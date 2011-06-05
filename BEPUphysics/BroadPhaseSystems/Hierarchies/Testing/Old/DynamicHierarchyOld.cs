using System;
using System.Collections.Generic;
using BEPUphysics.ResourceManagement;
using Microsoft.Xna.Framework;
using BEPUphysics.Threading;

namespace BEPUphysics.BroadPhaseSystems.Hierarchies.Testing.Old
{
    ///<summary>
    /// Broadphase which uses a dynamically changing bounding box tree to calculate overlaps.
    /// It has good insertion and update speed.
    ///</summary>
    public class DynamicHierarchyOld : BroadPhase
    {
        private readonly ResourcePool<DynamicHierarchyNodeOld> dynamicHierarchyNodes = new LockingResourcePool<DynamicHierarchyNodeOld>(128);

        internal DynamicHierarchyNodeOld GetNode()
        {
            //Debug.WriteLine("S: " + dynamicHierarchyNodes.sharedPool.Count);
            //Debug.WriteLine("TS: " + dynamicHierarchyNodes.threadSpecificPools.Length);

            var toReturn = dynamicHierarchyNodes.Take();

            toReturn.hierarchy = this;
            return toReturn;

        }

        internal void GiveBack(DynamicHierarchyNodeOld node)
        {
            foreach (var childNode in node.children)
            {
                GiveBack(childNode);
            }
            node.entries.Clear();
            node.currentVolume = 0;
            node.maximumAllowedVolume = 0;
            node.children.Clear();

            dynamicHierarchyNodes.GiveBack(node);
        }

        /// <summary>
        /// When an internal node is revalidated, its volume is stored.  When a node's volume exceeds its stored volume multiplied by this factor, it is revalidated again.
        /// </summary>
        public float MaximumAllowedVolumeFactor = 1.4f;

        /// <summary>
        /// Maximum fraction of a parent's entities that a child can inherit.
        /// If a child has as much or more, the validation process is done over again to ensure a more even split.
        /// </summary>
        public float MaximumChildEntityLoad = .8f;

        /// <summary>
        /// The maximum number of entities present in the leaf nodes of the hierarchy.
        /// </summary>
        public int MaximumEntitiesInLeaves = 2;

        /// <summary>
        /// The number of entities needed in a particular node to use the multithreaded reconstruction method on it.
        /// If the node has less, the current thread does the remainder of the subtree itself.
        /// This only applies if multithreading is currently being used.
        /// </summary>
        public int MinimumNodeEntitiesRequiredToMultithread = 100;

        /// <summary>
        /// Highest parent in the hierarchy.
        /// </summary>
        public DynamicHierarchyNodeOld Root;

        /// <summary>
        /// Constructs a new instance of the hierarchy and sets up the root node.
        /// </summary>
        public DynamicHierarchyOld()
        {
            Root = new DynamicHierarchyNodeOld(this);
            revalidateMultithreadedDelegate = RevalidateMultithreaded;
            updateCollisionMultithreadedSubFunctionDelegate = UpdateCollisionMultithreadedSubFunction;
            updateCollisionsMultithreadedSubFunctionTaskOverheadDelegate = UpdateCollisionsMultithreadedSubFunctionTaskOverhead;
            QueryAccelerator = new DynamicHierarchyQueryAcceleratorOld(this);
        }

        /// <summary>
        /// Constructs a new instance of the hierarchy and sets up the root node.
        /// </summary>
        public DynamicHierarchyOld(IThreadManager threadManager)
            : base(threadManager)
        {
            Root = new DynamicHierarchyNodeOld(this);
            revalidateMultithreadedDelegate = RevalidateMultithreaded;
            updateCollisionMultithreadedSubFunctionDelegate = UpdateCollisionMultithreadedSubFunction;
            updateCollisionsMultithreadedSubFunctionTaskOverheadDelegate = UpdateCollisionsMultithreadedSubFunctionTaskOverhead;

            QueryAccelerator = new DynamicHierarchyQueryAcceleratorOld(this);
        }

        /// <summary>
        /// Adds an entity to the hierarchy.  Called automatically by the Space owning this broad phase system when an entity is added.
        /// </summary>
        /// <param name="entry">Entry to add.</param>
        public override void Add(BroadPhaseEntry entry)
        {
            lock (Locker)
            {

                Root.Add(entry);

            }

        }


        /// <summary>
        /// Removes an entity from the hierarchy.  Called automatically by the Space owning this broad phase system when an entity is removed.
        /// </summary>
        /// <param name="entry">Entry to add.</param>
        public override void Remove(BroadPhaseEntry entry)
        {
            lock (Locker)
            {
                Root.Remove(entry, Root.entries.IndexOf(entry));

            }

        }

        internal void TryToAdd(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {
            bool intersects;
            entryA.boundingBox.Intersects(ref entryB.boundingBox, out intersects);
            if (intersects)
                TryToAddOverlap(entryA, entryB);
        }

        #region For Multithreading

        private readonly Queue<DynamicHierarchyNodeOld> aNodes = new Queue<DynamicHierarchyNodeOld>();
        private readonly Queue<DynamicHierarchyNodeOld> bNodes = new Queue<DynamicHierarchyNodeOld>();
        private readonly ResourcePool<HierarchyNodeComparison> nodeComparisons = new LockingResourcePool<HierarchyNodeComparison>(128);
        private readonly Action<object> revalidateMultithreadedDelegate;
        private readonly Action<object> updateCollisionMultithreadedSubFunctionDelegate;
        private readonly Action<object> updateCollisionsMultithreadedSubFunctionTaskOverheadDelegate;
        private readonly Queue<HierarchyNodeComparison> usedComparisons = new Queue<HierarchyNodeComparison>(128);

        private static void UpdateCollisionMultithreadedSubFunction(object information)
        {
            var comparison = information as HierarchyNodeComparison;

            comparison.a.BinaryCollideAgainst(comparison.b);
        }

        private HierarchyNodeComparison GetNodeComparison(DynamicHierarchyNodeOld aNode, DynamicHierarchyNodeOld bNode)
        {
            var toReturn = nodeComparisons.Take();
            toReturn.Setup(aNode, bNode);
            usedComparisons.Enqueue(toReturn);
            return toReturn;
        }

        private void GiveBack(HierarchyNodeComparison comparison)
        {
            nodeComparisons.GiveBack(comparison);
        }

        private void RedoSplitMultithreaded(object information)
        {
            var node = information as DynamicHierarchyNodeOld;
            foreach (var child in node.children)
            {
                GiveBack(child);
            }
            node.children.Clear();

            node.entries.Sort(node.axisComparer);

            var left = GetNode();
            var right = GetNode();

            for (int i = 0; i < node.entries.Count / 2; i++)
            {
                right.entries.Add(node.entries[i]);
            }
            for (int i = node.entries.Count / 2; i < node.entries.Count; i++)
            {
                left.entries.Add(node.entries[i]);
            }

            node.children.Add(left);
            node.children.Add(right);
            if (right.entries.Count >= MinimumNodeEntitiesRequiredToMultithread)
                ThreadManager.EnqueueTask(revalidateMultithreadedDelegate, right);
            else
                right.Revalidate();
            if (left.entries.Count >= MinimumNodeEntitiesRequiredToMultithread)
                ThreadManager.EnqueueTask(revalidateMultithreadedDelegate, left);
            else
                left.Revalidate();
        }

        private void RevalidateMultithreaded(object information)
        {
            var node = information as DynamicHierarchyNodeOld;
            if (node.entries.Count > 0)
            {
                node.BoundingBox = node.entries[0].boundingBox;
                for (int i = 1; i < node.entries.Count; i++)
                {
                    BoundingBox.CreateMerged(ref node.BoundingBox, ref node.entries[i].boundingBox, out node.BoundingBox);
                }
            }
            else
                node.BoundingBox = new BoundingBox();
            if (node.entries.Count <= MaximumEntitiesInLeaves)
            {
                //Revalidating this node won't do anything.
                if (node.children.Count > 0)
                {
                    //Debug.WriteLine("Whoa there nelly,.");
                    //Victim of removal.  Get rid of the children, they're not necessary.
                    foreach (var oldChild in node.children)
                    {
                        GiveBack(oldChild);
                    }
                    node.children.Clear();
                }
                return;
            }


            Vector3 difference;
            Vector3.Subtract(ref node.BoundingBox.Max, ref node.BoundingBox.Min, out difference);
            node.currentVolume = difference.X * difference.Y * difference.Z;
            node.maximumAllowedVolume = node.currentVolume * MaximumAllowedVolumeFactor;


            //Clear out old tree.
            foreach (var oldChild in node.children)
            {
                GiveBack(oldChild);
            }
            node.children.Clear();

            //Top-down reconstruction.
            Vector3 min = node.BoundingBox.Min;
            Vector3 max = node.BoundingBox.Max;
            var left = GetNode();
            var right = GetNode();
            float xDifference = max.X - min.X;
            float yDifference = max.Y - min.Y;
            float zDifference = max.Z - min.Z;
            float midpoint;
            DynamicHierarchyNodeOld.ComparerAxis minimumAxis;
            if (xDifference > yDifference && xDifference > zDifference)
            {
                minimumAxis = DynamicHierarchyNodeOld.ComparerAxis.X;
                midpoint = (max.X + min.X) * .5f;
                foreach (var e in node.entries)
                {
                    float x = (e.boundingBox.Max.X + e.boundingBox.Min.X) * .5f;
                    if (x > midpoint)
                        left.entries.Add(e);
                    else
                        right.entries.Add(e);
                }
            }
            else if (yDifference > xDifference && yDifference > zDifference)
            {
                minimumAxis = DynamicHierarchyNodeOld.ComparerAxis.Z;
                midpoint = (max.Y + min.Y) * .5f;
                foreach (var e in node.entries)
                {
                    float y = (e.boundingBox.Max.Y + e.boundingBox.Min.Y) * .5f;
                    if (y > midpoint)
                        left.entries.Add(e);
                    else
                        right.entries.Add(e);
                }
            }
            else // if (zDifference > xDifference && zDifference > yDifference)
            {
                minimumAxis = DynamicHierarchyNodeOld.ComparerAxis.Z;
                midpoint = (max.Z + min.Z) * .5f;
                foreach (var e in node.entries)
                {
                    float z = (e.boundingBox.Max.Z + e.boundingBox.Min.Z) * .5f;
                    if (z > midpoint)
                        left.entries.Add(e);
                    else
                        right.entries.Add(e);
                }
            }

            var maxEntityCount = (int)(node.entries.Count * MaximumChildEntityLoad);
            if (left.entries.Count >= maxEntityCount)
            {
                GiveBack(left);
                GiveBack(right);
                node.axisComparer.axis = minimumAxis;
                RedoSplitMultithreaded(node);
            }
            else
            {
                if (right.entries.Count >= maxEntityCount)
                {
                    GiveBack(left);
                    GiveBack(right);
                    node.axisComparer.axis = minimumAxis;
                    RedoSplitMultithreaded(node);
                }
                else
                {
                    node.children.Add(left);
                    node.children.Add(right);
                    if (right.entries.Count >= MinimumNodeEntitiesRequiredToMultithread)
                        ThreadManager.EnqueueTask(revalidateMultithreadedDelegate, right);
                    else
                        right.Revalidate();
                    if (left.entries.Count >= MinimumNodeEntitiesRequiredToMultithread)
                        ThreadManager.EnqueueTask(revalidateMultithreadedDelegate, left);
                    else
                        left.Revalidate();
                }
            }
        }

        private void UpdateCollisionsMultithreaded()
        {
            //Basic idea is to use a breadth first traversal.  When a certain number of nodes are visited in a tree of any significant complexity
            //compared to the number of threads available, all further visitable nodes are enqueued as tasks to be solved in parallel.
            //This doesn't scale infinitely, but works great for times when the number of nodes is much larger than the number of threads.
            //This avoids the task queue's significant overhead since there's only as many tasks enqueued as there are threads (within a factor of 4).
            aNodes.Enqueue(Root);
            bNodes.Enqueue(Root);
            DynamicHierarchyNodeOld aNode, bNode;
            bool shouldUseTasks = false;
            while (aNodes.Count > 0)
            {
                aNode = aNodes.Dequeue();
                bNode = bNodes.Dequeue();
                if (!shouldUseTasks && aNodes.Count >= ThreadManager.ThreadCount - 1)
                    shouldUseTasks = true;
                if (shouldUseTasks)
                {
                    //Toss the job off to a thread.
                    ThreadManager.EnqueueTask(updateCollisionMultithreadedSubFunctionDelegate, GetNodeComparison(aNode, bNode));
                }
                else
                {
                    //Base idea: recurse down the tree whenever child bounding boxes overlap.
                    //Attempt to get to the base case of leaf vs. leaf.
                    bool intersecting;
                    if (aNode == bNode)
                    {
                        //Very simple case; the test between the nodes
                        if (aNode.children.Count == 0)
                        {
                            //Leafwise tests
                            for (int i = 0; i < aNode.entries.Count - 1; i++)
                            {
                                for (int j = i + 1; j < aNode.entries.Count; j++)
                                {
                                    TryToAdd(aNode.entries[i], aNode.entries[j]);
                                }
                            }
                        }
                        else
                        {
                            //Nodewise test

                            aNode.children[0].BoundingBox.Intersects(ref aNode.children[1].BoundingBox, out intersecting);
                            if (intersecting)
                            {
                                //aNode.children[0].binaryCollideAgainst(aNode.children[1]);
                                aNodes.Enqueue(aNode.children[0]);
                                bNodes.Enqueue(aNode.children[1]);
                            }


                            //Self test
                            aNodes.Enqueue(aNode.children[0]);
                            bNodes.Enqueue(aNode.children[0]);
                            aNodes.Enqueue(aNode.children[1]);
                            bNodes.Enqueue(aNode.children[1]);
                        }
                    }
                    else
                    {
                        if (aNode.children.Count > 0 && bNode.children.Count > 0)
                        {
                            //Both involved nodes are not leaves.
                            aNode.children[0].BoundingBox.Intersects(ref bNode.children[0].BoundingBox, out intersecting);
                            if (intersecting)
                            {
                                //aNode.children[0].binaryCollideAgainst(bNode.children[0]);
                                aNodes.Enqueue(aNode.children[0]);
                                bNodes.Enqueue(bNode.children[0]);
                            }

                            aNode.children[0].BoundingBox.Intersects(ref bNode.children[1].BoundingBox, out intersecting);
                            if (intersecting)
                            {
                                //aNode.children[0].binaryCollideAgainst(bNode.children[1]);
                                aNodes.Enqueue(aNode.children[0]);
                                bNodes.Enqueue(bNode.children[1]);
                            }

                            aNode.children[1].BoundingBox.Intersects(ref bNode.children[0].BoundingBox, out intersecting);
                            if (intersecting)
                            {
                                //aNode.children[1].binaryCollideAgainst(bNode.children[0]);
                                aNodes.Enqueue(aNode.children[1]);
                                bNodes.Enqueue(bNode.children[0]);
                            }

                            aNode.children[1].BoundingBox.Intersects(ref bNode.children[1].BoundingBox, out intersecting);
                            if (intersecting)
                            {
                                //aNode.children[1].binaryCollideAgainst(bNode.children[1]);
                                aNodes.Enqueue(aNode.children[1]);
                                bNodes.Enqueue(bNode.children[1]);
                            }
                        }
                        else if (aNode.children.Count > 0)
                        {
                            //Opposing node is a leaf.
                            aNode.children[0].BoundingBox.Intersects(ref bNode.BoundingBox, out intersecting);
                            if (intersecting)
                            {
                                //aNode.children[0].binaryCollideAgainst(bNode);
                                aNodes.Enqueue(aNode.children[0]);
                                bNodes.Enqueue(bNode);
                            }

                            aNode.children[1].BoundingBox.Intersects(ref bNode.BoundingBox, out intersecting);
                            if (intersecting)
                            {
                                //aNode.children[1].binaryCollideAgainst(bNode);
                                aNodes.Enqueue(aNode.children[1]);
                                bNodes.Enqueue(bNode);
                            }
                        }
                        else if (bNode.children.Count > 0)
                        {
                            //I'm a leaf
                            aNode.BoundingBox.Intersects(ref bNode.children[0].BoundingBox, out intersecting);
                            if (intersecting)
                            {
                                //aNode.binaryCollideAgainst(bNode.children[0]);
                                aNodes.Enqueue(aNode);
                                bNodes.Enqueue(bNode.children[0]);
                            }

                            aNode.BoundingBox.Intersects(ref bNode.children[1].BoundingBox, out intersecting);
                            if (intersecting)
                            {
                                //aNode.binaryCollideAgainst(bNode.children[1]);
                                aNodes.Enqueue(aNode);
                                bNodes.Enqueue(bNode.children[1]);
                            }
                        }
                        else
                        {
                            //Both leaves!
                            //Entity vs. entity test.
                            for (int i = 0; i < aNode.entries.Count; i++)
                            {
                                for (int j = 0; j < bNode.entries.Count; j++)
                                {
                                    TryToAdd(aNode.entries[i], bNode.entries[j]);
                                }
                            }
                        }
                    }
                }
            }
        }

        private void UpdateCollisionsMultithreadedSubFunctionTaskOverhead(object information)
        {
            var comparison = information as HierarchyNodeComparison;
            var aNode = comparison.a;
            var bNode = comparison.b;

            //Base idea: recurse down the tree whenever child bounding boxes overlap.
            //Attempt to get to the base case of leaf vs. leaf.
            bool intersecting;
            if (aNode == bNode)
            {
                //Very simple case; the test between the nodes
                if (aNode.children.Count == 0)
                {
                    //Leafwise tests
                    for (int i = 0; i < aNode.entries.Count - 1; i++)
                    {
                        for (int j = i + 1; j < aNode.entries.Count; j++)
                        {
                            TryToAdd(aNode.entries[i], aNode.entries[j]);
                        }
                    }
                }
                else
                {
                    //Nodewise test

                    aNode.children[0].BoundingBox.Intersects(ref aNode.children[1].BoundingBox, out intersecting);
                    if (intersecting)
                    {
                        //aNode.children[0].binaryCollideAgainst(aNode.children[1]);
                        ThreadManager.EnqueueTask(updateCollisionsMultithreadedSubFunctionTaskOverheadDelegate, GetNodeComparison(aNode.children[0], bNode.children[1]));
                        //aNodes.Enqueue(aNode.children[0]);
                        //bNodes.Enqueue(aNode.children[1]);
                    }


                    //Self test
                    ThreadManager.EnqueueTask(updateCollisionsMultithreadedSubFunctionTaskOverheadDelegate, GetNodeComparison(aNode.children[0], aNode.children[0]));
                    ThreadManager.EnqueueTask(updateCollisionsMultithreadedSubFunctionTaskOverheadDelegate, GetNodeComparison(aNode.children[1], aNode.children[1]));
                    //aNodes.Enqueue(aNode.children[0]);
                    //bNodes.Enqueue(aNode.children[0]);
                    //aNodes.Enqueue(aNode.children[1]);
                    //bNodes.Enqueue(aNode.children[1]);
                }
            }
            else
            {
                if (aNode.children.Count > 0 && bNode.children.Count > 0)
                {
                    //Both involved nodes are not leaves.
                    aNode.children[0].BoundingBox.Intersects(ref bNode.children[0].BoundingBox, out intersecting);
                    if (intersecting)
                    {
                        //aNode.children[0].binaryCollideAgainst(bNode.children[0]);
                        ThreadManager.EnqueueTask(updateCollisionsMultithreadedSubFunctionTaskOverheadDelegate, GetNodeComparison(aNode.children[0], bNode.children[0]));
                        //aNodes.Enqueue(aNode.children[0]);
                        //bNodes.Enqueue(bNode.children[0]);
                    }

                    aNode.children[0].BoundingBox.Intersects(ref bNode.children[1].BoundingBox, out intersecting);
                    if (intersecting)
                    {
                        //aNode.children[0].binaryCollideAgainst(bNode.children[1]);
                        ThreadManager.EnqueueTask(updateCollisionsMultithreadedSubFunctionTaskOverheadDelegate, GetNodeComparison(aNode.children[0], bNode.children[1]));
                        //aNodes.Enqueue(aNode.children[0]);
                        //bNodes.Enqueue(bNode.children[1]);
                    }

                    aNode.children[1].BoundingBox.Intersects(ref bNode.children[0].BoundingBox, out intersecting);
                    if (intersecting)
                    {
                        //aNode.children[1].binaryCollideAgainst(bNode.children[0]);
                        ThreadManager.EnqueueTask(updateCollisionsMultithreadedSubFunctionTaskOverheadDelegate, GetNodeComparison(aNode.children[1], bNode.children[0]));
                        //aNodes.Enqueue(aNode.children[1]);
                        //bNodes.Enqueue(bNode.children[0]);
                    }

                    aNode.children[1].BoundingBox.Intersects(ref bNode.children[1].BoundingBox, out intersecting);
                    if (intersecting)
                    {
                        //aNode.children[1].binaryCollideAgainst(bNode.children[1]);
                        ThreadManager.EnqueueTask(updateCollisionsMultithreadedSubFunctionTaskOverheadDelegate, GetNodeComparison(aNode.children[1], bNode.children[1]));
                        //aNodes.Enqueue(aNode.children[1]);
                        //bNodes.Enqueue(bNode.children[1]);
                    }
                }
                else if (aNode.children.Count > 0)
                {
                    //Opposing node is a leaf.
                    aNode.children[0].BoundingBox.Intersects(ref bNode.BoundingBox, out intersecting);
                    if (intersecting)
                    {
                        //aNode.children[0].binaryCollideAgainst(bNode);
                        ThreadManager.EnqueueTask(updateCollisionsMultithreadedSubFunctionTaskOverheadDelegate, GetNodeComparison(aNode.children[0], bNode));
                        //aNodes.Enqueue(aNode.children[0]);
                        //bNodes.Enqueue(bNode);
                    }

                    aNode.children[1].BoundingBox.Intersects(ref bNode.BoundingBox, out intersecting);
                    if (intersecting)
                    {
                        //aNode.children[1].binaryCollideAgainst(bNode);
                        ThreadManager.EnqueueTask(updateCollisionsMultithreadedSubFunctionTaskOverheadDelegate, GetNodeComparison(aNode.children[1], bNode));
                        //aNodes.Enqueue(aNode.children[1]);
                        //bNodes.Enqueue(bNode);
                    }
                }
                else if (bNode.children.Count > 0)
                {
                    //I'm a leaf
                    aNode.BoundingBox.Intersects(ref bNode.children[0].BoundingBox, out intersecting);
                    if (intersecting)
                    {
                        //aNode.binaryCollideAgainst(bNode.children[0]);
                        ThreadManager.EnqueueTask(updateCollisionsMultithreadedSubFunctionTaskOverheadDelegate, GetNodeComparison(aNode, bNode.children[0]));
                        //aNodes.Enqueue(aNode);
                        //bNodes.Enqueue(bNode.children[0]);
                    }

                    aNode.BoundingBox.Intersects(ref bNode.children[1].BoundingBox, out intersecting);
                    if (intersecting)
                    {
                        //aNode.binaryCollideAgainst(bNode.children[1]);
                        ThreadManager.EnqueueTask(updateCollisionsMultithreadedSubFunctionTaskOverheadDelegate, GetNodeComparison(aNode, bNode.children[1]));
                        //aNodes.Enqueue(aNode);
                        //bNodes.Enqueue(bNode.children[1]);
                    }
                }
                else
                {
                    //Both leaves!
                    //Entity vs. entity test.
                    for (int i = 0; i < aNode.entries.Count; i++)
                    {
                        for (int j = 0; j < bNode.entries.Count; j++)
                        {
                            TryToAdd(aNode.entries[i], bNode.entries[j]);
                        }
                    }
                }
            }
        }

        private void UpdateNodeMultithreaded(DynamicHierarchyNodeOld node)
        {
            //revalidate();
            //return;
            /*float totalChildrenSum = 0;
            Vector3 diff;
                
            foreach (DynamicHierarchyNode child in children)
            {
                Vector3.Subtract(ref child.boundingBox.Max, ref child.boundingBox.Min, out diff);
                totalChildrenSum += (diff.X * diff.Y * diff.Z);
            }
            if(currentVolume / totalChildrenSum < .9f)*/
            if (node.currentVolume > node.maximumAllowedVolume)
            {
                //Debug.WriteLine("volume exceeded, current: " + currentVolume + ", max: " + maximumAllowedVolume);
                if (node.entries.Count >= MinimumNodeEntitiesRequiredToMultithread)
                    ThreadManager.EnqueueTask(revalidateMultithreadedDelegate, node);
                else
                    node.Revalidate();
            }
            else
            {
                if (node.children.Count > 0)
                {
                    //Internal Node
                    foreach (DynamicHierarchyNodeOld child in node.children)
                    {
                        UpdateNodeMultithreaded(child);
                    }
                    node.BoundingBox = node.children[0].BoundingBox;
                    for (int i = 1; i < node.children.Count; i++)
                    {
                        BoundingBox.CreateMerged(ref node.BoundingBox, ref node.children[i].BoundingBox, out node.BoundingBox);
                    }
                }
                else
                {
                    //Leaf Node
                    if (node.entries.Count > 0) //don't try to merge anything if there's nothing!
                    {
                        node.BoundingBox = node.entries[0].boundingBox;
                        for (int i = 1; i < node.entries.Count; i++)
                        {
                            BoundingBox.CreateMerged(ref node.BoundingBox, ref node.entries[i].boundingBox, out node.BoundingBox);
                        }
                    }
                    else
                        node.BoundingBox = new BoundingBox();
                }
                Vector3 difference;
                Vector3.Subtract(ref node.BoundingBox.Max, ref node.BoundingBox.Min, out difference);
                node.currentVolume = difference.X * difference.Y * difference.Z;
            }
        }

        private class HierarchyNodeComparison
        {
            internal DynamicHierarchyNodeOld a;
            internal DynamicHierarchyNodeOld b;

            internal void Setup(DynamicHierarchyNodeOld aNode, DynamicHierarchyNodeOld bNode)
            {
                a = aNode;
                b = bNode;
            }
        }

        #endregion


        protected override void UpdateMultithreaded()
        {
            Overlaps.Clear();
            lock (Locker)
            {
                UpdateNodeMultithreaded(Root);
                ThreadManager.WaitForTaskCompletion();


                //root.binaryUpdateNode();

                UpdateCollisionsMultithreaded();
                ThreadManager.WaitForTaskCompletion();
                while (usedComparisons.Count > 0)
                {
                    GiveBack(usedComparisons.Dequeue());
                }

                //root.binaryCollideAgainst(root);
            }
        }

        protected override void UpdateSingleThreaded()
        {
            Overlaps.Clear();
            lock (Locker)
            {
                Root.BinaryUpdateNode();
                Root.BinaryCollideAgainst(Root);
            }
        }
    }
}
