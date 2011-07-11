using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.DataStructures;
using Microsoft.Xna.Framework;
using System.Runtime.InteropServices;
using BEPUphysics.Threading;
using BEPUphysics.ResourceManagement;

namespace BEPUphysics.BroadPhaseSystems.Hierarchies
{
    /// <summary>
    /// Broad phase that incrementally updates the internal tree acceleration structure.
    /// </summary>
    /// <remarks>
    /// This is a good all-around broad phase; its performance is consistent and all queries are supported and speedy.
    /// The memory usage is higher than simple one-axis sort and sweep, but a bit lower than the Grid2DSortAndSweep option.
    /// </remarks>
    public class DynamicHierarchy : BroadPhase
    {
        internal Node root;

        /// <summary>
        /// Constructs a new dynamic hierarchy broad phase.
        /// </summary>
        public DynamicHierarchy()
        {
            multithreadedRefit = MultithreadedRefit;
            multithreadedOverlap = MultithreadedOverlap;
            QueryAccelerator = new DynamicHierarchyQueryAccelerator(this);
        }

        /// <summary>
        /// Constructs a new dynamic hierarchy broad phase.
        /// </summary>
        /// <param name="threadManager">Thread manager to use in the broad phase.</param>
        public DynamicHierarchy(IThreadManager threadManager)
            : base(threadManager)
        {
            multithreadedRefit = MultithreadedRefit;
            multithreadedOverlap = MultithreadedOverlap;
            QueryAccelerator = new DynamicHierarchyQueryAccelerator(this);
        }

        #region Multithreading

        protected override void UpdateMultithreaded()
        {
            lock (Locker)
            {
                Overlaps.Clear();
                if (root != null)
                {
                    //To multithread the tree traversals, we have to do a little single threaded work.
                    //Dive down into the tree far enough that there are enough nodes to split amongst all the threads in the thread manager.
                    int splitDepth = (int)Math.Ceiling(Math.Log(ThreadManager.ThreadCount, 2));

                    root.CollectMultithreadingNodes(splitDepth, 1, multithreadingSourceNodes);
                    //Go through every node and refit it.
                    ThreadManager.ForLoop(0, multithreadingSourceNodes.count, multithreadedRefit);
                    multithreadingSourceNodes.Clear();
                    //Now that the subtrees belonging to the source nodes are refit, refit the top nodes.
                    //Sometimes, this will go deeper than necessary because the refit process may require an extremely high level (nonmultithreaded) revalidation.
                    //The waste cost is a matter of nanoseconds due to the simplicity of the operations involved.
                    root.PostRefit(splitDepth, 1);

                    //The trees are now fully refit (and revalidated, if the refit process found it to be necessary).
                    //The overlap traversal is conceptually similar to the multithreaded refit, but is a bit easier since there's no need to go back up the stack.
                    if (!root.IsLeaf) //If the root is a leaf, it's alone- nothing to collide against! This test is required by the assumptions of the leaf-leaf test.
                    {
                        root.GetMultithreadedOverlaps(root, splitDepth, 1, this, multithreadingSourceOverlaps);
                        ThreadManager.ForLoop(0, multithreadingSourceOverlaps.count, multithreadedOverlap);
                        multithreadingSourceOverlaps.Clear();
                    }
                }
            }

        }

        internal struct NodePair
        {
            internal Node a;
            internal Node b;
        }

        RawList<Node> multithreadingSourceNodes = new RawList<Node>(4);
        Action<int> multithreadedRefit;
        void MultithreadedRefit(int i)
        {
            multithreadingSourceNodes.Elements[i].Refit();
        }

        RawList<NodePair> multithreadingSourceOverlaps = new RawList<NodePair>(10);
        Action<int> multithreadedOverlap;
        void MultithreadedOverlap(int i)
        {
            var overlap = multithreadingSourceOverlaps.Elements[i];
            //Note: It's okay not to check to see if a and b are equal and leaf nodes, because the systems which added nodes to the list already did it.
            overlap.a.GetOverlaps(overlap.b, this);
        }

        #endregion

        protected override void UpdateSingleThreaded()
        {
            lock (Locker)
            {
                Overlaps.Clear();
                if (root != null)
                {
                    root.Refit();

                    if (!root.IsLeaf) //If the root is a leaf, it's alone- nothing to collide against! This test is required by the assumptions of the leaf-leaf test.
                        root.GetOverlaps(root, this);
                }
            }
        }

        UnsafeResourcePool<LeafNode> leafNodes = new UnsafeResourcePool<LeafNode>();

        /// <summary>
        /// Adds an entry to the hierarchy.
        /// </summary>
        /// <param name="entry">Entry to remove.</param>
        public override void Add(BroadPhaseEntry entry)
        {
            //Entities do not set up their own bounding box before getting stuck in here.  If they're all zeroed out, the tree will be horrible.
            Vector3 offset;
            Vector3.Subtract(ref entry.boundingBox.Max, ref entry.boundingBox.Min, out offset);
            if (offset.X * offset.Y * offset.Z == 0)
                entry.UpdateBoundingBox();
            //Could buffer additions to get a better construction in the tree.
            var node = leafNodes.Take();
            node.Initialize(entry);
            if (root == null)
            {
                //Empty tree.  This is the first and only node.
                root = node;
            }
            else
            {
                if (root.IsLeaf) //Root is alone.
                    root.TryToInsert(node, out root);
                else
                {
                    BoundingBox.CreateMerged(ref node.BoundingBox, ref root.BoundingBox, out root.BoundingBox);
                    var internalNode = (InternalNode)root;
                    Vector3.Subtract(ref root.BoundingBox.Max, ref root.BoundingBox.Min, out offset);
                    internalNode.currentVolume = offset.X * offset.Y * offset.Z;
                    //internalNode.maximumVolume = internalNode.currentVolume * InternalNode.MaximumVolumeScale;
                    //The caller is responsible for the merge.
                    var treeNode = root;
                    while (!treeNode.TryToInsert(node, out treeNode)) ;//TryToInsert returns the next node, if any, and updates node bounding box.
                }
            }
        }
        /// <summary>
        /// Removes an entry from the hierarchy.
        /// </summary>
        /// <param name="entry">Entry to remove.</param>
        public override void Remove(BroadPhaseEntry entry)
        {
            if (root == null)
                throw new InvalidOperationException("Entry not present in the hierarchy.");
            LeafNode leafNode;
            if (root.Remove(entry, out leafNode, out root))
            {
                leafNode.CleanUp();
                leafNodes.GiveBack(leafNode);
            }
            else
                throw new InvalidOperationException("Entry not present in the hierarchy.");
        }

        #region Debug
        internal void Analyze(List<int> depths, out int nodeCount)
        {
            nodeCount = 0;
            root.Analyze(depths, 0, ref nodeCount);
        }

        internal void ForceRevalidation()
        {
            ((InternalNode)root).Revalidate();
        }
        #endregion
    }

}