using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.DataStructures;
using Microsoft.Xna.Framework;
using System.Runtime.InteropServices;
using BEPUphysics.Threading;

namespace BEPUphysics.BroadPhaseSystems.Hierarchies.TopDown
{
    public class DynamicHierarchy4 : BroadPhase
    {
        internal Node root;

        public DynamicHierarchy4()
        {
            multithreadedRefit = MultithreadedRefit;
            multithreadedOverlap = MultithreadedOverlap; 
            QueryAccelerator = new DynamicHierarchyQueryAccelerator4(this);
        }

        public DynamicHierarchy4(IThreadManager threadManager)
            : base(threadManager)
        {
            multithreadedRefit = MultithreadedRefit;
            multithreadedOverlap = MultithreadedOverlap;
            QueryAccelerator = new DynamicHierarchyQueryAccelerator4(this);
        }

        #region Multithreading

        protected override void UpdateMultithreaded()
        {
            lock (Locker)
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
                Overlaps.Clear();
                root.GetMultithreadedOverlaps(root, splitDepth, 1, this, multithreadingSourceOverlaps);
                ThreadManager.ForLoop(0, multithreadingSourceOverlaps.count, multithreadedOverlap);
                multithreadingSourceOverlaps.Clear();
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
            overlap.a.GetOverlaps(overlap.b, this);
        }

        #endregion

        protected override void UpdateSingleThreaded()
        {
            lock (Locker)
            {
                root.Refit();

                Overlaps.Clear();
                root.GetOverlaps(root, this);
            }
        }

        public override void Add(BroadPhaseEntry entry)
        {
            //Could buffer additions to get a better construction in the tree.
            var node = new LeafNode(entry);
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
                    Vector3 offset;
                    Vector3.Subtract(ref root.BoundingBox.Max, ref root.BoundingBox.Min, out offset);
                    internalNode.currentVolume = offset.X * offset.Y * offset.Z;
                    //internalNode.maximumVolume = internalNode.currentVolume * InternalNode.MaximumVolumeScale;
                    //The caller is responsible for the merge.
                    var treeNode = root;
                    while (!treeNode.TryToInsert(node, out treeNode)) ;//TryToInsert returns the next node, if any, and updates node bounding box.
                }
            }
        }
        public override void Remove(BroadPhaseEntry entry)
        {
        }

        #region Debug
        public void Analyze(List<int> depths, out int nodeCount)
        {
            nodeCount = 0;
            root.Analyze(depths, 0, ref nodeCount);
        }

        public void ForceRevalidation()
        {
            (root as InternalNode).Revalidate();
        }
        #endregion
    }

}