using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.DataStructures;
using Microsoft.Xna.Framework;
using System.Runtime.InteropServices;

namespace BEPUphysics.BroadPhaseSystems.Hierarchies.Insertion
{
    public class DynamicHierarchy2 : BroadPhase
    {
        Node root;

        protected override void UpdateMultithreaded()
        {
            UpdateSingleThreaded();
        }

        public static bool DEBUGAllowRefit = true;
        RawList<TreeOverlapPair<BroadPhaseEntry, BroadPhaseEntry>> treeOverlaps = new RawList<TreeOverlapPair<BroadPhaseEntry, BroadPhaseEntry>>();
        protected override void UpdateSingleThreaded()
        {
            if (DEBUGAllowRefit)
                root.Refit();
            //(root as InternalNode).Revalidate();

            Overlaps.Clear();
            root.GetOverlaps(root, this);
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
                    internalNode.maximumVolume = internalNode.currentVolume * InternalNode.MaximumVolumeScale;
                    //The caller is responsible for the merge.
                    var treeNode = root;
                    while (!treeNode.TryToInsert(node, out treeNode)) ;//TryToInsert returns the next node, if any, and updates node bounding box.
                }
            }
        }
        public override void Remove(BroadPhaseEntry entry)
        {
        }


        public void Analyze(List<int> depths, out int nodeCount)
        {
            nodeCount = 0;
            root.Analyze(depths, 0, ref nodeCount);
        }

        public void ForceRevalidation()
        {
            (root as InternalNode).Revalidate();
        }
    }

}