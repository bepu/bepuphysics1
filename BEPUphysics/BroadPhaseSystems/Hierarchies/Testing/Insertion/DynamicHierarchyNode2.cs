using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.DataStructures;
using BEPUphysics.ResourceManagement;
using System.Diagnostics;
using BEPUphysics.MathExtensions;

namespace BEPUphysics.BroadPhaseSystems.Hierarchies.Testing.Insertion
{
    internal abstract class Node
    {
        internal BoundingBox BoundingBox;
        internal abstract void GetOverlaps(ref BoundingBox boundingBox, IList<BroadPhaseEntry> outputOverlappedElements);
        internal abstract void GetOverlaps(ref BoundingSphere boundingSphere, IList<BroadPhaseEntry> outputOverlappedElements);
        //internal abstract void GetOverlaps(ref BoundingFrustum boundingFrustum, IList<BroadPhaseEntry> outputOverlappedElements);
        internal abstract void GetOverlaps(ref Ray ray, float maximumLength, IList<BroadPhaseEntry> outputOverlappedElements);
        internal abstract void GetOverlaps(Node node, DynamicHierarchy2 owner);

        internal abstract bool IsLeaf { get; }

        internal abstract Node ChildA { get; }
        internal abstract Node ChildB { get; }
        internal abstract BroadPhaseEntry Element { get; }

        internal abstract bool TryToInsert(LeafNode node, out Node treeNode);


        internal void BinaryCollideAgainst(Node node, DynamicHierarchy2 owner)
        {
            //Base idea: recurse down the tree whenever child bounding boxes overlap.
            //Attempt to get to the base case of leaf vs. leaf.
            bool intersecting;
            if (this == node)
            {
                //Very simple case; the test between the nodes
                if (IsLeaf)
                {
                    //Only one leaf in a leaf node. Do nothing.
                }
                else
                {
                    //Nodewise test

                    ChildA.BoundingBox.Intersects(ref ChildB.BoundingBox, out intersecting);
                    if (intersecting)
                    {
                        ChildA.BinaryCollideAgainst(ChildB, owner);
                    }


                    //Self test
                    ChildA.BinaryCollideAgainst(ChildA, owner);
                    ChildB.BinaryCollideAgainst(ChildB, owner);
                }
            }
            else
            {
                if (!IsLeaf && !node.IsLeaf)
                {
                    //Both involved nodes are not leaves.
                    ChildA.BoundingBox.Intersects(ref node.ChildA.BoundingBox, out intersecting);
                    if (intersecting)
                        ChildA.BinaryCollideAgainst(node.ChildA, owner);

                    ChildA.BoundingBox.Intersects(ref node.ChildB.BoundingBox, out intersecting);
                    if (intersecting)
                        ChildA.BinaryCollideAgainst(node.ChildB, owner);

                    ChildB.BoundingBox.Intersects(ref node.ChildA.BoundingBox, out intersecting);
                    if (intersecting)
                        ChildB.BinaryCollideAgainst(node.ChildA, owner);

                    ChildB.BoundingBox.Intersects(ref node.ChildB.BoundingBox, out intersecting);
                    if (intersecting)
                        ChildB.BinaryCollideAgainst(node.ChildB, owner);
                }
                else if (!IsLeaf)
                {
                    //Opposing node is a leaf.
                    ChildA.BoundingBox.Intersects(ref node.BoundingBox, out intersecting);
                    if (intersecting)
                        ChildA.BinaryCollideAgainst(node, owner);

                    ChildB.BoundingBox.Intersects(ref node.BoundingBox, out intersecting);
                    if (intersecting)
                        ChildB.BinaryCollideAgainst(node, owner);
                }
                else if (!node.IsLeaf)
                {
                    //I'm a leaf
                    BoundingBox.Intersects(ref node.ChildA.BoundingBox, out intersecting);
                    if (intersecting)
                        BinaryCollideAgainst(node.ChildA, owner);

                    BoundingBox.Intersects(ref node.ChildB.BoundingBox, out intersecting);
                    if (intersecting)
                        BinaryCollideAgainst(node.ChildB, owner);
                }
                else
                {
                    //Both leaves!
                    //Entity vs. entity test.

                    BoundingBox.Intersects(ref node.BoundingBox, out intersecting);
                    if (intersecting)
                        owner.TryToAddOverlap(Element, node.Element);

                }
            }
        }

        internal abstract void Analyze(List<int> depths, int depth, ref int nodeCount);

        internal abstract void Refit();

        internal abstract void RetrieveNodes(InternalNode parent);


    }

    internal sealed class InternalNode : Node
    {
        internal Node childA;
        internal Node childB;

        internal float currentVolume;
        internal float maximumVolume;

        internal static float MaximumVolumeScale = 1.4f;

        internal override Node ChildA
        {
            get
            {
                return childA;
            }
        }
        internal override Node ChildB
        {
            get
            {
                return childB;
            }
        }
        internal override BroadPhaseEntry Element
        {
            get
            {
                return default(BroadPhaseEntry);
            }
        }

        internal override bool IsLeaf
        {
            get { return false; }
        }


        internal override void GetOverlaps(ref BoundingBox boundingBox, IList<BroadPhaseEntry> outputOverlappedElements)
        {
            //Users of the GetOverlaps method will have to check the bounding box before calling
            //root.getoverlaps.  This is actually desired in some cases, since the outer bounding box is used
            //to determine a pair, and further overlap tests shouldn'BroadPhaseEntry bother retesting the root.
            bool intersects;
            childA.BoundingBox.Intersects(ref boundingBox, out intersects);
            if (intersects)
                childA.GetOverlaps(ref boundingBox, outputOverlappedElements);
            childB.BoundingBox.Intersects(ref boundingBox, out intersects);
            if (intersects)
                childB.GetOverlaps(ref boundingBox, outputOverlappedElements);
        }

        internal override void GetOverlaps(ref BoundingSphere boundingSphere, IList<BroadPhaseEntry> outputOverlappedElements)
        {
            bool intersects;
            childA.BoundingBox.Intersects(ref boundingSphere, out intersects);
            if (intersects)
                childA.GetOverlaps(ref boundingSphere, outputOverlappedElements);
            childB.BoundingBox.Intersects(ref boundingSphere, out intersects);
            if (intersects)
                childB.GetOverlaps(ref boundingSphere, outputOverlappedElements);
        }

        //internal override void GetOverlaps(ref BoundingFrustum boundingFrustum, IList<BroadPhaseEntry> outputOverlappedElements)
        //{
        //    bool intersects;
        //    boundingFrustum.Intersects(ref childA.BoundingBox, out intersects);
        //    if (intersects)
        //        childA.GetOverlaps(ref boundingFrustum, outputOverlappedElements);
        //    boundingFrustum.Intersects(ref childB.BoundingBox, out intersects);
        //    if (intersects)
        //        childB.GetOverlaps(ref boundingFrustum, outputOverlappedElements);
        //}

        internal override void GetOverlaps(ref Ray ray, float maximumLength, IList<BroadPhaseEntry> outputOverlappedElements)
        {
            float? result;
            ray.Intersects(ref childA.BoundingBox, out result);
            if (result != null && result < maximumLength)
                childA.GetOverlaps(ref ray, maximumLength, outputOverlappedElements);
            ray.Intersects(ref childB.BoundingBox, out result);
            if (result != null && result < maximumLength)
                childB.GetOverlaps(ref ray, maximumLength, outputOverlappedElements);
        }

        internal override void GetOverlaps(Node opposingNode, DynamicHierarchy2 owner)
        {
            bool intersects;

            if (this == opposingNode)
            {
                //We are being compared against ourselves!
                //Obviously we're an internal node, so spawn three children:
                //A versus A:
                if (!childA.IsLeaf) //This is performed in the child method usually by convention, but this saves some time.
                    childA.GetOverlaps(childA, owner);
                //B versus B:
                if (!childB.IsLeaf) //This is performed in the child method usually by convention, but this saves some time.
                    childB.GetOverlaps(childB, owner);
                //A versus B (if they intersect):
                childA.BoundingBox.Intersects(ref childB.BoundingBox, out intersects);
                if (intersects)
                    childA.GetOverlaps(childB, owner);

            }
            else
            {
                //Two different nodes.  The other one may be a leaf.
                if (opposingNode.IsLeaf)
                {
                    //If it's a leaf, go deeper in our hierarchy, but not the opposition.
                    childA.BoundingBox.Intersects(ref opposingNode.BoundingBox, out intersects);
                    if (intersects)
                        childA.GetOverlaps(opposingNode, owner);
                    childB.BoundingBox.Intersects(ref opposingNode.BoundingBox, out intersects);
                    if (intersects)
                        childB.GetOverlaps(opposingNode, owner);
                }
                else
                {
                    var opposingChildA = opposingNode.ChildA;
                    var opposingChildB = opposingNode.ChildB;
                    //If it's not a leaf, try to go deeper in both hierarchies.
                    childA.BoundingBox.Intersects(ref opposingChildA.BoundingBox, out intersects);
                    if (intersects)
                        childA.GetOverlaps(opposingChildA, owner);
                    childA.BoundingBox.Intersects(ref opposingChildB.BoundingBox, out intersects);
                    if (intersects)
                        childA.GetOverlaps(opposingChildB, owner);
                    childB.BoundingBox.Intersects(ref opposingChildA.BoundingBox, out intersects);
                    if (intersects)
                        childB.GetOverlaps(opposingChildA, owner);
                    childB.BoundingBox.Intersects(ref opposingChildB.BoundingBox, out intersects);
                    if (intersects)
                        childB.GetOverlaps(opposingChildB, owner);


                }
            }





        }

        internal static LockingResourcePool<InternalNode> nodePool = new LockingResourcePool<InternalNode>();
        internal override bool TryToInsert(LeafNode node, out Node treeNode)
        {
            ////The following can make the tree shorter, but it actually hurt query times in testing.
            //bool aIsLeaf = ChildA.IsLeaf;
            //bool bIsLeaf = ChildB.IsLeaf;
            //if (aIsLeaf && !bIsLeaf)
            //{
            //    //Just put us with the leaf.  Keeps the tree shallower.
            //    var newNode = nodePool.Take();
            //    BoundingBox.CreateMerged(ref childA.BoundingBox, ref node.BoundingBox, out newNode.BoundingBox);
            //    newNode.childA = this.childA;
            //    newNode.childB = node;
            //    childA = newNode;
            //    treeNode = null;
            //    return true;
            //}
            //else if (!aIsLeaf && bIsLeaf)
            //{
            //    //Just put us with the leaf.  Keeps the tree shallower.
            //    var newNode = nodePool.Take();
            //    BoundingBox.CreateMerged(ref childB.BoundingBox, ref node.BoundingBox, out newNode.BoundingBox);
            //    newNode.childA = node;
            //    newNode.childB = this.childB;
            //    childB = newNode;
            //    treeNode = null;
            //    return true;
            //}


            //Since we are an internal node, we know we have two children.
            //Regardless of what kind of nodes they are, figure out which would be a better choice to merge the new node with.

            //Use the path which produces the smallest 'volume.'
            BoundingBox mergedA, mergedB;
            BoundingBox.CreateMerged(ref childA.BoundingBox, ref node.BoundingBox, out mergedA);
            BoundingBox.CreateMerged(ref childB.BoundingBox, ref node.BoundingBox, out mergedB);

            Vector3 offset;
            float originalAVolume, originalBVolume;
            Vector3.Subtract(ref childA.BoundingBox.Max, ref childA.BoundingBox.Min, out offset);
            originalAVolume = offset.X * offset.Y * offset.Z;
            Vector3.Subtract(ref childB.BoundingBox.Max, ref childB.BoundingBox.Min, out offset);
            originalBVolume = offset.X * offset.Y * offset.Z;

            float mergedAVolume, mergedBVolume;
            Vector3.Subtract(ref mergedA.Max, ref mergedA.Min, out offset);
            mergedAVolume = offset.X * offset.Y * offset.Z;
            Vector3.Subtract(ref mergedB.Max, ref mergedB.Min, out offset);
            mergedBVolume = offset.X * offset.Y * offset.Z;

            //Could use factor increase or absolute difference
            if (mergedAVolume - originalAVolume < mergedBVolume - originalBVolume)
            {
                //merging A produces a better result.
                if (childA.IsLeaf)
                {
                    var newChildA = nodePool.Take();
                    newChildA.BoundingBox = mergedA;
                    newChildA.childA = this.childA;
                    newChildA.childB = node;
                    newChildA.currentVolume = mergedAVolume;
                    newChildA.maximumVolume = newChildA.currentVolume * MaximumVolumeScale;
                    childA = newChildA;
                    treeNode = null;
                    return true;
                }
                else
                {
                    childA.BoundingBox = mergedA;
                    var internalNode = (InternalNode)childA;
                    internalNode.currentVolume = mergedAVolume;
                    internalNode.maximumVolume = internalNode.currentVolume * MaximumVolumeScale;
                    treeNode = childA;
                    return false;
                }
            }
            else
            {
                //merging B produces a better result.
                if (childB.IsLeaf)
                {
                    //Target is a leaf! Return.
                    var newChildB = nodePool.Take();
                    newChildB.BoundingBox = mergedB;
                    newChildB.childA = node;
                    newChildB.childB = this.childB;
                    newChildB.currentVolume = mergedBVolume;
                    newChildB.maximumVolume = newChildB.currentVolume * MaximumVolumeScale;
                    childB = newChildB;
                    treeNode = null;
                    return true;
                }
                else
                {
                    childB.BoundingBox = mergedB;
                    treeNode = childB;
                    var internalNode = (InternalNode)childB;
                    internalNode.currentVolume = mergedBVolume;
                    internalNode.maximumVolume = internalNode.currentVolume * MaximumVolumeScale;
                    return false;
                }
            }



        }

        public override string ToString()
        {
            return "{" + childA.ToString() + ", " + childB.ToString() + "}";

        }

        internal override void Analyze(List<int> depths, int depth, ref int nodeCount)
        {
            nodeCount++;
            childA.Analyze(depths, depth + 1, ref nodeCount);
            childB.Analyze(depths, depth + 1, ref nodeCount);
        }

        internal override void Refit()
        {
            if (currentVolume > maximumVolume)
            {
                Revalidate();
                return;
            }
            childA.Refit();
            childB.Refit();
            BoundingBox.CreateMerged(ref childA.BoundingBox, ref childB.BoundingBox, out BoundingBox);
            //float DEBUGlastVolume = currentVolume;
            currentVolume = (BoundingBox.Max.X - BoundingBox.Min.X) * (BoundingBox.Max.Y - BoundingBox.Min.Y) * (BoundingBox.Max.Z - BoundingBox.Min.Z);
            //if (Math.Abs(currentVolume - DEBUGlastVolume) > .000001 * (DEBUGlastVolume + currentVolume))
            //    Debug.WriteLine(":Break>:)");
        }

        internal void Revalidate()
        {
            //The revalidation procedure 'reconstructs' a portion of the tree that has expanded beyond its old limits.
            //To reconstruct the tree, the nodes (internal and leaf) currently in use need to be retrieved.
            //The internal nodes can be put back into the nodePool.  LeafNodes are reinserted one by one into the new tree.
            //To retrieve the nodes, a depth-first search is used.

            //Given that an internal node is being revalidated, it is known that there are at least two children.
            var oldChildA = childA;
            var oldChildB = childB;
            childA = null;
            childB = null;
            oldChildA.RetrieveNodes(this);
            oldChildB.RetrieveNodes(this);
            currentVolume = (BoundingBox.Max.X - BoundingBox.Min.X) * (BoundingBox.Max.Y - BoundingBox.Min.Y) * (BoundingBox.Max.Z - BoundingBox.Min.Z);
            maximumVolume = currentVolume * MaximumVolumeScale;

        }

        internal override void RetrieveNodes(InternalNode parent)
        {
            var oldChildA = childA;
            var oldChildB = childB;
            childA = null;
            childB = null;
            nodePool.GiveBack(this); //Give internal nodes back to the pool before going deeper to minimize the creation of additional internal instances.
            oldChildA.RetrieveNodes(parent);
            oldChildB.RetrieveNodes(parent);


        }



        internal void Insert(LeafNode leafNode)
        {
            if (childA == null) //Check for an empty slot.
            {
                childA = leafNode; //Don't have to merge bounding boxes.  There must be at least two elements, so the bounding box will be computed in the second slot case.
            }
            else if (childB == null) //Check for another empty slot.
            {
                childB = leafNode;
                BoundingBox.CreateMerged(ref childA.BoundingBox, ref childB.BoundingBox, out BoundingBox);
                currentVolume = (BoundingBox.Max.X - BoundingBox.Min.X) * (BoundingBox.Max.Y - BoundingBox.Min.Y) * (BoundingBox.Max.Z - BoundingBox.Min.Z);
                maximumVolume = currentVolume * MaximumVolumeScale;
            }
            else
            {
                //Alrighty then! The current node is complete.  Do a normal insertion on ourselves..
                BoundingBox.CreateMerged(ref leafNode.BoundingBox, ref BoundingBox, out BoundingBox);
                Node treeNode = this;
                while (!treeNode.TryToInsert(leafNode, out treeNode)) ;//TryToInsert returns the next node, if any, and updates node bounding box.     
                currentVolume = (BoundingBox.Max.X - BoundingBox.Min.X) * (BoundingBox.Max.Y - BoundingBox.Min.Y) * (BoundingBox.Max.Z - BoundingBox.Min.Z);
                maximumVolume = currentVolume * MaximumVolumeScale;
            }

        }
    }


    internal sealed class LeafNode : Node
    {
        BroadPhaseEntry element;
        internal override Node ChildA
        {
            get
            {
                return null;
            }
        }
        internal override Node ChildB
        {
            get
            {
                return null;
            }
        }

        internal override BroadPhaseEntry Element
        {
            get
            {
                return element;
            }
        }

        internal override bool IsLeaf
        {
            get { return true; }
        }

        internal LeafNode(BroadPhaseEntry element)
        {
            this.element = element;
            BoundingBox = element.BoundingBox;
            //Having an ever-so-slight margin allows the hierarchy use a volume metric even for degenerate shapes (consider a flat tessellated plane).
            //BoundingBox.Max.X += LeafMargin;
            //BoundingBox.Max.Y += LeafMargin;
            //BoundingBox.Max.Z += LeafMargin;
            //BoundingBox.Min.X -= LeafMargin;
            //BoundingBox.Min.Y -= LeafMargin;
            //BoundingBox.Min.Z -= LeafMargin;
        }

        internal override void GetOverlaps(ref BoundingBox boundingBox, IList<BroadPhaseEntry> outputOverlappedElements)
        {
            //Our parent already tested the bounding box.  All that's left is to add myself to the list.
            outputOverlappedElements.Add(element);
        }

        internal override void GetOverlaps(ref BoundingSphere boundingSphere, IList<BroadPhaseEntry> outputOverlappedElements)
        {
            outputOverlappedElements.Add(element);
        }

        //internal override void GetOverlaps(ref BoundingFrustum boundingFrustum, IList<BroadPhaseEntry> outputOverlappedElements)
        //{
        //    outputOverlappedElements.Add(element);
        //}

        internal override void GetOverlaps(ref Ray ray, float maximumLength, IList<BroadPhaseEntry> outputOverlappedElements)
        {
            outputOverlappedElements.Add(element);
        }

        internal override void GetOverlaps(Node opposingNode, DynamicHierarchy2 owner)
        {
            bool intersects;
            //note: This is never executed when the opposing node is the current node.
            if (opposingNode.IsLeaf)
            {
                //We're both leaves!  Our parents have already done the testing for us, so we know we're overlapping.
                owner.TryToAddOverlap(element, opposingNode.Element);
            }
            else
            {
                var opposingChildA = opposingNode.ChildA;
                var opposingChildB = opposingNode.ChildB;
                //If it's not a leaf, try to go deeper in the opposing hierarchy.
                BoundingBox.Intersects(ref opposingChildA.BoundingBox, out intersects);
                if (intersects)
                    GetOverlaps(opposingChildA, owner);
                BoundingBox.Intersects(ref opposingChildB.BoundingBox, out intersects);
                if (intersects)
                    GetOverlaps(opposingChildB, owner);

            }
        }

        internal override bool TryToInsert(LeafNode node, out Node treeNode)
        {
            var newTreeNode = InternalNode.nodePool.Take();
            BoundingBox.CreateMerged(ref BoundingBox, ref node.BoundingBox, out newTreeNode.BoundingBox);
            Vector3 offset;
            Vector3.Subtract(ref newTreeNode.BoundingBox.Max, ref newTreeNode.BoundingBox.Min, out offset);
            newTreeNode.currentVolume = offset.X * offset.Y * offset.Z;
            newTreeNode.maximumVolume = newTreeNode.currentVolume * InternalNode.MaximumVolumeScale;
            newTreeNode.childA = this;
            newTreeNode.childB = node;
            treeNode = newTreeNode;
            return true;
        }

        public override string ToString()
        {
            return element.ToString();
        }

        internal override void Analyze(List<int> depths, int depth, ref int nodeCount)
        {
            nodeCount++;
            depths.Add(depth);
        }

        internal override void Refit()
        {
            BoundingBox = element.boundingBox;
        }

        internal override void RetrieveNodes(InternalNode parent)
        {
            Refit();
            parent.Insert(this);
        }
    }
}
