using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysics.BroadPhaseSystems.Hierarchies.Testing.Old
{
    /// <summary>
    /// Node within the binary hierarchy.
    /// </summary>
    public class DynamicHierarchyNodeOld
    {
        /// <summary>
        /// Bounding box all entities that are children of the node.
        /// </summary>
        public BoundingBox BoundingBox;

        internal AxisComparer axisComparer = new AxisComparer(ComparerAxis.X);

        internal List<DynamicHierarchyNodeOld> children = new List<DynamicHierarchyNodeOld>(2);
        internal float currentVolume;
        internal List<BroadPhaseEntry> entries = new List<BroadPhaseEntry>(8);
        internal DynamicHierarchyOld hierarchy;
        internal float maximumAllowedVolume;

        /// <summary>
        /// Constructs a DBH node.
        /// </summary>
        public DynamicHierarchyNodeOld()
        {
        }

        internal DynamicHierarchyNodeOld(DynamicHierarchyOld hierarchyOwner)
        {
            hierarchy = hierarchyOwner;
        }

        internal enum ComparerAxis
        {
            X,
            Y,
            Z
        } ;

        /// <summary>
        /// Collects all of the endpoints of lines of bounding boxes within the hierarchy.
        /// </summary>
        /// <param name="lineEndpoints">Endpoints of lines of bounding boxes within the hierarchy.</param>
        /// <param name="includeInternalNodes">Whether or not to collect the lines from internal node bounding boxes.</param>
        public void CollectBoundingBoxLines(List<VertexPositionColor> lineEndpoints, bool includeInternalNodes)
        {
            if (children.Count == 0 || includeInternalNodes)
            {
                Vector3[] boundingBoxCorners = BoundingBox.GetCorners();
                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[0], Color.DarkRed));
                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[1], Color.DarkRed));

                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[0], Color.DarkRed));
                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[3], Color.DarkRed));

                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[0], Color.DarkRed));
                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[4], Color.DarkRed));

                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[1], Color.DarkRed));
                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[2], Color.DarkRed));

                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[1], Color.DarkRed));
                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[5], Color.DarkRed));

                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[2], Color.DarkRed));
                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[3], Color.DarkRed));

                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[2], Color.DarkRed));
                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[6], Color.DarkRed));

                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[3], Color.DarkRed));
                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[7], Color.DarkRed));

                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[4], Color.DarkRed));
                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[5], Color.DarkRed));

                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[4], Color.DarkRed));
                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[7], Color.DarkRed));

                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[5], Color.DarkRed));
                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[6], Color.DarkRed));

                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[6], Color.DarkRed));
                lineEndpoints.Add(new VertexPositionColor(boundingBoxCorners[7], Color.DarkRed));
            }
            foreach (DynamicHierarchyNodeOld child in children)
            {
                child.CollectBoundingBoxLines(lineEndpoints, includeInternalNodes);
            }
        }


        /// <exception cref="InvalidOperationException">Thrown when the entity to add to the DynamicBinaryHierarchy has an invalid state.</exception>
        internal void Add(BroadPhaseEntry e)
        {
            entries.Add(e);
            if (children.Count > 0)
            {
                //Internal Node
                float minimumDistance = float.MaxValue;
                DynamicHierarchyNodeOld minimumNode = null;
                float x = (e.boundingBox.Max.X + e.boundingBox.Min.X) * .5f;
                float y = (e.boundingBox.Max.Y + e.boundingBox.Min.Y) * .5f;
                float z = (e.boundingBox.Max.Z + e.boundingBox.Min.Z) * .5f;
                Vector3 min, max;
                foreach (DynamicHierarchyNodeOld node in children)
                {
                    min = node.BoundingBox.Min;
                    max = node.BoundingBox.Max;
                    float midpointX = (max.X + min.X) * .5f;
                    float midpointY = (max.Y + min.Y) * .5f;
                    float midpointZ = (max.Z + min.Z) * .5f;
                    float distance = Math.Abs(midpointX - x) + Math.Abs(midpointY - y) + Math.Abs(midpointZ - z);
                    if (distance < minimumDistance)
                    {
                        minimumNode = node;
                        minimumDistance = distance;
                    }
                }
                if (minimumNode == null)
                    throw new InvalidOperationException(
                        "Cannot add entity to broadphase due to an invalid state.  Ensure that the entity's position and bounding box do not contain float.NaN, float.MaxValue, float.NegativeInfinity, or float.PositiveInfinity.");
                minimumNode.Add(e);
                //Refit
                BoundingBox.CreateMerged(ref children[0].BoundingBox, ref children[1].BoundingBox, out BoundingBox);
                Vector3 difference;
                Vector3.Subtract(ref BoundingBox.Max, ref BoundingBox.Min, out difference);
                currentVolume = difference.X * difference.Y * difference.Z;
            }
            else
            {
                Revalidate();
            }
        }

        internal void BinaryCollideAgainst(DynamicHierarchyNodeOld node)
        {
            //Base idea: recurse down the tree whenever child bounding boxes overlap.
            //Attempt to get to the base case of leaf vs. leaf.
            bool intersecting;
            if (this == node)
            {
                //Very simple case; the test between the nodes
                if (children.Count == 0)
                {
                    //Leafwise tests
                    for (int i = 0; i < entries.Count - 1; i++)
                    {
                        for (int j = i + 1; j < entries.Count; j++)
                        {
                            hierarchy.TryToAdd(entries[i], entries[j]);
                        }
                    }
                }
                else
                {
                    //Nodewise test

                    children[0].BoundingBox.Intersects(ref children[1].BoundingBox, out intersecting);
                    if (intersecting)
                    {
                        children[0].BinaryCollideAgainst(children[1]);
                    }


                    //Self test
                    children[0].BinaryCollideAgainst(children[0]);
                    children[1].BinaryCollideAgainst(children[1]);
                }
            }
            else
            {
                if (children.Count > 0 && node.children.Count > 0)
                {
                    //Both involved nodes are not leaves.
                    children[0].BoundingBox.Intersects(ref node.children[0].BoundingBox, out intersecting);
                    if (intersecting)
                        children[0].BinaryCollideAgainst(node.children[0]);

                    children[0].BoundingBox.Intersects(ref node.children[1].BoundingBox, out intersecting);
                    if (intersecting)
                        children[0].BinaryCollideAgainst(node.children[1]);

                    children[1].BoundingBox.Intersects(ref node.children[0].BoundingBox, out intersecting);
                    if (intersecting)
                        children[1].BinaryCollideAgainst(node.children[0]);

                    children[1].BoundingBox.Intersects(ref node.children[1].BoundingBox, out intersecting);
                    if (intersecting)
                        children[1].BinaryCollideAgainst(node.children[1]);
                }
                else if (children.Count > 0)
                {
                    //Opposing node is a leaf.
                    children[0].BoundingBox.Intersects(ref node.BoundingBox, out intersecting);
                    if (intersecting)
                        children[0].BinaryCollideAgainst(node);

                    children[1].BoundingBox.Intersects(ref node.BoundingBox, out intersecting);
                    if (intersecting)
                        children[1].BinaryCollideAgainst(node);
                }
                else if (node.children.Count > 0)
                {
                    //I'm a leaf
                    BoundingBox.Intersects(ref node.children[0].BoundingBox, out intersecting);
                    if (intersecting)
                        BinaryCollideAgainst(node.children[0]);

                    BoundingBox.Intersects(ref node.children[1].BoundingBox, out intersecting);
                    if (intersecting)
                        BinaryCollideAgainst(node.children[1]);
                }
                else
                {
                    //Both leaves!
                    //Entity vs. entity test.
                    for (int i = 0; i < entries.Count; i++)
                    {
                        for (int j = 0; j < node.entries.Count; j++)
                        {
                            hierarchy.TryToAdd(entries[i], node.entries[j]);
                        }
                    }
                }
            }
        }

        internal void BinaryUpdateNode()
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
            if (currentVolume > maximumAllowedVolume)
            {
                //Debug.WriteLine("volume exceeded, current: " + currentVolume + ", max: " + maximumAllowedVolume);
                Revalidate();
            }
            else
            {
                if (children.Count > 0)
                {
                    //Internal Node
                    foreach (DynamicHierarchyNodeOld node in children)
                    {
                        node.BinaryUpdateNode();
                    }

                    BoundingBox.CreateMerged(ref children[0].BoundingBox, ref children[1].BoundingBox, out BoundingBox);
                }
                else
                {
                    //Leaf Node

                    if (entries.Count > 0) //don't try to merge anything if there's nothing!
                    {
                        BoundingBox = entries[0].boundingBox;
                        for (int i = 1; i < entries.Count; i++)
                        {
                            BoundingBox.CreateMerged(ref BoundingBox, ref entries[i].boundingBox, out BoundingBox);
                        }
                    }
                }
                Vector3 difference;
                Vector3.Subtract(ref BoundingBox.Max, ref BoundingBox.Min, out difference);
                currentVolume = difference.X * difference.Y * difference.Z;
            }
        }
        
        internal void GetEntities(ref BoundingBox box, IList<BroadPhaseEntry> outputEntries)
        {
            bool intersecting;
            if (children.Count > 0)
            {
                BoundingBox.Intersects(ref box, out intersecting);
                if (intersecting)
                {
                    foreach (DynamicHierarchyNodeOld child in children)
                    {
                        child.BoundingBox.Intersects(ref box, out intersecting);
                        if (intersecting)
                        {
                            child.GetEntities(ref box, outputEntries);
                        }
                    }
                }
            }
            else
            {
                foreach (BroadPhaseEntry e in entries)
                {
                    e.boundingBox.Intersects(ref box, out intersecting);
                    if (intersecting)
                        outputEntries.Add(e);
                }
            }
        }

        internal void GetEntities(ref BoundingSphere sphere, IList<BroadPhaseEntry> outputEntries)
        {
            bool intersecting;
            if (children.Count > 0)
            {
                BoundingBox.Intersects(ref sphere, out intersecting);
                if (intersecting)
                {
                    foreach (DynamicHierarchyNodeOld child in children)
                    {
                        child.BoundingBox.Intersects(ref sphere, out intersecting);
                        if (intersecting)
                        {
                            child.GetEntities(ref sphere, outputEntries);
                        }
                    }
                }
            }
            else
            {
                foreach (BroadPhaseEntry e in entries)
                {
                    e.boundingBox.Intersects(ref sphere, out intersecting);
                    if (intersecting)
                        outputEntries.Add(e);
                }
            }
        }

        internal void GetEntities(ref BoundingFrustum frustum, IList<BroadPhaseEntry> outputEntries)
        {
            if (children.Count > 0)
            {
                if (BoundingBox.Intersects(frustum))
                {
                    foreach (DynamicHierarchyNodeOld child in children)
                    {
                        if (child.BoundingBox.Intersects(frustum))
                        {
                            child.GetEntities(ref frustum, outputEntries);
                        }
                    }
                }
            }
            else
            {
                foreach (BroadPhaseEntry e in entries)
                {
                    if (e.boundingBox.Intersects(frustum))
                        outputEntries.Add(e);
                }
            }
        }

        internal void RayCast(ref Ray ray, float maximumLength, IList<BroadPhaseEntry> outEntries)
        {
            if (children.Count > 0)
            {
                foreach (DynamicHierarchyNodeOld child in children)
                {
                    float? aabbtoi;
                    ray.Intersects(ref child.BoundingBox, out aabbtoi);
                    if (aabbtoi != null && aabbtoi < maximumLength)
                    {
                        child.RayCast(ref ray, maximumLength, outEntries);
                    }
                }
            }
            else
            {
                foreach (BroadPhaseEntry e in entries)
                {
                    float? toi;
                    ray.Intersects(ref e.boundingBox, out toi);
                    if (toi != null && toi < maximumLength)
                    {
                        outEntries.Add(e);
                    }
                }
            }
        }

        internal void RayCast(ref Ray ray, IList<BroadPhaseEntry> outEntries)
        {
            if (children.Count > 0)
            {
                foreach (DynamicHierarchyNodeOld child in children)
                {
                    float? aabbtoi;
                    ray.Intersects(ref child.BoundingBox, out aabbtoi);
                    if (aabbtoi != null)
                    {
                        child.RayCast(ref ray, outEntries);
                    }
                }
            }
            else
            {
                foreach (BroadPhaseEntry e in entries)
                {
                    float? toi;
                    ray.Intersects(ref e.boundingBox, out toi);
                    if (toi != null)
                    {
                        outEntries.Add(e);
                    }
                }
            }
        }

        internal void RedoSplit(ComparerAxis minimumAxis)
        {
            //Sort along maximum length axis.
            //Each half is given to a child.

            //Clear out old tree.
            foreach (DynamicHierarchyNodeOld node in children)
            {
                hierarchy.GiveBack(node);
            }
            children.Clear();

            axisComparer.axis = minimumAxis;
            entries.Sort(axisComparer);

            DynamicHierarchyNodeOld left = hierarchy.GetNode();
            DynamicHierarchyNodeOld right = hierarchy.GetNode();

            for (int i = 0; i < entries.Count / 2; i++)
            {
                right.entries.Add(entries[i]);
            }
            for (int i = entries.Count / 2; i < entries.Count; i++)
            {
                left.entries.Add(entries[i]);
            }
            children.Add(right);
            children.Add(left);


            foreach (DynamicHierarchyNodeOld node in children)
            {
                node.Revalidate();
            }
        }

        internal void Remove(BroadPhaseEntry e, int index)
        {
            entries.RemoveAt(index);
            for (int i = children.Count - 1; i >= 0; i--)
            {
                int childIndex = children[i].entries.IndexOf(e);
                if (childIndex != -1) // if it is contained
                {
                    if (children[i].entries.Count <= hierarchy.MaximumEntitiesInLeaves + 1)
                    {
                        //The child node in question is small enough to be a leaf, after it is removed we should restructure a little bit.
                        hierarchy.GiveBack(children[i]);
                        children.RemoveAt(i);
                        if (entries.Count > 0) //this != hierarchy.root)
                        {
                            Revalidate();
                            break;
                        }
                    }
                    else
                        children[i].Remove(e, childIndex);
                    //Refit
                    BoundingBox.CreateMerged(ref children[0].BoundingBox, ref children[1].BoundingBox, out BoundingBox);
                    Vector3 difference;
                    Vector3.Subtract(ref BoundingBox.Max, ref BoundingBox.Min, out difference);
                    currentVolume = difference.X * difference.Y * difference.Z;
                }
            }
        }

        internal void Revalidate()
        {
            if (entries.Count > 0)
            {
                BoundingBox = entries[0].boundingBox;
                for (int i = 1; i < entries.Count; i++)
                {
                    BoundingBox.CreateMerged(ref BoundingBox, ref entries[i].boundingBox, out BoundingBox);
                }
            }
            else
                BoundingBox = new BoundingBox();
            if (entries.Count <= hierarchy.MaximumEntitiesInLeaves)
            {
                //Revalidating this node won't do anything.
                if (children.Count > 0)
                {
                    //Debug.WriteLine("Whoa there nelly,.");
                    //Victim of removal.  Get rid of the children, they're not necessary.
                    foreach (DynamicHierarchyNodeOld node in children)
                    {
                        hierarchy.GiveBack(node);
                    }
                    children.Clear();
                }
                return;
            }


            Vector3 difference;
            Vector3.Subtract(ref BoundingBox.Max, ref BoundingBox.Min, out difference);
            currentVolume = difference.X * difference.Y * difference.Z;
            maximumAllowedVolume = currentVolume * hierarchy.MaximumAllowedVolumeFactor;


            //Clear out old tree.
            foreach (DynamicHierarchyNodeOld child in children)
            {
                hierarchy.GiveBack(child);
            }
            children.Clear();

            //Top-down reconstruction.
            Vector3 min = BoundingBox.Min;
            Vector3 max = BoundingBox.Max;
            DynamicHierarchyNodeOld left = hierarchy.GetNode();
            DynamicHierarchyNodeOld right = hierarchy.GetNode();
            float xDifference = max.X - min.X;
            float yDifference = max.Y - min.Y;
            float zDifference = max.Z - min.Z;
            float midpoint;
            ComparerAxis minimumAxis;
            if (xDifference > yDifference && xDifference > zDifference)
            {
                minimumAxis = ComparerAxis.X;
                midpoint = (max.X + min.X) * .5f;
                foreach (BroadPhaseEntry e in entries)
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
                minimumAxis = ComparerAxis.Z;
                midpoint = (max.Y + min.Y) * .5f;
                foreach (BroadPhaseEntry e in entries)
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
                minimumAxis = ComparerAxis.Z;
                midpoint = (max.Z + min.Z) * .5f;
                foreach (BroadPhaseEntry e in entries)
                {
                    float z = (e.boundingBox.Max.Y + e.boundingBox.Min.Y) * .5f;
                    if (z > midpoint)
                        left.entries.Add(e);
                    else
                        right.entries.Add(e);
                }
            }
            var maxEntityCount = (int)(entries.Count * hierarchy.MaximumChildEntityLoad);
            if (left.entries.Count >= maxEntityCount)
            {
                hierarchy.GiveBack(left);
                hierarchy.GiveBack(right);
                RedoSplit(minimumAxis);
            }
            else
            {
                if (right.entries.Count >= maxEntityCount)
                {
                    hierarchy.GiveBack(left);
                    hierarchy.GiveBack(right);
                    RedoSplit(minimumAxis);
                }
                else
                {
                    right.Revalidate();
                    children.Add(right);
                    left.Revalidate();
                    children.Add(left);
                }
            }
        }

        internal void UpdateNode()
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
            if (currentVolume > maximumAllowedVolume)
            {
                //Debug.WriteLine("volume exceeded, current: " + currentVolume + ", max: " + maximumAllowedVolume);
                Revalidate();
            }
            else
            {
                if (children.Count > 0)
                {
                    //Internal Node
                    foreach (DynamicHierarchyNodeOld node in children)
                    {
                        node.UpdateNode();
                    }
                    BoundingBox = children[0].BoundingBox;
                    for (int i = 1; i < children.Count; i++)
                    {
                        BoundingBox.CreateMerged(ref BoundingBox, ref children[i].BoundingBox, out BoundingBox);
                    }
                }
                else
                {
                    //Leaf Node
                    if (entries.Count > 0) //don't try to merge anything if there's nothing!
                    {
                        BoundingBox = entries[0].boundingBox;
                        for (int i = 1; i < entries.Count; i++)
                        {
                            BoundingBox.CreateMerged(ref BoundingBox, ref entries[i].boundingBox, out BoundingBox);
                        }
                    }
                    else
                        BoundingBox = new BoundingBox();
                }
                Vector3 difference;
                Vector3.Subtract(ref BoundingBox.Max, ref BoundingBox.Min, out difference);
                currentVolume = difference.X * difference.Y * difference.Z;
            }
        }


        internal class AxisComparer : IComparer<BroadPhaseEntry>
        {
            internal ComparerAxis axis;

            public AxisComparer(ComparerAxis comparerAxis)
            {
                axis = comparerAxis;
            }

            #region IComparer<Entity> Members

            public int Compare(BroadPhaseEntry a, BroadPhaseEntry b)
            {
                switch (axis)
                {
                    case ComparerAxis.X:
                        float x1 = (a.boundingBox.Max.X + a.boundingBox.Min.X) * .5f;
                        float x2 = (b.boundingBox.Max.X + b.boundingBox.Min.X) * .5f;
                        if (x1 > x2)
                            return 1;
                        if (x2 > x1)
                            return -1;
                        return 0;
                    case ComparerAxis.Y:
                        float y1 = (a.boundingBox.Max.Y + a.boundingBox.Min.Y) * .5f;
                        float y2 = (b.boundingBox.Max.Y + b.boundingBox.Min.Y) * .5f;
                        if (y1 > y2)
                            return 1;
                        if (y1 < y2)
                            return -1;
                        return 0;
                    case ComparerAxis.Z:
                        float z1 = (a.boundingBox.Max.Z + a.boundingBox.Min.Z) * .5f;
                        float z2 = (b.boundingBox.Max.Z + b.boundingBox.Min.Z) * .5f;
                        if (z1 > z2)
                            return 1;
                        if (z1 < z2)
                            return -1;
                        return 0;
                }
                return 0;
            }

            #endregion
        }

    }
}
