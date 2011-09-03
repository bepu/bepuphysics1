using System;
using System.Collections.Generic;
using BEPUphysics.BroadPhaseSystems;
 

namespace BEPUphysics.DataStructures
{
    ///<summary>
    /// Binary tree of objects with axis aligned bounding boxes, supporting various speedy queries.
    ///</summary>
    ///<typeparam name="T">Some IBoundingBoxOwner implementor.</typeparam>
    public class BoundingBoxTree<T> where T : class, IBoundingBoxOwner //It would be nice if this could be used with non-class owners.
    {
        internal Node[] nodeArray;

        ///<summary>
        /// Bounding box of the root of the tree.
        ///</summary>
        public BoundingBox BoundingBox
        {
            get
            {
                if (nodeArray.Length > 0)
                    return nodeArray[0].BoundingBox;
                return new BoundingBox();
            }
        }

        ///<summary>
        /// Constructs a new bounding box tree.
        ///</summary>
        ///<param name="elements">Elements contained by the tree.</param>
        public BoundingBoxTree(T[] elements)
        {
            Reconstruct(elements);
        }

        static void ComputeBoundingBox(T[] elements, out BoundingBox boundingBox)
        {
            int count = elements.Length;
            boundingBox = elements[0].BoundingBox;
            for (int i = 1; i < count; i++)
            {
                BoundingBox tempBox = elements[i].BoundingBox;
                if (tempBox.Min.X < boundingBox.Min.X)
                    boundingBox.Min.X = tempBox.Min.X;
                if (tempBox.Min.Y < boundingBox.Min.Y)
                    boundingBox.Min.Y = tempBox.Min.Y;
                if (tempBox.Min.Z < boundingBox.Min.Z)
                    boundingBox.Min.Z = tempBox.Min.Z;

                if (tempBox.Max.X > boundingBox.Max.X)
                    boundingBox.Max.X = tempBox.Max.X;
                if (tempBox.Max.Y > boundingBox.Max.Y)
                    boundingBox.Max.Y = tempBox.Max.Y;
                if (tempBox.Max.Z > boundingBox.Max.Z)
                    boundingBox.Max.Z = tempBox.Max.Z;
            }
        }

        ///<summary>
        /// Reconstructs the bounding box tree.
        /// This is a fairly expensive operation.  Consider using the refit method if the list of elements
        /// is the same.
        ///</summary>
        ///<param name="elements"></param>
        public void Reconstruct(T[] elements)
        {
            //Get the next highest power of 2.  A good starting estimate.  Might be able to trim a bit.
            int initialLength = elements.Length;
            initialLength = initialLength | (initialLength >> 1);
            initialLength = initialLength | (initialLength >> 2);
            initialLength = initialLength | (initialLength >> 4);
            initialLength = initialLength | (initialLength >> 8);
            initialLength = initialLength | (initialLength >> 16);
            initialLength = 2 * (initialLength + 1);
            if (nodeArray == null || nodeArray.Length != initialLength)
            {
                nodeArray = new Node[initialLength];
            }

            Reconstruct(0, elements);

            //Trim
            int trimmedLength = 0;
            for (int i = initialLength - 1; i >= 0; i--)
            {
                if (nodeArray[i].Value != null)
                {
                    trimmedLength = i + 1;
                    break;
                }
            }
            if (trimmedLength != initialLength)
            {
                var temp = new Node[trimmedLength];
                Array.Copy(nodeArray, temp, trimmedLength);
                nodeArray = temp;
            }
        }

        void Reconstruct(int index, T[] elements)
        {
            if (elements.Length > 1)
            {
                BoundingBox boundingBox;
                ComputeBoundingBox(elements, out boundingBox);
                nodeArray[index].BoundingBox = boundingBox;

                float xExtent = boundingBox.Max.X - boundingBox.Min.X;
                float yExtent = boundingBox.Max.Y - boundingBox.Min.Y;
                float zExtent = boundingBox.Max.Z - boundingBox.Min.Z;

                T[] firstHalf = new T[elements.Length / 2];
                T[] secondHalf = new T[elements.Length - firstHalf.Length];
                if (xExtent > yExtent && xExtent > zExtent)
                {
                    Array.Sort(elements, xAxisComparer);
                }
                else if (yExtent > zExtent)
                {
                    Array.Sort(elements, yAxisComparer);
                }
                else
                {
                    Array.Sort(elements, zAxisComparer);
                }
                Array.Copy(elements, firstHalf, firstHalf.Length);
                Array.Copy(elements, firstHalf.Length, secondHalf, 0, secondHalf.Length);
                int child = index + index + 1;
                Reconstruct(child, firstHalf);
                Reconstruct(child + 1, secondHalf);

            }
            else
            {
                nodeArray[index].BoundingBox = elements[0].BoundingBox;
                nodeArray[index].Value = elements[0];
            }
        }

        ///<summary>
        /// Refits the bounding box tree.  The structure of the tree
        /// is left unchanged.  This process traverses the existing tree
        /// and computes the bounding boxes so that everything is properly contained.
        ///</summary>
        public void Refit()
        {
            Refit(0);
        }

        void Refit(int currentNode)
        {
            IBoundingBoxOwner owner;
            if ((owner = nodeArray[currentNode].Value) != null)
            {
                nodeArray[currentNode].BoundingBox = owner.BoundingBox;
                //Has no children.
            }
            else
            {
                //Identify the children and add them.
                //Don't need to do any bounds check since the node didn't have a value.
                //Without a value, the node is guaranteed to have two children.
                int child = currentNode + currentNode + 1;
                Refit(child);
                int childB = child + 1;
                Refit(childB);
                BoundingBox.CreateMerged(ref nodeArray[child].BoundingBox, ref nodeArray[childB].BoundingBox, out nodeArray[currentNode].BoundingBox);
            }
        }

        #region Queries
        ///<summary>
        /// Tests a ray against the tree.
        ///</summary>
        ///<param name="ray">Ray to test.</param>
        ///<param name="hitElements">Elements with bounding boxes which intersect the ray.</param>
        ///<returns>Whether or not the ray hit any elements.</returns>
        public bool RayCast(Ray ray, IList<T> hitElements)
        {
            RayCastRecursive(0, ref ray, hitElements);
            return hitElements.Count > 0;
        }

        void RayCastRecursive(int index, ref Ray ray, IList<T> hitElements)
        {
            float? toi;
            ray.Intersects(ref nodeArray[index].BoundingBox, out toi);
            if (toi != null)
            {
                if (nodeArray[index].Value == null)
                {
                    //It's overlapping, so investigate and see if my children are overlapping too.
                    index = index + index + 1;
                    RayCastRecursive(index, ref ray, hitElements);
                    RayCastRecursive(++index, ref ray, hitElements);
                }
                else
                {
                    //I'm a leaf node! and overlapping!
                    hitElements.Add(nodeArray[index].Value);
                }
            }
        }

        ///<summary>
        /// Tests a ray against the tree.
        ///</summary>
        ///<param name="ray">Ray to test.</param>
        /// <param name="maximumDistance">Maximum length of the ray in units of the ray direction's length.</param>
        ///<param name="hitElements">Elements with bounding boxes which intersect the ray.</param>
        ///<returns>Whether or not the ray hit any elements.</returns>
        public bool RayCast(Ray ray, float maximumDistance, IList<T> hitElements)
        {
            RayCastRecursive(0, ref ray, maximumDistance, hitElements);
            return hitElements.Count > 0;
        }

        void RayCastRecursive(int index, ref Ray ray, float maximumDistance, IList<T> hitElements)
        {
            float? toi;
            ray.Intersects(ref nodeArray[index].BoundingBox, out toi);
            if (toi != null && toi < maximumDistance)
            {
                if (nodeArray[index].Value == null)
                {
                    //It's overlapping, so investigate and see if my children are overlapping too.
                    index = index + index + 1;
                    RayCastRecursive(index, ref ray, maximumDistance, hitElements);
                    RayCastRecursive(++index, ref ray, maximumDistance, hitElements);
                }
                else
                {
                    //I'm a leaf node! and overlapping!
                    hitElements.Add(nodeArray[index].Value);
                }
            }
        }

        ///<summary>
        /// Tests a volume against the tree.
        ///</summary>
        ///<param name="boundingBox">Volume to test.</param>
        ///<param name="overlappedElements">Elements with bounding boxes which intersect the volume.</param>
        ///<returns>Whether or not the volume intersected any elements.</returns>
        public bool GetOverlaps(BoundingBox boundingBox, IList<T> overlappedElements)
        {
            GetOverlapsRecursive(0, ref boundingBox, overlappedElements);
            return overlappedElements.Count > 0;
        }

        void GetOverlapsRecursive(int index, ref BoundingBox boundingBox, IList<T> overlappedElements)
        {
            bool isOverlapping;
            boundingBox.Intersects(ref nodeArray[index].BoundingBox, out isOverlapping);
            if (isOverlapping)
            {
                if (nodeArray[index].Value == null)
                {
                    //It's overlapping, so investigate and see if my children are overlapping too.
                    index = index + index + 1;
                    GetOverlapsRecursive(index, ref boundingBox, overlappedElements);
                    GetOverlapsRecursive(++index, ref boundingBox, overlappedElements);
                }
                else
                {
                    //I'm a leaf node! and overlapping!
                    overlappedElements.Add(nodeArray[index].Value);
                }
            }
        }

        ///<summary>
        /// Tests a volume against the tree.
        ///</summary>
        ///<param name="boundingSphere">Volume to test.</param>
        ///<param name="overlappedElements">Elements with bounding boxes which intersect the volume.</param>
        ///<returns>Whether or not the volume intersected any elements.</returns>
        public bool GetOverlaps(BoundingSphere boundingSphere, IList<T> overlappedElements)
        {
            GetOverlapsRecursive(0, ref boundingSphere, overlappedElements);
            return overlappedElements.Count > 0;
        }

        void GetOverlapsRecursive(int index, ref BoundingSphere boundingSphere, IList<T> overlappedElements)
        {
            bool isOverlapping;
            boundingSphere.Intersects(ref nodeArray[index].BoundingBox, out isOverlapping);
            if (isOverlapping)
            {
                if (nodeArray[index].Value == null)
                {
                    //It's overlapping, so investigate and see if my children are overlapping too.
                    index = index + index + 1;
                    GetOverlapsRecursive(index, ref boundingSphere, overlappedElements);
                    GetOverlapsRecursive(++index, ref boundingSphere, overlappedElements);
                }
                else
                {
                    //I'm a leaf node! and overlapping!
                    overlappedElements.Add(nodeArray[index].Value);
                }
            }
        }

        ///<summary>
        /// Tests a volume against the tree.
        ///</summary>
        ///<param name="boundingFrustum">Volume to test.</param>
        ///<param name="overlappedElements">Elements with bounding boxes which intersect the volume.</param>
        ///<returns>Whether or not the volume intersected any elements.</returns>
        public bool GetOverlaps(BoundingFrustum boundingFrustum, IList<T> overlappedElements)
        {
            GetOverlapsRecursive(0, ref boundingFrustum, overlappedElements);
            return overlappedElements.Count > 0;
        }

        void GetOverlapsRecursive(int index, ref BoundingFrustum boundingFrustum, IList<T> overlappedElements)
        {
            bool isOverlapping;
            boundingFrustum.Intersects(ref nodeArray[index].BoundingBox, out isOverlapping);
            if (isOverlapping)
            {
                if (nodeArray[index].Value == null)
                {
                    //It's overlapping, so investigate and see if my children are overlapping too.
                    index = index + index + 1;
                    GetOverlapsRecursive(index, ref boundingFrustum, overlappedElements);
                    GetOverlapsRecursive(++index, ref boundingFrustum, overlappedElements);
                }
                else
                {
                    //I'm a leaf node! and overlapping!
                    overlappedElements.Add(nodeArray[index].Value);
                }
            }
        }

        ///<summary>
        /// Tests a plane against the tree.
        ///</summary>
        ///<param name="plane">Volume to test.</param>
        /// <param name="intersectionType">Intersection type to search for.</param>
        ///<param name="overlappedElements">Elements with bounding boxes which intersect the plane.</param>
        ///<returns>Whether or not the plane intersected any elements.</returns>
        public bool GetOverlaps(Plane plane, PlaneIntersectionType intersectionType, IList<T> overlappedElements)
        {
            GetOverlapsRecursive(0, ref plane, intersectionType, overlappedElements);
            return overlappedElements.Count > 0;
        }

        void GetOverlapsRecursive(int index, ref Plane plane, PlaneIntersectionType intersectionType, IList<T> overlappedElements)
        {
            PlaneIntersectionType nodeIntersection;
            plane.Intersects(ref nodeArray[index].BoundingBox, out nodeIntersection);
            if (nodeIntersection == intersectionType)
            {
                if (nodeArray[index].Value == null)
                {
                    //It's overlapping, so investigate and see if my children are overlapping too.
                    index = index + index + 1;
                    GetOverlapsRecursive(index, ref plane, intersectionType, overlappedElements);
                    GetOverlapsRecursive(++index, ref plane, intersectionType, overlappedElements);
                }
                else
                {
                    //I'm a leaf node! and overlapping!
                    overlappedElements.Add(nodeArray[index].Value);
                }
            }
        }

        ///<summary>
        /// Gets the overlapping elements between two trees.
        ///</summary>
        ///<param name="opposingTree">Tree to test against.</param>
        ///<param name="collidingElements">Elements with overlapping AABB's.</param>
        ///<typeparam name="TTree">Type of the opposing tree's elements.</typeparam>
        ///<returns>Whether or not the trees overlapped.</returns>
        public bool GetOverlaps<TTree>(BoundingBoxTree<TTree> opposingTree, IList<TreeOverlapPair<T, TTree>> collidingElements)
            where TTree : class, IBoundingBoxOwner
        {
            bool isOverlapping;
            opposingTree.nodeArray[0].BoundingBox.Intersects(ref nodeArray[0].BoundingBox, out isOverlapping);
            if (isOverlapping)
                GetOverlapsWithoutRootTest(opposingTree, collidingElements);
            return collidingElements.Count > 0;

        }

        ///<summary>
        /// Gets the overlapping elements between two trees without testing the root.
        ///</summary>
        ///<param name="opposingTree">Tree to test against.</param>
        ///<param name="collidingElements">Elements with overlapping AABB's.</param>
        ///<typeparam name="TTree">Type of the opposing tree's elements.</typeparam>
        ///<returns>Whether or not the trees overlapped.</returns>
        public bool GetOverlapsWithoutRootTest<TTree>(BoundingBoxTree<TTree> opposingTree, IList<TreeOverlapPair<T, TTree>> collidingElements)
            where TTree : class, IBoundingBoxOwner
        {
            //We assume here that the opposing tree and current tree's roots overlap.
            //Also that both trees have been initialized.
            GetOverlaps(opposingTree.nodeArray, 0, 0, collidingElements);
            return collidingElements.Count > 0;
        }

        void GetOverlaps<TTree>(BoundingBoxTree<TTree>.Node[] opposingNodes, int parentA, int parentB, IList<TreeOverlapPair<T, TTree>> collidingElements)
            where TTree : class, IBoundingBoxOwner
        {
            if (nodeArray[parentA].Value == null)
            {
                //I have children.
                if (opposingNodes[parentB].Value == null)
                {
                    //Opposing node has children.
                    //Compare:
                    //A1 -> B1
                    //A1 -> B2
                    //A2 -> B1
                    //A2 -> B2
                    int childA1 = parentA + parentA + 1;
                    int childA2 = childA1 + 1;
                    int childB1 = parentB + parentB + 1;
                    int childB2 = childB1 + 1;
                    bool isOverlapping;
                    nodeArray[childA1].BoundingBox.Intersects(ref opposingNodes[childB1].BoundingBox, out isOverlapping);
                    if (isOverlapping)
                        GetOverlaps(opposingNodes, childA1, childB1, collidingElements);
                    nodeArray[childA1].BoundingBox.Intersects(ref opposingNodes[childB2].BoundingBox, out isOverlapping);
                    if (isOverlapping)
                        GetOverlaps(opposingNodes, childA1, childB2, collidingElements);
                    nodeArray[childA2].BoundingBox.Intersects(ref opposingNodes[childB1].BoundingBox, out isOverlapping);
                    if (isOverlapping)
                        GetOverlaps(opposingNodes, childA2, childB1, collidingElements);
                    nodeArray[childA2].BoundingBox.Intersects(ref opposingNodes[childB2].BoundingBox, out isOverlapping);
                    if (isOverlapping)
                        GetOverlaps(opposingNodes, childA2, childB2, collidingElements);

                }
                else
                {
                    //Opposing node has no children.
                    //Test my children against its bounding box to get closer to my leaves.
                    int childA1 = parentA + parentA + 1;
                    int childA2 = childA1 + 1;

                    bool isOverlapping;
                    nodeArray[childA1].BoundingBox.Intersects(ref opposingNodes[parentB].BoundingBox, out isOverlapping);
                    if (isOverlapping)
                        GetOverlaps(opposingNodes, childA1, parentB, collidingElements);
                    nodeArray[childA2].BoundingBox.Intersects(ref opposingNodes[parentB].BoundingBox, out isOverlapping);
                    if (isOverlapping)
                        GetOverlaps(opposingNodes, childA2, parentB, collidingElements);
                }
            }
            else
            {
                //I have no children.
                if (opposingNodes[parentB].Value == null)
                {
                    //Opposing node has children.
                    //Test its children against my bounding box to get closer to my leaves.

                    int childB1 = parentB + parentB + 1;
                    int childB2 = childB1 + 1;
                    bool isOverlapping;
                    nodeArray[parentA].BoundingBox.Intersects(ref opposingNodes[childB1].BoundingBox, out isOverlapping);
                    if (isOverlapping)
                        GetOverlaps(opposingNodes, parentA, childB1, collidingElements);
                    nodeArray[parentA].BoundingBox.Intersects(ref opposingNodes[childB2].BoundingBox, out isOverlapping);
                    if (isOverlapping)
                        GetOverlaps(opposingNodes, parentA, childB2, collidingElements);

                }
                else
                {
                    //Opposing node has no children.
                    //We are at leaf nodes.  If they overlap, they are added to the colliding elements.

                    bool isOverlapping;
                    nodeArray[parentA].BoundingBox.Intersects(ref opposingNodes[parentB].BoundingBox, out isOverlapping);
                    if (isOverlapping)
                        collidingElements.Add(new TreeOverlapPair<T, TTree>(nodeArray[parentA].Value, opposingNodes[parentB].Value));

                }
            }
        }
        #endregion


        internal struct Node
        {
            public BoundingBox BoundingBox;
            public T Value;
        }

        static XAxisComparer xAxisComparer = new XAxisComparer();

        //Epsilons in comparers help handle precision errors. 
        class XAxisComparer : IComparer<T>
        {
            public int Compare(T a, T b)
            {

                float x1 = (a.BoundingBox.Max.X + a.BoundingBox.Min.X) * .5f;
                float x2 = (b.BoundingBox.Max.X + b.BoundingBox.Min.X) * .5f;
                if (x1 > x2 + Toolbox.BigEpsilon)
                    return 1;
                if (x2 > x1 + Toolbox.BigEpsilon)
                    return -1;
                return 0;

            }
        }

        static YAxisComparer yAxisComparer = new YAxisComparer();
        class YAxisComparer : IComparer<T>
        {
            public int Compare(T a, T b)
            {

                float y1 = (a.BoundingBox.Max.Y + a.BoundingBox.Min.Y) * .5f;
                float y2 = (b.BoundingBox.Max.Y + b.BoundingBox.Min.Y) * .5f;
                if (y1 > y2 + Toolbox.BigEpsilon)
                    return 1;
                if (y2 > y1 + Toolbox.BigEpsilon)
                    return -1;
                return 0;

            }
        }
        static ZAxisComparer zAxisComparer = new ZAxisComparer();
        class ZAxisComparer : IComparer<T>
        {
            public int Compare(T a, T b)
            {

                float z1 = (a.BoundingBox.Max.Z + a.BoundingBox.Min.Z) * .5f;
                float z2 = (b.BoundingBox.Max.Z + b.BoundingBox.Min.Z) * .5f;
                if (z1 > z2 + Toolbox.BigEpsilon)
                    return 1;
                if (z2 > z1 + Toolbox.BigEpsilon)
                    return -1;
                return 0;

            }
        }


    }


}
