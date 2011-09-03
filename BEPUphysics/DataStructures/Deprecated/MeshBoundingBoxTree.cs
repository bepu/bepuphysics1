using System;
using System.Collections.Generic;
using BEPUphysics.BroadPhaseSystems;
 

namespace BEPUphysics.DataStructures
{
    //////////////////////////
    //  This is an old version of the MeshBoundingBoxTree.
    //  It uses a top-down construction approach and a contiguous array
    //  to store the nodes.
    //  It can occasionally outperform the newer incrementally constructed tree,
    //  but the construction time is more than 10 times slower.
    //
    //  If there's a very tight performance or memory requirement, trying this tree
    //  out might be worth it.  However, the mesh itself determines which tree works
    //  better.  For some, the incremental tree will be better in every way.
    //////////////////////////

    ///<summary>
    ///  Binary tree of triangles surrounded by axis aligned bounding boxes, supporting various speedy queries.
    ///</summary>
    public class MeshBoundingBoxTree
    {
        internal Node[] nodeArray;
        internal MeshBoundingBoxTreeData triangleMeshData;

        ///<summary>
        /// Gets the bounding box of the root of the tree.
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
        /// Gets the triangle mesh data used to build the tree.
        ///</summary>
        public MeshBoundingBoxTreeData TriangleMeshData
        {
            get
            {
                return triangleMeshData;
            }
        }

        ///<summary>
        /// Constructs a new triangle mesh tree.
        ///</summary>
        ///<param name="triangleMeshData">Data to use to construct the tree.</param>
        public MeshBoundingBoxTree(MeshBoundingBoxTreeData triangleMeshData)
        {
            xAxisComparer = new XAxisComparer(this);
            yAxisComparer = new YAxisComparer(this);
            zAxisComparer = new ZAxisComparer(this);
            Reconstruct(triangleMeshData);
        }
        ///<summary>
        /// Constructs a new triangle mesh tree.
        ///</summary>
        ///<param name="triangleMeshData">Data to use to construct the tree.</param>
        ///<param name="margin">Margin to expand the bounding box of elements by.</param>
        public MeshBoundingBoxTree(MeshBoundingBoxTreeData triangleMeshData, float margin)
        {
            xAxisComparer = new XAxisComparer(this);
            yAxisComparer = new YAxisComparer(this);
            zAxisComparer = new ZAxisComparer(this);
            Reconstruct(triangleMeshData, margin);
        }

        void ComputeBoundingBox(int[] triangleIndices, out BoundingBox boundingBox)
        {
            int count = triangleIndices.Length;
            triangleMeshData.GetBoundingBox(triangleIndices[0], out boundingBox);
            for (int i = 1; i < count; i++)
            {
                BoundingBox tempBox;
                triangleMeshData.GetBoundingBox(triangleIndices[i], out tempBox);
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
        /// Reconstructs the tree using the given data.
        /// This is a pretty slow operation; consider using the 
        /// refit function if the topology of the mesh is unchanged.
        ///</summary>
        ///<param name="triangleMeshData">Data used to construct the tree.</param>
        public void Reconstruct(MeshBoundingBoxTreeData triangleMeshData)
        {
            Reconstruct(triangleMeshData, 0);
        }

        ///<summary>
        /// Reconstructs the tree using the given data.
        /// This is a pretty slow operation; consider using the 
        /// refit function if the topology of the mesh is unchanged.
        ///</summary>
        ///<param name="triangleMeshData">Data used to construct the tree.</param>
        /// <param name="margin">Margin to expand the bounding box of elements by.</param>
        public void Reconstruct(MeshBoundingBoxTreeData triangleMeshData, float margin)
        {
            if (triangleMeshData.vertices.Length == 0)
                throw new InvalidOperationException("Triangle mesh data contains no vertices.  Ensure the mesh data was loaded properly.");
            if (triangleMeshData.indices.Length == 0)
                throw new InvalidOperationException("Triangle mesh data contains no indices.  Ensure the mesh data was loaded properly.");
            this.triangleMeshData = triangleMeshData;
            //Get the next highest power of 2.  A good starting estimate.  Might be able to trim a bit.
            int[] triangles = new int[triangleMeshData.indices.Length / 3];
            for (int i = 0; i < triangles.Length; i++)
            {
                triangles[i] = i * 3;
            }
            int initialLength = triangles.Length;
            initialLength = initialLength | (initialLength >> 1);
            initialLength = initialLength | (initialLength >> 2);
            initialLength = initialLength | (initialLength >> 4);
            initialLength = initialLength | (initialLength >> 8);
            initialLength = initialLength | (initialLength >> 16);
            initialLength = 2 * (initialLength + 1);
            if (nodeArray == null || nodeArray.Length != initialLength)
            {
                nodeArray = new Node[initialLength];
                for (int i = 0; i < initialLength; i++)
                {
                    nodeArray[i].Value = -2;
                }
            }


            Reconstruct(0, triangles, margin);

            //Trim
            int trimmedLength = 0;
            for (int i = initialLength - 1; i >= 0; i--)
            {
                if (nodeArray[i].Value != -2)
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

        void Reconstruct(int index, int[] triangleIndices, float margin)
        {
            if (triangleIndices.Length > 1)
            {
                BoundingBox boundingBox;
                ComputeBoundingBox(triangleIndices, out boundingBox);
                boundingBox.Min.X -= margin;
                boundingBox.Min.Y -= margin;
                boundingBox.Min.Z -= margin;

                boundingBox.Max.X += margin;
                boundingBox.Max.Y += margin;
                boundingBox.Max.Z += margin;
                nodeArray[index].BoundingBox = boundingBox;
                nodeArray[index].Value = -1;

                float xExtent = boundingBox.Max.X - boundingBox.Min.X;
                float yExtent = boundingBox.Max.Y - boundingBox.Min.Y;
                float zExtent = boundingBox.Max.Z - boundingBox.Min.Z;

                int[] firstHalf = new int[triangleIndices.Length / 2];
                int[] secondHalf = new int[triangleIndices.Length - firstHalf.Length];
                if (xExtent > yExtent && xExtent > zExtent)
                {
                    Array.Sort(triangleIndices, xAxisComparer);
                }
                else if (yExtent > zExtent)
                {
                    Array.Sort(triangleIndices, yAxisComparer);
                }
                else
                {
                    Array.Sort(triangleIndices, zAxisComparer);
                }
                Array.Copy(triangleIndices, firstHalf, firstHalf.Length);
                Array.Copy(triangleIndices, firstHalf.Length, secondHalf, 0, secondHalf.Length);
                int child = index + index + 1;
                Reconstruct(child, firstHalf, margin);
                Reconstruct(child + 1, secondHalf, margin);

            }
            else
            {
                triangleMeshData.GetBoundingBox(triangleIndices[0], out nodeArray[index].BoundingBox);
                nodeArray[index].Value = triangleIndices[0];
            }
        }

        ///<summary>
        /// Refits the bounding boxes of triangles.
        /// This does not alter the structure of the tree; it just traverses
        /// the structure and updates the bounding boxes as necessary.
        ///</summary>
        public void Refit()
        {
            Refit(0);
        }
        ///<summary>
        /// Refits the bounding boxes of triangles.
        /// This does not alter the structure of the tree; it just traverses
        /// the structure and updates the bounding boxes as necessary.
        /// </summary>
        /// <param name="margin">Margin to expand the elements of the mesh by.</param>
        public void Refit(float margin)
        {
            Refit(0, margin);
        }

        void Refit(int currentNode, float margin)
        {
            int owner;
            if ((owner = nodeArray[currentNode].Value) != -1)
            {
                triangleMeshData.GetBoundingBox(owner, out nodeArray[currentNode].BoundingBox);
                nodeArray[currentNode].BoundingBox.Min.X -= margin;
                nodeArray[currentNode].BoundingBox.Min.Y -= margin;
                nodeArray[currentNode].BoundingBox.Min.Z -= margin;

                nodeArray[currentNode].BoundingBox.Max.X += margin;
                nodeArray[currentNode].BoundingBox.Max.Y += margin;
                nodeArray[currentNode].BoundingBox.Max.Z += margin;
                //Has no children.
            }
            else
            {
                //Identify the children and add them.
                //Don't need to do any bounds check since the node didn't have a value.
                //Without a value, the node is guaranteed to have two children.
                int child = currentNode + currentNode + 1;
                Refit(child, margin);
                int childB = child + 1;
                Refit(childB, margin);
                BoundingBox.CreateMerged(ref nodeArray[child].BoundingBox, ref nodeArray[childB].BoundingBox, out nodeArray[currentNode].BoundingBox);
            }
        }

        void Analyze(out List<int> depths, out int minDepth, out int maxDepth, out int nodeCount)
        {
            depths = new List<int>();
            nodeCount = 0;
            AnalyzeRecursive(0, 0, depths, ref nodeCount);

            minDepth = int.MaxValue;
            maxDepth = 0;
            for (int i = 0; i < depths.Count; i++)
            {
                if (depths[i] > maxDepth)
                    maxDepth = depths[i];
                if (depths[i] < minDepth)
                    minDepth = depths[i];
            }
        }

        void AnalyzeRecursive(int index, int depth, List<int> depths, ref int nodeCount)
        {
            nodeCount++;
            if (nodeArray[index].Value == -1)
            {
                //It's overlapping, so investigate and see if my children are overlapping too.
                index = index + index + 1;
                AnalyzeRecursive(index, depth + 1, depths, ref nodeCount);
                AnalyzeRecursive(index + 1, depth + 1, depths, ref nodeCount);
            }
            else
            {
                //I'm a leaf node! and overlapping!
                depths.Add(depth);
            }
        }

        #region Queries
        ///<summary>
        /// Tests a ray against the tree.
        ///</summary>
        ///<param name="ray">Ray to test.</param>
        ///<param name="hitElements">Elements with bounding boxes which intersect the ray.</param>
        ///<returns>Whether or not the ray hit any elements.</returns>
        public bool RayCast(Ray ray, IList<int> hitElements)
        {
            RayCastRecursive(0, ref ray, hitElements);
            return hitElements.Count > 0;
        }

        void RayCastRecursive(int index, ref Ray ray, IList<int> hitElements)
        {
            float? toi;
            ray.Intersects(ref nodeArray[index].BoundingBox, out toi);
            if (toi != null)
            {
                if (nodeArray[index].Value == -1)
                {
                    //It's overlapping, so investigate and see if my children are overlapping too.
                    index = index + index + 1;
                    RayCastRecursive(index, ref ray, hitElements);
                    RayCastRecursive(index + 1, ref ray, hitElements);
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
        public bool RayCast(Ray ray, float maximumDistance, IList<int> hitElements)
        {
            RayCastRecursive(0, ref ray, maximumDistance, hitElements);
            return hitElements.Count > 0;
        }

        void RayCastRecursive(int index, ref Ray ray, float maximumDistance, IList<int> hitElements)
        {
            float? toi;
            ray.Intersects(ref nodeArray[index].BoundingBox, out toi);
            if (toi != null && toi < maximumDistance)
            {
                if (nodeArray[index].Value == -1)
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
        public bool GetOverlaps(BoundingBox boundingBox, IList<int> overlappedElements)
        {
            GetOverlapsRecursive(0, ref boundingBox, overlappedElements);
            return overlappedElements.Count > 0;
        }

        void GetOverlapsRecursive(int index, ref BoundingBox boundingBox, IList<int> overlappedElements)
        {
            bool isOverlapping;
            boundingBox.Intersects(ref nodeArray[index].BoundingBox, out isOverlapping);
            if (isOverlapping)
            {
                if (nodeArray[index].Value == -1)
                {
                    //It's overlapping, so investigate and see if my children are overlapping too.
                    index = index + index + 1;
                    GetOverlapsRecursive(index, ref boundingBox, overlappedElements);
                    GetOverlapsRecursive(index + 1, ref boundingBox, overlappedElements);
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
        public bool GetOverlaps(BoundingSphere boundingSphere, IList<int> overlappedElements)
        {
            GetOverlapsRecursive(0, ref boundingSphere, overlappedElements);
            return overlappedElements.Count > 0;
        }

        void GetOverlapsRecursive(int index, ref BoundingSphere boundingSphere, IList<int> overlappedElements)
        {
            bool isOverlapping;
            boundingSphere.Intersects(ref nodeArray[index].BoundingBox, out isOverlapping);
            if (isOverlapping)
            {
                if (nodeArray[index].Value == -1)
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
        public bool GetOverlaps(BoundingFrustum boundingFrustum, IList<int> overlappedElements)
        {
            GetOverlapsRecursive(0, ref boundingFrustum, overlappedElements);
            return overlappedElements.Count > 0;
        }

        void GetOverlapsRecursive(int index, ref BoundingFrustum boundingFrustum, IList<int> overlappedElements)
        {
            bool isOverlapping;
            boundingFrustum.Intersects(ref nodeArray[index].BoundingBox, out isOverlapping);
            if (isOverlapping)
            {
                if (nodeArray[index].Value == -1)
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
        public bool GetOverlaps(Plane plane, PlaneIntersectionType intersectionType, IList<int> overlappedElements)
        {
            GetOverlapsRecursive(0, ref plane, intersectionType, overlappedElements);
            return overlappedElements.Count > 0;
        }

        void GetOverlapsRecursive(int index, ref Plane plane, PlaneIntersectionType intersectionType, IList<int> overlappedElements)
        {
            PlaneIntersectionType nodeIntersection;
            plane.Intersects(ref nodeArray[index].BoundingBox, out nodeIntersection);
            if (nodeIntersection == intersectionType)
            {
                if (nodeArray[index].Value == -1)
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
        /// Gets the overlaps between the tree and another tree.
        ///</summary>
        ///<param name="opposingTree">The tree to test against.</param>
        ///<param name="collidingElements">Overlapping elements in the trees.</param>
        ///<typeparam name="TTree">Type of the elements in the opposing tree.</typeparam>
        ///<returns>Whether or not any elements were contained.</returns>
        public bool GetOverlaps<TTree>(BoundingBoxTree<TTree> opposingTree, IList<TreeOverlapPair<int, TTree>> collidingElements)
            where TTree : class, IBoundingBoxOwner
        {
            bool isOverlapping;
            opposingTree.nodeArray[0].BoundingBox.Intersects(ref nodeArray[0].BoundingBox, out isOverlapping);
            if (isOverlapping)
                GetOverlapsWithoutRootTest(opposingTree, collidingElements);
            return collidingElements.Count > 0;

        }

        ///<summary>
        /// Gets the overlaps between the tree and another tree without performing a root bounding box test.
        ///</summary>
        ///<param name="opposingTree">The tree to test against.</param>
        ///<param name="collidingElements">Overlapping elements in the trees.</param>
        ///<typeparam name="TTree">Type of the elements in the opposing tree.</typeparam>
        ///<returns>Whether or not any elements were contained.</returns>
        public bool GetOverlapsWithoutRootTest<TTree>(BoundingBoxTree<TTree> opposingTree, IList<TreeOverlapPair<int, TTree>> collidingElements)
            where TTree : class, IBoundingBoxOwner
        {
            //We assume here that the opposing tree and current tree's roots overlap.
            //Also that both trees have been initialized.
            GetOverlaps(opposingTree.nodeArray, 0, 0, collidingElements);
            return collidingElements.Count > 0;
        }

        void GetOverlaps<TTree>(BoundingBoxTree<TTree>.Node[] opposingNodes, int parentA, int parentB, IList<TreeOverlapPair<int, TTree>> collidingElements)
            where TTree : class,IBoundingBoxOwner
        {
            if (nodeArray[parentA].Value != -1)
            {
                //I have children.
                if (opposingNodes[parentB].Value != null)
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
                if (opposingNodes[parentB].Value != null)
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
                        collidingElements.Add(new TreeOverlapPair<int, TTree>(nodeArray[parentA].Value, opposingNodes[parentB].Value));

                }
            }
        }

        ///<summary>
        /// Gets the overlaps between the tree and another tree.
        ///</summary>
        ///<param name="opposingTree">The tree to test against.</param>
        ///<param name="collidingElements">Overlapping elements in the trees.</param>
        ///<returns>Whether or not any elements were contained.</returns>
        public bool GetOverlaps(MeshBoundingBoxTree opposingTree, IList<TreeOverlapPair<int, int>> collidingElements)
        {
            bool isOverlapping;
            opposingTree.nodeArray[0].BoundingBox.Intersects(ref nodeArray[0].BoundingBox, out isOverlapping);
            if (isOverlapping)
                GetOverlapsWithoutRootTest(opposingTree, collidingElements);
            return collidingElements.Count > 0;

        }

        ///<summary>
        /// Gets the overlaps between the tree and another tree without performing a root bounding box test.
        ///</summary>
        ///<param name="opposingTree">The tree to test against.</param>
        ///<param name="collidingElements">Overlapping elements in the trees.</param>
        ///<returns>Whether or not any elements were contained.</returns>
        public bool GetOverlapsWithoutRootTest(MeshBoundingBoxTree opposingTree, IList<TreeOverlapPair<int, int>> collidingElements)
        {
            //We assume here that the opposing tree and current tree's roots overlap.
            //Also that both trees have been initialized.
            GetOverlaps(opposingTree.nodeArray, 0, 0, collidingElements);
            return collidingElements.Count > 0;
        }

        void GetOverlaps(Node[] opposingNodes, int parentA, int parentB, IList<TreeOverlapPair<int, int>> collidingElements)
        {
            if (nodeArray[parentA].Value != -1)
            {
                //I have children.
                if (opposingNodes[parentB].Value != -1)
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
                if (opposingNodes[parentB].Value != -1)
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
                        collidingElements.Add(new TreeOverlapPair<int, int>(nodeArray[parentA].Value, opposingNodes[parentB].Value));

                }
            }
        }
        #endregion



        internal struct Node
        {
            public BoundingBox BoundingBox;
            public int Value;

        }

        //Epsilons in comparers help handle precision errors. 
        XAxisComparer xAxisComparer;
        class XAxisComparer : IComparer<int>
        {
            MeshBoundingBoxTree tree;
            public XAxisComparer(MeshBoundingBoxTree tree)
            {
                this.tree = tree;
            }
            public int Compare(int a, int b)
            {
                BoundingBox aBoundingBox, bBoundingBox;
                tree.triangleMeshData.GetBoundingBox(a, out aBoundingBox);
                tree.triangleMeshData.GetBoundingBox(b, out bBoundingBox);
                float x1 = (aBoundingBox.Max.X + aBoundingBox.Min.X) * .5f;
                float x2 = (bBoundingBox.Max.X + bBoundingBox.Min.X) * .5f;
                if (x1 > x2 + Toolbox.BigEpsilon)
                    return 1;
                if (x2 > x1 + Toolbox.BigEpsilon)
                    return -1;
                return 0;

            }
        }

        YAxisComparer yAxisComparer;
        class YAxisComparer : IComparer<int>
        {
            MeshBoundingBoxTree tree;
            public YAxisComparer(MeshBoundingBoxTree tree)
            {
                this.tree = tree;
            }
            public int Compare(int a, int b)
            {
                BoundingBox aBoundingBox, bBoundingBox;
                tree.triangleMeshData.GetBoundingBox(a, out aBoundingBox);
                tree.triangleMeshData.GetBoundingBox(b, out bBoundingBox);
                float y1 = (aBoundingBox.Max.Y + aBoundingBox.Min.Y) * .5f;
                float y2 = (bBoundingBox.Max.Y + bBoundingBox.Min.Y) * .5f;
                if (y1 > y2 + Toolbox.BigEpsilon)
                    return 1;
                if (y2 > y1 + Toolbox.BigEpsilon)
                    return -1;
                return 0;

            }
        }
        ZAxisComparer zAxisComparer;
        class ZAxisComparer : IComparer<int>
        {
            MeshBoundingBoxTree tree;
            public ZAxisComparer(MeshBoundingBoxTree tree)
            {
                this.tree = tree;
            }
            public int Compare(int a, int b)
            {
                BoundingBox aBoundingBox, bBoundingBox;
                tree.triangleMeshData.GetBoundingBox(a, out aBoundingBox);
                tree.triangleMeshData.GetBoundingBox(b, out bBoundingBox);
                float z1 = (aBoundingBox.Max.Z + aBoundingBox.Min.Z) * .5f;
                float z2 = (bBoundingBox.Max.Z + bBoundingBox.Min.Z) * .5f;
                if (z1 > z2 + Toolbox.BigEpsilon)
                    return 1;
                if (z2 > z1 + Toolbox.BigEpsilon)
                    return -1;
                return 0;

            }
        }
    }

}
