using System;
using System.Collections.Generic;
using BEPUphysics.CollisionTests.Manifolds;

using BEPUutilities;
using BEPUutilities.DataStructures;

namespace BEPUphysics.CollisionShapes
{
    ///<summary>
    /// The local space data needed by a Terrain collidable.
    /// Contains the Heightmap and other information.
    ///</summary>
    public class TerrainShape : CollisionShape
    {
        private float[,] heights;
        //note: changing heights in array does not fire OnShapeChanged automatically.
        //Need to notify parent manually if you do it.
        ///<summary>
        /// Gets or sets the height field of the terrain shape.
        ///</summary>
        public float[,] Heights
        {
            get
            {
                return heights;
            }
            set
            {
                heights = value;
                OnShapeChanged();
            }
        }



        QuadTriangleOrganization quadTriangleOrganization;
        ///<summary>
        /// Gets or sets the quad triangle organization.
        ///</summary>
        public QuadTriangleOrganization QuadTriangleOrganization
        {
            get
            {
                return quadTriangleOrganization;
            }
            set
            {
                quadTriangleOrganization = value;
                OnShapeChanged();
            }
        }

        ///<summary>
        /// Constructs a TerrainShape.
        ///</summary>
        ///<param name="heights">Heights array used for the shape.</param>
        ///<param name="triangleOrganization">Triangle organization of each quad.</param>
        ///<exception cref="ArgumentException">Thrown if the heights array has less than 2x2 vertices.</exception>
        public TerrainShape(float[,] heights, QuadTriangleOrganization triangleOrganization)
        {
            if (heights.GetLength(0) <= 1 || heights.GetLength(1) <= 1)
            {
                throw new ArgumentException("Terrains must have a least 2x2 vertices (one quad).");
            }
            this.heights = heights;
            quadTriangleOrganization = triangleOrganization;
        }

        ///<summary>
        /// Constructs a TerrainShape.
        ///</summary>
        ///<param name="heights">Heights array used for the shape.</param>
        public TerrainShape(float[,] heights)
            : this(heights, QuadTriangleOrganization.BottomLeftUpperRight)
        {
        }



        ///<summary>
        /// Constructs the bounding box of the terrain given a transform.
        ///</summary>
        ///<param name="transform">Transform to apply to the terrain during the bounding box calculation.</param>
        ///<param name="boundingBox">Bounding box of the terrain shape when transformed.</param>
        public void GetBoundingBox(ref AffineTransform transform, out BoundingBox boundingBox)
        {
#if !WINDOWS
            boundingBox = new BoundingBox();
#endif
            float minX = float.MaxValue, maxX = -float.MaxValue,
                  minY = float.MaxValue, maxY = -float.MaxValue,
                  minZ = float.MaxValue, maxZ = -float.MaxValue;
            Vector3 minXvertex = new Vector3(),
                    maxXvertex = new Vector3(),
                    minYvertex = new Vector3(),
                    maxYvertex = new Vector3(),
                    minZvertex = new Vector3(),
                    maxZvertex = new Vector3();

            //Find the extreme locations.
            for (int i = 0; i < heights.GetLength(0); i++)
            {
                for (int j = 0; j < heights.GetLength(1); j++)
                {
                    var vertex = new Vector3(i, heights[i, j], j);
                    Matrix3x3.Transform(ref vertex, ref transform.LinearTransform, out vertex);
                    if (vertex.X < minX)
                    {
                        minX = vertex.X;
                        minXvertex = vertex;
                    }
                    else if (vertex.X > maxX)
                    {
                        maxX = vertex.X;
                        maxXvertex = vertex;
                    }

                    if (vertex.Y < minY)
                    {
                        minY = vertex.Y;
                        minYvertex = vertex;
                    }
                    else if (vertex.Y > maxY)
                    {
                        maxY = vertex.Y;
                        maxYvertex = vertex;
                    }

                    if (vertex.Z < minZ)
                    {
                        minZ = vertex.Z;
                        minZvertex = vertex;
                    }
                    else if (vertex.Z > maxZ)
                    {
                        maxZ = vertex.Z;
                        maxZvertex = vertex;
                    }
                }
            }

            //Shift the bounding box.
            boundingBox.Min.X = minXvertex.X + transform.Translation.X;
            boundingBox.Min.Y = minYvertex.Y + transform.Translation.Y;
            boundingBox.Min.Z = minZvertex.Z + transform.Translation.Z;
            boundingBox.Max.X = maxXvertex.X + transform.Translation.X;
            boundingBox.Max.Y = maxYvertex.Y + transform.Translation.Y;
            boundingBox.Max.Z = maxZvertex.Z + transform.Translation.Z;
        }

        ///<summary>
        /// Tests a ray against the terrain shape.
        ///</summary>
        ///<param name="ray">Ray to test against the shape.</param>
        ///<param name="maximumLength">Maximum length of the ray in units of the ray direction's length.</param>
        ///<param name="transform">Transform to apply to the terrain shape during the test.</param>
        ///<param name="hit">Hit data of the ray cast, if any.</param>
        ///<returns>Whether or not the ray hit the transformed terrain shape.</returns>
        public bool RayCast(ref Ray ray, float maximumLength, ref AffineTransform transform, out RayHit hit)
        {
            return RayCast(ref ray, maximumLength, ref transform, TriangleSidedness.Counterclockwise, out hit);
        }
        ///<summary>
        /// Tests a ray against the terrain shape.
        ///</summary>
        ///<param name="ray">Ray to test against the shape.</param>
        ///<param name="maximumLength">Maximum length of the ray in units of the ray direction's length.</param>
        ///<param name="transform">Transform to apply to the terrain shape during the test.</param>
        ///<param name="sidedness">Sidedness of the triangles to use when raycasting.</param>
        ///<param name="hit">Hit data of the ray cast, if any.</param>
        ///<returns>Whether or not the ray hit the transformed terrain shape.</returns>
        public bool RayCast(ref Ray ray, float maximumLength, ref AffineTransform transform, TriangleSidedness sidedness, out RayHit hit)
        {
            hit = new RayHit();
            //Put the ray into local space.
            Ray localRay;
            AffineTransform inverse;
            AffineTransform.Invert(ref transform, out inverse);
            Matrix3x3.Transform(ref ray.Direction, ref inverse.LinearTransform, out localRay.Direction);
            AffineTransform.Transform(ref ray.Position, ref inverse, out localRay.Position);

            //Use rasterizey traversal.
            //The origin is at 0,0,0 and the map goes +X, +Y, +Z.
            //if it's before the origin and facing away, or outside the max and facing out, early out.
            float maxX = heights.GetLength(0) - 1;
            float maxZ = heights.GetLength(1) - 1;

            Vector3 progressingOrigin = localRay.Position;
            float distance = 0;
            //Check the outside cases first.
            if (progressingOrigin.X < 0)
            {
                if (localRay.Direction.X > 0)
                {
                    //Off the left side.
                    float timeToMinX = -progressingOrigin.X / localRay.Direction.X;
                    distance += timeToMinX;
                    Vector3 increment;
                    Vector3.Multiply(ref localRay.Direction, timeToMinX, out increment);
                    Vector3.Add(ref increment, ref progressingOrigin, out progressingOrigin);
                }
                else
                    return false; //Outside and pointing away from the terrain.
            }
            else if (progressingOrigin.X > maxX)
            {
                if (localRay.Direction.X < 0)
                {
                    //Off the left side.
                    float timeToMinX = -(progressingOrigin.X - maxX) / localRay.Direction.X;
                    distance += timeToMinX;
                    Vector3 increment;
                    Vector3.Multiply(ref localRay.Direction, timeToMinX, out increment);
                    Vector3.Add(ref increment, ref progressingOrigin, out progressingOrigin);
                }
                else
                    return false; //Outside and pointing away from the terrain.
            }

            if (progressingOrigin.Z < 0)
            {
                if (localRay.Direction.Z > 0)
                {
                    float timeToMinZ = -progressingOrigin.Z / localRay.Direction.Z;
                    distance += timeToMinZ;
                    Vector3 increment;
                    Vector3.Multiply(ref localRay.Direction, timeToMinZ, out increment);
                    Vector3.Add(ref increment, ref progressingOrigin, out progressingOrigin);
                }
                else
                    return false;
            }
            else if (progressingOrigin.Z > maxZ)
            {
                if (localRay.Direction.Z < 0)
                {
                    float timeToMinZ = -(progressingOrigin.Z - maxZ) / localRay.Direction.Z;
                    distance += timeToMinZ;
                    Vector3 increment;
                    Vector3.Multiply(ref localRay.Direction, timeToMinZ, out increment);
                    Vector3.Add(ref increment, ref progressingOrigin, out progressingOrigin);
                }
                else
                    return false;
            }

            if (distance > maximumLength)
                return false;



            //By now, we should be entering the main body of the terrain.

            int xCell = (int)progressingOrigin.X;
            int zCell = (int)progressingOrigin.Z;
            //If it's hitting the border and going in, then correct the index
            //so that it will initially target a valid quad.
            //Without this, a quad beyond the border would be tried and failed.
            if (xCell == heights.GetLength(0) - 1 && localRay.Direction.X < 0)
                xCell = heights.GetLength(0) - 2;
            if (zCell == heights.GetLength(1) - 1 && localRay.Direction.Z < 0)
                zCell = heights.GetLength(1) - 2;

            while (true)
            {
                //Check for a miss.
                if (xCell < 0 ||
                    zCell < 0 ||
                    xCell >= heights.GetLength(0) - 1 ||
                    zCell >= heights.GetLength(1) - 1)
                    return false;

                //Test the triangles of this cell.
                Vector3 v1, v2, v3, v4;
                // v3 v4
                // v1 v2
                GetLocalPosition(xCell, zCell, out v1);
                GetLocalPosition(xCell + 1, zCell, out v2);
                GetLocalPosition(xCell, zCell + 1, out v3);
                GetLocalPosition(xCell + 1, zCell + 1, out v4);
                RayHit hit1, hit2;
                bool didHit1;
                bool didHit2;

                //Don't bother doing ray intersection tests if the ray can't intersect it.

                float highest = v1.Y;
                float lowest = v1.Y;
                if (v2.Y > highest)
                    highest = v2.Y;
                else if (v2.Y < lowest)
                    lowest = v2.Y;
                if (v3.Y > highest)
                    highest = v3.Y;
                else if (v3.Y < lowest)
                    lowest = v3.Y;
                if (v4.Y > highest)
                    highest = v4.Y;
                else if (v4.Y < lowest)
                    lowest = v4.Y;


                if (!(progressingOrigin.Y > highest && localRay.Direction.Y > 0 ||
                    progressingOrigin.Y < lowest && localRay.Direction.Y < 0))
                {


                    if (quadTriangleOrganization == QuadTriangleOrganization.BottomLeftUpperRight)
                    {
                        //Always perform the raycast as if Y+ in local space is the way the triangles are facing.
                        didHit1 = Toolbox.FindRayTriangleIntersection(ref localRay, maximumLength, sidedness, ref v1, ref v2, ref v3, out hit1);
                        didHit2 = Toolbox.FindRayTriangleIntersection(ref localRay, maximumLength, sidedness, ref v2, ref v4, ref v3, out hit2);
                    }
                    else //if (quadTriangleOrganization == CollisionShapes.QuadTriangleOrganization.BottomRightUpperLeft)
                    {
                        didHit1 = Toolbox.FindRayTriangleIntersection(ref localRay, maximumLength, sidedness, ref v1, ref v2, ref v4, out hit1);
                        didHit2 = Toolbox.FindRayTriangleIntersection(ref localRay, maximumLength, sidedness, ref v1, ref v4, ref v3, out hit2);
                    }
                    if (didHit1 && didHit2)
                    {
                        if (hit1.T < hit2.T)
                        {
                            Vector3.Multiply(ref ray.Direction, hit1.T, out hit.Location);
                            Vector3.Add(ref hit.Location, ref ray.Position, out hit.Location);
                            Matrix3x3.TransformTranspose(ref hit1.Normal, ref inverse.LinearTransform, out hit.Normal);
                            hit.T = hit1.T;
                            return true;
                        }
                        Vector3.Multiply(ref ray.Direction, hit2.T, out hit.Location);
                        Vector3.Add(ref hit.Location, ref ray.Position, out hit.Location);
                        Matrix3x3.TransformTranspose(ref hit2.Normal, ref inverse.LinearTransform, out hit.Normal);
                        hit.T = hit2.T;
                        return true;
                    }
                    else if (didHit1)
                    {
                        Vector3.Multiply(ref ray.Direction, hit1.T, out hit.Location);
                        Vector3.Add(ref hit.Location, ref ray.Position, out hit.Location);
                        Matrix3x3.TransformTranspose(ref hit1.Normal, ref inverse.LinearTransform, out hit.Normal);
                        hit.T = hit1.T;
                        return true;
                    }
                    else if (didHit2)
                    {
                        Vector3.Multiply(ref ray.Direction, hit2.T, out hit.Location);
                        Vector3.Add(ref hit.Location, ref ray.Position, out hit.Location);
                        Matrix3x3.TransformTranspose(ref hit2.Normal, ref inverse.LinearTransform, out hit.Normal);
                        hit.T = hit2.T;
                        return true;
                    }
                }

                //Move to the next cell.

                float timeToX;
                if (localRay.Direction.X < 0)
                    timeToX = -(progressingOrigin.X - xCell) / localRay.Direction.X;
                else if (ray.Direction.X > 0)
                    timeToX = (xCell + 1 - progressingOrigin.X) / localRay.Direction.X;
                else
                    timeToX = float.MaxValue;

                float timeToZ;
                if (localRay.Direction.Z < 0)
                    timeToZ = -(progressingOrigin.Z - zCell) / localRay.Direction.Z;
                else if (localRay.Direction.Z > 0)
                    timeToZ = (zCell + 1 - progressingOrigin.Z) / localRay.Direction.Z;
                else
                    timeToZ = float.MaxValue;

                //Move to the next cell.
                if (timeToX < timeToZ)
                {
                    if (localRay.Direction.X < 0)
                        xCell--;
                    else
                        xCell++;

                    distance += timeToX;
                    if (distance > maximumLength)
                        return false;

                    Vector3 increment;
                    Vector3.Multiply(ref localRay.Direction, timeToX, out increment);
                    Vector3.Add(ref increment, ref progressingOrigin, out progressingOrigin);
                }
                else
                {
                    if (localRay.Direction.Z < 0)
                        zCell--;
                    else
                        zCell++;

                    distance += timeToZ;
                    if (distance > maximumLength)
                        return false;

                    Vector3 increment;
                    Vector3.Multiply(ref localRay.Direction, timeToZ, out increment);
                    Vector3.Add(ref increment, ref progressingOrigin, out progressingOrigin);
                }

            }


        }

        ///<summary>
        /// Gets the position of a vertex at the given indices in local space.
        ///</summary>
        ///<param name="columnIndex">Index in the first dimension.</param>
        ///<param name="rowIndex">Index in the second dimension.</param>
        ///<param name="v">Local space position at the given vertice.s</param>
        public void GetLocalPosition(int columnIndex, int rowIndex, out Vector3 v)
        {
#if !WINDOWS
            v = new Vector3();
#endif
            v.X = columnIndex;
            v.Y = heights[columnIndex, rowIndex];
            v.Z = rowIndex;
        }

        /// <summary>
        /// Gets the world space position of a vertex in the terrain at the given indices.
        /// </summary>
        ///<param name="columnIndex">Index in the first dimension.</param>
        ///<param name="rowIndex">Index in the second dimension.</param>
        /// <param name="transform">Transform to apply to the vertex.</param>
        /// <param name="position">Transformed position of the vertex at the given indices.</param>
        public void GetPosition(int columnIndex, int rowIndex, ref AffineTransform transform, out Vector3 position)
        {
            if (columnIndex <= 0)
                columnIndex = 0;
            else if (columnIndex >= heights.GetLength(0))
                columnIndex = heights.GetLength(0) - 1;
            if (rowIndex <= 0)
                rowIndex = 0;
            else if (rowIndex >= heights.GetLength(1))
                rowIndex = heights.GetLength(1) - 1;
#if !WINDOWS
            position = new Vector3();
#endif
            position.X = columnIndex;
            position.Y = heights[columnIndex, rowIndex];
            position.Z = rowIndex;
            AffineTransform.Transform(ref position, ref transform, out position);


        }


        /// <summary>
        /// Gets the non-normalized local space vertex normal at the given indices.
        /// </summary>
        ///<param name="columnIndex">Vertex index in the first dimension.</param>
        ///<param name="rowIndex">Vertex index in the second dimension.</param>
        /// <param name="normal">Non-normalized local space normal at the given indices.</param>
        public void GetLocalNormal(int columnIndex, int rowIndex, out Vector3 normal)
        {

            float topHeight = heights[columnIndex, Math.Min(rowIndex + 1, heights.GetLength(1) - 1)];
            float bottomHeight = heights[columnIndex, Math.Max(rowIndex - 1, 0)];
            float rightHeight = heights[Math.Min(columnIndex + 1, heights.GetLength(0) - 1), rowIndex];
            float leftHeight = heights[Math.Max(columnIndex - 1, 0), rowIndex];

            //Since the horizontal offsets are known to be 1 in local space, we can omit quite a few operations compared to a full Vector3 and cross product.

            //The implicit vectors are:
            //var leftToRight = new Vector3(2, rightHeight - leftHeight, 0);
            //var bottomToTop = new Vector3(0, topHeight - bottomHeight, 2);
            //the result is then:
            //Vector3.Cross(bottomToTop, leftToRight);
            //Which is:
            //float resultX = bottomToTop.Y * leftToRight.Z - bottomToTop.Z * leftToRight.Y;
            //float resultY = bottomToTop.Z * leftToRight.X - bottomToTop.X * leftToRight.Z;
            //float resultZ = bottomToTop.X * leftToRight.Y - bottomToTop.Y * leftToRight.X;
            //Which becomes:
            //float resultX = bottomToTop.Y * 0 - 2 * leftToRight.Y;
            //float resultY = 2 * 2 - 0 * 0;
            //float resultZ = 0 * leftToRight.Y - bottomToTop.Y * 2;
            //Which becomes:
            normal.X = rightHeight - leftHeight;
            normal.Y = 2;
            normal.Z = topHeight - bottomHeight;

        }


        ///<summary>
        /// Gets overlapped triangles with the terrain shape with a bounding box in the local space of the shape.
        ///</summary>
        ///<param name="localBoundingBox">Bounding box in the local space of the terrain shape.</param>
        ///<param name="overlappedElements">Indices of triangles whose bounding boxes overlap the input bounding box. Encoded as 2 * (quadRowIndex * terrainWidthInQuads + quadColumnIndex) + isFirstTriangleOfQuad ? 0 : 1, where isFirstTriangleOfQuad refers to which of the two triangles in a quad is being requested. Matches the input of the TerrainShape.GetTriangle function.</param>
        ///<typeparam name="T">Type of the list to fill with overlaps.</typeparam>
        public bool GetOverlaps<T>(BoundingBox localBoundingBox, ref T overlappedElements) where T : IList<int> //Designed to work with value type ILists, hence anti-boxing interface constraint and ref.
        {
            int width = heights.GetLength(0);
            int minX = Math.Max((int)localBoundingBox.Min.X, 0);
            int minY = Math.Max((int)localBoundingBox.Min.Z, 0);
            int maxX = Math.Min((int)localBoundingBox.Max.X, width - 2);
            int maxY = Math.Min((int)localBoundingBox.Max.Z, heights.GetLength(1) - 2);
            for (int i = minX; i <= maxX; i++)
            {
                for (int j = minY; j <= maxY; j++)
                {
                    //Before adding a triangle to the list, make sure the object isn't too high or low from the quad.
                    float highest, lowest;
                    float y1 = heights[i, j];
                    float y2 = heights[i + 1, j];
                    float y3 = heights[i, j + 1];
                    float y4 = heights[i + 1, j + 1];

                    highest = y1;
                    lowest = y1;
                    if (y2 > highest)
                        highest = y2;
                    else if (y2 < lowest)
                        lowest = y2;
                    if (y3 > highest)
                        highest = y3;
                    else if (y3 < lowest)
                        lowest = y3;
                    if (y4 > highest)
                        highest = y4;
                    else if (y4 < lowest)
                        lowest = y4;


                    if (localBoundingBox.Max.Y < lowest ||
                        localBoundingBox.Min.Y > highest)
                        continue;

                    //Now the local bounding box is very likely intersecting those of the triangles.
                    //Add the triangles to the list.
                    int quadIndex = (i + j * width) << 1;
                    overlappedElements.Add(quadIndex);
                    overlappedElements.Add(quadIndex | 1);


                }
            }
            return overlappedElements.Count > 0;
        }


        ///<summary>
        /// Gets the first triangle of the quad at the given indices in world space.
        ///</summary>
        ///<param name="columnIndex">Index of the triangle's quad in the first dimension.</param>
        ///<param name="rowIndex">Index of the triangle's quad in the second dimension.</param>
        ///<param name="transform">Transform to apply to the triangle vertices.</param>
        ///<param name="a">First vertex of the triangle.</param>
        ///<param name="b">Second vertex of the triangle.</param>
        ///<param name="c">Third vertex of the triangle.</param>
        public void GetFirstTriangle(int columnIndex, int rowIndex, ref AffineTransform transform, out Vector3 a, out Vector3 b, out Vector3 c)
        {
            if (quadTriangleOrganization == QuadTriangleOrganization.BottomLeftUpperRight)
            {
                GetPosition(columnIndex, rowIndex, ref transform, out a);
                GetPosition(columnIndex + 1, rowIndex, ref transform, out b);
                GetPosition(columnIndex, rowIndex + 1, ref transform, out c);
            }
            else
            {
                GetPosition(columnIndex, rowIndex, ref transform, out a);
                GetPosition(columnIndex + 1, rowIndex, ref transform, out b);
                GetPosition(columnIndex + 1, rowIndex + 1, ref transform, out c);
            }
        }

        ///<summary>
        /// Gets the second triangle of the quad at the given indices in world space.
        ///</summary>
        ///<param name="columnIndex">Index of the triangle's quad in the first dimension.</param>
        ///<param name="rowIndex">Index of the triangle's quad in the second dimension.</param>
        ///<param name="transform">Transform to apply to the triangle vertices.</param>
        ///<param name="a">First vertex of the triangle.</param>
        ///<param name="b">Second vertex of the triangle.</param>
        ///<param name="c">Third vertex of the triangle.</param>
        public void GetSecondTriangle(int columnIndex, int rowIndex, ref AffineTransform transform, out Vector3 a, out Vector3 b, out Vector3 c)
        {
            if (quadTriangleOrganization == QuadTriangleOrganization.BottomLeftUpperRight)
            {
                GetPosition(columnIndex, rowIndex + 1, ref transform, out a);
                GetPosition(columnIndex + 1, rowIndex + 1, ref transform, out b);
                GetPosition(columnIndex + 1, rowIndex, ref transform, out c);
            }
            else
            {
                GetPosition(columnIndex, rowIndex, ref transform, out a);
                GetPosition(columnIndex, rowIndex + 1, ref transform, out b);
                GetPosition(columnIndex + 1, rowIndex + 1, ref transform, out c);
            }
        }


        ///<summary>
        /// Gets a world space triangle in the terrain at the given triangle index.
        ///</summary>
        ///<param name="index">Index of the triangle. Encoded as 2 * (quadRowIndex * terrainWidthInQuads + quadColumnIndex) + isFirstTriangleOfQuad ? 0 : 1, where isFirstTriangleOfQuad refers to which of the two triangles in a quad is being requested. Matches the output of the TerrainShape.GetOverlaps function.</param>
        ///<param name="transform">Transform to apply to the triangle vertices.</param>
        ///<param name="a">First vertex of the triangle.</param>
        ///<param name="b">Second vertex of the triangle.</param>
        ///<param name="c">Third vertex of the triangle.</param>
        public void GetTriangle(int index, ref AffineTransform transform, out Vector3 a, out Vector3 b, out Vector3 c)
        {
            //Find the quad.
            int quadIndex = index / 2;
            //TODO: This division could be avoided if you're willing to get tricky or impose some size requirements.
            int rowIndex = quadIndex / heights.GetLength(0);
            int columnIndex = quadIndex - rowIndex * heights.GetLength(0);
            if ((index & 1) == 0) //Check if this is the first or second triangle.
            {
                GetFirstTriangle(columnIndex, rowIndex, ref transform, out a, out b, out c);
            }
            else
            {
                GetSecondTriangle(columnIndex, rowIndex, ref transform, out a, out b, out c);
            }
        }



        internal void GetLocalIndices(int i, out TerrainVertexIndices a, out TerrainVertexIndices b, out TerrainVertexIndices c)
        {
            int quadIndex = i / 2;
            //TODO: This division could be avoided if you're willing to get tricky or impose some size requirements.
            int rowIndex = quadIndex / heights.GetLength(0);
            int columnIndex = quadIndex - rowIndex * heights.GetLength(0);
            if ((i & 1) == 0) //Check if this is the first or second triangle.
            {
                if (quadTriangleOrganization == QuadTriangleOrganization.BottomLeftUpperRight)
                {
                    a = new TerrainVertexIndices { ColumnIndex = columnIndex, RowIndex = rowIndex };
                    b = new TerrainVertexIndices { ColumnIndex = columnIndex + 1, RowIndex = rowIndex };
                    c = new TerrainVertexIndices { ColumnIndex = columnIndex, RowIndex = rowIndex + 1 };
                }
                else
                {
                    a = new TerrainVertexIndices { ColumnIndex = columnIndex, RowIndex = rowIndex };
                    b = new TerrainVertexIndices { ColumnIndex = columnIndex + 1, RowIndex = rowIndex };
                    c = new TerrainVertexIndices { ColumnIndex = columnIndex + 1, RowIndex = rowIndex + 1 };
                }
            }
            else
            {
                if (quadTriangleOrganization == QuadTriangleOrganization.BottomLeftUpperRight)
                {
                    a = new TerrainVertexIndices { ColumnIndex = columnIndex, RowIndex = rowIndex + 1 };
                    c = new TerrainVertexIndices { ColumnIndex = columnIndex + 1, RowIndex = rowIndex + 1 };
                    b = new TerrainVertexIndices { ColumnIndex = columnIndex + 1, RowIndex = rowIndex };
                }
                else
                {
                    a = new TerrainVertexIndices { ColumnIndex = columnIndex, RowIndex = rowIndex };
                    b = new TerrainVertexIndices { ColumnIndex = columnIndex + 1, RowIndex = rowIndex + 1 };
                    c = new TerrainVertexIndices { ColumnIndex = columnIndex, RowIndex = rowIndex + 1 };
                }
            }
        }
    }

    /// <summary>
    /// Defines how a Terrain organizes triangles in its quads.
    /// </summary>
    public enum QuadTriangleOrganization
    {
        /// <summary>
        /// Triangle with a right angle at the (-i,-j) position and another at the (+i,+j) position.
        /// </summary>
        BottomLeftUpperRight,
        /// <summary>
        /// Triangle with a right angle at the (+i,-j) position and another at the high (-i,+j) position.
        /// </summary>
        BottomRightUpperLeft
    }
}
