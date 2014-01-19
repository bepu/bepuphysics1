using System;
using System.Collections.Generic;
using BEPUphysics.CollisionTests.Manifolds;
using BEPUutilities;
using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;

namespace BEPUphysics.CollisionShapes.ConvexShapes
{
    ///<summary>
    /// Helper class used to compute volume distribution information, which is in turn used to compute inertia tensor information.
    ///</summary>
    public static class InertiaHelper
    {
        /// <summary>
        /// Value to scale any created entities' inertia tensors by.
        /// Larger tensors (above 1) improve stiffness of constraints and contacts, while smaller values (towards 1) are closer to 'realistic' behavior.
        /// Defaults to 2.5.
        /// </summary>
        public static float InertiaTensorScale = 2.5f;

        ///<summary>
        /// Number of samples the system takes along a side of an object's AABB when voxelizing it.
        ///</summary>
        public static int NumberOfSamplesPerDimension = 10;



        ///<summary>
        /// Computes the volume contribution of a point.
        ///</summary>
        ///<param name="pointWeight">Weight of the point.</param>
        ///<param name="center">Location to use as the center for the purposes of computing the contribution.</param>
        ///<param name="point">Point to compute the contribution of.</param>
        ///<param name="contribution">Contribution of the point.</param>
        public static void GetPointContribution(float pointWeight, ref Vector3 center, ref Vector3 point, out Matrix3x3 contribution)
        {
            Vector3.Subtract(ref point, ref center, out point);
            float xx = pointWeight * point.X * point.X;
            float yy = pointWeight * point.Y * point.Y;
            float zz = pointWeight * point.Z * point.Z;
            contribution.M11 = yy + zz;
            contribution.M22 = xx + zz;
            contribution.M33 = xx + yy;
            contribution.M12 = -pointWeight * point.X * point.Y;
            contribution.M13 = -pointWeight * point.X * point.Z;
            contribution.M23 = -pointWeight * point.Y * point.Z;
            contribution.M21 = contribution.M12;
            contribution.M31 = contribution.M13;
            contribution.M32 = contribution.M23;
        }




        /// <summary>
        /// Set of directions sampled by the inertia helper when constructing a mesh representation of a convex object.
        /// </summary>
        public static Vector3[] SampleDirections;

        /// <summary>
        /// Set of triangles represented by groups of three indices into the SampleDirections array.
        /// </summary>
        public static int[] SampleTriangleIndices;


        static InertiaHelper()
        {
            GenerateSphere(1, out SampleDirections, out SampleTriangleIndices);
        }


        /// <summary>
        /// Generates a sphere by progressively refining a tetrahedral mesh.
        /// </summary>
        /// <param name="subdivisionCount">Number of recursive splits to perform on the triangles.</param>
        /// <param name="vertices">Vertex buffer of the sphere.</param>
        /// <param name="triangles">Index buffer of the sphere. Each group of three integers represents one triangle.</param>
        public static void GenerateSphere(int subdivisionCount, out Vector3[] vertices, out int[] triangles)
        {

            //int finalSize = 3 * (4 << (subdivisionCount << 1));
            //int secondToLastSize = finalSize >> 2;

            int finalSize = 3 * (20 << (subdivisionCount * 2));
            int secondToLastSize = finalSize >> 2;


            triangles = new int[finalSize];
            //Grabbing a RawList<int> for this purpose is a little gross. Really, just an int[] is desired.
            var swapTriangles = new int[secondToLastSize];

            int[] currentTriangles, nextTriangles;
            if ((subdivisionCount & 1) == 0)
            {
                //Subdivision count is even.
                //The "currentTriangles" reference will end up being the final container.
                currentTriangles = triangles;
                nextTriangles = swapTriangles;
            }
            else
            {
                //Subdivision count is odd.
                //The "nextTriangles" reference will end up being the final container.
                currentTriangles = swapTriangles;
                nextTriangles = triangles;
            }

            //The number of vertices that we'll end up with is known analytically.
            vertices = new Vector3[GetExpectedVertexCount(subdivisionCount)];

            ////Create the regular tetrahedron vertices.
            //float x = (float)(1 / Math.Sqrt(3));
            //float z = (float)(-1 / (2 * Math.Sqrt(6)));
            //vertices[0] = Vector3.Normalize(new Vector3(0, 0, (float)(Math.Sqrt(2.0 / 3.0) + z)));
            //vertices[1] = Vector3.Normalize(new Vector3(-0.5f * x, -0.5f, z));
            //vertices[2] = Vector3.Normalize(new Vector3(-0.5f * x, 0.5f, z));
            //vertices[3] = Vector3.Normalize(new Vector3(x, 0, z));
            ////Just treat this array as a list.
            //int vertexCount = 4;

            ////Create the regular tetrahedron triangles.
            ////The winding matters. They should be consistent so that the end result is consistent.
            //currentTriangles[0] = 0;
            //currentTriangles[1] = 1;
            //currentTriangles[2] = 2;

            //currentTriangles[3] = 0;
            //currentTriangles[4] = 2;
            //currentTriangles[5] = 3;

            //currentTriangles[6] = 1;
            //currentTriangles[7] = 3;
            //currentTriangles[8] = 2;

            //currentTriangles[9] = 0;
            //currentTriangles[10] = 3;
            //currentTriangles[11] = 1;

            //int currentTriangleIndexCount = 12;
            //int nextTriangleIndexCount = 0;

            //Create the regular icosahedron vertices.
            //Vector3[] vertices = new Vector3[12];
            var goldenRatio = (1 + (float)Math.Sqrt(5)) / 2;
            float length = (float)Math.Sqrt(1 + goldenRatio * goldenRatio);
            float x = 1 / length;
            float y = goldenRatio / length;
            vertices[0] = new Vector3(0, x, y);
            vertices[1] = new Vector3(0, -x, y);
            vertices[2] = new Vector3(0, x, -y);
            vertices[3] = new Vector3(0, -x, -y);

            vertices[4] = new Vector3(x, y, 0);
            vertices[5] = new Vector3(-x, y, 0);
            vertices[6] = new Vector3(x, -y, 0);
            vertices[7] = new Vector3(-x, -y, 0);

            vertices[8] = new Vector3(y, 0, x);
            vertices[9] = new Vector3(-y, 0, x);
            vertices[10] = new Vector3(y, 0, -x);
            vertices[11] = new Vector3(-y, 0, -x);

            //Just treat this array as a list.
            int vertexCount = 12;

            //The winding matters. They should be consistent so that the end result is consistent.
            //[This was generated using GetConvexHull on the above vertices, so it's known to be consistent with GetConvexHull!]
            currentTriangles[0] = 8;
            currentTriangles[1] = 10;
            currentTriangles[2] = 6;
            currentTriangles[3] = 2;
            currentTriangles[4] = 3;
            currentTriangles[5] = 10;
            currentTriangles[6] = 8;
            currentTriangles[7] = 6;
            currentTriangles[8] = 1;
            currentTriangles[9] = 9;
            currentTriangles[10] = 11;
            currentTriangles[11] = 5;
            currentTriangles[12] = 6;
            currentTriangles[13] = 10;
            currentTriangles[14] = 3;
            currentTriangles[15] = 9;
            currentTriangles[16] = 5;
            currentTriangles[17] = 0;
            currentTriangles[18] = 2;
            currentTriangles[19] = 5;
            currentTriangles[20] = 11;
            currentTriangles[21] = 8;
            currentTriangles[22] = 4;
            currentTriangles[23] = 10;
            currentTriangles[24] = 8;
            currentTriangles[25] = 0;
            currentTriangles[26] = 4;
            currentTriangles[27] = 2;
            currentTriangles[28] = 10;
            currentTriangles[29] = 4;
            currentTriangles[30] = 2;
            currentTriangles[31] = 11;
            currentTriangles[32] = 3;
            currentTriangles[33] = 8;
            currentTriangles[34] = 1;
            currentTriangles[35] = 0;
            currentTriangles[36] = 9;
            currentTriangles[37] = 0;
            currentTriangles[38] = 1;
            currentTriangles[39] = 5;
            currentTriangles[40] = 2;
            currentTriangles[41] = 4;
            currentTriangles[42] = 5;
            currentTriangles[43] = 4;
            currentTriangles[44] = 0;
            currentTriangles[45] = 9;
            currentTriangles[46] = 1;
            currentTriangles[47] = 7;
            currentTriangles[48] = 6;
            currentTriangles[49] = 7;
            currentTriangles[50] = 1;
            currentTriangles[51] = 9;
            currentTriangles[52] = 7;
            currentTriangles[53] = 11;
            currentTriangles[54] = 11;
            currentTriangles[55] = 7;
            currentTriangles[56] = 3;
            currentTriangles[57] = 6;
            currentTriangles[58] = 3;
            currentTriangles[59] = 7;

            int currentTriangleIndexCount = 60;
            int nextTriangleIndexCount = 0;


            var edges = new Dictionary<TriangleMeshConvexContactManifold.Edge, int>();

            for (int i = 0; i < subdivisionCount; ++i)
            {
                for (int triangleIndex = 0; triangleIndex < currentTriangleIndexCount; triangleIndex += 3)
                {
                    //For each edge of this triangle, insert a new vertex if the edge hasn't already been taken care of.
                    var aIndex = currentTriangles[triangleIndex];
                    var bIndex = currentTriangles[triangleIndex + 1];
                    var cIndex = currentTriangles[triangleIndex + 2];
                    var a = vertices[aIndex];
                    var b = vertices[bIndex];
                    var c = vertices[cIndex];
                    int abMidIndex;
                    var edge = new TriangleMeshConvexContactManifold.Edge(aIndex, bIndex);
                    if (!edges.TryGetValue(edge, out abMidIndex))
                    {
                        //This edge hasn't yet been handled by another triangle.
                        //Create a vertex.
                        Vector3 mid;
                        Vector3.Add(ref a, ref b, out mid);
                        mid.Normalize();
                        abMidIndex = vertexCount;
                        vertices.Add(ref mid, ref vertexCount);

                        //Mark this edge as handled.
                        edges.Add(edge, abMidIndex);
                    }

                    int bcMidIndex;
                    edge = new TriangleMeshConvexContactManifold.Edge(bIndex, cIndex);
                    if (!edges.TryGetValue(edge, out bcMidIndex))
                    {
                        //This edge hasn't yet been handled by another triangle.
                        //Create a vertex.
                        Vector3 mid;
                        Vector3.Add(ref b, ref c, out mid);
                        mid.Normalize();
                        bcMidIndex = vertexCount;
                        vertices.Add(ref mid, ref vertexCount);

                        //Mark this edge as handled.
                        edges.Add(edge, bcMidIndex);
                    }

                    int acMidIndex;
                    edge = new TriangleMeshConvexContactManifold.Edge(aIndex, cIndex);
                    if (!edges.TryGetValue(edge, out acMidIndex))
                    {
                        //This edge hasn't yet been handled by another triangle.
                        Vector3 mid;
                        Vector3.Add(ref a, ref c, out mid);
                        mid.Normalize();
                        acMidIndex = vertexCount;
                        vertices.Add(ref mid, ref vertexCount);

                        //Mark this edge as handled.
                        edges.Add(edge, acMidIndex);
                    }

                    //Create the new triangles with consistent winding.
                    nextTriangles.Add(aIndex, ref nextTriangleIndexCount);
                    nextTriangles.Add(abMidIndex, ref nextTriangleIndexCount);
                    nextTriangles.Add(acMidIndex, ref nextTriangleIndexCount);

                    nextTriangles.Add(abMidIndex, ref nextTriangleIndexCount);
                    nextTriangles.Add(bIndex, ref nextTriangleIndexCount);
                    nextTriangles.Add(bcMidIndex, ref nextTriangleIndexCount);

                    nextTriangles.Add(abMidIndex, ref nextTriangleIndexCount);
                    nextTriangles.Add(bcMidIndex, ref nextTriangleIndexCount);
                    nextTriangles.Add(acMidIndex, ref nextTriangleIndexCount);

                    nextTriangles.Add(acMidIndex, ref nextTriangleIndexCount);
                    nextTriangles.Add(bcMidIndex, ref nextTriangleIndexCount);
                    nextTriangles.Add(cIndex, ref nextTriangleIndexCount);

                }

                //Clear out the edges collection. It will get reused when we iterate through the next deeper set of triangles.
                edges.Clear();

                //Swap the triangle buffer references.
                currentTriangleIndexCount = nextTriangleIndexCount;
                nextTriangleIndexCount = 0;
                var swap = currentTriangles;
                currentTriangles = nextTriangles;
                nextTriangles = swap;

            }

            //RawList<int> indices = new RawList<int>(SampleTriangleIndices.Length);
            //RawList<Vector3> surfacePoints = new RawList<Vector3>();
            //ConvexHullHelper.GetConvexHull(vertices, indices, surfacePoints);
            //indices.CopyTo(SampleTriangleIndices, 0);


        }

        private static int GetExpectedVertexCount(int subdivisionCount)
        {
            return 2 + 5 * (1 << (2 * subdivisionCount + 1));
        }


        /// <summary>
        /// Computes the center, volume, and volume distribution of a shape represented by a mesh.
        /// </summary>
        /// <param name="vertices">Vertices of the mesh.</param>
        /// <param name="triangleIndices">Groups of 3 indices into the vertices array which represent the triangles of the mesh.</param>
        /// <param name="volume">Volume of the shape.</param>
        /// <param name="volumeDistribution">Distribution of the volume as measured from the computed center.</param>
        public static void ComputeShapeDistribution(IList<Vector3> vertices, IList<int> triangleIndices, out float volume, out Matrix3x3 volumeDistribution)
        {
            //TODO: Whole bunch of repeat code here. If you ever need to change this, refactor the two methods to share.
            //Explanation for the tetrahedral integration bits: Explicit Exact Formulas for the 3-D Tetrahedron Inertia Tensor in Terms of its Vertex Coordinates
            //http://www.scipub.org/fulltext/jms2/jms2118-11.pdf
            //x1, x2, x3, x4 are origin, triangle1, triangle2, triangle3
            //Looking to find inertia tensor matrix of the form
            // [  a  -b' -c' ]
            // [ -b'  b  -a' ]
            // [ -c' -a'  c  ]
            float a = 0, b = 0, c = 0, ao = 0, bo = 0, co = 0;

            float scaledVolume = 0;
            for (int i = 0; i < triangleIndices.Count; i += 3)
            {
                Vector3 v2 = vertices[triangleIndices[i]];
                Vector3 v3 = vertices[triangleIndices[i + 1]];
                Vector3 v4 = vertices[triangleIndices[i + 2]];

                //Determinant is 6 * volume.  It's signed, though; the mesh isn't necessarily convex and the origin isn't necessarily in the mesh even if it is convex.
                float scaledTetrahedronVolume = v2.X * (v3.Z * v4.Y - v3.Y * v4.Z) -
                                                v3.X * (v2.Z * v4.Y - v2.Y * v4.Z) +
                                                v4.X * (v2.Z * v3.Y - v2.Y * v3.Z);

                scaledVolume += scaledTetrahedronVolume;

                a += scaledTetrahedronVolume * (v2.Y * v2.Y + v2.Y * v3.Y + v3.Y * v3.Y + v2.Y * v4.Y + v3.Y * v4.Y + v4.Y * v4.Y +
                                                v2.Z * v2.Z + v2.Z * v3.Z + v3.Z * v3.Z + v2.Z * v4.Z + v3.Z * v4.Z + v4.Z * v4.Z);
                b += scaledTetrahedronVolume * (v2.X * v2.X + v2.X * v3.X + v3.X * v3.X + v2.X * v4.X + v3.X * v4.X + v4.X * v4.X +
                                                v2.Z * v2.Z + v2.Z * v3.Z + v3.Z * v3.Z + v2.Z * v4.Z + v3.Z * v4.Z + v4.Z * v4.Z);
                c += scaledTetrahedronVolume * (v2.X * v2.X + v2.X * v3.X + v3.X * v3.X + v2.X * v4.X + v3.X * v4.X + v4.X * v4.X +
                                                v2.Y * v2.Y + v2.Y * v3.Y + v3.Y * v3.Y + v2.Y * v4.Y + v3.Y * v4.Y + v4.Y * v4.Y);
                ao += scaledTetrahedronVolume * (2 * v2.Y * v2.Z + v3.Y * v2.Z + v4.Y * v2.Z + v2.Y * v3.Z + 2 * v3.Y * v3.Z + v4.Y * v3.Z + v2.Y * v4.Z + v3.Y * v4.Z + 2 * v4.Y * v4.Z);
                bo += scaledTetrahedronVolume * (2 * v2.X * v2.Z + v3.X * v2.Z + v4.X * v2.Z + v2.X * v3.Z + 2 * v3.X * v3.Z + v4.X * v3.Z + v2.X * v4.Z + v3.X * v4.Z + 2 * v4.X * v4.Z);
                co += scaledTetrahedronVolume * (2 * v2.X * v2.Y + v3.X * v2.Y + v4.X * v2.Y + v2.X * v3.Y + 2 * v3.X * v3.Y + v4.X * v3.Y + v2.X * v4.Y + v3.X * v4.Y + 2 * v4.X * v4.Y);
            }
            volume = scaledVolume / 6;
            float scaledDensity = 1 / volume;
            float diagonalFactor = scaledDensity / 60;
            float offFactor = -scaledDensity / 120;
            a *= diagonalFactor;
            b *= diagonalFactor;
            c *= diagonalFactor;
            ao *= offFactor;
            bo *= offFactor;
            co *= offFactor;
            volumeDistribution = new Matrix3x3(a, bo, co,
                                               bo, b, ao,
                                               co, ao, c);


        }

        /// <summary>
        /// Computes the center, volume, and volume distribution of a shape represented by a mesh.
        /// </summary>
        /// <param name="vertices">Vertices of the mesh.</param>
        /// <param name="triangleIndices">Groups of 3 indices into the vertices array which represent the triangles of the mesh.</param>
        /// <param name="center">Computed center of the shape's volume.</param>
        /// <param name="volume">Volume of the shape.</param>
        /// <param name="volumeDistribution">Distribution of the volume as measured from the computed center.</param>
        public static void ComputeShapeDistribution(IList<Vector3> vertices, IList<int> triangleIndices, out Vector3 center, out float volume, out Matrix3x3 volumeDistribution)
        {
            //Explanation for the tetrahedral integration bits: Explicit Exact Formulas for the 3-D Tetrahedron Inertia Tensor in Terms of its Vertex Coordinates
            //http://www.scipub.org/fulltext/jms2/jms2118-11.pdf
            //x1, x2, x3, x4 are origin, triangle1, triangle2, triangle3
            //Looking to find inertia tensor matrix of the form
            // [  a  -b' -c' ]
            // [ -b'  b  -a' ]
            // [ -c' -a'  c  ]
            float a = 0, b = 0, c = 0, ao = 0, bo = 0, co = 0;

            Vector3 summedCenter = new Vector3();
            float scaledVolume = 0;
            for (int i = 0; i < triangleIndices.Count; i += 3)
            {
                Vector3 v2 = vertices[triangleIndices[i]];
                Vector3 v3 = vertices[triangleIndices[i + 1]];
                Vector3 v4 = vertices[triangleIndices[i + 2]];

                //Determinant is 6 * volume.  It's signed, though; the mesh isn't necessarily convex and the origin isn't necessarily in the mesh even if it is convex.
                float scaledTetrahedronVolume = v2.X * (v3.Z * v4.Y - v3.Y * v4.Z) -
                                                v3.X * (v2.Z * v4.Y - v2.Y * v4.Z) +
                                                v4.X * (v2.Z * v3.Y - v2.Y * v3.Z);

                scaledVolume += scaledTetrahedronVolume;

                Vector3 tetrahedronCentroid;
                Vector3.Add(ref v2, ref v3, out tetrahedronCentroid);
                Vector3.Add(ref tetrahedronCentroid, ref v4, out tetrahedronCentroid);
                Vector3.Multiply(ref tetrahedronCentroid, scaledTetrahedronVolume, out tetrahedronCentroid);
                Vector3.Add(ref tetrahedronCentroid, ref summedCenter, out summedCenter);

                a += scaledTetrahedronVolume * (v2.Y * v2.Y + v2.Y * v3.Y + v3.Y * v3.Y + v2.Y * v4.Y + v3.Y * v4.Y + v4.Y * v4.Y +
                                                v2.Z * v2.Z + v2.Z * v3.Z + v3.Z * v3.Z + v2.Z * v4.Z + v3.Z * v4.Z + v4.Z * v4.Z);
                b += scaledTetrahedronVolume * (v2.X * v2.X + v2.X * v3.X + v3.X * v3.X + v2.X * v4.X + v3.X * v4.X + v4.X * v4.X +
                                                v2.Z * v2.Z + v2.Z * v3.Z + v3.Z * v3.Z + v2.Z * v4.Z + v3.Z * v4.Z + v4.Z * v4.Z);
                c += scaledTetrahedronVolume * (v2.X * v2.X + v2.X * v3.X + v3.X * v3.X + v2.X * v4.X + v3.X * v4.X + v4.X * v4.X +
                                                v2.Y * v2.Y + v2.Y * v3.Y + v3.Y * v3.Y + v2.Y * v4.Y + v3.Y * v4.Y + v4.Y * v4.Y);
                ao += scaledTetrahedronVolume * (2 * v2.Y * v2.Z + v3.Y * v2.Z + v4.Y * v2.Z + v2.Y * v3.Z + 2 * v3.Y * v3.Z + v4.Y * v3.Z + v2.Y * v4.Z + v3.Y * v4.Z + 2 * v4.Y * v4.Z);
                bo += scaledTetrahedronVolume * (2 * v2.X * v2.Z + v3.X * v2.Z + v4.X * v2.Z + v2.X * v3.Z + 2 * v3.X * v3.Z + v4.X * v3.Z + v2.X * v4.Z + v3.X * v4.Z + 2 * v4.X * v4.Z);
                co += scaledTetrahedronVolume * (2 * v2.X * v2.Y + v3.X * v2.Y + v4.X * v2.Y + v2.X * v3.Y + 2 * v3.X * v3.Y + v4.X * v3.Y + v2.X * v4.Y + v3.X * v4.Y + 2 * v4.X * v4.Y);
            }
            if (scaledVolume < Toolbox.Epsilon)
            {
                //This function works on the assumption that there is volume.
                //If there is no volume, then volume * density is 0, so the mass is considered to be zero.
                //If the mass is zero, then a zero matrix is the consistent result.
                //In other words, this function shouldn't be used with things with no volume.
                //A special case should be used instead.
                volumeDistribution = new Matrix3x3();
                volume = 0;
                center = new Vector3();
            }
            else
            {
                Vector3.Multiply(ref summedCenter, 0.25f / scaledVolume, out center);
                volume = scaledVolume / 6;
                float scaledDensity = 1 / volume;
                float diagonalFactor = scaledDensity / 60;
                float offFactor = -scaledDensity / 120;
                a *= diagonalFactor;
                b *= diagonalFactor;
                c *= diagonalFactor;
                ao *= offFactor;
                bo *= offFactor;
                co *= offFactor;
                //volumeDistribution = new Matrix3x3(a, bo, co,
                //                                   bo, b, ao,
                //                                   co, ao, c);
                volumeDistribution = new Matrix3x3(a, co, bo,
                                                   co, b, ao,
                                                   bo, ao, c);

                //The volume distribution, as computed, is currently offset from the origin.
                //There's a operation that moves a local inertia tensor to a displaced position.
                //The inverse of that operation can be computed and applied to the displaced inertia to center it on the origin.

                Matrix3x3 additionalInertia;
                GetPointContribution(1, ref Toolbox.ZeroVector, ref center, out additionalInertia);
                Matrix3x3.Subtract(ref volumeDistribution, ref additionalInertia, out volumeDistribution);

                //The derivation that shows the above point mass usage is valid goes something like this, with lots of details left out:
                //Consider the usual form of the tensor, created from the summation of a bunch of pointmasses representing the shape.
                //Each sum contribution relies on a particular offset, r. When the center of mass isn't aligned with (0,0,0), 
                //r = c + b, where c is the center of mass and b is the offset of r from the center of mass.
                //So, each term of the matrix (like M11 = sum(mi * (ry*ry + rz*rz))) can be rephrased in terms of the center and the offset:
                //M11 = sum(mi * ((cy + by) * (cy + by) + (cz + bz) * (cz + bz)))
                //Expanding that gets you to:
                //M11 = sum(mi * (cycy + 2cyby + byby + czcz + 2czbz + bzbz))
                //A couple of observations move things along.
                //1) Since it's constant over the summation, the c terms can be pulled out of a sum.
                //2) sum(mi * by) and sum(mi * bz) are zero, because 'by' and 'bz' are offsets from the center of mass. In other words, if you averaged all of the offsets, it would equal (0,0,0).
                //(uniform density assumed)
                //With a little more massaging, the constant c terms can be fully separated into an additive term on each matrix element.
            }

        }

        //public static void GetInertiaOffset(Vector3 offset, float mass, out Matrix3x3 additionalInertia)
        //{
        //    additionalInertia.M11 = mass * (offset.Y * offset.Y + offset.Z * offset.Z);
        //    additionalInertia.M12 = -mass * offset.X * offset.Y;
        //    additionalInertia.M13 = -mass * offset.X * offset.Z;

        //    additionalInertia.M21 = -mass * offset.Y * offset.X;
        //    additionalInertia.M22 = mass * (offset.X * offset.X + offset.Z * offset.Z);
        //    additionalInertia.M23 = -mass * offset.Y * offset.Z;

        //    additionalInertia.M31 = -mass * offset.Z * offset.X;
        //    additionalInertia.M32 = -mass * offset.Z * offset.Y;
        //    additionalInertia.M33 = mass * (offset.X * offset.X + offset.Y * offset.Y);

     
        //}

        
        /// <summary>
        /// Computes a minimum radius estimate of a shape based on a convex mesh representation.
        /// </summary>
        /// <param name="vertices">Vertices of the convex mesh.</param>
        /// <param name="triangleIndices">Groups of 3 indices into the vertices array which represent the triangles of the convex mesh.</param>
        /// <param name="center">Center of the convex shape.</param>
        public static float ComputeMinimumRadius(IList<Vector3> vertices, IList<int> triangleIndices, ref Vector3 center)
        {
            //Walk through all of the triangles. Treat them as a bunch of planes which bound the shape.
            //The closest distance on any of those planes to the center is the radius of the largest sphere,
            //centered on the... center, which can fit in the shape.

            //While this shares a lot of math with the volume distribution computation (volume of a parallelepiped),
            //it requires that a center be available. So, it's a separate calculation.
            float minimumDistance = float.MaxValue;
            for (int i = 0; i < triangleIndices.Count; i += 3)
            {
                Vector3 v2 = vertices[triangleIndices[i]];
                Vector3 v3 = vertices[triangleIndices[i + 1]];
                Vector3 v4 = vertices[triangleIndices[i + 2]];


                //This normal calculation creates a dependency on winding.
                //It needs to be consistent with the SampleDirections triangle winding.
                Vector3 v2v3, v2v4;
                Vector3.Subtract(ref v3, ref v2, out v2v3);
                Vector3.Subtract(ref v4, ref v2, out v2v4);
                Vector3 normal;
                Vector3.Cross(ref v2v4, ref v2v3, out normal);

                //Watch out: this could very easily be a degenerate triangle; the sampling approach tends to create them.
                float lengthSquared = normal.LengthSquared();
                if (lengthSquared > 1e-10f)
                    Vector3.Divide(ref normal, (float)Math.Sqrt(lengthSquared), out normal);
                else
                    continue;

                Vector3 fromCenterToPlane;
                Vector3.Subtract(ref v2, ref center, out fromCenterToPlane);

                float distance;
                Vector3.Dot(ref normal, ref fromCenterToPlane, out distance);
                if (distance < 0)
                    throw new ArgumentException("Invalid distance. Ensure the mesh is convex, has consistent winding, and contains the passed-in center.");

                if (distance < minimumDistance)
                    minimumDistance = distance;
            }
            return minimumDistance;

            //Technically, we could also compute a maximum radius estimate... 
            //but that amounts to finding the furthest distance contained by the set of planes defined by the sampled extreme points and their associated sample directions.
            //That's a trickier thing to compute quickly, and it's not all that important to make the estimate ultra tight.

        }

        //TODO: These will be replaced when a better resource pooling system is implemented.

        /// <summary>
        /// Treats an array like a list and adds an element to it.
        /// Does not perform any resizing.
        /// </summary>
        /// <typeparam name="T">Type of the array.</typeparam>
        /// <param name="array">Array to add an item to.</param>
        /// <param name="item">The item to add.</param>
        /// <param name="count">Length of the array. Will be incremented.</param>
        private static void Add<T>(this T[] array, T item, ref int count)
        {
            array[count++] = item;
        }

        /// <summary>
        /// Treats an array like a list and adds an element to it.
        /// Does not perform any resizing.
        /// </summary>
        /// <typeparam name="T">Type of the array.</typeparam>
        /// <param name="array">Array to add an item to.</param>
        /// <param name="item">The item to add.</param>
        /// <param name="count">Length of the array. Will be incremented.</param>
        private static void Add<T>(this T[] array, ref T item, ref int count)
        {
            array[count++] = item;
        }



    }
}
