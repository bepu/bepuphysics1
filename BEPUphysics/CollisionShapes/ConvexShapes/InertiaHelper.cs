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
        /// Computes the center of a convex shape.
        ///</summary>
        ///<param name="shape">Shape to compute the center of.</param>
        ///<returns>Center of the shape.</returns>
        public static Vector3 ComputeCenter(ConvexShape shape)
        {
            float volume;
            return ComputeCenter(shape, out volume);
        }

        ///<summary>
        /// Computes the center and volume of a convex shape.
        ///</summary>
        ///<param name="shape">Shape to compute the center of.</param>
        ///<param name="volume">Volume of the shape.</param>
        ///<returns>Center of the shape.</returns>
        public static Vector3 ComputeCenter(ConvexShape shape, out float volume)
        {
            var pointContributions = CommonResources.GetVectorList();
            GetPoints(shape, out volume, pointContributions);
            Vector3 center = AveragePoints(pointContributions);
            CommonResources.GiveBack(pointContributions);
            MathChecker.Validate(center);
            return center;
        }

        ///<summary>
        /// Averages together all the points in the point list.
        ///</summary>
        ///<param name="pointContributions">Point list to average.</param>
        ///<returns>Averaged point.</returns>
        public static Vector3 AveragePoints(RawList<Vector3> pointContributions)
        {
            var center = new Vector3();
            for (int i = 0; i < pointContributions.Count; i++)
            {
                center += pointContributions[i]; //Every point has equal weight.
            }
            return center / pointContributions.Count;
        }

        ///<summary>
        /// Computes the volume and volume distribution of a shape.
        ///</summary>
        ///<param name="shape">Shape to compute the volume information of.</param>
        ///<param name="volume">Volume of the shape.</param>
        ///<returns>Volume distribution of the shape.</returns>
        public static Matrix3x3 ComputeVolumeDistribution(ConvexShape shape, out float volume)
        {
            var pointContributions = CommonResources.GetVectorList();
            GetPoints(shape, out volume, pointContributions);
            Vector3 center = AveragePoints(pointContributions);
            Matrix3x3 volumeDistribution = ComputeVolumeDistribution(pointContributions, ref center);
            CommonResources.GiveBack(pointContributions);
            return volumeDistribution;
        }


        ///<summary>
        /// Computes the volume and volume distribution of a shape based on a given center.
        ///</summary>
        ///<param name="shape">Shape to compute the volume information of.</param>
        ///<param name="center">Location to use as the center of the shape when computing the volume distribution.</param>
        ///<param name="volume">Volume of the shape.</param>
        ///<returns>Volume distribution of the shape.</returns>
        public static Matrix3x3 ComputeVolumeDistribution(ConvexShape shape, ref Vector3 center, out float volume)
        {
            var pointContributions = CommonResources.GetVectorList();
            GetPoints(shape, out volume, pointContributions);
            Matrix3x3 volumeDistribution = ComputeVolumeDistribution(pointContributions, ref center);
            CommonResources.GiveBack(pointContributions);
            return volumeDistribution;
        }

        ///<summary>
        /// Computes a volume distribution based on a bunch of point contributions.
        ///</summary>
        ///<param name="pointContributions">Point contributions to the volume distribution.</param>
        ///<param name="center">Location to use as the center for purposes of computing point contributions.</param>
        ///<returns>Volume distribution of the point contributions.</returns>
        public static Matrix3x3 ComputeVolumeDistribution(RawList<Vector3> pointContributions, ref Vector3 center)
        {
            var volumeDistribution = new Matrix3x3();
            float pointWeight = 1f / pointContributions.Count;
            for (int i = 0; i < pointContributions.Count; i++)
            {
                Matrix3x3 contribution;
                GetPointContribution(pointWeight, ref center, pointContributions[i], out contribution);
                Matrix3x3.Add(ref volumeDistribution, ref contribution, out volumeDistribution);
            }
            return volumeDistribution;
        }



        ///<summary>
        /// Gets the point contributions within a convex shape.
        ///</summary>
        ///<param name="shape">Shape to compute the point contributions of.</param>
        ///<param name="volume">Volume of the shape.</param>
        ///<param name="outputPointContributions">Point contributions of the shape.</param>
        public static void GetPoints(ConvexShape shape, out float volume, RawList<Vector3> outputPointContributions)
        {
            RigidTransform transform = RigidTransform.Identity;
            BoundingBox boundingBox;
            shape.GetBoundingBox(ref transform, out boundingBox);

            //Find the direction which maximizes the possible hits.  Generally, this is the smallest area axis.
            //Possible options are:
            //YZ -> use X
            //XZ -> use Y
            //XY -> use Z
            Ray ray;
            float width = boundingBox.Max.X - boundingBox.Min.X;
            float height = boundingBox.Max.Y - boundingBox.Min.Y;
            float length = boundingBox.Max.Z - boundingBox.Min.Z;
            float yzArea = height * length;
            float xzArea = width * length;
            float xyArea = width * height;
            Vector3 increment1, increment2;
            float incrementMultiplier = 1f / NumberOfSamplesPerDimension;
            float maxLength;
            float rayIncrement;
            if (yzArea > xzArea && yzArea > xyArea)
            {
                //use the x axis as the direction.
                ray.Direction = Vector3.Right;
                ray.Position = new Vector3(boundingBox.Min.X, boundingBox.Min.Y + .5f * incrementMultiplier * height, boundingBox.Min.Z + .5f * incrementMultiplier * length);
                increment1 = new Vector3(0, incrementMultiplier * height, 0);
                increment2 = new Vector3(0, 0, incrementMultiplier * length);
                rayIncrement = incrementMultiplier * width;
                maxLength = width;
            }
            else if (xzArea > xyArea) //yz is not the max, given by the previous if.  Is xz or xy the max?
            {
                //use the y axis as the direction.
                ray.Direction = Vector3.Up;
                ray.Position = new Vector3(boundingBox.Min.X + .5f * incrementMultiplier * width, boundingBox.Min.Y, boundingBox.Min.Z + .5f * incrementMultiplier * length);
                increment1 = new Vector3(incrementMultiplier * width, 0, 0);
                increment2 = new Vector3(0, 0, incrementMultiplier * height);
                rayIncrement = incrementMultiplier * height;
                maxLength = height;
            }
            else
            {
                //use the z axis as the direction.
                ray.Direction = Vector3.Backward;
                ray.Position = new Vector3(boundingBox.Min.X + .5f * incrementMultiplier * width, boundingBox.Min.Y + .5f * incrementMultiplier * height, boundingBox.Min.Z);
                increment1 = new Vector3(incrementMultiplier * width, 0, 0);
                increment2 = new Vector3(0, incrementMultiplier * height, 0);
                rayIncrement = incrementMultiplier * length;
                maxLength = length;
            }


            Ray oppositeRay;
            volume = 0;
            for (int i = 0; i < NumberOfSamplesPerDimension; i++)
            {
                for (int j = 0; j < NumberOfSamplesPerDimension; j++)
                {
                    //Ray cast from one direction.  If it succeeds, try the other way.  This forms an interval in which inertia tensor contributions are contained.
                    RayHit hit;
                    if (shape.RayTest(ref ray, ref transform, maxLength, out hit))
                    {
                        Vector3.Multiply(ref ray.Direction, maxLength, out oppositeRay.Position);
                        Vector3.Add(ref oppositeRay.Position, ref ray.Position, out oppositeRay.Position);
                        Vector3.Negate(ref ray.Direction, out oppositeRay.Direction);
                        RayHit oppositeHit;
                        if (shape.RayTest(ref oppositeRay, ref transform, maxLength, out oppositeHit))
                        {
                            //It should always get here if one direction casts, but there may be numerical issues.
                            float scanVolume;
                            ScanObject(rayIncrement, maxLength, ref increment1, ref increment2, ref ray, ref hit, ref oppositeHit, outputPointContributions, out scanVolume);
                            volume += scanVolume;
                        }
                    }
                    Vector3.Add(ref ray.Position, ref increment2, out ray.Position);
                }
                Vector3.Add(ref ray.Position, ref increment1, out ray.Position);
                //Move the ray back to the starting position along the other axis.
                Vector3 subtract;
                Vector3.Multiply(ref increment2, NumberOfSamplesPerDimension, out subtract);
                Vector3.Subtract(ref ray.Position, ref subtract, out ray.Position);
            }


        }



        private static void ScanObject(float rayIncrement, float maxLength, ref Vector3 increment1, ref Vector3 increment2, ref Ray ray, ref RayHit startHit, ref RayHit endHit, RawList<Vector3> pointContributions, out float volume)
        {
            Vector3 cell;
            Vector3.Multiply(ref ray.Direction, rayIncrement, out cell);
            Vector3.Add(ref increment1, ref cell, out cell);
            Vector3.Add(ref increment2, ref cell, out cell);
            float perCellVolume = cell.X * cell.Y * cell.Z;

            volume = 0;

            for (int i = (int)(startHit.T / rayIncrement); i <= (int)((maxLength - endHit.T) / rayIncrement); i++)
            {
                Vector3 position;
                Vector3.Multiply(ref ray.Direction, (i + .5f) * rayIncrement, out position);
                Vector3.Add(ref position, ref ray.Position, out position);
                pointContributions.Add(position);
                volume += perCellVolume;
            }
        }



        ///<summary>
        /// Computes the volume contribution of a point.
        ///</summary>
        ///<param name="pointWeight">Weight of the point.</param>
        ///<param name="center">Location to use as the center for the purposes of computing the contribution.</param>
        ///<param name="p">Point to compute the contribution of.</param>
        ///<param name="contribution">Contribution of the point.</param>
        public static void GetPointContribution(float pointWeight, ref Vector3 center, Vector3 p, out Matrix3x3 contribution)
        {
            Vector3.Subtract(ref p, ref center, out p);
            float xx = pointWeight * p.X * p.X;
            float yy = pointWeight * p.Y * p.Y;
            float zz = pointWeight * p.Z * p.Z;
            contribution.M11 = yy + zz;
            contribution.M22 = xx + zz;
            contribution.M33 = xx + yy;
            contribution.M12 = -pointWeight * p.X * p.Y;
            contribution.M13 = -pointWeight * p.X * p.Z;
            contribution.M23 = -pointWeight * p.Y * p.Z;
            contribution.M21 = contribution.M12;
            contribution.M31 = contribution.M13;
            contribution.M32 = contribution.M23;
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
            float length = (float) Math.Sqrt(1 + goldenRatio * goldenRatio);
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
            //2 + 2^(2n + 1)
            return 2 + 5 * (1 << (2 * subdivisionCount + 1));// 2 + (1 << (2 * subdivisionCount + 1));
        }

        private static int GetHighestIndex(int[] indices)
        {
            int highest = 0;
            foreach (var index in indices)
            {
                if (index > highest)
                    highest = index;
            }
            return highest;
        }

        /// <summary>
        /// Set of directions sampled by the inertia helper when constructing a mesh representation of a convex object.
        /// </summary>
        public static Vector3[] SampleDirections;

        /// <summary>
        /// Set of triangles represented by groups of three indices into the SampleDirections array.
        /// </summary>
        public static int[] SampleTriangleIndices;

        /// <summary>
        /// Fills an array with the samples taken according to the SampleDirections.
        /// </summary>
        /// <param name="shape">Shape to compute samples for. Must be initialized enough to support extreme point queries.</param>
        /// <param name="samples">Array to fill with samples.</param>
        public static void ComputeSamples(ConvexShape shape, Vector3[] samples)
        {
            if (SampleDirections.Length > samples.Length)
                throw new ArgumentException("Samples array must be large enough to contain all samples.", "samples");

            for (int i = 0; i < SampleDirections.Length; ++i)
            {
                shape.GetLocalExtremePointWithoutMargin(ref SampleDirections[i], out samples[i]);
            }
        }

        /// <summary>
        /// Computes the center, volume, and volume distribution of a shape represented by a mesh.
        /// </summary>
        /// <param name="vertices">Vertices of the mesh.</param>
        /// <param name="triangleIndices">Groups of 3 indices into the vertices array which represent the triangles of the mesh.</param>
        /// <param name="center">Computed center of the shape's volume.</param>
        /// <param name="volume">Volume of the shape.</param>
        /// <param name="volumeDistribution">Distribution of the volume as measured from the computed center.</param>
        public static void ComputeShapeDistribution(Vector3[] vertices, int[] triangleIndices, out Vector3 center, out float volume, out Matrix3x3 volumeDistribution)
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
            for (int i = 0; i < triangleIndices.Length; i += 3)
            {
                Vector3 v2 = vertices[triangleIndices[i]];
                Vector3 v3 = vertices[triangleIndices[i + 1]];
                Vector3 v4 = vertices[triangleIndices[i + 2]];

                //Determinant is 6 * volume.  It's signed, though; the mesh isn't necessarily convex and the origin isn't necessarily in the mesh even if it is convex.
                float scaledTetrahedronVolume = v2.X * (v3.Y * v4.Z - v3.Z * v4.Y) -
                                                v3.X * (v2.Y * v4.Z - v2.Z * v4.Y) +
                                                v4.X * (v2.Y * v3.Z - v2.Z * v3.Y);

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
            volumeDistribution = new Matrix3x3(a, bo, co,
                                               bo, b, ao,
                                               co, ao, c);

            //The volume distribution, as computed, is currently offset from the origin.
            //There's a operation that moves a local inertia tensor to a displaced position.
            //The inverse of that operation can be computed and applied to the displaced inertia to center it on the origin.
            //Fortunately, it's pretty simple.
            volumeDistribution.M11 -= center.Y * center.Y + center.Z * center.Z;
            volumeDistribution.M12 += center.X * center.Y;
            volumeDistribution.M13 += center.X * center.Z;

            volumeDistribution.M21 += center.Y * center.X;
            volumeDistribution.M22 -= center.X * center.X + center.Z * center.Z;
            volumeDistribution.M23 += center.Y * center.Z;

            volumeDistribution.M31 += center.Z * center.X;
            volumeDistribution.M32 += center.Z * center.Y;
            volumeDistribution.M33 -= center.X * center.X + center.Y * center.Y;

            //The derivation for the above goes something like this, with lots of details left out:
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
            //(The volume distribution assumes mass is 1, hence the lack of any mass term in the implementation above.)

        }


        //public static void ComputeShapeDistribution(ConvexShape shape, Vector3 center, out float volume, out Matrix3x3 volumeDistribution)
        //{
        //}

        /// <summary>
        /// Computes a minimum radius estimate of a shape based on a convex mesh representation.
        /// </summary>
        /// <param name="vertices">Vertices of the convex mesh.</param>
        /// <param name="triangleIndices">Groups of 3 indices into the vertices array which represent the triangles of the convex mesh.</param>
        /// <param name="center">Center of the convex shape.</param>
        public static float ComputeMinimumRadius(Vector3[] vertices, int[] triangleIndices, ref Vector3 center)
        {
            //Walk through all of the triangles. Treat them as a bunch of planes which bound the shape.
            //The closest distance on any of those planes to the center is the radius of the largest sphere,
            //centered on the... center, which can fit in the shape.

            //While this shares a lot of math with the volume distribution computation (volume of a parallelepiped),
            //it requires that a center be available. So, it's a separate calculation.
            float minimumDistance = float.MaxValue;
            for (int i = 0; i < triangleIndices.Length; i += 3)
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
                Vector3.Cross(ref v2v3, ref v2v4, out normal);

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
                    throw new ArgumentException("Invalid distance: mesh may not be concave, winding may not be consistent, or the center may be outside the mesh.");

                if (distance < minimumDistance)
                    minimumDistance = distance;
            }
            return minimumDistance;

            //Technically, we could also compute a maximum radius estimate... 
            //but that amounts to finding the furthest distance contained by the set of planes defined by the sampled extreme points and their associated sample directions.
            //That's a trickier thing to compute quickly, and it's not all that important to make the estimate ultra tight.

        }


        /// <summary>
        /// Treats an array like a list and adds an element to it.
        /// Does not perform any resizing.
        /// </summary>
        /// <typeparam name="T">Type of the array.</typeparam>
        /// <param name="array">Array to add an item to.</param>
        /// <param name="item">The item to add.</param>
        /// <param name="count">Length of the array. Will be incremented.</param>
        public static void Add<T>(this T[] array, T item, ref int count)
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
        public static void Add<T>(this T[] array, ref T item, ref int count)
        {
            array[count++] = item;
        }

        /// <summary>
        /// Treats an array like a list and removes an element from it.
        /// Puts the last element of the array at the removed index.
        /// Does not attempt to erase values.
        /// </summary>
        /// <typeparam name="T">Type of the array</typeparam>
        /// <param name="array">Array to remove from.</param>
        /// <param name="index">Index to remove at.</param>
        /// <param name="count">Length of the list.</param>
        public static void RemoveValueAt<T>(this T[] array, int index, ref int count)
        {
            --count;
            if (index < count - 1)
            {
                array[index] = array[count];
            }
        }

        /// <summary>
        /// Treats an array like a list and removes an element from it.
        /// Puts the last element of the array at the removed index.
        /// Sets emptied indices to default values.
        /// </summary>
        /// <typeparam name="T">Type of the array</typeparam>
        /// <param name="array">Array to remove from.</param>
        /// <param name="index">Index to remove at.</param>
        /// <param name="count">Length of the list.</param>
        public static void RemoveAt<T>(this T[] array, int index, ref int count)
        {
            --count;
            if (index < count - 1)
            {
                array[index] = array[count];
                array[count] = default(T);
            }
        }


    }
}
