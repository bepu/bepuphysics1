using System;
using System.Collections.Generic;
using System.Linq;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;

using BEPUutilities;
using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;

namespace BEPUphysics.CollisionShapes.ConvexShapes
{
    ///<summary>
    /// Convex wrapping around a point set.
    ///</summary>
    public class ConvexHullShape : ConvexShape
    {
        ///<summary>
        /// Gets the point set of the convex hull.
        ///</summary>
        public ReadOnlyList<Vector3> Vertices
        {
            get
            {
                return new ReadOnlyList<Vector3>(vertices);
            }
        }
        Vector3[] vertices;

        private readonly float unexpandedMinimumRadius;
        private readonly float unexpandedMaximumRadius;

        ///<summary>
        /// Constructs a new convex hull shape.
        /// The point set will be recentered on the local origin.
        /// If that offset is needed, use the other constructor which outputs the computed center.
        ///</summary>
        ///<param name="vertices">Point set to use to construct the convex hull.</param>
        ///<exception cref="ArgumentException">Thrown when the point set is empty.</exception>
        public ConvexHullShape(IList<Vector3> vertices)
        {
            if (vertices.Count == 0)
                throw new ArgumentException("Vertices list used to create a ConvexHullShape cannot be empty.");

            var surfaceVertices = CommonResources.GetVectorList();
            var hullTriangleIndices = CommonResources.GetIntList();

            Vector3 center;
            UpdateConvexShapeInfo(ComputeDescription(vertices, collisionMargin, out center, hullTriangleIndices, surfaceVertices));
            this.vertices = surfaceVertices.ToArray();


            CommonResources.GiveBack(hullTriangleIndices);
            CommonResources.GiveBack(surfaceVertices);

            unexpandedMaximumRadius = MaximumRadius - collisionMargin;
            unexpandedMinimumRadius = MinimumRadius - collisionMargin;
        }

        ///<summary>
        /// Constructs a new convex hull shape.
        /// The point set will be recentered on the local origin.
        ///</summary>
        ///<param name="vertices">Point set to use to construct the convex hull.</param>
        /// <param name="center">Computed center of the convex hull shape prior to recentering.</param>
        ///<exception cref="ArgumentException">Thrown when the point set is empty.</exception>
        public ConvexHullShape(IList<Vector3> vertices, out Vector3 center)
        {
            if (vertices.Count == 0)
                throw new ArgumentException("Vertices list used to create a ConvexHullShape cannot be empty.");

            var surfaceVertices = CommonResources.GetVectorList();
            var hullTriangleIndices = CommonResources.GetIntList();

            UpdateConvexShapeInfo(ComputeDescription(vertices, collisionMargin, out center, hullTriangleIndices, surfaceVertices));
            this.vertices = surfaceVertices.ToArray();

            CommonResources.GiveBack(hullTriangleIndices);
            CommonResources.GiveBack(surfaceVertices);

            unexpandedMaximumRadius = MaximumRadius - collisionMargin;
            unexpandedMinimumRadius = MinimumRadius - collisionMargin;

        }

        ///<summary>
        /// Constructs a new convex hull shape.
        /// The point set will be recentered on the local origin.
        ///</summary>
        ///<param name="vertices">Point set to use to construct the convex hull.</param>
        /// <param name="center">Computed center of the convex hull shape prior to recentering.</param>
        /// <param name="outputHullTriangleIndices">Triangle indices computed on the surface of the point set.</param>
        /// <param name="outputUniqueSurfaceVertices">Unique vertices on the surface of the convex hull.</param>
        ///<exception cref="ArgumentException">Thrown when the point set is empty.</exception>
        public ConvexHullShape(IList<Vector3> vertices, out Vector3 center, IList<int> outputHullTriangleIndices, IList<Vector3> outputUniqueSurfaceVertices)
        {
            if (vertices.Count == 0)
                throw new ArgumentException("Vertices list used to create a ConvexHullShape cannot be empty.");

            UpdateConvexShapeInfo(ComputeDescription(vertices, collisionMargin, out center, outputHullTriangleIndices, outputUniqueSurfaceVertices));
            this.vertices = new Vector3[outputUniqueSurfaceVertices.Count];
            outputUniqueSurfaceVertices.CopyTo(this.vertices, 0);

            unexpandedMaximumRadius = MaximumRadius - collisionMargin;
            unexpandedMinimumRadius = MinimumRadius - collisionMargin;

        }


        /// <summary>
        /// Creates a ConvexHullShape from cached information. Assumes all data provided is accurate- no pre-processing is performed.
        /// </summary>
        /// <param name="localSurfaceVertices">List of vertex positions on the surface of the convex hull shape, centered on the desired origin. These vertices are used as-is for the shape representation; no additional processing occurs.</param>
        /// <param name="description">Cached information about the shape. Assumed to be correct; no extra processing or validation is performed.</param>
        public ConvexHullShape(IList<Vector3> localSurfaceVertices, ConvexShapeDescription description)
        {
            if (localSurfaceVertices.Count == 0)
                throw new ArgumentException("Vertices list used to create a ConvexHullShape cannot be empty.");

            unexpandedMaximumRadius = description.MaximumRadius - collisionMargin;
            unexpandedMinimumRadius = description.MinimumRadius - collisionMargin;
            vertices = new Vector3[localSurfaceVertices.Count];
            localSurfaceVertices.CopyTo(vertices, 0);
            UpdateConvexShapeInfo(description);

        }

        protected override void OnShapeChanged()
        {
            //The convex hull shape's vertices are immutable.
            //The only way for this to occur is if the collision margin changed.
            //In that case, we only need to update the radius.

            //The (immutable) unexpanded radii are cached, so all that needs to be done is to add the new margin.
            UpdateConvexShapeInfo(new ConvexShapeDescription
            {
                EntityShapeVolume = new EntityShapeVolumeDescription { Volume = Volume, VolumeDistribution = VolumeDistribution },
                MinimumRadius = unexpandedMinimumRadius + collisionMargin,
                MaximumRadius = unexpandedMaximumRadius + collisionMargin,
                CollisionMargin = collisionMargin
            });
            base.OnShapeChanged();
        }

        /// <summary>
        /// Computes a convex shape description for a ConvexHullShape.
        /// </summary>
        /// <param name="vertices">Vertices describing the convex hull shape.</param>
        /// <param name="collisionMargin">Collision margin of the shape.</param>
        /// <param name="center">Computed center of the convex hull shape. Used as the origin of the outputUniqueSurfaceVertices.</param>
        /// <param name="outputHullTriangleIndices">Computed list of indices into the input point set composing the triangulated surface of the convex hull.
        /// Each group of 3 indices represents a triangle on the surface of the hull.</param>
        /// <param name="outputUniqueSurfaceVertices">Computed nonredundant list of vertices composing the outer shell of the input point set.</param>
        /// <returns>Description required to define a convex shape.</returns>
        public static ConvexShapeDescription ComputeDescription(IList<Vector3> vertices, float collisionMargin, out Vector3 center, IList<int> outputHullTriangleIndices, IList<Vector3> outputUniqueSurfaceVertices)
        {
            if (outputHullTriangleIndices.Count != 0 || outputUniqueSurfaceVertices.Count != 0)
                throw new ArgumentException("Output lists must start empty.");

            ConvexHullHelper.GetConvexHull(vertices, outputHullTriangleIndices, outputUniqueSurfaceVertices);

            //Grab the center and volume.
            ConvexShapeDescription description;
            ComputeCenterAndVolume(vertices, outputHullTriangleIndices, out center, out description.EntityShapeVolume.Volume);

            //Recenter the surface vertices. This puts the vertices into the shape's local space.
            for (int i = 0; i < outputUniqueSurfaceVertices.Count; ++i)
            {
                outputUniqueSurfaceVertices[i] -= center;
            }

            description.EntityShapeVolume.VolumeDistribution = ComputeVolumeDistribution(vertices, outputHullTriangleIndices, center, description.EntityShapeVolume.Volume, collisionMargin, out description.MinimumRadius);

            description.MaximumRadius = ComputeMaximumRadius(outputUniqueSurfaceVertices, collisionMargin);

            description.CollisionMargin = collisionMargin;
            return description;
        }


        /// <summary>
        /// Computes the center and volume of a convex hull.
        /// </summary>
        /// <param name="vertices">Vertices used as input to the convex hull.</param>
        /// <param name="hullTriangleIndices">List of indices into the input point set composing the triangulated surface of the convex hull.
        /// Each group of 3 indices represents a triangle on the surface of the hull.</param>
        /// <param name="center">Center of the hull.</param>
        /// <param name="volume">Volume of the hull.</param>
        public static void ComputeCenterAndVolume(IList<Vector3> vertices, IList<int> hullTriangleIndices, out Vector3 center, out float volume)
        {
            volume = 0;
            var volumes = CommonResources.GetFloatList();
            var centroids = CommonResources.GetVectorList();
            for (int k = 0; k < hullTriangleIndices.Count; k += 3)
            {
                //Compute the signed volume of each tetrahedron.
                //The fourth vertex is chosen to be (0,0,0).
                //Since that's potentially outside of the hull, some of the volumes will be negative.
                //That's perfectly fine! Some weights should be negative. Once everything is summed together, it comes out correct.
                float subvolume = Vector3.Dot(
                    Vector3.Cross(vertices[hullTriangleIndices[k + 1]] - vertices[hullTriangleIndices[k]],
                                  vertices[hullTriangleIndices[k + 2]] - vertices[hullTriangleIndices[k]]),
                    -vertices[hullTriangleIndices[k]]);
                volumes.Add(subvolume);
                volume += subvolume;
                centroids.Add((vertices[hullTriangleIndices[k]] + vertices[hullTriangleIndices[k + 1]] + vertices[hullTriangleIndices[k + 2]]) / 4);
            }
            //Perform a weighted sum of the centroids.
            center = Toolbox.ZeroVector;
            for (int k = 0; k < centroids.Count; k++)
            {
                center += centroids[k] * (volumes[k] / volume);
            }
            //The computed volume was for a parallelepiped. Apply a scaling factor to get down to the tetrahedron's volume.
            //[The scale factor cancels out in the above computation; correctness is unaffected by deferring the division.]
            volume /= 6;
            CommonResources.GiveBack(centroids);
            CommonResources.GiveBack(volumes);
        }

        /// <summary>
        /// Computes a volume distribution for the given convex hull data.
        /// </summary>
        /// <param name="vertices">Vertices used as input to the convex hull.</param>
        /// <param name="hullTriangleIndices">List of indices into the input point set composing the triangulated surface of the convex hull.
        /// Each group of 3 indices represents a triangle on the surface of the hull.</param>
        /// <param name="center">Center of the hull.</param>
        /// <param name="volume">Volume of the hull.</param>
        /// <param name="collisionMargin">Collision margin of the shape.</param>
        /// <param name="minimumRadius">Computed minimum radius of the shape.</param>
        /// <returns>Volume distribution of the convex hull.</returns>
        public static Matrix3x3 ComputeVolumeDistribution(IList<Vector3> vertices, IList<int> hullTriangleIndices, Vector3 center, float volume, float collisionMargin, out float minimumRadius)
        {
            //Source: Explicit Exact Formulas for the 3-D Tetrahedron Inertia Tensor in Terms of its Vertex Coordinates
            //http://www.scipub.org/fulltext/jms2/jms2118-11.pdf
            //x1, x2, x3, x4 are origin, triangle1, triangle2, triangle3
            //Looking to find inertia tensor matrix of the form
            // [  a  -b' -c' ]
            // [ -b'  b  -a' ]
            // [ -c' -a'  c  ]
            float a = 0, b = 0, c = 0, ao = 0, bo = 0, co = 0;
            Vector3 v2, v3, v4;
            float density = 1 / volume;
            float diagonalFactor = density / 60;
            float offFactor = -density / 120;
            minimumRadius = float.MaxValue;
            for (int i = 0; i < hullTriangleIndices.Count; i += 3)
            {
                v2 = vertices[hullTriangleIndices[i]];
                v3 = vertices[hullTriangleIndices[i + 1]];
                v4 = vertices[hullTriangleIndices[i + 2]];
                Vector3.Subtract(ref v2, ref center, out v2);
                Vector3.Subtract(ref v3, ref center, out v3);
                Vector3.Subtract(ref v4, ref center, out v4);

                //The determinant of the jacobian referred to by the paper above can be thought of as the volume of a parallelepiped created from the tetrahedron's edges.
                //The volume of a tetrahedron is 1/6 that of a parallelepiped sharing the same edges.
                //That scaling factor's compensation is included in the final multipliers.

                //More importantly, we can share work: the minimum radius requires knowing the shortest distance from the center to each plane.
                //Since everything has been shifted to the local origin, the shortest distance is dot(N, v2) / ||N||, where N is cross(v2v3, v2v4).
                //dot(cross(v2v3, v2v4), a) is the volume of the parallelepiped, and is the determinant!
                Vector3 v2v3, v2v4;
                Vector3.Subtract(ref v3, ref v2, out v2v3);
                Vector3.Subtract(ref v4, ref v2, out v2v4);
                Vector3 n;
                Vector3.Cross(ref v2v4, ref v2v3, out n);
                float determinant;
                Vector3.Dot(ref v2, ref n, out determinant);
                float distanceFromOriginToTriangle = determinant / n.Length();
                if (distanceFromOriginToTriangle < minimumRadius)
                    minimumRadius = distanceFromOriginToTriangle;

                a += determinant * (v2.Y * v2.Y + v2.Y * v3.Y + v3.Y * v3.Y + v2.Y * v4.Y + v3.Y * v4.Y + v4.Y * v4.Y +
                                    v2.Z * v2.Z + v2.Z * v3.Z + v3.Z * v3.Z + v2.Z * v4.Z + v3.Z * v4.Z + v4.Z * v4.Z);
                b += determinant * (v2.X * v2.X + v2.X * v3.X + v3.X * v3.X + v2.X * v4.X + v3.X * v4.X + v4.X * v4.X +
                                    v2.Z * v2.Z + v2.Z * v3.Z + v3.Z * v3.Z + v2.Z * v4.Z + v3.Z * v4.Z + v4.Z * v4.Z);
                c += determinant * (v2.X * v2.X + v2.X * v3.X + v3.X * v3.X + v2.X * v4.X + v3.X * v4.X + v4.X * v4.X +
                                    v2.Y * v2.Y + v2.Y * v3.Y + v3.Y * v3.Y + v2.Y * v4.Y + v3.Y * v4.Y + v4.Y * v4.Y);
                ao += determinant * (2 * v2.Y * v2.Z + v3.Y * v2.Z + v4.Y * v2.Z + v2.Y * v3.Z + 2 * v3.Y * v3.Z + v4.Y * v3.Z + v2.Y * v4.Z + v3.Y * v4.Z + 2 * v4.Y * v4.Z);
                bo += determinant * (2 * v2.X * v2.Z + v3.X * v2.Z + v4.X * v2.Z + v2.X * v3.Z + 2 * v3.X * v3.Z + v4.X * v3.Z + v2.X * v4.Z + v3.X * v4.Z + 2 * v4.X * v4.Z);
                co += determinant * (2 * v2.X * v2.Y + v3.X * v2.Y + v4.X * v2.Y + v2.X * v3.Y + 2 * v3.X * v3.Y + v4.X * v3.Y + v2.X * v4.Y + v3.X * v4.Y + 2 * v4.X * v4.Y);

            }
            minimumRadius += collisionMargin;

            a *= diagonalFactor;
            b *= diagonalFactor;
            c *= diagonalFactor;
            ao *= offFactor;
            bo *= offFactor;
            co *= offFactor;
            var distribution = new Matrix3x3(a, bo, co,
                                             bo, b, ao,
                                             co, ao, c);

            return distribution;
        }

        /// <summary>
        /// Computes the minimum radius for the given convex hull data.
        /// </summary>
        /// <param name="localSurfaceVertices">Surface vertices of the convex hull.</param>
        /// <param name="collisionMargin">Collision margin of the shape.</param>
        /// <returns>Maximum radius of the convex hull.</returns>
        public static float ComputeMaximumRadius(IList<Vector3> localSurfaceVertices, float collisionMargin)
        {
            float longestLengthSquared = 0;
            for (int i = 0; i < localSurfaceVertices.Count; ++i)
            {
                float lengthCandidate = localSurfaceVertices[i].LengthSquared();
                if (lengthCandidate > longestLengthSquared)
                {
                    longestLengthSquared = lengthCandidate;
                }
            }
            return (float)Math.Sqrt(longestLengthSquared) + collisionMargin;
        }




        /// <summary>
        /// Gets the bounding box of the shape given a transform.
        /// </summary>
        /// <param name="shapeTransform">Transform to use.</param>
        /// <param name="boundingBox">Bounding box of the transformed shape.</param>
        public override void GetBoundingBox(ref RigidTransform shapeTransform, out BoundingBox boundingBox)
        {
#if !WINDOWS
            boundingBox = new BoundingBox();
#endif

            Matrix3x3 o;
            Matrix3x3.CreateFromQuaternion(ref shapeTransform.Orientation, out o);

            float minX, maxX;
            float minY, maxY;
            float minZ, maxZ;
            var right = new Vector3(o.M11, o.M21, o.M31);
            var up = new Vector3(o.M12, o.M22, o.M32);
            var backward = new Vector3(o.M13, o.M23, o.M33);
            Vector3.Dot(ref vertices[0], ref right, out maxX);
            minX = maxX;
            Vector3.Dot(ref vertices[0], ref up, out maxY);
            minY = maxY;
            Vector3.Dot(ref vertices[0], ref backward, out maxZ);
            minZ = maxZ;
            int minXIndex = 0;
            int maxXIndex = 0;
            int minYIndex = 0;
            int maxYIndex = 0;
            int minZIndex = 0;
            int maxZIndex = 0;
            for (int i = 1; i < vertices.Length; ++i)
            {
                float dot;
                Vector3.Dot(ref vertices[i], ref right, out dot);
                if (dot < minX)
                {
                    minX = dot;
                    minXIndex = i;
                }
                else if (dot > maxX)
                {
                    maxX = dot;
                    maxXIndex = i;
                }

                Vector3.Dot(ref vertices[i], ref up, out dot);
                if (dot < minY)
                {
                    minY = dot;
                    minYIndex = i;
                }
                else if (dot > maxY)
                {
                    maxY = dot;
                    maxYIndex = i;
                }

                Vector3.Dot(ref vertices[i], ref backward, out dot);
                if (dot < minZ)
                {
                    minZ = dot;
                    minZIndex = i;
                }
                else if (dot > maxZ)
                {
                    maxZ = dot;
                    maxZIndex = i;
                }
            }

            //Rather than transforming each axis independently (and doing three times as many operations as required), just get the 6 required values directly.
            Vector3 positive, negative;
            TransformLocalExtremePoints(ref vertices[maxXIndex], ref vertices[maxYIndex], ref vertices[maxZIndex], ref o, out positive);
            TransformLocalExtremePoints(ref vertices[minXIndex], ref vertices[minYIndex], ref vertices[minZIndex], ref o, out negative);

            //The positive and negative vectors represent the X, Y and Z coordinates of the extreme points in world space along the world space axes.
            boundingBox.Max.X = shapeTransform.Position.X + positive.X + collisionMargin;
            boundingBox.Max.Y = shapeTransform.Position.Y + positive.Y + collisionMargin;
            boundingBox.Max.Z = shapeTransform.Position.Z + positive.Z + collisionMargin;

            boundingBox.Min.X = shapeTransform.Position.X + negative.X - collisionMargin;
            boundingBox.Min.Y = shapeTransform.Position.Y + negative.Y - collisionMargin;
            boundingBox.Min.Z = shapeTransform.Position.Z + negative.Z - collisionMargin;
        }


        public override void GetLocalExtremePointWithoutMargin(ref Vector3 direction, out Vector3 extremePoint)
        {
            float max;
            Vector3.Dot(ref vertices[0], ref direction, out max);
            int maxIndex = 0;
            for (int i = 1; i < vertices.Length; i++)
            {
                float dot;
                Vector3.Dot(ref vertices[i], ref direction, out dot);
                if (dot > max)
                {
                    max = dot;
                    maxIndex = i;
                }
            }
            extremePoint = vertices[maxIndex];
        }



        /// <summary>
        /// Retrieves an instance of an EntityCollidable that uses this EntityShape.  Mainly used by compound bodies.
        /// </summary>
        /// <returns>EntityCollidable that uses this shape.</returns>
        public override EntityCollidable GetCollidableInstance()
        {
            return new ConvexCollidable<ConvexHullShape>(this);
        }


    }
}
