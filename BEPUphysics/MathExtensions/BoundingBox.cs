using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysics.MathExtensions
{
    /// <summary>
    /// Provides XNA-like axis-aligned bounding box functionality needed by the engine.
    /// </summary>
    public struct BoundingBox
    {
        /// <summary>
        /// Location with the lowest X, Y, and Z coordinates in the axis-aligned bounding box.
        /// </summary>
        public Vector3 Min;

        /// <summary>
        /// Location with the highest X, Y, and Z coordinates in the axis-aligned bounding box.
        /// </summary>
        public Vector3 Max;

        /// <summary>
        /// Constructs a bounding box from the specified minimum and maximum.
        /// </summary>
        /// <param name="min">Location with the lowest X, Y, and Z coordinates contained by the axis-aligned bounding box.</param>
        /// <param name="max">Location with the highest X, Y, and Z coordinates contained by the axis-aligned bounding box.</param>
        public BoundingBox(Vector3 min, Vector3 max)
        {
            this.Min = min;
            this.Max = max;
        }

        /// <summary>
        /// Gets an array of locations corresponding to the 8 corners of the bounding box.
        /// </summary>
        /// <returns>Corners of the bounding box.</returns>
        public Vector3[] GetCorners()
        {
            var toReturn = new Vector3[8];
            toReturn[0] = new Vector3(Min.X, Max.Y, Max.Z);
            toReturn[1] = Max;
            toReturn[2] = new Vector3(Max.X, Min.Y, Max.Z);
            toReturn[3] = new Vector3(Min.X, Min.Y, Max.Z);
            toReturn[4] = new Vector3(Min.X, Max.Y, Min.Z);
            toReturn[5] = new Vector3(Max.X, Max.Y, Min.Z);
            toReturn[6] = new Vector3(Max.X, Min.Y, Min.Z);
            toReturn[7] = Min;
            return toReturn;
        }


        /// <summary>
        /// Determines if a bounding box intersects another bounding box.
        /// </summary>
        /// <param name="boundingBox">Bounding box to test against.</param>
        /// <returns>Whether the bounding boxes intersected.</returns>
        public bool Intersects(BoundingBox boundingBox)
        {
            if (boundingBox.Min.X > Max.X || boundingBox.Min.Y > Max.Y || boundingBox.Min.Z > Max.Z)
                return false;
            if (Min.X > boundingBox.Max.X || Min.Y > boundingBox.Max.Y || Min.Z > boundingBox.Max.Z)
                return false;
            return true;

        }

        /// <summary>
        /// Determines if a bounding box intersects another bounding box.
        /// </summary>
        /// <param name="boundingBox">Bounding box to test against.</param>
        /// <param name="intersects">Whether the bounding boxes intersect.</param>
        public void Intersects(ref BoundingBox boundingBox, out bool intersects)
        {
            if (boundingBox.Min.X > Max.X || boundingBox.Min.Y > Max.Y || boundingBox.Min.Z > Max.Z)
            {
                intersects = false;
                return;
            }
            if (Min.X > boundingBox.Max.X || Min.Y > boundingBox.Max.Y || Min.Z > boundingBox.Max.Z)
            {
                intersects = false;
                return;
            }
            intersects = true;
        }

        /// <summary>
        /// Determines if a bounding box intersects a bounding sphere.
        /// </summary>
        /// <param name="boundingSphere">Sphere to test for intersection.</param>
        /// <param name="intersects">Whether the bounding shapes intersect.</param>
        public void Intersects(ref BoundingSphere boundingSphere, out bool intersects)
        {
            Vector3 clampedLocation;
            if (boundingSphere.Center.X > Max.X)
                clampedLocation.X = Max.X;
            else if (boundingSphere.Center.X < Min.X)
                clampedLocation.X = Min.X;
            else
                clampedLocation.X = boundingSphere.Center.X;

            if (boundingSphere.Center.Y > Max.Y)
                clampedLocation.Y = Max.Y;
            else if (boundingSphere.Center.Y < Min.Y)
                clampedLocation.Y = Min.Y;
            else
                clampedLocation.Y = boundingSphere.Center.Y;

            if (boundingSphere.Center.Z > Max.Z)
                clampedLocation.Z = Max.Z;
            else if (boundingSphere.Center.Z < Min.Z)
                clampedLocation.Z = Min.Z;
            else
                clampedLocation.Z = boundingSphere.Center.Z;

            float distanceSquared;
            Vector3.DistanceSquared(ref clampedLocation, ref boundingSphere.Center, out distanceSquared);
            intersects = distanceSquared <= boundingSphere.Radius * boundingSphere.Radius;

        }

        //public bool Intersects(BoundingFrustum frustum)
        //{
        //    bool intersects;
        //    frustum.Intersects(ref this, out intersects);
        //    return intersects;
        //}

        /// <summary>
        /// Creates the smallest possible bounding box that contains a list of points.
        /// </summary>
        /// <param name="points">Points to enclose with a bounding box.</param>
        /// <returns>Bounding box which contains the list of points.</returns>
        public static BoundingBox CreateFromPoints(IList<Vector3> points)
        {
            BoundingBox aabb;
            if (points.Count == 0)
                throw new Exception("Cannot construct a bounding box from an empty list.");
            aabb.Min = points[0];
            aabb.Max = aabb.Min;
            for (int i = points.Count - 1; i >= 1; i--)
            {
                Vector3 v = points[i];
                if (v.X < aabb.Min.X)
                    aabb.Min.X = v.X;
                else if (v.X > aabb.Max.X)
                    aabb.Max.X = v.X;

                if (v.Y < aabb.Min.Y)
                    aabb.Min.Y = v.Y;
                else if (v.Y > aabb.Max.Y)
                    aabb.Max.Y = v.Y;

                if (v.Z < aabb.Min.Z)
                    aabb.Min.Z = v.Z;
                else if (v.Z > aabb.Max.Z)
                    aabb.Max.Z = v.Z;
            }
            return aabb;
        }



        /// <summary>
        /// Creates the smallest bounding box which contains two other bounding boxes.
        /// </summary>
        /// <param name="a">First bounding box to be contained.</param>
        /// <param name="b">Second bounding box to be contained.</param>
        /// <param name="merged">Smallest bounding box which contains the two input bounding boxes.</param>
        public static void CreateMerged(ref BoundingBox a, ref BoundingBox b, out BoundingBox merged)
        {
            if (a.Min.X < b.Min.X)
                merged.Min.X = a.Min.X;
            else
                merged.Min.X = b.Min.X;
            if (a.Min.Y < b.Min.Y)
                merged.Min.Y = a.Min.Y;
            else
                merged.Min.Y = b.Min.Y;
            if (a.Min.Z < b.Min.Z)
                merged.Min.Z = a.Min.Z;
            else
                merged.Min.Z = b.Min.Z;

            if (a.Max.X > b.Max.X)
                merged.Max.X = a.Max.X;
            else
                merged.Max.X = b.Max.X;
            if (a.Max.Y > b.Max.Y)
                merged.Max.Y = a.Max.Y;
            else
                merged.Max.Y = b.Max.Y;
            if (a.Max.Z > b.Max.Z)
                merged.Max.Z = a.Max.Z;
            else
                merged.Max.Z = b.Max.Z;
        }

    }
}
