using System;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;

using BEPUutilities;

namespace BEPUphysics.CollisionShapes.ConvexShapes
{
    ///<summary>
    /// Convex shape with width, length, and height.
    ///</summary>
    public class BoxShape : ConvexShape
    {
        internal float halfWidth;
        internal float halfHeight;
        internal float halfLength;


        /// <summary>
        /// Width of the box divided by two.
        /// </summary>
        public float HalfWidth
        {
            get { return halfWidth; }
            set { halfWidth = value; OnShapeChanged(); }
        }

        /// <summary>
        /// Height of the box divided by two.
        /// </summary>
        public float HalfHeight
        {
            get { return halfHeight; }
            set { halfHeight = value; OnShapeChanged(); }
        }

        /// <summary>
        /// Length of the box divided by two.
        /// </summary>
        public float HalfLength
        {
            get { return halfLength; }
            set { halfLength = value; OnShapeChanged(); }
        }

        /// <summary>
        /// Width of the box.
        /// </summary>
        public float Width
        {
            get { return halfWidth * 2; }
            set { halfWidth = value * 0.5f; OnShapeChanged(); }
        }

        /// <summary>
        /// Height of the box.
        /// </summary>
        public float Height
        {
            get { return halfHeight * 2; }
            set { halfHeight = value * 0.5f; OnShapeChanged(); }
        }

        /// <summary>
        /// Length of the box.
        /// </summary>
        public float Length
        {
            get { return halfLength * 2; }
            set { halfLength = value * 0.5f; OnShapeChanged(); }
        }


        ///<summary>
        /// Constructs a new box shape.
        ///</summary>
        ///<param name="width">Width of the box.</param>
        ///<param name="height">Height of the box.</param>
        ///<param name="length">Length of the box.</param>
        public BoxShape(float width, float height, float length)
        {
            halfWidth = width * 0.5f;
            halfHeight = height * 0.5f;
            halfLength = length * 0.5f;

            UpdateConvexShapeInfo(ComputeDescription(width, height, length, collisionMargin));
        }

        ///<summary>
        /// Constructs a new box shape from cached information.
        ///</summary>
        ///<param name="width">Width of the box.</param>
        ///<param name="height">Height of the box.</param>
        ///<param name="length">Length of the box.</param>
        /// <param name="description">Cached information about the shape. Assumed to be correct; no extra processing or validation is performed.</param>
        public BoxShape(float width, float height, float length, ConvexShapeDescription description)
        {
            halfWidth = width * 0.5f;
            halfHeight = height * 0.5f;
            halfLength = length * 0.5f;

            UpdateConvexShapeInfo(description);
        }

        protected override void OnShapeChanged()
        {
            UpdateConvexShapeInfo(ComputeDescription(halfWidth, halfHeight, halfLength, collisionMargin));
            base.OnShapeChanged();
        }

        /// <summary>
        /// Computes a convex shape description for a BoxShape.
        /// </summary>
        ///<param name="width">Width of the box.</param>
        ///<param name="height">Height of the box.</param>
        ///<param name="length">Length of the box.</param>
        /// <param name="collisionMargin">Collision margin of the shape.</param>
        /// <returns>Description required to define a convex shape.</returns>
        public static ConvexShapeDescription ComputeDescription(float width, float height, float length, float collisionMargin)
        {
            ConvexShapeDescription description;
            description.EntityShapeVolume.Volume = width * height * length;

            float widthSquared = width * width;
            float heightSquared = height * height;
            float lengthSquared = length * length;
            const float inv12 = 1 / 12f;

            description.EntityShapeVolume.VolumeDistribution = new Matrix3x3();
            description.EntityShapeVolume.VolumeDistribution.M11 = (heightSquared + lengthSquared) * inv12;
            description.EntityShapeVolume.VolumeDistribution.M22 = (widthSquared + lengthSquared) * inv12;
            description.EntityShapeVolume.VolumeDistribution.M33 = (widthSquared + heightSquared) * inv12;

            description.MaximumRadius = 0.5f * (float)Math.Sqrt(width * width + height * height + length * length);
            description.MinimumRadius = 0.5f * Math.Min(width, Math.Min(height, length));

            description.CollisionMargin = collisionMargin;
            return description;
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
            //Sample the local directions from the orientation matrix, implicitly transposed.
            //Notice only three directions are used.  Due to box symmetry, 'left' is just -right.
            var right = new Vector3(Math.Sign(o.M11) * halfWidth, Math.Sign(o.M21) * halfHeight, Math.Sign(o.M31) * halfLength);

            var up = new Vector3(Math.Sign(o.M12) * halfWidth, Math.Sign(o.M22) * halfHeight, Math.Sign(o.M32) * halfLength);

            var backward = new Vector3(Math.Sign(o.M13) * halfWidth, Math.Sign(o.M23) * halfHeight, Math.Sign(o.M33) * halfLength);


            //Rather than transforming each axis independently (and doing three times as many operations as required), just get the 3 required values directly.
            Vector3 offset;
            TransformLocalExtremePoints(ref right, ref up, ref backward, ref o, out offset);

            //The positive and negative vectors represent the X, Y and Z coordinates of the extreme points in world space along the world space axes.
            Vector3.Add(ref shapeTransform.Position, ref offset, out boundingBox.Max);
            Vector3.Subtract(ref shapeTransform.Position, ref offset, out boundingBox.Min);

        }


        ///<summary>
        /// Gets the extreme point of the shape in local space in a given direction.
        ///</summary>
        ///<param name="direction">Direction to find the extreme point in.</param>
        ///<param name="extremePoint">Extreme point on the shape.</param>
        public override void GetLocalExtremePointWithoutMargin(ref Vector3 direction, out Vector3 extremePoint)
        {
            extremePoint = new Vector3(Math.Sign(direction.X) * (halfWidth - collisionMargin), Math.Sign(direction.Y) * (halfHeight - collisionMargin), Math.Sign(direction.Z) * (halfLength - collisionMargin));
        }




        /// <summary>
        /// Gets the intersection between the box and the ray.
        /// </summary>
        /// <param name="ray">Ray to test against the box.</param>
        /// <param name="transform">Transform of the shape.</param>
        /// <param name="maximumLength">Maximum distance to travel in units of the direction vector's length.</param>
        /// <param name="hit">Hit data for the raycast, if any.</param>
        /// <returns>Whether or not the ray hit the target.</returns>
        public override bool RayTest(ref Ray ray, ref RigidTransform transform, float maximumLength, out RayHit hit)
        {
            hit = new RayHit();

            Quaternion conjugate;
            Quaternion.Conjugate(ref transform.Orientation, out conjugate);
            Vector3 localOrigin;
            Vector3.Subtract(ref ray.Position, ref transform.Position, out localOrigin);
            Quaternion.Transform(ref localOrigin, ref conjugate, out localOrigin);
            Vector3 localDirection;
            Quaternion.Transform(ref ray.Direction, ref conjugate, out localDirection);
            Vector3 normal = Toolbox.ZeroVector;
            float temp, tmin = 0, tmax = maximumLength;

            if (Math.Abs(localDirection.X) < Toolbox.Epsilon && (localOrigin.X < -halfWidth || localOrigin.X > halfWidth))
                return false;
            float inverseDirection = 1 / localDirection.X;
            float t1 = (-halfWidth - localOrigin.X) * inverseDirection;
            float t2 = (halfWidth - localOrigin.X) * inverseDirection;
            var tempNormal = new Vector3(-1, 0, 0);
            if (t1 > t2)
            {
                temp = t1;
                t1 = t2;
                t2 = temp;
                tempNormal *= -1;
            }
            temp = tmin;
            tmin = Math.Max(tmin, t1);
            if (temp != tmin)
                normal = tempNormal;
            tmax = Math.Min(tmax, t2);
            if (tmin > tmax)
                return false;
            if (Math.Abs(localDirection.Y) < Toolbox.Epsilon && (localOrigin.Y < -halfHeight || localOrigin.Y > halfHeight))
                return false;
            inverseDirection = 1 / localDirection.Y;
            t1 = (-halfHeight - localOrigin.Y) * inverseDirection;
            t2 = (halfHeight - localOrigin.Y) * inverseDirection;
            tempNormal = new Vector3(0, -1, 0);
            if (t1 > t2)
            {
                temp = t1;
                t1 = t2;
                t2 = temp;
                tempNormal *= -1;
            }
            temp = tmin;
            tmin = Math.Max(tmin, t1);
            if (temp != tmin)
                normal = tempNormal;
            tmax = Math.Min(tmax, t2);
            if (tmin > tmax)
                return false;
            if (Math.Abs(localDirection.Z) < Toolbox.Epsilon && (localOrigin.Z < -halfLength || localOrigin.Z > halfLength))
                return false;
            inverseDirection = 1 / localDirection.Z;
            t1 = (-halfLength - localOrigin.Z) * inverseDirection;
            t2 = (halfLength - localOrigin.Z) * inverseDirection;
            tempNormal = new Vector3(0, 0, -1);
            if (t1 > t2)
            {
                temp = t1;
                t1 = t2;
                t2 = temp;
                tempNormal *= -1;
            }
            temp = tmin;
            tmin = Math.Max(tmin, t1);
            if (temp != tmin)
                normal = tempNormal;
            tmax = Math.Min(tmax, t2);
            if (tmin > tmax)
                return false;
            hit.T = tmin;
            Vector3.Multiply(ref ray.Direction, tmin, out hit.Location);
            Vector3.Add(ref hit.Location, ref ray.Position, out hit.Location);
            Quaternion.Transform(ref normal, ref transform.Orientation, out normal);
            hit.Normal = normal;
            return true;
        }

        /// <summary>
        /// Retrieves an instance of an EntityCollidable that uses this EntityShape.  Mainly used by compound bodies.
        /// </summary>
        /// <returns>EntityCollidable that uses this shape.</returns>
        public override EntityCollidable GetCollidableInstance()
        {
            return new ConvexCollidable<BoxShape>(this);
        }

    }
}
