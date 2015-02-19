using System;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
 
using BEPUutilities;

namespace BEPUphysics.CollisionShapes.ConvexShapes
{
    ///<summary>
    /// Symmetrical object with a circular bottom and top.
    ///</summary>
    public class CylinderShape : ConvexShape
    {
        private float radius;
        ///<summary>
        /// Gets or sets the radius of the cylinder.
        ///</summary>
        public float Radius { get { return radius; } set { radius = value; OnShapeChanged(); } }

        private float halfHeight;
        ///<summary>
        /// Gets or sets the height of the cylinder.
        ///</summary>
        public float Height { get { return halfHeight * 2; } set { halfHeight = value * 0.5f; OnShapeChanged(); } }

        ///<summary>
        /// Constructs a new cylinder shape.
        ///</summary>
        ///<param name="height">Height of the cylinder.</param>
        ///<param name="radius">Radius of the cylinder.</param>
        public CylinderShape(float height, float radius)
        {
            halfHeight = height * .5f;
            this.radius = radius;
            UpdateConvexShapeInfo(ComputeDescription(height, radius, collisionMargin));
        }

        ///<summary>
        /// Constructs a new cylinder shape from cached data.
        ///</summary>
        ///<param name="height">Height of the cylinder.</param>
        ///<param name="radius">Radius of the cylinder.</param>
        /// <param name="description">Cached information about the shape. Assumed to be correct; no extra processing or validation is performed.</param>
        public CylinderShape(float height, float radius, ConvexShapeDescription description)
        {
            halfHeight = height * .5f;
            this.radius = radius;
            UpdateConvexShapeInfo(description);
        }

        protected override void OnShapeChanged()
        {
            UpdateConvexShapeInfo(ComputeDescription(Height, radius, collisionMargin));
            base.OnShapeChanged();
        }
        /// <summary>
        /// Computes a convex shape description for a CylinderShape.
        /// </summary>
        ///<param name="height">Height of the cylinder.</param>
        ///<param name="radius">Radius of the cylinder.</param>
        ///<param name="collisionMargin">Collision margin of the shape.</param>
        /// <returns>Description required to define a convex shape.</returns>
        public static ConvexShapeDescription ComputeDescription(float height, float radius, float collisionMargin)
        {
            ConvexShapeDescription description;
            description.EntityShapeVolume.Volume = MathHelper.Pi * radius * radius * height;

            description.EntityShapeVolume.VolumeDistribution = new Matrix3x3();
            float diagValue = (.0833333333f * height * height + .25f * radius * radius);
            description.EntityShapeVolume.VolumeDistribution.M11 = diagValue;
            description.EntityShapeVolume.VolumeDistribution.M22 = .5f * radius * radius;
            description.EntityShapeVolume.VolumeDistribution.M33 = diagValue;

            float halfHeight = height * 0.5f;
            description.MinimumRadius = Math.Min(radius, halfHeight);
            description.MaximumRadius = (float)Math.Sqrt(radius * radius + halfHeight * halfHeight);
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
            //Notice only three directions are used.  Due to cylinder symmetry, 'left' is just -right.
            var direction = new Vector3(o.M11, o.M21, o.M31);
            Vector3 right;
            GetLocalExtremePointWithoutMargin(ref direction, out right);

            direction = new Vector3(o.M12, o.M22, o.M32);
            Vector3 up;
            GetLocalExtremePointWithoutMargin(ref direction, out up);

            direction = new Vector3(o.M13, o.M23, o.M33);
            Vector3 backward;
            GetLocalExtremePointWithoutMargin(ref direction, out backward);

            //Rather than transforming each axis independently (and doing three times as many operations as required), just get the 3 required values directly.
            Vector3 positive;
            TransformLocalExtremePoints(ref right, ref up, ref backward, ref o, out positive);

            //The positive and negative vectors represent the X, Y and Z coordinates of the extreme points in world space along the world space axes.
            boundingBox.Max.X = shapeTransform.Position.X + positive.X + collisionMargin;
            boundingBox.Max.Y = shapeTransform.Position.Y + positive.Y + collisionMargin;
            boundingBox.Max.Z = shapeTransform.Position.Z + positive.Z + collisionMargin;

            boundingBox.Min.X = shapeTransform.Position.X - positive.X - collisionMargin;
            boundingBox.Min.Y = shapeTransform.Position.Y - positive.Y - collisionMargin;
            boundingBox.Min.Z = shapeTransform.Position.Z - positive.Z - collisionMargin;
        }


        ///<summary>
        /// Gets the extreme point of the shape in local space in a given direction.
        ///</summary>
        ///<param name="direction">Direction to find the extreme point in.</param>
        ///<param name="extremePoint">Extreme point on the shape.</param>
        public override void GetLocalExtremePointWithoutMargin(ref Vector3 direction, out Vector3 extremePoint)
        {
            float horizontalLengthSquared = direction.X * direction.X + direction.Z * direction.Z;
            if (horizontalLengthSquared > Toolbox.Epsilon)
            {
                float multiplier = (radius - collisionMargin) / (float)Math.Sqrt(horizontalLengthSquared);
                extremePoint = new Vector3(direction.X * multiplier, Math.Sign(direction.Y) * (halfHeight - collisionMargin), direction.Z * multiplier);
            }
            else
            {
                extremePoint = new Vector3(0, Math.Sign(direction.Y) * (halfHeight - collisionMargin), 0);
            }

        }

        
        /// <summary>
        /// Retrieves an instance of an EntityCollidable that uses this EntityShape.  Mainly used by compound bodies.
        /// </summary>
        /// <returns>EntityCollidable that uses this shape.</returns>
        public override EntityCollidable GetCollidableInstance()
        {
            return new ConvexCollidable<CylinderShape>(this);
        }

        /// <summary>
        /// Gets the intersection between the convex shape and the ray.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="transform">Transform of the convex shape.</param>
        /// <param name="maximumLength">Maximum distance to travel in units of the ray direction's length.</param>
        /// <param name="hit">Ray hit data, if any.</param>
        /// <returns>Whether or not the ray hit the target.</returns>
        public override bool RayTest(ref Ray ray, ref RigidTransform transform, float maximumLength, out RayHit hit)
        {
            //Put the ray into local space.
            Quaternion conjugate;
            Quaternion.Conjugate(ref transform.Orientation, out conjugate);
            Ray localRay;
            Vector3.Subtract(ref ray.Position, ref transform.Position, out localRay.Position);
            Quaternion.Transform(ref localRay.Position, ref conjugate, out localRay.Position);
            Quaternion.Transform(ref ray.Direction, ref conjugate, out localRay.Direction);

            //Check for containment.
            if (localRay.Position.Y >= -halfHeight && localRay.Position.Y <= halfHeight && localRay.Position.X * localRay.Position.X + localRay.Position.Z * localRay.Position.Z <= radius * radius)
            {
                //It's inside!
                hit.T = 0;
                hit.Location = localRay.Position;
                hit.Normal = new Vector3(hit.Location.X, 0, hit.Location.Z);
                float normalLengthSquared = hit.Normal.LengthSquared();
                if (normalLengthSquared > 1e-9f)
                    Vector3.Divide(ref hit.Normal, (float)Math.Sqrt(normalLengthSquared), out hit.Normal);
                else
                    hit.Normal = new Vector3();
                //Pull the hit into world space.
                Quaternion.Transform(ref hit.Normal, ref transform.Orientation, out hit.Normal);
                RigidTransform.Transform(ref hit.Location, ref transform, out hit.Location);
                return true;
            }

            //Project the ray direction onto the plane where the cylinder is a circle.
            //The projected ray is then tested against the circle to compute the time of impact.
            //That time of impact is used to compute the 3d hit location.
            Vector2 planeDirection = new Vector2(localRay.Direction.X, localRay.Direction.Z);
            float planeDirectionLengthSquared = planeDirection.LengthSquared();

            if (planeDirectionLengthSquared < Toolbox.Epsilon)
            {
                //The ray is nearly parallel with the axis.
                //Skip the cylinder-sides test.  We're either inside the cylinder and won't hit the sides, or we're outside
                //and won't hit the sides.  
                if (localRay.Position.Y > halfHeight)
                    goto upperTest;
                if (localRay.Position.Y < -halfHeight)
                    goto lowerTest;


                hit = new RayHit();
                return false;

            }
            Vector2 planeOrigin = new Vector2(localRay.Position.X, localRay.Position.Z);
            float dot;
            Vector2.Dot(ref planeDirection, ref planeOrigin, out dot);
            float closestToCenterT = -dot / planeDirectionLengthSquared;

            Vector2 closestPoint;
            Vector2.Multiply(ref planeDirection, closestToCenterT, out closestPoint);
            Vector2.Add(ref planeOrigin, ref closestPoint, out closestPoint);
            //How close does the ray come to the circle?
            float squaredDistance = closestPoint.LengthSquared();
            if (squaredDistance > radius * radius)
            {
                //It's too far!  The ray cannot possibly hit the capsule.
                hit = new RayHit();
                return false;
            }



            //With the squared distance, compute the distance backward along the ray from the closest point on the ray to the axis.
            float backwardsDistance = radius * (float)Math.Sqrt(1 - squaredDistance / (radius * radius));
            float tOffset = backwardsDistance / (float)Math.Sqrt(planeDirectionLengthSquared);

            hit.T = closestToCenterT - tOffset;

            //Compute the impact point on the infinite cylinder in 3d local space.
            Vector3.Multiply(ref localRay.Direction, hit.T, out hit.Location);
            Vector3.Add(ref hit.Location, ref localRay.Position, out hit.Location);

            //Is it intersecting the cylindrical portion of the capsule?
            if (hit.Location.Y <= halfHeight && hit.Location.Y >= -halfHeight && hit.T < maximumLength)
            {
                //Yup!
                hit.Normal = new Vector3(hit.Location.X, 0, hit.Location.Z);
                float normalLengthSquared = hit.Normal.LengthSquared();
                if (normalLengthSquared > 1e-9f)
                    Vector3.Divide(ref hit.Normal, (float)Math.Sqrt(normalLengthSquared), out hit.Normal);
                else
                    hit.Normal = new Vector3();
                //Pull the hit into world space.
                Quaternion.Transform(ref hit.Normal, ref transform.Orientation, out hit.Normal);
                RigidTransform.Transform(ref hit.Location, ref transform, out hit.Location);
                return true;
            }

            if (hit.Location.Y < halfHeight)
                goto lowerTest;
        upperTest:
            //Nope! It may be intersecting the ends of the cylinder though.
            //We're above the cylinder, so cast a ray against the upper cap.
            if (localRay.Direction.Y > -1e-9)
            {
                //Can't hit the upper cap if the ray isn't pointing down.
                hit = new RayHit();
                return false;
            }
            float t = (halfHeight - localRay.Position.Y) / localRay.Direction.Y;
            Vector3 planeIntersection;
            Vector3.Multiply(ref localRay.Direction, t, out planeIntersection);
            Vector3.Add(ref localRay.Position, ref planeIntersection, out planeIntersection);
            if(planeIntersection.X * planeIntersection.X + planeIntersection.Z * planeIntersection.Z < radius * radius + 1e-9 && t < maximumLength)
            {
                //Pull the hit into world space.
                Quaternion.Transform(ref Toolbox.UpVector, ref transform.Orientation, out hit.Normal);
                RigidTransform.Transform(ref planeIntersection, ref transform, out hit.Location);
                hit.T = t;
                return true;
            }
            //No intersection! We can't be hitting the other sphere, so it's over!
            hit = new RayHit();
            return false;

        lowerTest:
            //Is it intersecting the bottom cap?
            if (localRay.Direction.Y < 1e-9)
            {
                //Can't hit the bottom cap if the ray isn't pointing up.
                hit = new RayHit();
                return false;
            }
            t = (-halfHeight - localRay.Position.Y) / localRay.Direction.Y;
            Vector3.Multiply(ref localRay.Direction, t, out planeIntersection);
            Vector3.Add(ref localRay.Position, ref planeIntersection, out planeIntersection);
            if (planeIntersection.X * planeIntersection.X + planeIntersection.Z * planeIntersection.Z < radius * radius + 1e-9 && t < maximumLength)
            {
                //Pull the hit into world space.
                Quaternion.Transform(ref Toolbox.DownVector, ref transform.Orientation, out hit.Normal);
                RigidTransform.Transform(ref planeIntersection, ref transform, out hit.Location);
                hit.T = t;
                return true;
            }
            //No intersection! We can't be hitting the other sphere, so it's over!
            hit = new RayHit();
            return false;

        }


    }
}
