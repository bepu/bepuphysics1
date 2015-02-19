using System;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;

using BEPUutilities;

namespace BEPUphysics.CollisionShapes.ConvexShapes
{
    ///<summary>
    /// Ball-like shape.
    ///</summary>
    public class SphereShape : ConvexShape
    {

        //This is a convenience method.  People expect to see a 'radius' of some kind.
        ///<summary>
        /// Gets or sets the radius of the sphere.
        ///</summary>
        public float Radius { get { return collisionMargin; } set { CollisionMargin = value; } }

        ///<summary>
        /// Constructs a new sphere shape.
        ///</summary>
        ///<param name="radius">Radius of the sphere.</param>
        public SphereShape(float radius)
        {
            Radius = radius;

            UpdateConvexShapeInfo(ComputeDescription(radius));
        }


        ///<summary>
        /// Constructs a new sphere shape.
        ///</summary>
        /// <param name="description">Cached information about the shape. Assumed to be correct; no extra processing or validation is performed.</param>
        public SphereShape(ConvexShapeDescription description)
        {
            UpdateConvexShapeInfo(description);
        }

        protected override void OnShapeChanged()
        {
            UpdateConvexShapeInfo(ComputeDescription(Radius));
            base.OnShapeChanged();
        }

        /// <summary>
        /// Computes a convex shape description for a SphereShape.
        /// </summary>
        ///<param name="radius">Radius of the sphere.</param>
        /// <returns>Description required to define a convex shape.</returns>
        public static ConvexShapeDescription ComputeDescription(float radius)
        {
            ConvexShapeDescription description;
            description.EntityShapeVolume.Volume = 1.333333f * MathHelper.Pi * radius * radius * radius;
            description.EntityShapeVolume.VolumeDistribution = new Matrix3x3();
            float diagValue = ((2f / 5f) * radius * radius);
            description.EntityShapeVolume.VolumeDistribution.M11 = diagValue;
            description.EntityShapeVolume.VolumeDistribution.M22 = diagValue;
            description.EntityShapeVolume.VolumeDistribution.M33 = diagValue;

            description.MinimumRadius = radius;
            description.MaximumRadius = radius;

            description.CollisionMargin = radius;
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
            boundingBox.Min.X = shapeTransform.Position.X - collisionMargin;
            boundingBox.Min.Y = shapeTransform.Position.Y - collisionMargin;
            boundingBox.Min.Z = shapeTransform.Position.Z - collisionMargin;
            boundingBox.Max.X = shapeTransform.Position.X + collisionMargin;
            boundingBox.Max.Y = shapeTransform.Position.Y + collisionMargin;
            boundingBox.Max.Z = shapeTransform.Position.Z + collisionMargin;
        }


        //TODO: Could do a little optimizing.  If the methods were virtual, could override and save a conjugate/transform.
        ///<summary>
        /// Gets the extreme point of the shape in local space in a given direction.
        ///</summary>
        ///<param name="direction">Direction to find the extreme point in.</param>
        ///<param name="extremePoint">Extreme point on the shape.</param>
        public override void GetLocalExtremePointWithoutMargin(ref Vector3 direction, out Vector3 extremePoint)
        {
            extremePoint = Toolbox.ZeroVector;
        }


        /// <summary>
        /// Gets the intersection between the sphere and the ray.
        /// </summary>
        /// <param name="ray">Ray to test against the sphere.</param>
        /// <param name="transform">Transform applied to the convex for the test.</param>
        /// <param name="maximumLength">Maximum distance to travel in units of the ray direction's length.</param>
        /// <param name="hit">Ray hit data, if any.</param>
        /// <returns>Whether or not the ray hit the target.</returns>
        public override bool RayTest(ref Ray ray, ref RigidTransform transform, float maximumLength, out RayHit hit)
        {
            return Toolbox.RayCastSphere(ref ray, ref transform.Position, collisionMargin, maximumLength, out hit);
        }


        /// <summary>
        /// Retrieves an instance of an EntityCollidable that uses this EntityShape.  Mainly used by compound bodies.
        /// </summary>
        /// <returns>EntityCollidable that uses this shape.</returns>
        public override EntityCollidable GetCollidableInstance()
        {
            return new ConvexCollidable<SphereShape>(this);
        }

    }
}
