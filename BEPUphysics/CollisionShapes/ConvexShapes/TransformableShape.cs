using System;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;

using BEPUutilities;
using BEPUutilities.ResourceManagement;

namespace BEPUphysics.CollisionShapes.ConvexShapes
{

    ///<summary>
    /// Shape which can take any convex shape and use a linear transform to shear, scale, and rotate it.
    ///</summary>
    public class TransformableShape : ConvexShape
    {
        protected ConvexShape shape;
        ///<summary>
        /// Gets or sets the convex shape to be transformed.
        ///</summary>
        public ConvexShape Shape
        {
            get
            {
                return shape;
            }
            set
            {
                shape = value;
                OnShapeChanged();
            }
        }

        protected Matrix3x3 transform;
        ///<summary>
        /// Gets or sets the linear transform used to transform the convex shape.
        ///</summary>
        public Matrix3x3 Transform
        {
            get
            {
                return transform;
            }
            set
            {
                transform = value;
                OnShapeChanged();
            }
        }

        ///<summary>
        /// Constructs a new transformable shape.
        ///</summary>
        ///<param name="shape">Base shape to transform.</param>
        ///<param name="transform">Transform to use.</param>
        public TransformableShape(ConvexShape shape, Matrix3x3 transform)
        {
            this.shape = shape;
            this.transform = transform;

            UpdateConvexShapeInfo();

        }        
        
        ///<summary>
        /// Constructs a new transformable shape.
        ///</summary>
        /// <param name="shape">Base shape to transform.</param>
        /// <param name="transform">Transform to use.</param>
        /// <param name="description">Cached information about the shape. Assumed to be correct; no extra processing or validation is performed.</param>
        public TransformableShape(ConvexShape shape, Matrix3x3 transform, ConvexShapeDescription description)
        {
            this.shape = shape;
            this.transform = transform;

            UpdateConvexShapeInfo(description);
        }

        protected override void OnShapeChanged()
        {
            UpdateConvexShapeInfo();
            base.OnShapeChanged();
        }

        /// <summary>
        /// Computes a convex shape description for a TransformableShape and applies it.
        /// </summary>
        public void UpdateConvexShapeInfo()
        {
            //Compute the volume distribution.
            var samples = CommonResources.GetVectorList();
            if (samples.Capacity < InertiaHelper.SampleDirections.Length)
                samples.Capacity = InertiaHelper.SampleDirections.Length;
            samples.Count = InertiaHelper.SampleDirections.Length;
            for (int i = 0; i < InertiaHelper.SampleDirections.Length; ++i)
            {
                shape.GetLocalExtremePointWithoutMargin(ref InertiaHelper.SampleDirections[i], out samples.Elements[i]);
            }

            var triangles = CommonResources.GetIntList();
            ConvexHullHelper.GetConvexHull(samples, triangles);

            float volume;
            InertiaHelper.ComputeShapeDistribution(samples, triangles, out volume, out volumeDistribution);
            Volume = volume;

            //Estimate the minimum radius based on the surface mesh.
            MinimumRadius = InertiaHelper.ComputeMinimumRadius(samples, triangles, ref Toolbox.ZeroVector) + collisionMargin;
            MaximumRadius = ComputeMaximumRadius();
            CommonResources.GiveBack(samples);
            CommonResources.GiveBack(triangles);


        }


        ///<summary>
        /// Gets the extreme point of the shape in local space in a given direction.
        ///</summary>
        ///<param name="direction">Direction to find the extreme point in.</param>
        ///<param name="extremePoint">Extreme point on the shape.</param>
        public override void GetLocalExtremePointWithoutMargin(ref Vector3 direction, out Vector3 extremePoint)
        {
            Vector3 d;
            Matrix3x3.TransformTranspose(ref direction, ref transform, out d);
            shape.GetLocalExtremePoint(d, out extremePoint);
            Matrix3x3.Transform(ref extremePoint, ref transform, out extremePoint);
        }


        /// <summary>
        /// Computes the maximum radius of the shape.
        /// This is often larger than the actual maximum radius;
        /// it is simply an approximation that avoids underestimating.
        /// </summary>
        /// <returns>Maximum radius of the shape.</returns>
        public float ComputeMaximumRadius()
        {
            //This will overestimate the actual maximum radius, but such is the defined behavior of the ComputeMaximumRadius function.  It's not exact; it's an upper bound on the actual maximum.
            RigidTransform identity = RigidTransform.Identity;
            BoundingBox boundingBox;
            GetBoundingBox(ref identity, out boundingBox);
            Vector3 diameter;
            Vector3.Subtract(ref boundingBox.Max, ref boundingBox.Min, out diameter);
            return diameter.Length();

        }

        

       

        /// <summary>
        /// Retrieves an instance of an EntityCollidable that uses this EntityShape.  Mainly used by compound bodies.
        /// </summary>
        /// <returns>EntityCollidable that uses this shape.</returns>
        public override EntityCollidable GetCollidableInstance()
        {
            return new ConvexCollidable<TransformableShape>(this);
        }

    }
}
