using System;
using System.Collections.Generic;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUutilities;
 
using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;

namespace BEPUphysics.CollisionShapes.ConvexShapes
{
    ///<summary>
    /// Convex shape entry to a WrappedShape.
    ///</summary>
    public struct ConvexShapeEntry
    {
        /// <summary>
        /// Convex shape of the entry.
        /// </summary>
        public ConvexShape CollisionShape;
        /// <summary>
        /// Local transform of the entry.
        /// </summary>
        public RigidTransform Transform;

        /// <summary>
        /// Constructs a convex shape entry.
        /// </summary>
        /// <param name="position">Local position of the entry.</param>
        /// <param name="shape">Shape of the entry.</param>
        public ConvexShapeEntry(Vector3 position, ConvexShape shape)
        {
            Transform = new RigidTransform(position);
            CollisionShape = shape;
        }

        /// <summary>
        /// Constructs a convex shape entry.
        /// </summary>
        /// <param name="orientation">Local orientation of the entry.</param>
        /// <param name="shape">Shape of the entry.</param>
        public ConvexShapeEntry(Quaternion orientation, ConvexShape shape)
        {
            Transform = new RigidTransform(orientation);
            CollisionShape = shape;
        }

        /// <summary>
        /// Constructs a convex shape entry.
        /// </summary>
        /// <param name="transform">Local transform of the entry.</param>
        /// <param name="shape">Shape of the entry.</param>
        public ConvexShapeEntry(RigidTransform transform, ConvexShape shape)
        {
            Transform = transform;
            CollisionShape = shape;
        }

        ///<summary>
        /// Constructs a convex shape entry with identity transformation.
        ///</summary>
        ///<param name="shape">Shape of the entry.</param>
        public ConvexShapeEntry(ConvexShape shape)
        {
            Transform = RigidTransform.Identity;
            CollisionShape = shape;
        }
    }
    ///<summary>
    /// Shape that wraps other convex shapes in a convex hull.
    /// One way to think of it is to collect a bunch of items and wrap shrinkwrap around them.
    /// That surface is the shape of the WrappedShape.
    ///</summary>
    public class WrappedShape : ConvexShape
    {
        ObservableList<ConvexShapeEntry> shapes = new ObservableList<ConvexShapeEntry>();
        ///<summary>
        /// Gets the shapes in wrapped shape.
        ///</summary>
        public ObservableList<ConvexShapeEntry> Shapes
        {
            get
            {
                return shapes;
            }
        }


        ///<summary>
        /// Constructs a wrapped shape.
        /// A constructor is also available which takes a list of objects rather than just a pair.
        /// The shape will be recentered.  If the center is needed, use the other constructor.
        ///</summary>
        ///<param name="firstShape">First shape in the wrapped shape.</param>
        ///<param name="secondShape">Second shape in the wrapped shape.</param>
        public WrappedShape(ConvexShapeEntry firstShape, ConvexShapeEntry secondShape)
        {
            shapes.Add(firstShape);
            shapes.Add(secondShape);

            Vector3 center;
            UpdateConvexShapeInfo(out center);

            shapes.Changed += ShapesChanged;
        }

        ///<summary>
        /// Constructs a wrapped shape.
        /// A constructor is also available which takes a list of objects rather than just a pair.
        /// The shape will be recentered.
        ///</summary>
        ///<param name="firstShape">First shape in the wrapped shape.</param>
        ///<param name="secondShape">Second shape in the wrapped shape.</param>
        ///<param name="center">Center of the shape before recentering..</param>
        public WrappedShape(ConvexShapeEntry firstShape, ConvexShapeEntry secondShape, out Vector3 center)
        {
            shapes.Add(firstShape);
            shapes.Add(secondShape);

            UpdateConvexShapeInfo(out center);

            shapes.Changed += ShapesChanged;
        }

        ///<summary>
        /// Constructs a wrapped shape.
        /// The shape will be recentered; if the center is needed, use the other constructor.
        ///</summary>
        ///<param name="shapeEntries">Shape entries used to construct the shape.</param>
        ///<exception cref="Exception">Thrown when the shape list is empty.</exception>
        public WrappedShape(IList<ConvexShapeEntry> shapeEntries)
        {
            if (shapeEntries.Count == 0)
                throw new ArgumentException("Cannot create a wrapped shape with no contained shapes.");
            for (int i = 0; i < shapeEntries.Count; i++)
            {
                shapes.Add(shapeEntries[i]);
            }

            Vector3 center;
            UpdateConvexShapeInfo(out center);
            shapes.Changed += ShapesChanged;
        }

        ///<summary>
        /// Constructs a wrapped shape.
        /// The shape will be recentered.
        ///</summary>
        ///<param name="shapeEntries">Shape entries used to construct the shape.</param>
        /// <param name="center">Center of the shape before recentering.</param>
        ///<exception cref="Exception">Thrown when the shape list is empty.</exception>
        public WrappedShape(IList<ConvexShapeEntry> shapeEntries, out Vector3 center)
        {
            if (shapeEntries.Count == 0)
                throw new ArgumentException("Cannot create a wrapped shape with no contained shapes.");
            for (int i = 0; i < shapeEntries.Count; i++)
            {
                shapes.Add(shapeEntries[i]);
            }

            UpdateConvexShapeInfo(out center);
            shapes.Changed += ShapesChanged;
        }

        ///<summary>
        /// Constructs a wrapped shape from cached data.
        ///</summary>
        ///<param name="shapeEntries">Already centered shape entries used to construct the shape.</param>
        /// <param name="description">Cached information about the shape. Assumed to be correct; no extra processing or validation is performed.</param>
        ///<exception cref="Exception">Thrown when the shape list is empty.</exception>
        public WrappedShape(IList<ConvexShapeEntry> shapeEntries, ConvexShapeDescription description)
        {
            if (shapeEntries.Count == 0)
                throw new ArgumentException("Cannot create a wrapped shape with no contained shapes."); 
            for (int i = 0; i < shapeEntries.Count; i++)
            {
                shapes.Add(shapeEntries[i]);
            }

            UpdateConvexShapeInfo(description);
            shapes.Changed += ShapesChanged;
        }


        protected override void OnShapeChanged()
        {
            Vector3 center;
            UpdateConvexShapeInfo(out center);
            base.OnShapeChanged();
        }

        void ShapesChanged(ObservableList<ConvexShapeEntry> list)
        {
            OnShapeChanged();
        }


        /// <summary>
        /// Computes and applies a convex shape description for this WrappedShape.
        /// </summary>
        /// <param name="center">Computed center of the shape before recentering.</param>
        public void UpdateConvexShapeInfo(out Vector3 center)
        {
            //Compute the volume distribution.
            var samples = CommonResources.GetVectorList();
            if (samples.Capacity < InertiaHelper.SampleDirections.Length)
                samples.Capacity = InertiaHelper.SampleDirections.Length;
            samples.Count = InertiaHelper.SampleDirections.Length;
            for (int i = 0; i < InertiaHelper.SampleDirections.Length; ++i)
            {
                GetLocalExtremePoint(InertiaHelper.SampleDirections[i], out samples.Elements[i]);
            }

            var triangles = CommonResources.GetIntList();
            ConvexHullHelper.GetConvexHull(samples, triangles);

            float volume;
            InertiaHelper.ComputeShapeDistribution(samples, triangles, out center, out volume, out volumeDistribution);
            Volume = volume;

            CommonResources.GiveBack(samples);
            CommonResources.GiveBack(triangles);

            //Now recenter the shape and compute the radii estimates.
            for (int i = 0; i < shapes.Count; i++)
            {
                shapes.WrappedList.Elements[i].Transform.Position -= center;
            }
            MinimumRadius = ComputeMinimumRadius();
            MaximumRadius = ComputeMaximumRadius();

        }



        /// <summary>
        /// Gets the bounding box of the shape given a transform.
        /// </summary>
        /// <param name="shapeTransform">Transform to use.</param>
        /// <param name="boundingBox">Bounding box of the transformed shape.</param>
        public override void GetBoundingBox(ref RigidTransform shapeTransform, out BoundingBox boundingBox)
        {
            RigidTransform subTransform;
            RigidTransform.Multiply(ref shapes.WrappedList.Elements[0].Transform, ref shapeTransform, out subTransform);
            shapes.WrappedList.Elements[0].CollisionShape.GetBoundingBox(ref subTransform, out boundingBox);
            for (int i = 1; i < shapes.WrappedList.Count; i++)
            {
                RigidTransform.Multiply(ref shapes.WrappedList.Elements[i].Transform, ref shapeTransform, out subTransform);
                BoundingBox toMerge;
                shapes.WrappedList.Elements[i].CollisionShape.GetBoundingBox(ref subTransform, out toMerge);
                BoundingBox.CreateMerged(ref boundingBox, ref toMerge, out boundingBox);
            }

            boundingBox.Min.X -= collisionMargin;
            boundingBox.Min.Y -= collisionMargin;
            boundingBox.Min.Z -= collisionMargin;

            boundingBox.Max.X += collisionMargin;
            boundingBox.Max.Y += collisionMargin;
            boundingBox.Max.Z += collisionMargin;
        }


        ///<summary>
        /// Gets the extreme point of the shape in local space in a given direction.
        ///</summary>
        ///<param name="direction">Direction to find the extreme point in.</param>
        ///<param name="extremePoint">Extreme point on the shape.</param>
        public override void GetLocalExtremePointWithoutMargin(ref Vector3 direction, out Vector3 extremePoint)
        {
            shapes.WrappedList.Elements[0].CollisionShape.GetExtremePoint(direction, ref shapes.WrappedList.Elements[0].Transform, out extremePoint);
            float maxDot;
            Vector3.Dot(ref extremePoint, ref direction, out maxDot);
            for (int i = 1; i < shapes.WrappedList.Count; i++)
            {
                float dot;
                Vector3 temp;

                shapes.WrappedList.Elements[i].CollisionShape.GetExtremePoint(direction, ref shapes.WrappedList.Elements[i].Transform, out temp);
                Vector3.Dot(ref direction, ref temp, out dot);
                if (dot > maxDot)
                {
                    extremePoint = temp;
                    maxDot = dot;
                }
            }
        }


        /// <summary>
        /// Computes the maximum radius of the shape.
        /// This is often larger than the actual maximum radius;
        /// it is simply an approximation that avoids underestimating.
        /// </summary>
        /// <returns>Maximum radius of the shape.</returns>
        public float ComputeMaximumRadius()
        {
            //This can overestimate the actual maximum radius, but such is the defined behavior of the ComputeMaximumRadius function.  It's not exact; it's an upper bound on the actual maximum.
            float maxRadius = 0;
            for (int i = 0; i < shapes.Count; i++)
            {
                float radius = shapes.WrappedList.Elements[i].CollisionShape.MaximumRadius +
                               shapes.WrappedList.Elements[i].Transform.Position.Length();
                if (radius > maxRadius)
                    maxRadius = radius;
            }
            return maxRadius + collisionMargin;
        }
        /// <summary>
        /// Computes the minimum radius of the shape.
        /// This is often smaller than the actual minimum radius;
        /// it is simply an approximation that avoids overestimating.
        /// </summary>
        /// <returns>Minimum radius of the shape.</returns>
        public float ComputeMinimumRadius()
        {
            //Could also use the tetrahedron approximation approach.
            float minRadius = 0;
            for (int i = 0; i < shapes.Count; i++)
            {
                float radius = shapes.WrappedList.Elements[i].CollisionShape.MinimumRadius;
                if (radius < minRadius)
                    minRadius = radius;
            }
            return minRadius + collisionMargin;
        }

        /// <summary>
        /// Retrieves an instance of an EntityCollidable that uses this EntityShape.  Mainly used by compound bodies.
        /// </summary>
        /// <returns>EntityCollidable that uses this shape.</returns>
        public override EntityCollidable GetCollidableInstance()
        {
            return new ConvexCollidable<WrappedShape>(this);
        }
    }
}
