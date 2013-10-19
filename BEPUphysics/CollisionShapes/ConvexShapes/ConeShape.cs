using System;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;

using BEPUutilities;

namespace BEPUphysics.CollisionShapes.ConvexShapes
{
    ///<summary>
    /// Symmetrical shape with a circular base and a point at the top.
    ///</summary>
    public class ConeShape : ConvexShape
    {

        float height;
        ///<summary>
        /// Gets or sets the height of the cone.
        ///</summary>
        public float Height
        {
            get { return height; }
            set
            {
                height = value;
                OnShapeChanged();
            }
        }

        float radius;
        ///<summary>
        /// Gets or sets the radius of the cone base.
        ///</summary>
        public float Radius
        {
            get { return radius; }
            set
            {
                radius = value;
                OnShapeChanged();
            }
        }

        ///<summary>
        /// Constructs a new cone shape.
        ///</summary>
        ///<param name="height">Height of the cone.</param>
        ///<param name="radius">Radius of the cone base.</param>
        public ConeShape(float height, float radius)
        {
            this.height = height;
            this.radius = radius;

            UpdateConvexShapeInfo(ComputeDescription(height, radius, collisionMargin));
        }

        ///<summary>
        /// Constructs a new cone shape.
        ///</summary>
        ///<param name="height">Height of the cone.</param>
        ///<param name="radius">Radius of the cone base.</param>
        /// <param name="description">Cached information about the shape. Assumed to be correct; no extra processing or validation is performed.</param>
        public ConeShape(float height, float radius, ConvexShapeDescription description)
        {
            this.height = height;
            this.radius = radius;

            UpdateConvexShapeInfo(description);
        }


        protected override void OnShapeChanged()
        {
            UpdateConvexShapeInfo(ComputeDescription(height, radius, collisionMargin));
            base.OnShapeChanged();
        }


        /// <summary>
        /// Computes a convex shape description for a ConeShape.
        /// </summary>
        ///<param name="height">Height of the cone.</param>
        ///<param name="radius">Radius of the cone base.</param>
        ///<param name="collisionMargin">Collision margin of the shape.</param>
        /// <returns>Description required to define a convex shape.</returns>
        public static ConvexShapeDescription ComputeDescription(float height, float radius, float collisionMargin)
        {
            ConvexShapeDescription description;
            description.EntityShapeVolume.Volume = (float)(.333333 * Math.PI * radius * radius * height);

            description.EntityShapeVolume.VolumeDistribution = new Matrix3x3();
            float diagValue = (.1f * height * height + .15f * radius * radius);
            description.EntityShapeVolume.VolumeDistribution.M11 = diagValue;
            description.EntityShapeVolume.VolumeDistribution.M22 = .3f * radius * radius;
            description.EntityShapeVolume.VolumeDistribution.M33 = diagValue;

            description.MaximumRadius = (float)(collisionMargin + Math.Max(.75 * height, Math.Sqrt(.0625f * height * height + radius * radius)));

            double denominator = radius / height;
            denominator = denominator / Math.Sqrt(denominator * denominator + 1);
            description.MinimumRadius = (float)(collisionMargin + Math.Min(.25f * height, denominator * .75 * height));

            description.CollisionMargin = collisionMargin;
            return description;
        }


        ///<summary>
        /// Gets the extreme point of the shape in local space in a given direction.
        ///</summary>
        ///<param name="direction">Direction to find the extreme point in.</param>
        ///<param name="extremePoint">Extreme point on the shape.</param>
        public override void GetLocalExtremePointWithoutMargin(ref Vector3 direction, out Vector3 extremePoint)
        {
            //Is it the tip of the cone?
            float sinThetaSquared = radius * radius / (radius * radius + height * height);
            //If d.Y * d.Y / d.LengthSquared >= sinthetaSquared
            if (direction.Y > 0 && direction.Y * direction.Y >= direction.LengthSquared() * sinThetaSquared)
            {
                extremePoint = new Vector3(0, .75f * height, 0);
                return;
            }
            //Is it a bottom edge of the cone?
            float horizontalLengthSquared = direction.X * direction.X + direction.Z * direction.Z;
            if (horizontalLengthSquared > Toolbox.Epsilon)
            {
                var radOverSigma = radius / Math.Sqrt(horizontalLengthSquared);
                extremePoint = new Vector3((float)(radOverSigma * direction.X), -.25f * height, (float)(radOverSigma * direction.Z));
            }
            else // It's pointing almost straight down...
                extremePoint = new Vector3(0, -.25f * height, 0);


        }


        /// <summary>
        /// Retrieves an instance of an EntityCollidable that uses this EntityShape.  Mainly used by compound bodies.
        /// </summary>
        /// <returns>EntityCollidable that uses this shape.</returns>
        public override EntityCollidable GetCollidableInstance()
        {
            return new ConvexCollidable<ConeShape>(this);
        }

    }
}
