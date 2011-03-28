using BEPUphysics.CollisionShapes.ConvexShapes;
using Microsoft.Xna.Framework;
using System;
using BEPUphysics.PositionUpdating;
using BEPUphysics.Settings;

namespace BEPUphysics.Collidables.MobileCollidables
{
    ///<summary>
    /// Collidable with a convex shape.
    ///</summary>
    public abstract class ConvexCollidable : EntityCollidable
    {

        protected ConvexCollidable(ConvexShape shape)
            : base(shape)
        {

        }

        ///<summary>
        /// Gets the shape of the collidable.
        ///</summary>
        public new ConvexShape Shape
        {
            get
            {
                return base.Shape as ConvexShape;
            }
        }





    }

    ///<summary>
    /// Collidable with a convex shape of a particular type.
    ///</summary>
    ///<typeparam name="T">ConvexShape type.</typeparam>
    public class ConvexCollidable<T> : ConvexCollidable where T : ConvexShape
    {
        ///<summary>
        /// Gets the shape of the collidable.
        ///</summary>
        public new T Shape
        {
            get
            {
                return base.Shape as T;
            }
        }

        ///<summary>
        /// Constructs a new convex collidable.
        ///</summary>
        ///<param name="shape">Shape to use in the collidable.</param>
        public ConvexCollidable(T shape)
            : base(shape)
        {

        }


        /// <summary>
        /// Tests a ray against the entry.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length, in units of the ray's direction's length, to test.</param>
        /// <param name="rayHit">Hit location of the ray on the entry, if any.</param>
        /// <returns>Whether or not the ray hit the entry.</returns>
        public override bool RayCast(Ray ray, float maximumLength, out RayHit rayHit)
        {
            return Shape.RayTest(ref ray, ref worldTransform, maximumLength, out rayHit);
        }



        protected internal override void UpdateBoundingBoxInternal(float dt)
        {
            Shape.GetBoundingBox(ref worldTransform, out boundingBox);

            //Expand bounding box with velocity.
            if (dt > 0)
            {
                bool useExtraExpansion = MotionSettings.UseExtraExpansionForContinuousBoundingBoxes && entity.PositionUpdateMode == PositionUpdateMode.Continuous;
                float velocityScaling = useExtraExpansion ? 2 : 1;
                if (entity.linearVelocity.X > 0)
                    boundingBox.Max.X += entity.linearVelocity.X * dt * velocityScaling;
                else
                    boundingBox.Min.X += entity.linearVelocity.X * dt * velocityScaling;

                if (entity.linearVelocity.Y > 0)
                    boundingBox.Max.Y += entity.linearVelocity.Y * dt * velocityScaling;
                else
                    boundingBox.Min.Y += entity.linearVelocity.Y * dt * velocityScaling;

                if (entity.linearVelocity.Z > 0)
                    boundingBox.Max.Z += entity.linearVelocity.Z * dt * velocityScaling;
                else
                    boundingBox.Min.Z += entity.linearVelocity.Z * dt * velocityScaling;



                
                if (useExtraExpansion)
                {
                    float expansion = 0;
                    //It's possible that an object could have a small bounding box since its own
                    //velocity is low, but then a collision with a high velocity object sends
                    //it way out of its bounding box.  By taking into account high velocity objects
                    //in danger of hitting us and expanding our own bounding box by their speed,
                    //we stand a much better chance of not missing secondary collisions.
                    foreach (var e in OverlappedEntities)
                    {

                        float velocity = e.linearVelocity.LengthSquared();
                        if (velocity > expansion)
                            expansion = velocity;
                    }
                    expansion = (float)Math.Sqrt(expansion) * dt;


                    boundingBox.Min.X -= expansion;
                    boundingBox.Min.Y -= expansion;
                    boundingBox.Min.Z -= expansion;

                    boundingBox.Max.X += expansion;
                    boundingBox.Max.Y += expansion;
                    boundingBox.Max.Z += expansion;
                    
                }

                //Could use this to incorporate angular motion.  Since the bounding box is an approximation to begin with,
                //this isn't too important.  If an updating system is used where the bounding box MUST fully contain the frame's motion
                //then the commented area should be used.
                //Math.Min(entity.angularVelocity.Length() * dt, Shape.maximumRadius) * velocityScaling;
                //TODO: consider using minimum radius 

            }
        }


    }
}
