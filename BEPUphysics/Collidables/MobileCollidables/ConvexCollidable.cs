using BEPUphysics.CollisionShapes.ConvexShapes;
using Microsoft.Xna.Framework;

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

        protected ConvexCollidable(ConvexShape shape, float minimumRadius, float maximumRadius)
            : base(shape, minimumRadius, maximumRadius)
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

        ///<summary>
        /// Constructs a new convex collidable.
        ///</summary>
        ///<param name="shape">Shape to use for the collidable.</param>
        ///<param name="minimumRadius">Precomputed minimum radius of the collidable.
        /// If the other constructor is used, these are calculated automatically; sometimes re-using known values can help construction speeds.</param>
        ///<param name="maximumRadius">Precomputed maximum radius of the collidable.
        /// If the other constructor is used, these are calculated automatically; sometimes re-using known values can help construction speeds.</param>
        public ConvexCollidable(T shape, float minimumRadius, float maximumRadius)
            : base(shape, minimumRadius, maximumRadius)
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
                if (entity.linearVelocity.X > 0)
                    boundingBox.Max.X += entity.linearVelocity.X * dt;
                else
                    boundingBox.Min.X += entity.linearVelocity.X * dt;

                if (entity.linearVelocity.Y > 0)
                    boundingBox.Max.Y += entity.linearVelocity.Y * dt;
                else
                    boundingBox.Min.Y += entity.linearVelocity.Y * dt;

                if (entity.linearVelocity.Z > 0)
                    boundingBox.Max.Z += entity.linearVelocity.Z * dt;
                else
                    boundingBox.Min.Z += entity.linearVelocity.Z * dt;
            }
        }

    }
}
