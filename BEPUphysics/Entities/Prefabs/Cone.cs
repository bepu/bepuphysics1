using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.EntityStateManagement;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionShapes.ConvexShapes;

namespace BEPUphysics.Entities.Prefabs
{
    /// <summary>
    /// Cone-shaped object that can collide and move.  After making an entity, add it to a Space so that the engine can manage it.
    /// </summary>
    public class Cone : Entity<ConvexCollidable<ConeShape>>
    {
        /// <summary>
        /// Gets or sets the length of the cone.
        /// </summary>
        public float Height
        {
            get
            {
                return CollisionInformation.Shape.Height;
            }
            set
            {
                CollisionInformation.Shape.Height = value;
            }
        }

        /// <summary>
        /// Gets or sets the radius of the cone.
        /// </summary>
        public float Radius
        {
            get
            {
                return CollisionInformation.Shape.Radius;
            }
            set
            {
                CollisionInformation.Shape.Radius = value;
            }
        }


        private Cone(float high, float rad)
            :base(new ConvexCollidable<ConeShape>(new ConeShape(high, rad)))
        {
        }

        private Cone(float high, float rad, float mass)
            :base(new ConvexCollidable<ConeShape>(new ConeShape(high, rad)), mass)
        {
        }



        /// <summary>
        /// Constructs a physically simulated cone.
        /// </summary>
        /// <param name="pos">Position of the cone.</param>
        /// <param name="high">Height of the cone.</param>
        /// <param name="rad">Radius of the cone.</param>
        /// <param name="m">Mass of the object.</param>
        public Cone(Vector3 pos, float high, float rad, float m)
            : this(high, rad, m)
        {
            Position = pos;
        }

        /// <summary>
        /// Constructs a nondynamic cone.
        /// </summary>
        /// <param name="pos">Position of the cone.</param>
        /// <param name="high">Height of the cone.</param>
        /// <param name="rad">Radius of the cone.</param>
        public Cone(Vector3 pos, float high, float rad)
            : this(high, rad)
        {
            Position = pos;
        }

        /// <summary>
        /// Constructs a physically simulated cone.
        /// </summary>
        /// <param name="motionState">Motion state specifying the entity's initial state.</param>
        /// <param name="high">Height of the cone.</param>
        /// <param name="rad">Radius of the cone.</param>
        /// <param name="m">Mass of the object.</param>
        public Cone(MotionState motionState, float high, float rad, float m)
            : this(high, rad, m)
        {
            MotionState = motionState;
        }

        /// <summary>
        /// Constructs a nondynamic cone.
        /// </summary>
        /// <param name="motionState">Motion state specifying the entity's initial state.</param>
        /// <param name="high">Height of the cone.</param>
        /// <param name="rad">Radius of the cone.</param>
        public Cone(MotionState motionState, float high, float rad)
            : this(high, rad)
        {
            MotionState = motionState;
        }

 


    }
}