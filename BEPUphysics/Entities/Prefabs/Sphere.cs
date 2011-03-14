using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.EntityStateManagement;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionShapes.ConvexShapes;

namespace BEPUphysics.Entities.Prefabs
{
    /// <summary>
    /// Ball-shaped object that can collide and move.  After making an entity, add it to a Space so that the engine can manage it.
    /// </summary>
    public class Sphere : Entity<ConvexCollidable<SphereShape>>
    {
        /// <summary>
        /// Radius of the sphere.
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

        private Sphere(float radius)
            :base(new ConvexCollidable<SphereShape>(new SphereShape(radius)))
        {
        }

        private Sphere(float radius, float mass)
            :base(new ConvexCollidable<SphereShape>(new SphereShape(radius)), mass)
        {
        }



        /// <summary>
        /// Constructs a physically simulated sphere.
        /// </summary>
        /// <param name="pos">Position of the sphere.</param>
        /// <param name="rad">Radius of the sphere.</param>
        /// <param name="m">Mass of the object.</param>
        public Sphere(Vector3 pos, float rad, float m)
            : this(rad, m)
        {
            Position = pos;
        }

        /// <summary>
        /// Constructs a nondynamic sphere.
        /// </summary>
        /// <param name="pos">Position of the sphere.</param>
        /// <param name="rad">Radius of the sphere.</param>
        public Sphere(Vector3 pos, float rad)
            : this(rad)
        {
            Position = pos;
        }

        /// <summary>
        /// Constructs a physically simulated sphere.
        /// </summary>
        /// <param name="motionState">Motion state specifying the entity's initial state.</param>
        /// <param name="rad">Radius of the sphere.</param>
        /// <param name="m">Mass of the object.</param>
        public Sphere(MotionState motionState, float rad, float m)
            : this(rad, m)
        {
            MotionState = motionState;
        }

        /// <summary>
        /// Constructs a nondynamic sphere.
        /// </summary>
        /// <param name="motionState">Motion state specifying the entity's initial state.</param>
        /// <param name="rad">Radius of the sphere.</param>
        public Sphere(MotionState motionState, float rad)
            : this(rad)
        {
            MotionState = motionState;
        }



    }
}