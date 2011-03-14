using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.EntityStateManagement;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionShapes.ConvexShapes;

namespace BEPUphysics.Entities.Prefabs
{
    /// <summary>
    /// Cylinder-shaped object that can collide and move.  After making an entity, add it to a Space so that the engine can manage it.
    /// </summary>
    public class Cylinder : Entity<ConvexCollidable<CylinderShape>>
    {
        /// <summary>
        /// Gets or sets the height of the cylinder.
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
        /// Gets or sets the radius of the cylinder.
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


        private Cylinder(float high, float rad, float m)
            : base(new ConvexCollidable<CylinderShape>(new CylinderShape(high, rad)), m)
        {
        }

        private Cylinder(float high, float rad)
            : base(new ConvexCollidable<CylinderShape>(new CylinderShape(high, rad)))
        {
        }

        /// <summary>
        /// Constructs a physically simulated cylinder.
        /// </summary>
        /// <param name="pos">Position of the cylinder.</param>
        /// <param name="high">Height of the cylinder.</param>
        /// <param name="rad">Radius of the cylinder.</param>
        /// <param name="m">Mass of the object.</param>
        public Cylinder(Vector3 pos, float high, float rad, float m)
            : this(high, rad, m)
        {
            Position = pos;
        }

        /// <summary>
        /// Constructs a nondynamic cylinder.
        /// </summary>
        /// <param name="pos">Position of the cylinder.</param>
        /// <param name="high">Height of the cylinder.</param>
        /// <param name="rad">Radius of the cylinder.</param>
        public Cylinder(Vector3 pos, float high, float rad)
            : this(high, rad)
        {
            Position = pos;
        }

        /// <summary>
        /// Constructs a physically simulated cylinder.
        /// </summary>
        /// <param name="motionState">Motion state specifying the entity's initial state.</param>
        /// <param name="high">Height of the cylinder.</param>
        /// <param name="rad">Radius of the cylinder.</param>
        /// <param name="m">Mass of the object.</param>
        public Cylinder(MotionState motionState, float high, float rad, float m)
            : this(high, rad, m)
        {
            MotionState = motionState;
        }

        /// <summary>
        /// Constructs a nondynamic cylinder.
        /// </summary>
        /// <param name="motionState">Motion state specifying the entity's initial state.</param>
        /// <param name="high">Height of the cylinder.</param>
        /// <param name="rad">Radius of the cylinder.</param>
        public Cylinder(MotionState motionState, float high, float rad)
            : this(high, rad)
        {
            MotionState = motionState;
        }




    }
}