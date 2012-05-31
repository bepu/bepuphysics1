using BEPUphysics.Entities;
using BEPUphysics.UpdateableSystems;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.SampleCode
{
    /// <summary>
    /// Simple example updateable that acts like a rocket engine.
    /// </summary>
    public class Thruster : Updateable, IDuringForcesUpdateable
    {
        private float myAge;
        private float myLifeSpan;

        /// <summary>
        /// Constructs a thruster originating at the given position, pushing in the given direction.
        /// </summary>
        /// <param name="targetEntity">Entity that the force will be applied to.</param>
        /// <param name="pos">Origin of the force.</param>
        /// <param name="dir">Direction of the force.</param>
        /// <param name="time">Total lifespan of the force.  A lifespan of zero is infinite.</param>
        public Thruster(Entity targetEntity, Vector3 pos, Vector3 dir, float time)
        {
            Target = targetEntity;
            Position = pos;
            Direction = dir;
            LifeSpan = time;
        }

        /// <summary>
        /// Gets or sets the position of the thruster in the local space of the target entity.
        /// </summary>
        public Vector3 Position { get; set; }

        /// <summary>
        /// Gets or sets the direction of the force in the local space of the target entity.  Magnitude of the force is equal to the magnitude of the direction.
        /// </summary>
        public Vector3 Direction { get; set; }

        /// <summary>
        /// Gets or sets the entity to apply force to.
        /// </summary>
        public Entity Target { get; set; }

        /// <summary>
        /// Gets or sets the length of time that the thruster has been firing.
        /// This can be reset to 'refresh' the life of the force.
        /// </summary>
        public float Age
        {
            get { return myAge; }
            set
            {
                myAge = value;
                if (myAge < LifeSpan)
                    IsUpdating = true; //IsUpdating is a property of the Updateable class.  The updateDuringForces method won't be called if IsUpdating is false.
            }
        }

        /// <summary>
        /// Maximum life span of the force, after which the thruster will deactivate.
        /// Set to 0 for infinite lifespan.
        /// </summary>
        public float LifeSpan
        {
            get { return myLifeSpan; }
            set
            {
                myLifeSpan = value;
                if (myLifeSpan > Age || myLifeSpan == 0)
                    IsUpdating = true; //Wake the thruster up if it's young again.
            }
        }


        /// <summary>
        /// Applies the thruster forces.
        /// Called automatically by the owning space during a space update.
        /// </summary>
        /// <param name="dt">Simulation timestep.</param>
        void IDuringForcesUpdateable.Update(float dt)
        {
            //Transform the local position and direction into world space.
            Vector3 worldPosition = Target.Position + Matrix3X3.Transform(Position, Target.OrientationMatrix);
            Vector3 worldDirection = Matrix3X3.Transform(Direction, Target.OrientationMatrix);
            //Apply the force.
            Target.ApplyImpulse(worldPosition, worldDirection * dt);


            Age += dt;
            if (LifeSpan > 0 && Age > LifeSpan)
            {
                IsUpdating = false; //The thruster has finished firing.
            }
        }
    }
}