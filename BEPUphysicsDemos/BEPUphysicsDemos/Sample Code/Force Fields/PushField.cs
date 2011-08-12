using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Entities;
using BEPUphysics.UpdateableSystems.ForceFields;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.SampleCode
{
    /// <summary>
    /// Simple force field which applies a constant force to contained entities.
    /// </summary>
    public class PushField : ForceField
    {
        private Vector3 forceDirection;
        private float forceMagnitude;
        private Vector3 myForce;

        /// <summary>
        /// Creates a simple, constant force field.
        /// </summary>
        /// <param name="shape">Shape representing the volume of the force field.</param>
        /// <param name="forceToApply">Force to apply to entities within the field.  Magnitude of the vector represents the magnitude of the force.</param>
        /// <param name="maxPushSpeed">Maximum speed that the field will accelerate objects to, regardless of force applied. Set to a non-positive for infinite.</param>
        public PushField(ForceFieldShape shape, Vector3 forceToApply, float maxPushSpeed, IQueryAccelerator accelerator)
            : base(shape, accelerator)
        {
            Force = forceToApply;
            MaximumPushSpeed = maxPushSpeed;
        }

        /// <summary>
        /// Gets or sets the force to apply to entities within the field.  Magnitude of the vector represents the magnitude of the force.
        /// </summary>
        public Vector3 Force
        {
            get { return myForce; }
            set
            {
                myForce = value;
                forceMagnitude = myForce.Length();
                forceDirection = myForce / forceMagnitude;
            }
        }

        /// <summary>
        /// Gets or sets the maximum speed that the field will accelerate objects to, regardless of force applied.  Set to a non-positive value for infinite.
        /// </summary>
        public float MaximumPushSpeed { get; set; }

        /// <summary>
        /// Calculates the impulse to apply to the center of mass of physically simulated bodies within the field.
        /// </summary>
        /// <param name="e">Target of the impulse.</param>
        /// <param name="dt">Time since the last frame in simulation seconds.</param>
        /// <param name="impulse">Force to apply at the given position.</param>
        protected override void CalculateImpulse(Entity e, float dt, out Vector3 impulse)
        {
            if (MaximumPushSpeed > 0)
            {
                //Current velocity along the tangent direction.
                float dot = Vector3.Dot(e.LinearVelocity, forceDirection);
                //Compute the velocity difference between the current and the maximum
                dot = MaximumPushSpeed - dot;
                //Compute the force needed to reach the maximum, but clamp it to the amount of force that the field can apply
                //Also, don't apply a force that would slow an object down.
                dot = MathHelper.Clamp(dot * e.Mass, 0, forceMagnitude * dt);
                Vector3.Multiply(ref forceDirection, dot, out impulse);
            }
            else
                impulse = Force * dt;
        }
    }
}