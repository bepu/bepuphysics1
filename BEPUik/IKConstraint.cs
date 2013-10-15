using System;

namespace BEPUik
{
    public abstract class IKConstraint
    {
        protected float softness;

        protected float errorCorrectionFactor;

        private float damping = 16;
        /// <summary>
        /// Gets or sets the damping coefficient of the constraint. Acts like the damping coefficient of a spring.
        /// </summary>
        public float Damping
        {
            get { return damping; }
            set { damping = Math.Max(0, value); }
        }


        private float stiffness = 4;
        /// <summary>
        /// Gets or sets the stiffness coefficient of the constraint. Acts like the stiffness coefficient of a spring.
        /// </summary>
        public float Stiffness
        {
            get { return stiffness; }
            set { stiffness = Math.Max(0, value); }
        }

        protected float maximumImpulse;
        protected float maximumImpulseSquared;
        protected float maximumForce = float.MaxValue;

        /// <summary>
        /// Gets or sets the maximum force that the constraint can apply.
        /// </summary>
        public float MaximumForce
        {
            get { return maximumForce; }
            set
            {
                maximumForce = Math.Max(value, 0);
            }
        }

        /// <summary>
        /// Updates the softness, bias factor, and maximum impulse based on the current time step.
        /// </summary>
        /// <param name="dt">Time step duration.</param>
        /// <param name="updateRate">Inverse time step duration.</param>
        protected internal void Preupdate(float dt, float updateRate)
        {
            if (stiffness == 0 && damping == 0)
                throw new InvalidOperationException("Constraints cannot have both 0 stiffness and 0 damping.");
            float multiplier = 1 / (dt * stiffness + damping);
            errorCorrectionFactor = stiffness * multiplier;
            softness = updateRate * multiplier;
            maximumImpulse = maximumForce * dt;
            maximumImpulseSquared = Math.Min(float.MaxValue, maximumImpulse * maximumImpulse);

        }

        /// <summary>
        /// Update the jacobians for the latest position and orientation bone states and store a velocity bias based on the error.
        /// </summary>
        protected internal abstract void UpdateJacobiansAndVelocityBias();

        /// <summary>
        /// Computes the effective mass matrix for the constraint for the current jacobians.
        /// </summary>
        protected internal abstract void ComputeEffectiveMass();

        /// <summary>
        /// Applies the accumulated impulse to warm up the constraint solving process.
        /// </summary>
        protected internal abstract void WarmStart();

        /// <summary>
        /// Applies impulses to satisfy the velocity constraint.
        /// </summary>
        protected internal abstract void SolveVelocityIteration();

        /// <summary>
        /// Clears out the accumulated impulse.
        /// </summary>
        protected internal abstract void ClearAccumulatedImpulses();
    }
}
