using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    public abstract class IKConstraint
    {
        protected float softness = .05f;
        /// <summary>
        /// Gets or sets the softness of the constraint. The higher the softness is, the more the constraint can be violated. Must be nonnegative; 0 corresponds to complete rigidity.
        /// </summary>
        public float Softness
        {
            get { return softness; }
            set { softness = MathHelper.Max(value, 0); }
        }

        protected float errorCorrectionFactor = .2f;
        /// <summary>
        /// Gets or sets the error correction factor of the constraint. Values range from 0 to 1. 0 means the constraint will not attempt to correct any error.
        /// 1 means the constraint will attempt to correct all error in a single iteration. This factor, combined with Softness, define the springlike behavior of a constraint.
        /// </summary>
        public float ErrorCorrectionFactor
        {
            get { return errorCorrectionFactor; }
            set { errorCorrectionFactor = MathHelper.Clamp(value, 0, 1); }
        }

        protected float maximumImpulse = float.MaxValue;
        protected float maximumImpulseSquared = float.MaxValue;
        /// <summary>
        /// Gets or sets the maximum impulse that the constraint can apply.
        /// Velocity error requiring a greater impulse will result in the impulse being clamped to the maximum impulse.
        /// </summary>
        public float MaximumImpulse
        {
            get { return (float)Math.Sqrt(maximumImpulseSquared); }
            set
            {
                maximumImpulse = Math.Max(value, 0);
                if (maximumImpulse >= float.MaxValue)
                    maximumImpulseSquared = float.MaxValue;
                else
                    maximumImpulseSquared = maximumImpulse * maximumImpulse;
            }
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
