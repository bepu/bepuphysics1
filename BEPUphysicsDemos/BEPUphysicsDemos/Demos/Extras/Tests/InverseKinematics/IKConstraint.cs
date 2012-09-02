using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    public abstract class IKConstraint
    {
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
