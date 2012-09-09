using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    /// <summary>
    /// Constrains an individual bone in an attempt to reach some goal.
    /// Controls act as groups of single bone constraints. They are used
    /// by the solver to determine the active set of body constraints.
    /// </summary>
    public abstract class Control
    {
        /// <summary>
        /// Gets or sets the controlled bone.
        /// </summary>
        public abstract Bone TargetBone { get; set; }

        /// <summary>
        /// Gets whether or not the control is used by an IK solver.
        /// </summary>
        public bool IsActive
        {
            get { return solverIndex >= 0; }
        }

        /// <summary>
        /// Stores where the control is in the solver listing for quick adds and removes.
        /// </summary>
        internal int solverIndex = -1;


        protected internal abstract void UpdateJacobiansAndVelocityBias();

        protected internal abstract void ComputeEffectiveMass();

        protected internal abstract void WarmStart();

        protected internal abstract void SolveVelocityIteration();

        protected internal abstract void ClearAccumulatedImpulses();

        protected internal abstract float MaximumImpulse { get; set; }
    }
}