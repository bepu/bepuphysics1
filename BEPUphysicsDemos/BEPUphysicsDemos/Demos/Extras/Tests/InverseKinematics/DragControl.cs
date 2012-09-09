using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    /// <summary>
    /// Constrains an individual bone in an attempt to reach some position goal.
    /// </summary>
    public class DragControl : Control
    {
        /// <summary>
        /// Gets or sets the controlled bone.
        /// </summary>
        public override Bone TargetBone
        {
            get { return LinearMotor.TargetBone; }
            set
            {
                LinearMotor.TargetBone = value;
            }
        }

        /// <summary>
        /// Gets or sets the linear motor used by the control.
        /// </summary>
        public SingleBoneLinearMotor LinearMotor
        {
            get;
            private set;
        }

        public DragControl()
        {
            LinearMotor = new SingleBoneLinearMotor();
            LinearMotor.Softness = .2f;
        }

        protected internal override void UpdateJacobiansAndVelocityBias()
        {
            LinearMotor.UpdateJacobiansAndVelocityBias();
        }

        protected internal override void ComputeEffectiveMass()
        {
            LinearMotor.ComputeEffectiveMass();
        }

        protected internal override void WarmStart()
        {
            LinearMotor.WarmStart();
        }

        protected internal override void SolveVelocityIteration()
        {
            LinearMotor.SolveVelocityIteration();
        }

        protected internal override void ClearAccumulatedImpulses()
        {
            LinearMotor.ClearAccumulatedImpulses();
        }

        protected internal override float MaximumImpulse
        {
            get { return LinearMotor.MaximumImpulse; }
            set
            {
                LinearMotor.MaximumImpulse = value;
            }
        }
    }
}