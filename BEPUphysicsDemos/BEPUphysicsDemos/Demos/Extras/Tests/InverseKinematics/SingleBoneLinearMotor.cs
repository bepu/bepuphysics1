using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    public class SingleBoneLinearMotor : SingleBoneConstraint
    {        
        /// <summary>
        /// Gets or sets the target position to apply to the target bone.
        /// </summary>
        public Vector3 TargetPosition;

        protected internal override void UpdateJacobiansAndVelocityBias()
        {
            linearJacobian = Matrix3X3.Identity;
            angularJacobian = new Matrix3X3();

            //Error is in world space. It gets projected onto the jacobians later.
            Vector3 linearError;
            Vector3.Subtract(ref TargetPosition, ref TargetBone.Position, out linearError);
            Vector3 angularError = Vector3.Zero;
            ComputeVelocityBias(ref linearError, ref angularError);
        }

    }
}
