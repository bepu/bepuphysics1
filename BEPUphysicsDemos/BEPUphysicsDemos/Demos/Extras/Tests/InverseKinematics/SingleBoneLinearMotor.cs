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

        /// <summary>
        /// Gets or sets the offset in the bone's local space to the point which will be pulled towards the target position.
        /// </summary>
        public Vector3 LocalOffset;


        public Vector3 Offset
        {
            get { return Vector3.Transform(LocalOffset, TargetBone.Orientation); }
            set { LocalOffset = Vector3.Transform(value, Quaternion.Conjugate(TargetBone.Orientation)); }
        }

        protected internal override void UpdateJacobiansAndVelocityBias()
        {
            linearJacobian = Matrix3X3.Identity;
            Vector3 r;
            Vector3.Transform(ref LocalOffset, ref TargetBone.Orientation, out r);
            Matrix3X3.CreateCrossProduct(ref r, out angularJacobian);

            Vector3 worldPosition;
            Vector3.Add(ref TargetBone.Position, ref r, out worldPosition);

            //Error is in world space. It gets projected onto the jacobians later.
            Vector3 linearError;
            Vector3.Subtract(ref TargetPosition, ref worldPosition, out linearError);
            Vector3 angularError = Vector3.Zero;
            ComputeVelocityBias(ref linearError, ref angularError);
        }


    }
}
