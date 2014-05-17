using System;
using BEPUutilities;

namespace BEPUik
{
    public class SingleBoneRevoluteConstraint : SingleBoneConstraint
    {
        private Vector3 freeAxis;
        private Vector3 constrainedAxis1;
        private Vector3 constrainedAxis2;

        /// <summary>
        /// Gets or sets the direction to constrain the bone free axis to.
        /// </summary>
        public Vector3 FreeAxis
        {
            get { return freeAxis; }
            set
            {
                freeAxis = value;
                constrainedAxis1 = Vector3.Cross(freeAxis, Vector3.Up);
                if (constrainedAxis1.LengthSquared() < Toolbox.Epsilon)
                {
                    constrainedAxis1 = Vector3.Cross(freeAxis, Vector3.Right);
                }
                constrainedAxis1.Normalize();
                constrainedAxis2 = Vector3.Cross(freeAxis, constrainedAxis1);
            }
        }


        /// <summary>
        /// Axis of allowed rotation in the bone's local space.
        /// </summary>
        public Vector3 BoneLocalFreeAxis;

        protected internal override void UpdateJacobiansAndVelocityBias()
        {
 

            linearJacobian = new Matrix3x3();

            Vector3 boneAxis;
            Quaternion.Transform(ref BoneLocalFreeAxis, ref TargetBone.Orientation, out boneAxis);


            angularJacobian = new Matrix3x3
            {
                M11 = constrainedAxis1.X,
                M12 = constrainedAxis1.Y,
                M13 = constrainedAxis1.Z,
                M21 = constrainedAxis2.X,
                M22 = constrainedAxis2.Y,
                M23 = constrainedAxis2.Z
            };


            Vector3 error;
            Vector3.Cross(ref boneAxis, ref freeAxis, out error);
            Vector2 constraintSpaceError;
            Vector3.Dot(ref error, ref constrainedAxis1, out constraintSpaceError.X);
            Vector3.Dot(ref error, ref constrainedAxis2, out constraintSpaceError.Y);
            velocityBias.X = errorCorrectionFactor * constraintSpaceError.X;
            velocityBias.Y = errorCorrectionFactor * constraintSpaceError.Y;


        }


    }
}
