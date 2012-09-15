using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    public class IKRevoluteJoint : IKJoint
    {
        /// <summary>
        /// Gets or sets the free axis in connection A's local space.
        /// Updates the internal restricted axes.
        /// </summary>
        public Vector3 LocalFreeAxisA;
        /// <summary>
        /// Gets or sets the free axis in connection B's local space.
        /// Updates the internal restricted axes.
        /// </summary>
        public Vector3 LocalFreeAxisB;


        /// <summary>
        /// Gets or sets the free axis attached to connection A in world space.
        /// This does not change the other connection's free axis.
        /// Updates the internal restricted axes.
        /// </summary>
        public Vector3 WorldFreeAxisA
        {
            get { return LocalFreeAxisA; }
            set
            {
                LocalFreeAxisA = Vector3.Transform(value, Quaternion.Conjugate(ConnectionA.Orientation));
            }
        }

        /// <summary>
        /// Gets or sets the free axis attached to connection B in world space.
        /// This does not change the other connection's free axis.
        /// </summary>
        public Vector3 WorldFreeAxisB
        {
            get { return LocalFreeAxisB; }
            set
            {
                LocalFreeAxisB = Vector3.Transform(value, Quaternion.Conjugate(ConnectionB.Orientation));
            }
        }

        /// <summary>
        /// Constructs a new orientation joint.
        /// Orientation joints can be used to simulate the angular portion of a hinge.
        /// Orientation joints allow rotation around only a single axis.
        /// </summary>
        /// <param name="connectionA">First entity connected in the orientation joint.</param>
        /// <param name="connectionB">Second entity connected in the orientation joint.</param>
        /// <param name="freeAxis">Axis allowed to rotate freely in world space.</param>
        public IKRevoluteJoint(Bone connectionA, Bone connectionB, Vector3 freeAxis)
            : base(connectionA, connectionB)
        {
            WorldFreeAxisA = freeAxis;
            WorldFreeAxisB = freeAxis;
        }

        protected internal override void UpdateJacobiansAndVelocityBias()
        {
            linearJacobianA = linearJacobianB = new Matrix3X3();

            //While we could technically treat this as a 1DOF 'hinge' like the swing limit,
            //better robustness is achieved by using a 2DOF restriction.

            //We know the one free axis. We need the two restricted axes. This amounts to completing the orthonormal basis.
            //We can grab one of the restricted axes using a cross product of the two world axes. This is not guaranteed
            //to be nonzero, so the normalization requires protection.

            Vector3 worldAxisA, worldAxisB;
            Vector3.Transform(ref LocalFreeAxisA, ref ConnectionA.Orientation, out worldAxisA);
            Vector3.Transform(ref LocalFreeAxisB, ref ConnectionB.Orientation, out worldAxisB);

            Vector3 error;
            Vector3.Cross(ref worldAxisA, ref worldAxisB, out error);

            Vector3 worldConstrainedAxis1, worldConstrainedAxis2;
            float lengthSquared = error.LengthSquared();
            //Find the first constrained axis.
            if (lengthSquared > Toolbox.Epsilon)
            {
                //The error direction can be used as the first axis!
                Vector3.Divide(ref error, (float)Math.Sqrt(lengthSquared), out worldConstrainedAxis1);
            }
            else
            {
                //There's not enough error for it to be a good constrained axis.
                //We'll need to create the constrained axes arbitrarily.
                Vector3.Cross(ref Toolbox.UpVector, ref worldAxisA, out worldConstrainedAxis1);
                lengthSquared = worldConstrainedAxis1.LengthSquared();
                if (lengthSquared > Toolbox.Epsilon)
                {
                    //The up vector worked!
                    Vector3.Divide(ref worldConstrainedAxis1, (float)Math.Sqrt(lengthSquared), out worldConstrainedAxis1);
                }
                else
                {
                    //The up vector didn't work. Just try the right vector.
                    Vector3.Cross(ref Toolbox.RightVector, ref worldAxisA, out worldConstrainedAxis1);
                    worldConstrainedAxis1.Normalize();
                }
            }
            //Don't have to normalize the second constraint axis; it's the cross product of two perpendicular normalized vectors.
            Vector3.Cross(ref worldAxisA, ref worldConstrainedAxis1, out worldConstrainedAxis2);

            angularJacobianA = new Matrix3X3
            {
                M11 = worldConstrainedAxis1.X,
                M12 = worldConstrainedAxis1.Y,
                M13 = worldConstrainedAxis1.Z,
                M21 = worldConstrainedAxis2.X,
                M22 = worldConstrainedAxis2.Y,
                M23 = worldConstrainedAxis2.Z
            };
            Matrix3X3.Negate(ref angularJacobianA, out angularJacobianB);


            Vector2 constraintSpaceError;
            Vector3.Dot(ref error, ref worldConstrainedAxis1, out constraintSpaceError.X);
            Vector3.Dot(ref error, ref worldConstrainedAxis2, out constraintSpaceError.Y);
            velocityBias.X = errorCorrectionFactor * constraintSpaceError.X;
            velocityBias.Y = errorCorrectionFactor * constraintSpaceError.Y;


        }
    }
}
