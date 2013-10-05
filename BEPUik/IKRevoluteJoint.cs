using System;
using BEPUutilities;

namespace BEPUik
{
    public class IKRevoluteJoint : IKJoint
    {
        private Vector3 localFreeAxisA;
        /// <summary>
        /// Gets or sets the free axis in connection A's local space.
        /// Must be unit length.
        /// </summary>
        public Vector3 LocalFreeAxisA
        {
            get { return localFreeAxisA; }
            set
            {
                localFreeAxisA = value;
                ComputeConstrainedAxes();
            }
        }

        private Vector3 localFreeAxisB;
        /// <summary>
        /// Gets or sets the free axis in connection B's local space.
        /// Must be unit length.
        /// </summary>
        public Vector3 LocalFreeAxisB
        {
            get { return localFreeAxisB; }
            set
            {
                localFreeAxisB = value;
                ComputeConstrainedAxes();
            }
        }



        /// <summary>
        /// Gets or sets the free axis attached to connection A in world space.
        /// This does not change the other connection's free axis.
        /// </summary>
        public Vector3 WorldFreeAxisA
        {
            get { return Quaternion.Transform(localFreeAxisA, ConnectionA.Orientation); }
            set
            {
                LocalFreeAxisA = Quaternion.Transform(value, Quaternion.Conjugate(ConnectionA.Orientation));
            }
        }

        /// <summary>
        /// Gets or sets the free axis attached to connection B in world space.
        /// This does not change the other connection's free axis.
        /// </summary>
        public Vector3 WorldFreeAxisB
        {
            get { return Quaternion.Transform(localFreeAxisB, ConnectionB.Orientation); }
            set
            {
                LocalFreeAxisB = Quaternion.Transform(value, Quaternion.Conjugate(ConnectionB.Orientation));
            }
        }

        private Vector3 localConstrainedAxis1, localConstrainedAxis2;
        void ComputeConstrainedAxes()
        {
            Vector3 worldAxisA = WorldFreeAxisA;
            Vector3 error = Vector3.Cross(worldAxisA, WorldFreeAxisB);
            float lengthSquared = error.LengthSquared();
            Vector3 worldConstrainedAxis1, worldConstrainedAxis2;
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

            localConstrainedAxis1 = Quaternion.Transform(worldConstrainedAxis1, Quaternion.Conjugate(ConnectionA.Orientation));
            localConstrainedAxis2 = Quaternion.Transform(worldConstrainedAxis2, Quaternion.Conjugate(ConnectionA.Orientation));
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
            linearJacobianA = linearJacobianB = new Matrix3x3();

            //We know the one free axis. We need the two restricted axes. This amounts to completing the orthonormal basis.
            //We can grab one of the restricted axes using a cross product of the two world axes. This is not guaranteed
            //to be nonzero, so the normalization requires protection.

            Vector3 worldAxisA, worldAxisB;
            Quaternion.Transform(ref localFreeAxisA, ref ConnectionA.Orientation, out worldAxisA);
            Quaternion.Transform(ref localFreeAxisB, ref ConnectionB.Orientation, out worldAxisB);

            Vector3 error;
            Vector3.Cross(ref worldAxisA, ref worldAxisB, out error);

            Vector3 worldConstrainedAxis1, worldConstrainedAxis2;
            Quaternion.Transform(ref localConstrainedAxis1, ref ConnectionA.Orientation, out worldConstrainedAxis1);
            Quaternion.Transform(ref localConstrainedAxis2, ref ConnectionA.Orientation, out worldConstrainedAxis2);


            angularJacobianA = new Matrix3x3
            {
                M11 = worldConstrainedAxis1.X,
                M12 = worldConstrainedAxis1.Y,
                M13 = worldConstrainedAxis1.Z,
                M21 = worldConstrainedAxis2.X,
                M22 = worldConstrainedAxis2.Y,
                M23 = worldConstrainedAxis2.Z
            };
            Matrix3x3.Negate(ref angularJacobianA, out angularJacobianB);


            Vector2 constraintSpaceError;
            Vector3.Dot(ref error, ref worldConstrainedAxis1, out constraintSpaceError.X);
            Vector3.Dot(ref error, ref worldConstrainedAxis2, out constraintSpaceError.Y);
            velocityBias.X = errorCorrectionFactor * constraintSpaceError.X;
            velocityBias.Y = errorCorrectionFactor * constraintSpaceError.Y;


        }
    }
}
