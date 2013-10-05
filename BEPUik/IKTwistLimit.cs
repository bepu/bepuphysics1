using System;
using BEPUutilities;

namespace BEPUik
{
    /// <summary>
    /// Prevents two bones from twisting beyond a certain angle away from each other as measured by the twist between two measurement axes.
    /// </summary>
    public class IKTwistLimit : IKLimit
    {
        /// <summary>
        /// Gets or sets the axis attached to ConnectionA in its local space.
        /// Must be unit length and perpendicular to LocalMeasurementAxisA.
        /// </summary>
        public Vector3 LocalAxisA;
        /// <summary>
        /// Gets or sets the axis attached to ConnectionB in its local space.
        /// Must be unit length and perpendicular to LocalMeasurementAxisB.
        /// </summary>
        public Vector3 LocalAxisB;

        /// <summary>
        /// Gets or sets the measurement axis attached to connection A.
        /// Must be unit length and perpendicular to LocalAxisA.
        /// </summary>
        public Vector3 LocalMeasurementAxisA;
        /// <summary>
        /// Gets or sets the measurement axis attached to connection B.
        /// Must be unit length and perpendicular to LocalAxisB.
        /// </summary>
        public Vector3 LocalMeasurementAxisB;

        /// <summary>
        /// Gets or sets the axis attached to ConnectionA in world space.
        /// Must be unit length and perpendicular to MeasurementAxisA.
        /// </summary>
        public Vector3 AxisA
        {
            get { return Quaternion.Transform(LocalAxisA, ConnectionA.Orientation); }
            set { LocalAxisA = Quaternion.Transform(value, Quaternion.Conjugate(ConnectionA.Orientation)); }
        }

        /// <summary>
        /// Gets or sets the axis attached to ConnectionB in world space.
        /// Must be unit length and perpendicular to MeasurementAxisB.
        /// </summary>
        public Vector3 AxisB
        {
            get { return Quaternion.Transform(LocalAxisB, ConnectionB.Orientation); }
            set { LocalAxisB = Quaternion.Transform(value, Quaternion.Conjugate(ConnectionB.Orientation)); }
        }

        /// <summary>
        /// Gets or sets the measurement axis attached to ConnectionA in world space.
        /// This axis is compared against the other connection's measurement axis to determine the twist.
        /// Must be unit length and perpendicular to AxisA.
        /// </summary>
        public Vector3 MeasurementAxisA
        {
            get { return Quaternion.Transform(LocalMeasurementAxisA, ConnectionA.Orientation); }
            set { LocalMeasurementAxisA = Quaternion.Transform(value, Quaternion.Conjugate(ConnectionA.Orientation)); }
        }

        /// <summary>
        /// Gets or sets the measurement axis attached to ConnectionB in world space.
        /// This axis is compared against the other connection's measurement axis to determine the twist.
        /// Must be unit length and perpendicular to AxisB.
        /// </summary>
        public Vector3 MeasurementAxisB
        {
            get { return Quaternion.Transform(LocalMeasurementAxisB, ConnectionB.Orientation); }
            set { LocalMeasurementAxisB = Quaternion.Transform(value, Quaternion.Conjugate(ConnectionB.Orientation)); }
        }

        private float maximumAngle;
        /// <summary>
        /// Gets or sets the maximum angle between the two axes allowed by the constraint.
        /// </summary>
        public float MaximumAngle
        {
            get { return maximumAngle; }
            set { maximumAngle = Math.Max(0, value); }
        }

        /// <summary>
        /// Automatically computes the measurement axes for the current local axes.
        /// The current relative state of the entities will be considered 0 twist angle.
        /// </summary>
        public void ComputeMeasurementAxes()
        {
            Vector3 axisA, axisB;
            Quaternion.Transform(ref LocalAxisA, ref ConnectionA.Orientation, out axisA);
            Quaternion.Transform(ref LocalAxisB, ref ConnectionB.Orientation, out axisB);
            //Pick an axis perpendicular to axisA to use as the measurement axis.
            Vector3 worldMeasurementAxisA;
            Vector3.Cross(ref Toolbox.UpVector, ref axisA, out worldMeasurementAxisA);
            float lengthSquared = worldMeasurementAxisA.LengthSquared();
            if (lengthSquared > Toolbox.Epsilon)
            {
                Vector3.Divide(ref worldMeasurementAxisA, (float)Math.Sqrt(lengthSquared), out worldMeasurementAxisA);
            }
            else
            {
                //Oops! It was parallel to the up vector. Just try again with the right vector.
                Vector3.Cross(ref Toolbox.RightVector, ref axisA, out worldMeasurementAxisA);
                worldMeasurementAxisA.Normalize();
            }
            //Attach the measurement axis to entity B.
            //'Push' A's axis onto B by taking into account the swing transform.
            Quaternion alignmentRotation;
            Quaternion.GetQuaternionBetweenNormalizedVectors(ref axisA, ref axisB, out alignmentRotation);
            Vector3 worldMeasurementAxisB;
            Quaternion.Transform(ref worldMeasurementAxisA, ref alignmentRotation, out worldMeasurementAxisB);
            //Plop them on!
            MeasurementAxisA = worldMeasurementAxisA;
            MeasurementAxisB = worldMeasurementAxisB;

        }


        /// <summary>
        /// Builds a new twist limit. Prevents two bones from rotating beyond a certain angle away from each other as measured by attaching an axis to each connected bone.
        /// </summary>
        /// <param name="connectionA">First connection of the limit.</param>
        /// <param name="connectionB">Second connection of the limit.</param>
        /// <param name="axisA">Axis attached to connectionA in world space.</param>
        /// <param name="axisB">Axis attached to connectionB in world space.</param>
        /// <param name="maximumAngle">Maximum angle allowed between connectionA's axis and connectionB's axis.</param>
        public IKTwistLimit(Bone connectionA, Bone connectionB, Vector3 axisA, Vector3 axisB, float maximumAngle)
            : base(connectionA, connectionB)
        {
            AxisA = axisA;
            AxisB = axisB;
            MaximumAngle = maximumAngle;

            ComputeMeasurementAxes();
        }

        protected internal override void UpdateJacobiansAndVelocityBias()
        {

            //This constraint doesn't consider linear motion.
            linearJacobianA = linearJacobianB = new Matrix3x3();

            //Compute the world axes.
            Vector3 axisA, axisB;
            Quaternion.Transform(ref LocalAxisA, ref ConnectionA.Orientation, out axisA);
            Quaternion.Transform(ref LocalAxisB, ref ConnectionB.Orientation, out axisB);

            Vector3 twistMeasureAxisA, twistMeasureAxisB;
            Quaternion.Transform(ref LocalMeasurementAxisA, ref ConnectionA.Orientation, out twistMeasureAxisA);
            Quaternion.Transform(ref LocalMeasurementAxisB, ref ConnectionB.Orientation, out twistMeasureAxisB);

            //Compute the shortest rotation to bring axisB into alignment with axisA.
            Quaternion alignmentRotation;
            Quaternion.GetQuaternionBetweenNormalizedVectors(ref axisB, ref axisA, out alignmentRotation);

            //Transform the measurement axis on B by the alignment quaternion.
            Quaternion.Transform(ref twistMeasureAxisB, ref alignmentRotation, out twistMeasureAxisB);

            //We can now compare the angle between the twist axes.
            float angle;
            Vector3.Dot(ref twistMeasureAxisA, ref twistMeasureAxisB, out angle);
            angle = (float)Math.Acos(MathHelper.Clamp(angle, -1, 1));

            //Compute the bias based upon the error.
            if (angle > maximumAngle)
                velocityBias = new Vector3(errorCorrectionFactor * (angle - maximumAngle), 0, 0);
            else //If the constraint isn't violated, set up the velocity bias to allow a 'speculative' limit.
                velocityBias = new Vector3(angle - maximumAngle, 0, 0);

            //We can't just use the axes directly as jacobians. Consider 'cranking' one object around the other.
            Vector3 jacobian;
            Vector3.Add(ref axisA, ref axisB, out jacobian);
            float lengthSquared = jacobian.LengthSquared();
            if (lengthSquared > Toolbox.Epsilon)
            {
                Vector3.Divide(ref jacobian, (float)Math.Sqrt(lengthSquared), out jacobian);
            }
            else
            {
                //The constraint is in an invalid configuration. Just ignore it.
                jacobian = new Vector3();
            }

            //In addition to the absolute angle value, we need to know which side of the limit we're hitting.
            //The jacobian will be negated on one side. This is because limits can only 'push' in one direction;
            //if we didn't flip the direction of the jacobian, it would be trying to push the same direction on both ends of the limit.
            //One side would end up doing nothing!
            Vector3 cross;
            Vector3.Cross(ref twistMeasureAxisA, ref twistMeasureAxisB, out cross);
            float limitSide;
            Vector3.Dot(ref cross, ref axisA, out limitSide);
            //Negate the jacobian based on what side of the limit we're on.
            if (limitSide < 0)
                Vector3.Negate(ref jacobian, out jacobian);

            angularJacobianA = new Matrix3x3 { M11 = jacobian.X, M12 = jacobian.Y, M13 = jacobian.Z };
            angularJacobianB = new Matrix3x3 { M11 = -jacobian.X, M12 = -jacobian.Y, M13 = -jacobian.Z };




        }
    }
}
