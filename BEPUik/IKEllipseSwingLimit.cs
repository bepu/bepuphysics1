using System;
using BEPUutilities;

namespace BEPUik
{
    /// <summary>
    /// Prevents two bones from rotating beyond a certain angle away from each other as measured by attaching an axis to each connected bone.
    /// </summary>
    public class IKEllipseSwingLimit : IKLimit
    {
        /// <summary>
        /// Gets or sets the axis attached to ConnectionA in its local space.
        /// </summary>
        public Vector3 LocalAxisA;
        /// <summary>
        /// Gets or sets the axis attached to ConnectionB in its local space.
        /// </summary>
        public Vector3 LocalAxisB;

        /// <summary>
        /// Gets or sets the axis attached to ConnectionA in world space.
        /// </summary>
        public Vector3 AxisA
        {
            get { return Quaternion.Transform(LocalAxisA, ConnectionA.Orientation); }
            set { LocalAxisA = Quaternion.Transform(value, Quaternion.Conjugate(ConnectionA.Orientation)); }
        }

        /// <summary>
        ///  Gets or sets the axis attached to ConnectionB in world space.
        /// </summary>
        public Vector3 AxisB
        {
            get { return Quaternion.Transform(LocalAxisB, ConnectionB.Orientation); }
            set { LocalAxisB = Quaternion.Transform(value, Quaternion.Conjugate(ConnectionB.Orientation)); }
        }

        private float maximumAngleX;
        /// <summary>
        /// Gets or sets the maximum x angle between the two axes allowed by the constraint.
        /// </summary>
        public float MaximumAngleX
        {
            get { return maximumAngleX; }
            set { maximumAngleX = Math.Max(0, value); }
        }

        private float maximumAngleY;
        /// <summary>
        /// Gets or sets the maximum y angle between the two axes allowed by the constraint.
        /// </summary>
        public float MaximumAngleY
        {
            get { return maximumAngleY; }
            set { maximumAngleY = Math.Max(0, value); }
        }

        private Vector3 axisX;
        private Vector3 axisY;

        /// <summary>
        /// Builds a new swing limit. Prevents two bones from rotating beyond a certain angle away from each other as measured by attaching an axis to each connected bone.
        /// </summary>
        /// <param name="connectionA">First connection of the limit.</param>
        /// <param name="connectionB">Second connection of the limit.</param>
        /// <param name="axisA">Axis attached to connectionA in world space.</param>
        /// <param name="axisB">Axis attached to connectionB in world space.</param>
        /// <param name="maximumAngleX">Maximum x angle allowed between connectionA's axis and connectionB's axis.</param>
        /// <param name="maximumAngleY">Maximum y angle allowed between connectionA's axis and connectionB's axis.</param>
        public IKEllipseSwingLimit(Bone connectionA, Bone connectionB, Vector3 axisA, Vector3 axisB, float maximumAngleX, float maximumAngleY)
            : base(connectionA, connectionB)
        {
            AxisA = axisA;
            AxisB = axisB;
            MaximumAngleX = maximumAngleX;
            MaximumAngleY = maximumAngleY;

            SetupJointTransforms(axisA);
        }

        /// <summary>
        /// Sets up the joint transform axes by automatically creating perpendicular vectors to complete the bases.
        /// </summary>
        /// <param name="twistAxis">Axis around which rotation is allowed.</param>
        private void SetupJointTransforms(Vector3 twistAxis)
        {
            //Compute a vector which is perpendicular to the axis.  It'll be added in local space to both connections.
            Vector3.Cross(ref twistAxis, ref Toolbox.UpVector, out axisX);
            float length = axisX.LengthSquared();
            if (length < Toolbox.Epsilon)
            {
                Vector3.Cross(ref twistAxis, ref Toolbox.RightVector, out axisX);
            }

            Vector3.Cross(ref twistAxis, ref axisX, out axisY);

            axisX = Vector3.Normalize(axisX);
            axisY = Vector3.Normalize(axisY);
            Quaternion.Transform(axisX, Quaternion.Inverse(ConnectionA.Orientation));
            Quaternion.Transform(axisY, Quaternion.Inverse(ConnectionA.Orientation));
        }

        protected internal override void UpdateJacobiansAndVelocityBias()
        {

            //This constraint doesn't consider linear motion.
            linearJacobianA = linearJacobianB = new Matrix3x3();

            //Compute the world axes.
            Vector3 axisA, axisB;
            Quaternion.Transform(ref LocalAxisA, ref ConnectionA.Orientation, out axisA);
            Quaternion.Transform(ref LocalAxisB, ref ConnectionB.Orientation, out axisB);

            Quaternion relativeRotation;
            Quaternion.GetQuaternionBetweenNormalizedVectors(ref axisB, ref axisA, out relativeRotation);

            float angle;
            Vector3 axis;
            Quaternion.GetAxisAngleFromQuaternion(ref relativeRotation, out axis, out angle);

            Vector3 axisAngle = new Vector3();
            //This combined axis-angle representation is similar to angular velocity in describing a rotation.
            //Just like you can dot an axis with angular velocity to get a velocity around that axis,
            //dotting an axis with the axis-angle representation gets the angle of rotation around that axis.
            //(As far as the constraint is concerned, anyway.)
            axisAngle.X = axis.X * angle;
            axisAngle.Y = axis.Y * angle;
            axisAngle.Z = axis.Z * angle;

            Vector3 basisXAxis = axisX;
            basisXAxis = Quaternion.Transform(basisXAxis, ConnectionA.Orientation);

            Vector3 basisYAxis = axisY;
            basisYAxis = Quaternion.Transform(basisYAxis, ConnectionA.Orientation);

            //Compute the individual swing angles.
            float angleX = Vector3.Dot(axisAngle, basisXAxis);
            float angleY = Vector3.Dot(axisAngle, basisYAxis);

            //The position constraint states that the angles must be within an ellipse. The following is just a reorganization of the x^2 / a^2 + y^2 / b^2 <= 1 definition of an ellipse's area.
            float maxAngleXSquared = maximumAngleX * maximumAngleX;
            float maxAngleYSquared = maximumAngleY * maximumAngleY;
            float error = angleX * angleX * maxAngleYSquared + angleY * angleY * maxAngleXSquared - maxAngleXSquared * maxAngleYSquared;

            //One angular DOF is constrained by this limit.
            Vector3 hingeAxis;
            Vector3.Cross(ref axisA, ref axisB, out hingeAxis);

            angularJacobianA = new Matrix3x3 { M11 = hingeAxis.X, M12 = hingeAxis.Y, M13 = hingeAxis.Z };
            angularJacobianB = new Matrix3x3 { M11 = -hingeAxis.X, M12 = -hingeAxis.Y, M13 = -hingeAxis.Z };

            //Note how we've computed the jacobians despite the limit being potentially inactive.
            //This is to enable 'speculative' limits.
            if (error >= 0.0)
            {
                velocityBias = new Vector3(errorCorrectionFactor * error, 0, 0);
            }
            else
            {
                //The constraint is not yet violated. But, it may be- allow only as much motion as could occur without violating the constraint.
                //Limits can't 'pull,' so this will not result in erroneous sticking.
                velocityBias = new Vector3(error, 0, 0);
            }


        }
    }
}
