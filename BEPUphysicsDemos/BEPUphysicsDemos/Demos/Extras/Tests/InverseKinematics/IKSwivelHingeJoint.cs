using System;
using BEPUutilities;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    public class IKSwivelHingeJoint : IKJoint
    {
        /// <summary>
        /// Gets or sets the free hinge axis attached to connection A in its local space.
        /// </summary>
        public Vector3 LocalHingeAxis;
        /// <summary>
        /// Gets or sets the free twist axis attached to connection B in its local space.
        /// </summary>
        public Vector3 LocalTwistAxis;


        /// <summary>
        /// Gets or sets the free hinge axis attached to connection A in world space.
        /// </summary>
        public Vector3 WorldHingeAxis
        {
            get { return Vector3.Transform(LocalHingeAxis, ConnectionA.Orientation); }
            set
            {
                LocalHingeAxis = Vector3.Transform(value, Quaternion.Conjugate(ConnectionA.Orientation));
            }
        }

        /// <summary>
        /// Gets or sets the free twist axis attached to connection B in world space.
        /// </summary>
        public Vector3 WorldTwistAxis
        {
            get { return Vector3.Transform(LocalTwistAxis, ConnectionB.Orientation); }
            set
            {
                LocalTwistAxis = Vector3.Transform(value, Quaternion.Conjugate(ConnectionB.Orientation));
            }
        }

        /// <summary>
        /// Constructs a new constraint which allows relative angular motion around a hinge axis and a twist axis.
        /// </summary>
        /// <param name="connectionA">First connection of the pair.</param>
        /// <param name="connectionB">Second connection of the pair.</param>
        /// <param name="worldHingeAxis">Hinge axis attached to connectionA.
        /// The connected bone will be able to rotate around this axis relative to each other.</param>
        /// <param name="worldTwistAxis">Twist axis attached to connectionB.
        /// The connected bones will be able to rotate around this axis relative to each other.</param>
        public IKSwivelHingeJoint(Bone connectionA, Bone connectionB, Vector3 worldHingeAxis, Vector3 worldTwistAxis)
            : base(connectionA, connectionB)
        {
            WorldHingeAxis = worldHingeAxis;
            WorldTwistAxis = worldTwistAxis;
        }

        protected internal override void UpdateJacobiansAndVelocityBias()
        {
            linearJacobianA = linearJacobianB = new Matrix3X3();


            //There are two free axes and one restricted axis.
            //The constraint attempts to keep the hinge axis attached to connection A and the twist axis attached to connection B perpendicular to each other.
            //The restricted axis is the cross product between the twist and hinge axes.

            Vector3 worldTwistAxis, worldHingeAxis;
            Vector3.Transform(ref LocalHingeAxis, ref ConnectionA.Orientation, out worldHingeAxis);
            Vector3.Transform(ref LocalTwistAxis, ref ConnectionB.Orientation, out worldTwistAxis);

            Vector3 restrictedAxis;
            Vector3.Cross(ref worldHingeAxis, ref worldTwistAxis, out restrictedAxis);
            //Attempt to normalize the restricted axis.
            float lengthSquared = restrictedAxis.LengthSquared();
            if (lengthSquared > Toolbox.Epsilon)
            {
                Vector3.Divide(ref restrictedAxis, (float)Math.Sqrt(lengthSquared), out restrictedAxis);
            }
            else
            {
                restrictedAxis = new Vector3();
            }


            angularJacobianA = new Matrix3X3
              {
                  M11 = restrictedAxis.X,
                  M12 = restrictedAxis.Y,
                  M13 = restrictedAxis.Z,
              };
            Matrix3X3.Negate(ref angularJacobianA, out angularJacobianB);

            float error;
            Vector3.Dot(ref worldHingeAxis, ref worldTwistAxis, out error);
            error = (float)Math.Acos(MathHelper.Clamp(error, -1, 1)) - MathHelper.PiOver2;

            velocityBias = new Vector3(errorCorrectionFactor * error, 0, 0);


        }
    }
}
