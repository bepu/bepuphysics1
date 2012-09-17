using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    /// <summary>
    /// Attempts to maintain the relative orientation between two bones.
    /// </summary>
    public class IKAngularJoint : IKJoint
    {
        /// <summary>
        /// Gets or sets the relative orientation between the connections to maintain.
        /// </summary>
        public Quaternion GoalRelativeOrientation;


        /// <summary>
        /// Constructs a 3DOF angular joint which tries to keep two bones in angular alignment.
        /// </summary>
        /// <param name="connectionA">First bone to connect to the joint.</param>
        /// <param name="connectionB">Second bone to connect to the joint.</param>
        public IKAngularJoint(Bone connectionA, Bone connectionB)
            : base(connectionA, connectionB)
        {  
            Quaternion orientationAConjugate;
            Quaternion.Conjugate(ref ConnectionA.Orientation, out orientationAConjugate);
            Quaternion.Multiply(ref ConnectionB.Orientation, ref orientationAConjugate, out GoalRelativeOrientation);
        }

        protected internal override void UpdateJacobiansAndVelocityBias()
        {
            linearJacobianA = linearJacobianB = new Matrix3X3();
            angularJacobianA = new Matrix3X3 { M11 = 1, M22 = 1, M33 = 1 };
            angularJacobianB = new Matrix3X3 { M11 = -1, M22 = -1, M33 = -1 };

            //Compute the error between A and B's orientations.
            Quaternion conjugateB;
            Quaternion.Conjugate(ref ConnectionB.Orientation, out conjugateB);
            Quaternion errorOrientation;
            Quaternion.Multiply(ref ConnectionA.Orientation, ref conjugateB, out errorOrientation);

            //Construct the goal in world space using the basis.
            Quaternion goal;
            Quaternion conjugateA;
            Quaternion.Conjugate(ref ConnectionA.Orientation, out conjugateA);
            Quaternion.Multiply(ref GoalRelativeOrientation, ref conjugateA, out goal);
            Quaternion.Multiply(ref ConnectionA.Orientation, ref goal, out goal);
            Quaternion.Multiply(ref goal, ref errorOrientation, out errorOrientation);      
            //TODO: The above could really use some clearing up. Bad names, indirect computations...

            //Convert the error into an axis-angle vector usable for bias velocity.
            float angle;
            Vector3 axis;
            Toolbox.GetAxisAngleFromQuaternion(ref errorOrientation, out axis, out angle);

            velocityBias.X = -errorCorrectionFactor * axis.X * angle;
            velocityBias.Y = -errorCorrectionFactor * axis.Y * angle;
            velocityBias.Z = -errorCorrectionFactor * axis.Z * angle;


        }
    }
}
