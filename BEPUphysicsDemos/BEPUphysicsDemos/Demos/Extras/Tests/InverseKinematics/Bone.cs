using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.DataStructures;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    /// <summary>
    /// Piece of a character which is moved by constraints.
    /// </summary>
    public class Bone
    {
        internal List<IKJoint> joints = new List<IKJoint>();

        /// <summary>
        /// Gets or sets the position of the bone.
        /// </summary>
        public Vector3 Position;

        /// <summary>
        /// Gets or sets the orientation of the bone.
        /// </summary>
        public Quaternion Orientation;

        /// <summary>
        /// The mid-iteration angular velocity associated with the bone.
        /// This is computed during the velocity subiterations and then applied to the orientation at the end of each position iteration.
        /// </summary>
        internal Vector3 angularVelocity;

        /// <summary>
        /// The mid-iteration linear velocity associated with the bone.
        /// This is computed during the velocity subiterations and then applied to the position at the end of each position iteration.
        /// </summary>
        internal Vector3 linearVelocity;


        internal float inverseMass;

        internal Matrix3X3 inertiaTensorInverse;
        internal Matrix3X3 localInertiaTensorInverse;

        /// <summary>
        /// Gets the list of joints affecting this bone.
        /// </summary>
        public ReadOnlyList<IKJoint> Joints
        {
            get { return new ReadOnlyList<IKJoint>(joints); }
        }


        /// <summary>
        /// Gets or sets whether or not this bone is pinned. Pinned bones cannot be moved by constraints.
        /// </summary>
        public bool Pinned { get; set; }

        /// <summary>
        /// Gets whether or not the bone is a member of the active set as determined by the last IK solver execution.
        /// </summary>
        public bool IsActive { get; internal set; }

        /// <summary>
        /// Updates the world inertia tensor based upon the local inertia tensor and current orientation.
        /// </summary>
        internal void UpdateInertiaTensor()
        {
            //This is separate from the position update because the orientation can change outside of our iteration loop, so this has to run first.
            Matrix3X3 orientationMatrix;
            Matrix3X3.CreateFromQuaternion(ref Orientation, out orientationMatrix);
            Matrix3X3.MultiplyTransposed(ref orientationMatrix, ref localInertiaTensorInverse, out inertiaTensorInverse);
        }

        /// <summary>
        /// Integrates the position and orientation of the bone forward based upon the current linear and angular velocity.
        /// </summary>
        internal void UpdatePosition()
        {
            //Update the position based on the linear velocity.
            Vector3.Add(ref linearVelocity, ref Position, out Position);

            //Update the orientation based on the angular velocity.
            Vector3 increment;
            Vector3.Multiply(ref angularVelocity, .5f, out increment);
            var multiplier = new Quaternion(increment.X, increment.Y, increment.Z, 0);
            Quaternion.Multiply(ref multiplier, ref Orientation, out multiplier);
            Quaternion.Add(ref Orientation, ref multiplier, out Orientation);
            Orientation.Normalize();

            //Eliminate any latent velocity in the bone to prevent unwanted simulation feedback.
            //This is the only thing conceptually separating this "IK" solver from the regular dynamics loop in BEPUphysics.
            //(Well, that and the whole lack of collision detection...)
            linearVelocity = new Vector3();
            angularVelocity = new Vector3();
        }

        internal void ApplyLinearImpulse(ref Vector3 impulse)
        {
            Vector3 velocityChange;
            Vector3.Multiply(ref impulse, inverseMass, out velocityChange);
            Vector3.Add(ref linearVelocity, ref velocityChange, out linearVelocity);
        }

        internal void ApplyAngularImpulse(ref Vector3 impulse)
        {
            Vector3 velocityChange;
            Matrix3X3.Transform(ref impulse, ref inertiaTensorInverse, out velocityChange);
            Vector3.Add(ref velocityChange, ref angularVelocity, out angularVelocity);
        }
    }
}
