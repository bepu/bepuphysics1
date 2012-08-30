using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.DataStructures;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    /// <summary>
    /// Piece of a character which is moved by constraints.
    /// </summary>
    public class IKBone
    {
        internal List<IKJoint> joints = new List<IKJoint>();

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
    }
}
