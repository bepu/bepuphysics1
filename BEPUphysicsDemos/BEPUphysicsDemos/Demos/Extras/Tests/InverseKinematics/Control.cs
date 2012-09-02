using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    /// <summary>
    /// Constrains an individual bone in an attempt to reach some goal.
    /// </summary>
    public class Control
    {
        /// <summary>
        /// Gets or sets the controlled bone.
        /// </summary>
        public Bone TargetBone
        {
            get { return LinearMotor.TargetBone; }
            set
            {
                LinearMotor.TargetBone = value;
                AngularMotor.TargetBone = value;
            }
        }


        /// <summary>
        /// Stores where the control is in the solver listing for quick adds and removes.
        /// </summary>
        internal int solverIndex;

        /// <summary>
        /// Gets or sets the linear motor used by the control.
        /// </summary>
        public SingleBoneLinearMotor LinearMotor
        {
            get;
            private set;

        }

        /// <summary>
        /// Gets or sets the angular motor used by the control.
        /// </summary>
        public SingleBoneAngularMotor AngularMotor
        {
            get;
            private set;

        }
    }
}