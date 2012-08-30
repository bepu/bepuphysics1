using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    /// <summary>
    /// Constrains an individual bone in an attempt to reach some goal.
    /// </summary>
    public abstract class IKControl : IKConstraint
    {
        public IKBone TargetBone { get; set; }

        /// <summary>
        /// Stores where the control is in the solver listing for quick adds and removes.
        /// </summary>
        internal int solverIndex; 
        //TODO: Should probably throw an exception or something if the target bone is pinned.
    }
}
