using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.DataStructures;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    public class Bone
    {
        protected internal List<IKConstraint> constraints = new List<IKConstraint>();
        public ReadOnlyList<IKConstraint> Constraints
        {
            get { return new ReadOnlyList<IKConstraint>(constraints); }
        }
    }
}
