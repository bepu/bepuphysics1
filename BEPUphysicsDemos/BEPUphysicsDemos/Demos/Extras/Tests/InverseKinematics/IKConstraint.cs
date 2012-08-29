using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    public abstract class IKConstraint
    {
        public Bone ConnectionA { get; protected set; }
        public Bone ConnectionB { get; protected set; }


    }
}
