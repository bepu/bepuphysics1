using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUutilities;

namespace BEPUphysicsDemos.Demos.Extras.SolverTypeTests
{
    abstract class Constraint
    {

        public float Softness = 0f;
        public float BiasFactor = 0.2f;

        public abstract void Preupdate(float inverseDt, bool useConstraintCounts);

        public abstract void SolveIteration();

        public abstract void ApplyImpulse(LinearDynamic dynamic);

        public abstract void ApplyAccumulatedImpulse(LinearDynamic dynamic);

        public abstract void ApplyImpulses();

        public abstract void ApplyAccumulatedImpulses();

        internal abstract void AddToConnections();

        public abstract void EnterLock();
        public abstract void ExitLock();
    }
}
