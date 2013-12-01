using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.Threading;
using BEPUutilities.DataStructures;

namespace BEPUphysicsDemos.Demos.Extras.SolverTypeTests
{
    internal class JacobiSimulator : Simulator
    {


        private void Preupdate(int i)
        {
            constraints.Elements[i].Preupdate(inverseDt);
        }

        private void ApplyAccumulatedImpulses(int i)
        {
            var dynamic = dynamics.Elements[i];
            var count = dynamic.Constraints.Count;
            for (int constraintIndex = 0; constraintIndex < count; ++constraintIndex)
            {
                dynamic.Constraints.Elements[constraintIndex].ApplyAccumulatedImpulse(dynamic);
            }
        }

        private void SolveIteration(int i)
        {
            constraints.Elements[i].SolveIteration();
        }

        private void ApplyImpulse(int i)
        {
            var dynamic = dynamics.Elements[i];
            var count = dynamic.Constraints.Count;
            for (int constraintIndex = 0; constraintIndex < count; ++constraintIndex)
            {
                dynamic.Constraints.Elements[constraintIndex].ApplyImpulse(dynamic);
            }
        }

        private void UpdatePosition(int i)
        {
            var dynamic = dynamics.Elements[i];
            dynamic.UpdatePosition(dt);
        }

        private float inverseDt;
        private float dt;
        public override void Update(float dt)
        {
            this.dt = dt;
            inverseDt = 1 / dt;

            ApplyGravity(dt);

            foreach (var constraint in constraints)
            {
                constraint.Preupdate(inverseDt);
            }
            for (int i = 0; i < dynamics.Count; ++i)
            {
                ApplyAccumulatedImpulses(i);
            }

            for (int iterationIndex = 0; iterationIndex < IterationCount; ++iterationIndex)
            {
                foreach (var constraint in constraints)
                {
                    constraint.SolveIteration();
                }
                for (int dynamicIndex = 0; dynamicIndex < dynamics.Count; ++dynamicIndex)
                {
                    ApplyImpulse(dynamicIndex);
                }
            }

            foreach (var dynamic in dynamics)
            {
                dynamic.UpdatePosition(dt);
            }
        }

        public override void Update(float dt, IParallelLooper looper)
        {
            this.dt = dt;
            inverseDt = 1 / dt;


            ApplyGravity(dt, looper);

            looper.ForLoop(0, constraints.Count, Preupdate);
            looper.ForLoop(0, dynamics.Count, ApplyAccumulatedImpulses);

            for (int i = 0; i < IterationCount; ++i)
            {
                looper.ForLoop(0, constraints.Count, SolveIteration);
                looper.ForLoop(0, dynamics.Count, ApplyImpulse);
            }

            looper.ForLoop(0, constraints.Count, UpdatePosition);
        }

        
    }
}
