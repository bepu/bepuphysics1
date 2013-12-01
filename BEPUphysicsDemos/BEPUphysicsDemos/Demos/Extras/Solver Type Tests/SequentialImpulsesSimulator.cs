using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.Threading;
using BEPUutilities.DataStructures;

namespace BEPUphysicsDemos.Demos.Extras.SolverTypeTests
{
    internal class SequentialImpulsesSimulator : Simulator
    {

        private void Preupdate(int i)
        {
            constraints.Elements[i].Preupdate(inverseDt);
        }

        private void ApplyAccumulatedImpulses(int i)
        {
            var constraint = constraints.Elements[i];
            constraint.EnterLock();
            constraint.ApplyAccumulatedImpulses();
            constraint.ExitLock();
        }

        private void SolveIteration(int i)
        {
            var constraint = constraints.Elements[i];
            constraint.EnterLock();
            constraints.Elements[i].SolveIteration();
            constraints.Elements[i].ApplyImpulses();
            constraint.ExitLock();
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

            for (int i = 0; i < constraints.Count; ++i)
            {
                Preupdate(i);
            }
            for (int i = 0; i < constraints.Count; ++i)
            {
                constraints.Elements[i].ApplyAccumulatedImpulses();
            }

            for (int iterationIndex = 0; iterationIndex < IterationCount; ++iterationIndex)
            {
                foreach (var constraint in constraints)
                {
                    constraint.SolveIteration();
                    constraint.ApplyImpulses();
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
            looper.ForLoop(0, constraints.Count, ApplyAccumulatedImpulses);

            for (int i = 0; i < IterationCount; ++i)
            {
                looper.ForLoop(0, constraints.Count, SolveIteration);
            }

            looper.ForLoop(0, constraints.Count, UpdatePosition);
        }

    }
}
