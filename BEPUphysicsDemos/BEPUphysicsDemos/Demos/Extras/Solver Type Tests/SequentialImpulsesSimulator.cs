using System.Diagnostics;
using BEPUutilities;
using BEPUutilities.Threading;

namespace BEPUphysicsDemos.Demos.Extras.SolverTypeTests
{
    internal class SequentialImpulsesSimulator : Simulator
    {

        private void Preupdate(int i)
        {
            constraints.Elements[i].Preupdate(inverseDt, false);
        }

        private void ApplyAccumulatedImpulses(int i)
        {
            var constraint = constraints.Elements[i];
            constraint.EnterLock();
            constraint.ApplyAccumulatedImpulses();
            constraint.ExitLock();
        }

        private void ApplyAccumulatedImpulsesToDynamic(int i)
        {
            var dynamic = dynamics.Elements[i];
            var count = dynamic.Constraints.Count;
            for (int constraintIndex = 0; constraintIndex < count; ++constraintIndex)
            {
                dynamic.Constraints.Elements[constraintIndex].ApplyAccumulatedImpulse(dynamic);
            }
        }

        private PermutationMapper permutationMapper = new PermutationMapper();
        private void SolveIteration(int i)
        {
            var constraint = constraints.Elements[permutationMapper.GetMappedIndex(i, constraints.Count)];
            constraint.EnterLock();
            constraint.SolveIteration();
            constraint.ApplyImpulses();
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
            var wholeStartTime = Stopwatch.GetTimestamp();
            this.dt = dt;
            inverseDt = 1 / dt;

            ApplyGravity(dt);

            var solverStartTime = Stopwatch.GetTimestamp();
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
            var solverEndTime = Stopwatch.GetTimestamp();

            foreach (var dynamic in dynamics)
            {
                dynamic.UpdatePosition(dt);
            }


            var wholeEndTime = Stopwatch.GetTimestamp();

            SolveTime = (solverEndTime - solverStartTime) / (double)Stopwatch.Frequency;
            TotalTime = (wholeEndTime - wholeStartTime) / (double)Stopwatch.Frequency;
        }

        public override void Update(float dt, IParallelLooper looper)
        {
            var wholeStartTime = Stopwatch.GetTimestamp();
            this.dt = dt;
            inverseDt = 1 / dt;

            ApplyGravity(dt, looper);

            var solverStartTime = Stopwatch.GetTimestamp();
            looper.ForLoop(0, constraints.Count, Preupdate);
            //looper.ForLoop(0, constraints.Count, ApplyAccumulatedImpulses);
            looper.ForLoop(0, dynamics.Count, ApplyAccumulatedImpulsesToDynamic);

            looper.ForLoop(0, constraints.Count * IterationCount, SolveIteration);

            var solverEndTime = Stopwatch.GetTimestamp();

            looper.ForLoop(0, dynamics.Count, UpdatePosition);
            var wholeEndTime = Stopwatch.GetTimestamp();

            SolveTime = (solverEndTime - solverStartTime) / (double)Stopwatch.Frequency;
            TotalTime = (wholeEndTime - wholeStartTime) / (double)Stopwatch.Frequency;
        }

    }
}
