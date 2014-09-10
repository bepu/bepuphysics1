using System.Diagnostics;
using BEPUutilities.Threading;

namespace BEPUphysicsDemos.Demos.Extras.SolverTypeTests
{
    internal class JacobiSimulator : Simulator
    {


        private void Preupdate(int i)
        {
            constraints.Elements[i].Preupdate(inverseDt, true);
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

            var wholeStartTime = Stopwatch.GetTimestamp();

            this.dt = dt;
            inverseDt = 1 / dt;


            ApplyGravity(dt);

            var solverStartTime = Stopwatch.GetTimestamp();
            foreach (var constraint in constraints)
            {
                constraint.Preupdate(inverseDt, true);
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
            looper.ForLoop(0, dynamics.Count, ApplyAccumulatedImpulses);

            for (int i = 0; i < IterationCount; ++i)
            {
                looper.ForLoop(0, constraints.Count, SolveIteration);
                looper.ForLoop(0, dynamics.Count, ApplyImpulse);
            }
            var solverEndTime = Stopwatch.GetTimestamp();

            looper.ForLoop(0, dynamics.Count, UpdatePosition);
            var wholeEndTime = Stopwatch.GetTimestamp();

            SolveTime = (solverEndTime - solverStartTime) / (double)Stopwatch.Frequency;
            TotalTime = (wholeEndTime - wholeStartTime) / (double)Stopwatch.Frequency;
        }


    }
}
