using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using BEPUutilities;

namespace BEPUphysicsDemos.Demos.Extras.SolverTypeTests
{
    class DistanceConstraint : Constraint
    {
        public LinearDynamic A { get; private set; }
        public LinearDynamic B { get; private set; }


        private Vector3 jacobian;
        private float effectiveMass;


        private float biasVelocity;

        private float accumulatedImpulse;
        private float impulse;

        private float distance;

        public DistanceConstraint(LinearDynamic a, LinearDynamic b)
        {
            A = a;
            B = b;
            ++a.ConstraintCount;
            ++b.ConstraintCount;

            distance = (a.Position - b.Position).Length();
        }

        public override void Preupdate(float inverseDt, bool useConstraintCounts)
        {
            Vector3.Subtract(ref B.Position, ref A.Position, out jacobian);
            float currentDistance = jacobian.LengthSquared();
            if (currentDistance > Toolbox.Epsilon)
            {
                currentDistance = (float)Math.Sqrt(currentDistance);
                Vector3.Divide(ref jacobian, currentDistance, out jacobian);
            }
            else
            {
                currentDistance = 0;
                jacobian = Toolbox.UpVector;
            }

            if (useConstraintCounts)
                effectiveMass = 1f / (A.ConstraintCount * A.InverseMass + B.ConstraintCount * B.InverseMass + Softness);
            else
                effectiveMass = 1f / (A.InverseMass + B.InverseMass + Softness);
            accumulatedImpulse = 0;
            biasVelocity = (distance - currentDistance) * BiasFactor * inverseDt;

        }

        public override void SolveIteration()
        {
            Vector3 relativeVelocity;
            Vector3.Subtract(ref B.Velocity, ref A.Velocity, out relativeVelocity);
            float relativeVelocityAlongJacobian;
            Vector3.Dot(ref relativeVelocity, ref jacobian, out relativeVelocityAlongJacobian);


            float changeInVelocity = relativeVelocityAlongJacobian - biasVelocity - Softness * accumulatedImpulse;

            impulse = changeInVelocity * effectiveMass;

            accumulatedImpulse += impulse;

        }

        public override void ApplyImpulse(LinearDynamic dynamic)
        {
            ApplyImpulse(dynamic, impulse);
        }

        public override void ApplyAccumulatedImpulse(LinearDynamic dynamic)
        {
            ApplyImpulse(dynamic, accumulatedImpulse);
        }

        private void ApplyImpulse(LinearDynamic dynamic, float impulseToApply)
        {
            Vector3 worldSpaceImpulse;
            if (A == dynamic)
            {
                Vector3.Multiply(ref jacobian, impulseToApply, out worldSpaceImpulse);
            }
            else
            {
                Vector3.Multiply(ref jacobian, -impulseToApply, out worldSpaceImpulse);
            }
            dynamic.ApplyImpulse(ref worldSpaceImpulse);

        }

        public override void ApplyImpulses()
        {
            Vector3 worldSpaceImpulse;
            Vector3.Multiply(ref jacobian, impulse, out worldSpaceImpulse);
            A.ApplyImpulse(ref worldSpaceImpulse);
            Vector3.Negate(ref worldSpaceImpulse, out worldSpaceImpulse);
            B.ApplyImpulse(ref worldSpaceImpulse);
        }

        public override void ApplyAccumulatedImpulses()
        {
            Vector3 worldSpaceImpulse;
            Vector3.Multiply(ref jacobian, accumulatedImpulse, out worldSpaceImpulse);
            A.ApplyImpulse(ref worldSpaceImpulse);
            Vector3.Negate(ref worldSpaceImpulse, out worldSpaceImpulse);
            B.ApplyImpulse(ref worldSpaceImpulse);
        }


        internal override void AddToConnections()
        {
            A.Constraints.Add(this);
            B.Constraints.Add(this);
        }

        public override void EnterLock()
        {
            if (A.Id <= B.Id)
            {
                A.SolverSpinLock.Enter();
                B.SolverSpinLock.Enter();
            }
            else
            {
                B.SolverSpinLock.Enter();
                A.SolverSpinLock.Enter();
            }
        }

        public override void ExitLock()
        {
            if (A.Id <= B.Id)
            {
                B.SolverSpinLock.Exit();
                A.SolverSpinLock.Exit();
            }
            else
            {
                A.SolverSpinLock.Exit();
                B.SolverSpinLock.Exit();
            }
        }
    }
}
