using BEPUutilities;
using BEPUutilities.DataStructures;
using BEPUutilities.Threading;

namespace BEPUphysicsDemos.Demos.Extras.SolverTypeTests
{
    abstract class Simulator
    {
        protected RawList<LinearDynamic> dynamics = new RawList<LinearDynamic>();
        protected RawList<Constraint> constraints = new RawList<Constraint>();

        public ReadOnlyList<LinearDynamic> Dynamics
        {
            get { return new ReadOnlyList<LinearDynamic>(dynamics); }
        }

        public ReadOnlyList<Constraint> Constraints
        {
            get { return new ReadOnlyList<Constraint>(constraints); }
        }

        public int IterationCount = 10;
        public Vector3 Gravity = new Vector3(0, -10, 0);

        public void Add(LinearDynamic dynamic)
        {
            dynamics.Add(dynamic);
        }

        public void Add(Constraint constraint)
        {
            constraints.Add(constraint);
            constraint.AddToConnections();
        }

        public double TotalTime { get; protected set; }
        public double SolveTime { get; protected set; }

        public abstract void Update(float dt);

        public abstract void Update(float dt, IParallelLooper looper);


        Vector3 gravityVelocityChange;

        protected void ApplyGravity(int i)
        {
            Vector3.Add(ref dynamics.Elements[i].Velocity, ref gravityVelocityChange, out dynamics.Elements[i].Velocity);
        }

        protected void ApplyGravity(float dt)
        {
            Vector3.Multiply(ref Gravity, dt, out gravityVelocityChange);
            for (int i = dynamics.Count - 1; i >= 0; --i)
            {
                ApplyGravity(i);
            }
        }
        protected void ApplyGravity(float dt, IParallelLooper looper)
        {
            Vector3.Multiply(ref Gravity, dt, out gravityVelocityChange);
            looper.ForLoop(0, dynamics.Count, ApplyGravity);
        }
    }
}
