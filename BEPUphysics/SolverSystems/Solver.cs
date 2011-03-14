using System;
using System.Threading;
using System.Collections.ObjectModel;
using BEPUphysics.DeactivationManagement;
using BEPUphysics.Threading;
using BEPUphysics.Constraints;
using BEPUphysics.DataStructures;

namespace BEPUphysics.SolverSystems
{
    ///<summary>
    /// Iteratively solves solver updateables, converging to a solution for simulated joints and collision pair contact constraints.
    ///</summary>
    public class Solver : MultithreadedProcessingStage
    {
        RawList<SolverUpdateable> solverUpdateables = new RawList<SolverUpdateable>();
        internal int iterationLimit = 10;
        ///<summary>
        /// Gets or sets the maximum number of iterations the solver will attempt to use to solve the simulation's constraints.
        ///</summary>
        public int IterationLimit { get { return iterationLimit; } set { iterationLimit = Math.Max(value, 0); } }
        ///<summary>
        /// Gets the list of solver updateables in the solver.
        ///</summary>
        public ReadOnlyCollection<SolverUpdateable> SolverUpdateables { get; private set; }
        protected internal TimeStepSettings timeStepSettings;
        ///<summary>
        /// Gets or sets the time step settings used by the solver.
        ///</summary>
        public TimeStepSettings TimeStepSettings
        {
            get
            {
                return timeStepSettings;
            }
            set
            {
                timeStepSettings = value;
            }
        }

        ///<summary>
        /// Gets or sets the deactivation manager used by the solver.
        /// When constraints are added and removed, the deactivation manager
        /// gains and loses simulation island connections that affect simulation islands
        /// and activity states.
        ///</summary>
        public DeactivationManager DeactivationManager { get; set; }

        ///<summary>
        /// Constructs a Solver.
        ///</summary>
        ///<param name="timeStepSettings">Time step settings used by the solver.</param>
        ///<param name="deactivationManager">Deactivation manager used by the solver.</param>
        public Solver(TimeStepSettings timeStepSettings, DeactivationManager deactivationManager)
        {
            TimeStepSettings = timeStepSettings;
            DeactivationManager = deactivationManager;
            multithreadedPrestepDelegate = MultithreadedPrestep;
            multithreadedIterationDelegate = MultithreadedIteration;
            SolverUpdateables = new ReadOnlyCollection<SolverUpdateable>(solverUpdateables);
            Enabled = true;
        }
        ///<summary>
        /// Constructs a Solver.
        ///</summary>
        ///<param name="timeStepSettings">Time step settings used by the solver.</param>
        ///<param name="deactivationManager">Deactivation manager used by the solver.</param>
        /// <param name="threadManager">Thread manager used by the solver.</param>
        public Solver(TimeStepSettings timeStepSettings, DeactivationManager deactivationManager, IThreadManager threadManager)
            : this(timeStepSettings, deactivationManager)
        {
            ThreadManager = threadManager;
            AllowMultithreading = true;
        }

        ///<summary>
        /// Adds a solver updateable to the solver.
        ///</summary>
        ///<param name="item">Updateable to add.</param>
        ///<exception cref="ArgumentException">Thrown when the item already belongs to a solver.</exception>
        public void Add(SolverUpdateable item)
        {
            if (item.Solver == null)
            {
                item.Solver = this;
                solverUpdateables.Add(item);
                DeactivationManager.Add(item);
                item.OnAdditionToSolver(this);
            }
            else
                throw new ArgumentException("Solver updateable already belongs to something; it can't be added.", "item");
        }
        ///<summary>
        /// Removes a solver updateable from the solver.
        ///</summary>
        ///<param name="item">Updateable to remove.</param>
        ///<exception cref="ArgumentException">Thrown when the item does not belong to the solver.</exception>
        public void Remove(SolverUpdateable item)
        {
            if (item.Solver == this)
            {
                item.Solver = null;
                solverUpdateables.Remove(item);
                DeactivationManager.Remove(item);
                item.OnRemovalFromSolver(this);
            }
            else
                throw new ArgumentException("Solver updateable doesn't belong to this solver; it can't be removed.", "item");
        }

        Action<int> multithreadedPrestepDelegate;
        void MultithreadedPrestep(int i)
        {
            SolverUpdateable updateable = solverUpdateables.Elements[i];
            updateable.UpdateSolverActivity();
            if (updateable.isActiveInSolver)
            {
                updateable.SolverSettings.currentIterations = 0;
                updateable.SolverSettings.iterationsAtZeroImpulse = 0;
                updateable.Update(timeStepSettings.TimeStepDuration);

                updateable.EnterLock();
                try
                {
                    updateable.ExclusiveUpdate();
                }
                finally
                {
                    updateable.ExitLock();
                }
            }
        }



        static int[] primes =  { 45589, 45599, 45613, 45631, 45641, 45659, 45667, 45673, 45677, 45691, 
                                 45697, 45707, 45737, 45751, 45757, 45763, 45767, 45779, 45817, 45821,
                                 45823, 45827, 45833, 45841, 45853, 45863, 45869, 45887, 45893, 45943, 
                                 45949, 45953, 45959, 45971, 45979, 45989, 46021, 46027, 46049, 46051, 
                                 46061, 46073, 46091, 46093, 46099, 46103, 46133, 46141, 46147, 46153, 
                                 46171, 46181, 46183, 46187, 46199, 46219, 46229, 46237, 46261, 46271, 
                                 46273, 46279, 46301, 46307, 46309, 46327, 46337};
        int primeIndex;
        private void GetCoefficients(out int a, out int b)
        {
            //Interlocked increment wraps on overflow.
            int index = Math.Abs(Interlocked.Increment(ref primeIndex)) % primes.Length;
            if (solverUpdateables.count >= primes[0])
            {
                //Very few simulations have such a crazy number of updateables, but it's not unheard of.
                //Can't use a larger prime than 46337, since that would overflow and destroy the permutation.
                //This leaves us with shifting alone.
                a = 1;
            }
            else
            {
                a = primes[index];
            }
            //Compute the shift amount.  This should be a positive number from 0 to 366078, keeping it from wrapping.
            b = (int)(((uint)index * (uint)primes[index]) % 366079);
        }

        Action<int> multithreadedIterationDelegate;
        void MultithreadedIteration(int i)
        {
            int a, b;
            GetCoefficients(out a, out b);
            int solverCount = solverUpdateables.Count;
            for (int j = 0; j < solverCount; j++)
            {
                var updateable = solverUpdateables.Elements[(a * j + b) % solverCount];

                SolverSettings solverSettings = updateable.solverSettings;
                //Updateables only ever go from active to inactive during iterations,
                //so it's safe to check for activity before we do hard (synchronized) work.
                if (updateable.isActiveInSolver)
                {

                    int incrementedIterations = Interlocked.Increment(ref solverSettings.currentIterations);
                    if (incrementedIterations > iterationLimit ||
                        incrementedIterations > solverSettings.maximumIterations)
                    {
                        updateable.isActiveInSolver = false;
                    }
                    else
                    {
                        updateable.EnterLock();
                        try
                        {
                            if (updateable.SolveIteration() < solverSettings.minimumImpulse)
                            {
                                solverSettings.iterationsAtZeroImpulse++;
                                if (solverSettings.iterationsAtZeroImpulse > solverSettings.minimumIterations)
                                    updateable.isActiveInSolver = false;
                            }
                            else
                            {
                                solverSettings.iterationsAtZeroImpulse = 0;
                            }
                        }
                        finally
                        {
                            updateable.ExitLock();
                        }
                    }


                }
            }


        }

        protected override void UpdateMultithreaded()
        {
            ThreadManager.ForLoop(0, solverUpdateables.Count, multithreadedPrestepDelegate);
            ThreadManager.ForLoop(0, iterationLimit, multithreadedIterationDelegate);
        }

        protected override void UpdateSingleThreaded()
        {
            int count = solverUpdateables.count;
            for (int i = 0; i < count; i++)
            {
                UnsafePrestep(solverUpdateables.Elements[i]);
            }

            for (int i = 0; i < iterationLimit; i++)
            {
                int a, b;
                GetCoefficients(out a, out b);
                for (int j = 0; j < count; j++)
                {
                    UnsafeSolveIteration(solverUpdateables.Elements[(a * j + b) % count]);
                }
            }


        }

        protected internal void UnsafePrestep(SolverUpdateable updateable)
        {
            updateable.UpdateSolverActivity();
            if (updateable.isActiveInSolver)
            {
                SolverSettings solverSettings = updateable.solverSettings;
                solverSettings.currentIterations = 0;
                solverSettings.iterationsAtZeroImpulse = 0;
                updateable.Update(timeStepSettings.TimeStepDuration);
                updateable.ExclusiveUpdate();
            }
        }

        protected internal void UnsafeSolveIteration(SolverUpdateable updateable)
        {
            if (updateable.isActiveInSolver)
            {
                SolverSettings solverSettings = updateable.solverSettings;


                solverSettings.currentIterations++;
                if (solverSettings.currentIterations <= iterationLimit &&
                    solverSettings.currentIterations <= solverSettings.maximumIterations)
                {
                    if (updateable.SolveIteration() < solverSettings.minimumImpulse)
                    {
                        solverSettings.iterationsAtZeroImpulse++;
                        if (solverSettings.iterationsAtZeroImpulse > solverSettings.minimumIterations)
                            updateable.isActiveInSolver = false;

                    }
                    else
                    {
                        solverSettings.iterationsAtZeroImpulse = 0;
                    }
                }
                else
                {
                    updateable.isActiveInSolver = false;
                }

                //if (++solverSettings.currentIterations > iterationLimit ||
                //    solverSettings.currentIterations > solverSettings.maximumIterations ||
                //    (updateable.SolveIteration() < solverSettings.minimumImpulse &&
                //    ++solverSettings.iterationsAtZeroImpulse > solverSettings.minimumIterations))
                //{
                //    updateable.isActiveInSolver = false;
                //}
                //else //If it's greater than the minimum impulse, reset the count.
                //    solverSettings.iterationsAtZeroImpulse = 0;
            }
        }


    }
}
