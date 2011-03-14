//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using System.Collections.ObjectModel;
//using BEPUphysics.ResourceManagement;

//namespace BEPUphysics
//{
//    public class BatchedSolver : MultithreadedProcessingStage, ISolver
//    {
//        ResourcePool<SolverBatch> solverBatchPool = new UnsafeResourcePool<SolverBatch>();
//        Action<int> multithreadedPrestepDelegate;
//        protected internal TimeStepSettings timeStepSettings;
//        public TimeStepSettings TimeStepSettings
//        {
//            get
//            {
//                return timeStepSettings;
//            }
//            set
//            {
//                timeStepSettings = value;
//            }
//        }

//        void MultithreadedPrestep(int i)
//        {
//            SolverUpdateable updateable = solverUpdateables[i];
//            updateable.UpdateSolverActivity();
//            if (updateable.IsActiveInSolver)
//            {
//                updateable.SolverSettings.currentIterations = 0;
//                updateable.SolverSettings.iterationsAtZeroImpulse = 0;
//                updateable.Update(timeStepSettings.TimeStepDuration);

//                updateable.EnterLock();
//                try
//                {
//                    updateable.ExclusiveUpdate();
//                }
//                finally
//                {
//                    updateable.ExitLock();
//                }
//            }
//        }
    
//        public DeactivationManager DeactivationManager { get; set; }

//        Action<int> batchSolveDelegate;
//        void BatchSolve(int i)
//        {
//            SolverUpdateable updateable = currentBatch.SolverUpdateables[i] as SolverUpdateable;
//            if (updateable.IsActiveInSolver)
//            {
//                if (++updateable.SolverSettings.currentIterations <= iterationLimit &&
//                    updateable.SolverSettings.currentIterations <= updateable.SolverSettings.maximumIterations &&
//                    updateable.SolverSettings.iterationsAtZeroImpulse <= updateable.SolverSettings.minimumIterations)
//                {
//                    if (updateable.SolveIteration() < updateable.SolverSettings.minimumImpulse)
//                        updateable.SolverSettings.iterationsAtZeroImpulse++;
//                }
//                else
//                    updateable.isActiveInSolver = false;
//            }
//        }

//        SolverBatch currentBatch;
//        protected override void UpdateMultithreaded()
//        {
//            ThreadManager.ForLoop(0, solverUpdateables.Count, multithreadedPrestepDelegate);
//            for (int i = 0; i < iterationLimit; i++)
//            {
//                foreach (SolverBatch batch in solverBatches)
//                {
//                    currentBatch = batch;
//                    ThreadManager.ForLoop(0, batch.SolverUpdateables.Count, batchSolveDelegate);
//                }
//            }
//        }

//        protected override void UpdateSingleThreaded()
//        {
//            //TODO: This is identical to the normal Solver single threaded... and there's other re-used stuff too.  Consider doing some reorganization to eliminate codecopy.
//            ISolver solver = this as ISolver;
//            foreach (SolverUpdateable updateable in solverUpdateables)
//            {
//                solver.UnsafePrestep(updateable);
//            }

//            for (int i = 0; i < iterationLimit; i++)
//            {
//                foreach (SolverUpdateable updateable in solverUpdateables)
//                {
//                    solver.UnsafeSolveIteration(updateable);
//                }
//            }
//        }


//        public ReadOnlyCollection<SolverUpdateable> SolverUpdateables { get; private set; }
//        List<SolverUpdateable> solverUpdateables = new List<SolverUpdateable>();
//        List<SolverBatch> solverBatches = new List<SolverBatch>();
//        public ReadOnlyCollection<SolverBatch> SolverBatches { get; private set; }

//        public BatchedSolver()
//        {
//            SolverUpdateables = new ReadOnlyCollection<SolverUpdateable>(solverUpdateables);
//            SolverBatches = new ReadOnlyCollection<SolverBatch>(solverBatches);
//        }

//        int iterationLimit = 10;
//        public int IterationLimit { get { return iterationLimit; } set { iterationLimit = Math.Max(value, 0); } }

//        public void Add(SolverUpdateable item)
//        {
//            ISolverBatchUpdateable newItem = item as ISolverBatchUpdateable;
//            if (newItem != null)
//            {
//                if (item.Solver == null)
//                {
//                    item.Solver = this;
//                    solverUpdateables.Add(item);
//                    DeactivationManager.Add(item);

//                    //TODO: Add to batch.

//                    ISolverBatchUpdateable batchItem = item as ISolverBatchUpdateable;
//                    foreach (SolverBatch batch in solverBatches)
//                    {
//                        if (batch.TryToAdd(batchItem))
//                            break;
//                    }
//                }
//                else
//                    throw new InvalidOperationException("Solver updateable already belongs to something; it can't be added.");
//            }
//            else
//                throw new InvalidOperationException("Added ISolverUpdateable is not an ISolverBatchUpdateable; cannot add to the batched solver.");
//        }

//        public void Remove(SolverUpdateable item)
//        {
//            ISolverBatchUpdateable newItem = item as ISolverBatchUpdateable;
//            if (newItem != null)
//            {
//                if (item.Solver == this)
//                {
//                    item.Solver = null;
//                    solverUpdateables.Remove(item);
//                    DeactivationManager.Remove(item);

//                    ISolverBatchUpdateable batchItem = item as ISolverBatchUpdateable;
//                    batchItem.SolverBatch.Remove(batchItem);

//                }
//                else
//                    throw new InvalidOperationException("Solver updateable doesn't belong to this solver; it can't be removed.");
//            }
//            else
//                throw new InvalidOperationException("Removed ISolverUpdateable is not an ISolverBatchUpdateable; cannot belong to the batched solver.");
//        }




//        internal void GiveBack(SolverBatch solverBatch)
//        {
//            solverBatch.Clear();
//            solverBatchPool.GiveBack(solverBatch);
//        }

//        void ISolver.UnsafePrestep(SolverUpdateable updateable)
//        {
//            updateable.UpdateSolverActivity();
//            if (updateable.IsActiveInSolver)
//            {
//                updateable.SolverSettings.currentIterations = 0;
//                updateable.SolverSettings.iterationsAtZeroImpulse = 0;
//                updateable.Update(timeStepSettings.TimeStepDuration);
//                updateable.ExclusiveUpdate();
//            }
//        }

//        void ISolver.UnsafeSolveIteration(SolverUpdateable updateable)
//        {
//            if (updateable.IsActiveInSolver)
//            {
//                if (++updateable.SolverSettings.currentIterations <= iterationLimit &&
//                    updateable.SolverSettings.currentIterations <= updateable.SolverSettings.maximumIterations &&
//                    updateable.SolverSettings.iterationsAtZeroImpulse <= updateable.SolverSettings.minimumIterations)
//                {
//                    if (updateable.SolveIteration() < updateable.SolverSettings.minimumImpulse)
//                        updateable.SolverSettings.iterationsAtZeroImpulse++;
//                    else
//                        updateable.SolverSettings.iterationsAtZeroImpulse = 0; //TODO: Should it really set it to zero?  Probably, but..
//                }
//                else
//                    updateable.isActiveInSolver = false;
//            }
//        }

//    }
//}
