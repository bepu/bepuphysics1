using System.Collections.Generic;
using System.Threading;
using BEPUphysics.Constraints.SolverGroups;
using BEPUphysics.DeactivationManagement;
using BEPUphysics.Entities;
using BEPUphysics.ResourceManagement;
using System.Collections.ObjectModel;
using BEPUphysics.DataStructures;
using BEPUphysics.SolverSystems;

namespace BEPUphysics.Constraints
{
    /// <summary>
    /// Superclass of objects types which require solving by the velocity solver.
    /// These are updated within the internal iterative solver when owned by a space.
    /// </summary>
    public abstract class EntitySolverUpdateable : SolverUpdateable//, ISolverBatchUpdateable
    {
        
        //protected internal SolverBatch batch;



        /// <summary>
        /// List of all entities affected by this updateable.
        /// </summary>
        protected internal readonly RawList<Entity> involvedEntities = new RawList<Entity>(2);

        ///<summary>
        /// Gets the entities that this solver updateable is involved with.
        ///</summary>
        public ReadOnlyList<Entity> InvolvedEntities
        {
            get
            {
                return new ReadOnlyList<Entity>(involvedEntities);
            }
        }


        
        /// <summary>
        /// Number of entities used in the solver updateable.
        /// Note that this is set automatically by the sortInvolvedEntities method
        /// if it is called.
        /// </summary>
        protected internal int numberOfInvolvedEntities;




        /// <summary>
        /// Gets the solver group that manages this solver updateable, if any.
        /// Null if not owned by a solver group.
        /// </summary>
        public SolverGroup SolverGroup { get; protected internal set; }


        /// <summary>
        /// Acquires exclusive access to all entities involved in the solver updateable.
        /// </summary>
        public override void EnterLock()
        {
            for (int i = 0; i < numberOfInvolvedEntities; i++)
            {
                if (involvedEntities.Elements[i].isDynamic) //Only need to lock dynamic entities.
                {
                    involvedEntities.Elements[i].locker.Enter();
                    //Monitor.Enter(involvedEntities.Elements[i].locker);
                }
            }
        }

        /// <summary>
        /// Releases exclusive access to the updateable's entities.
        /// This should be called within a 'finally' block following a 'try' block containing the locked operations.
        /// </summary>
        public override void ExitLock()
        {
            for (int i = numberOfInvolvedEntities - 1; i >= 0; i--)
            {
                if (involvedEntities.Elements[i].isDynamic) //Only need to lock dynamic entities.
                    involvedEntities.Elements[i].locker.Exit();
                    //Monitor.Exit(involvedEntities[i].locker);
            }
        }

        /// <summary>
        /// Attempts to acquire exclusive access to all entities involved in the solver updateable.
        /// </summary>
        /// <returns>Whether or not the lock was entered successfully.</returns>
        public override bool TryEnterLock()
        {
            for (int i = 0; i < numberOfInvolvedEntities; i++)
            {
                if (involvedEntities.Elements[i].isDynamic) //Only need to lock dynamic entities.
                    if (!involvedEntities.Elements[i].locker.TryEnter())
                    {
                        //Turns out we can't take all the resources! Immediately drop everything.
                        for (i = i - 1 /*failed on the ith element, so start at the previous*/; i >= 0; i--)
                        {
                            if (involvedEntities[i].isDynamic)
                                involvedEntities.Elements[i].locker.Exit();
                        }
                        return false;
                    }
            }
            return true;

            //for (int i = 0; i < numberOfInvolvedEntities; i++)
            //{
            //    if (involvedEntities[i].isDynamic) //Only need to lock dynamic entities.
            //        if (!Monitor.TryEnter(involvedEntities[i].locker))
            //        {
            //            //Turns out we can't take all the resources! Immediately drop everything.
            //            for (i = i - 1 /*failed on the ith element, so start at the previous*/; i >= 0; i--)
            //            {
            //                if (involvedEntities[i].isDynamic)
            //                    Monitor.Exit(involvedEntities[i].locker);
            //            }
            //            return false;
            //        }
            //}
            //return true;
        }


        void UpdateConnectedMembers()
        {
            connectedMembers.Clear();
            for (int i = 0; i < involvedEntities.count; i++)
            {
                connectedMembers.Add(involvedEntities.Elements[i]);
            }
        }


        /// <summary>
        /// Adds entities associated with the solver item to the involved entities list.
        /// This allows the non-batched multithreading system to lock properly.
        /// </summary>
        protected internal abstract void CollectInvolvedEntities(RawList<Entity> outputInvolvedEntities);



        /// <summary>
        /// Handle any bookkeeping needed when the entities involved in this SolverUpdateable change.
        /// </summary>
        protected internal virtual void OnInvolvedEntitiesChanged()
        {
            //First verify that something really changed.
            bool entitiesChanged = false;
            RawList<Entity> newInvolvedEntities = Resources.GetEntityRawList();
            CollectInvolvedEntities(newInvolvedEntities);
            if (newInvolvedEntities.count == involvedEntities.count)
            {
                for (int i = 0; i < newInvolvedEntities.Count; i++)
                {
                    if (newInvolvedEntities.Elements[i] != involvedEntities.Elements[i])
                    {
                        entitiesChanged = true;
                        break;
                    }
                }
            }
            else
            {
                entitiesChanged = true;
            }

            //TODO: This code is bad.  It refers back to a specific kind of solver (batched) and is just messy overall.
            if (entitiesChanged)
            {
                //Probably need to wake things up given that such a significant change was made.
                for (int i = 0; i < involvedEntities.count; i++)
                {
                    Entity e = involvedEntities.Elements[i];
                    if (e.isDynamic && !e.isActive)
                        e.IsActive = true;
                }


                //Since we're about to change this updateable's connections, make sure the 
                //simulation islands hear about it.  This is NOT thread safe.
                var activityManager = (this as ISimulationIslandConnection).DeactivationManager;
                if (activityManager != null) //solver != null && SolverGroup == null)
                    activityManager.Remove(this);
                //Note that the above means that anything in the Solver must belong to an DeactivationManager in order to maintain
                //the correct connected references.
                //Since the default implementation of the Solver does behave in such a way, it's okay, but watch out when 
                //making any large changes.


                //if it's a batched solver, the batches need to be updated.
                //BatchedSolver batchedSolver;
                //if (solver != null && SolverGroup == null && solver.AllowMultithreading && (batchedSolver = solver as BatchedSolver) != null)
                //{
                //    batchedSolver.Remove(this);
                //    involvedEntities.Clear();
                //    involvedEntities.AddRange(newInvolvedEntities);
                //    SortInvolvedEntities();
                //    batchedSolver.Add(this);
                //}
                //else
                //{
                    //If it's not a batched solver, 
                    involvedEntities.Clear();
                    involvedEntities.AddRange(newInvolvedEntities);
                    SortInvolvedEntities();
                 //}
                //Connected members need to be updated too..
                UpdateConnectedMembers();

                
                //Add the new references back.
                if (activityManager != null)//solver != null && SolverGroup == null)
                    activityManager.Add(this);

              

                if (SolverGroup != null)
                    SolverGroup.OnInvolvedEntitiesChanged();

                //We woke up the FORMER involved entities, now wake up the current involved entities.
                for (int i = 0; i < involvedEntities.count; i++)
                {
                    Entity e = involvedEntities.Elements[i];
                    if (e.isDynamic && !e.isActive)
                        e.IsActive = true;
                }
            }
            Resources.GiveBack(newInvolvedEntities);
        }

        /// <summary>
        /// Collects the entities involved in a solver updateable and sets up the internal listings.
        /// </summary>
        protected internal void CollectInvolvedEntities()
        {
            involvedEntities.Clear();
            CollectInvolvedEntities(involvedEntities);
            SortInvolvedEntities();
            UpdateConnectedMembers();
        }

        /// <summary>
        /// Sorts the involved entities according to their hashcode to allow non-batched multithreading to avoid deadlocks.
        /// </summary>
        protected internal void SortInvolvedEntities()
        {
            numberOfInvolvedEntities = involvedEntities.Count;
            involvedEntities.Sort(comparer);
        }

        private static EntityComparer comparer = new EntityComparer();
        private class EntityComparer : IComparer<Entity>
        {
            #region IComparer<Entity> Members

            int IComparer<Entity>.Compare(Entity x, Entity y)
            {
                if (x.GetHashCode() > y.GetHashCode())
                    return 1;
                if (x.GetHashCode() < y.GetHashCode())
                    return -1;
                return 0;
            }

            #endregion
        }


        //SolverBatch ISolverBatchUpdateable.SolverBatch
        //{
        //    get
        //    {
        //        return batch;
        //    }
        //    set
        //    {
        //        batch = value;
        //    }
        //}

        //ReadOnlyCollection<ISimulationIslandMember> ISolverBatchUpdateable.ConnectedMembers { get { return (this as ISimulationIslandConnection).ConnectedMembers; } }


        /// <summary>
        /// Sets the activity state of the constraint based on the activity state of its connections.
        /// Called automatically by the space owning a constaint.  If a constraint is a sub-constraint that hasn't been directly added to the space,
        /// this may need to be called alongside the preStep from within the parent constraint.
        /// </summary>
        public override void UpdateSolverActivity()
        {
            if (isActive)
            {
                for (int i = 0; i < involvedEntities.Count && !isActiveInSolver; i++)
                {
                    isActiveInSolver |= involvedEntities[i].isActive;
                }
            }
            else
            {
                isActiveInSolver = false;
            }
        }

  

    }
}