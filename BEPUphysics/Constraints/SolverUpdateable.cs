using System.Collections.Generic;
using BEPUphysics.Constraints.SolverGroups;
using BEPUphysics.DeactivationManagement;
using BEPUphysics.Entities;
using BEPUutilities.DataStructures;

namespace BEPUphysics.Constraints
{
    /// <summary>
    /// Superclass of objects types which require solving by the velocity solver.
    /// These are updated within the internal iterative solver when owned by a space.
    /// </summary>
    public abstract class SolverUpdateable : ISimulationIslandConnectionOwner, ISpaceObject
    {

        internal int solverIndex;

        protected internal Solver solver;

        ///<summary>
        /// Gets the solver to which the solver updateable belongs.
        ///</summary>
        public virtual Solver Solver //Note: this is virtual because some child classes (SolverGroups) need to perform their own logic when a solver gets set.
        {
            get { return solver; }
            protected internal set { solver = value; }
        }

        protected internal SimulationIslandConnection simulationIslandConnection;

        /// <summary>
        /// Gets the simulation island connection associated with this updateable.
        /// </summary>
        public SimulationIslandConnection SimulationIslandConnection
        {
            get { return simulationIslandConnection; }
        }


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

        protected internal SolverSettings solverSettings = new SolverSettings();
        ///<summary>
        /// Gets the solver settings that manage how the solver updates.
        ///</summary>
        public SolverSettings SolverSettings
        {
            get { return solverSettings; }
        }

        protected internal bool isActive = true;
        /// <summary>
        /// Gets or sets whether or not this solver updateable is active.
        /// 
        /// When set to false, this solver updateable will be idle and its 
        /// isActiveInSolver field will always be false.
        /// 
        /// When set to true, the solver updateable will run normally and update if
        /// the type's activity conditions allow it.
        /// </summary>
        public bool IsActive
        {
            get { return isActive; }
            set
            {
                if (value != isActive)
                {
                    //A constraint appearing or disappearing changes the constraint system.
                    //Something that went to sleep because a constraint reached a rest state
                    //could need to solve if the constraint turned on or off.
                    ActivateInvolvedEntities();
                    isActive = value;
                }
            }
        }

        protected internal bool isActiveInSolver = true;
        /// <summary>
        /// Gets whether or not the space's solver should try to solve this object.
        /// Depends on conditions specific to each solver updateable type and whether or not
        /// it has completed its computations early.  Recomputed each frame.
        /// </summary>
        public bool IsActiveInSolver
        {
            get
            {
                return isActiveInSolver;
            }
        }

        /// <summary>
        /// Activates all entities involved with this solver updateable.
        /// </summary>
        public void ActivateInvolvedEntities()
        {
            for (int i = 0; i < involvedEntities.Count; i++)
            {
                if (involvedEntities[i].isDynamic)
                {
                    //Only need to wake up one dynamic entity.  That will wake up the rest.
                    //Wouldn't want to pointlessly force-wake a kinematic object.
                    involvedEntities[i].activityInformation.Activate();
                    break;
                }
            }
        }


        /// <summary>
        /// Gets the solver group that manages this solver updateable, if any.
        /// Null if not owned by a solver group.
        /// </summary>
        public SolverGroup SolverGroup { get; protected internal set; }


        protected SolverUpdateable()
        {
            //Initialize the connection.
            //It will usually be overridden and end up floating on back to the resource pool.
            simulationIslandConnection = PhysicsResources.GetSimulationIslandConnection();
            simulationIslandConnection.Owner = this;
        }




        /// <summary>
        /// Acquires exclusive access to all entities involved in the solver updateable.
        /// </summary>
        public void EnterLock()
        {
            for (int i = 0; i < numberOfInvolvedEntities; i++)
            {
                if (involvedEntities.Elements[i].isDynamic) //Only need to lock dynamic entities.
                {
                    involvedEntities.Elements[i].locker.Enter();
                }
            }
        }

        /// <summary>
        /// Releases exclusive access to the updateable's entities.
        /// This should be called within a 'finally' block following a 'try' block containing the locked operations.
        /// </summary>
        public void ExitLock()
        {
            for (int i = numberOfInvolvedEntities - 1; i >= 0; i--)
            {
                if (involvedEntities.Elements[i].isDynamic) //Only need to lock dynamic entities.
                    involvedEntities.Elements[i].locker.Exit();
            }
        }

        /// <summary>
        /// Attempts to acquire exclusive access to all entities involved in the solver updateable.
        /// If it is contested, the lock attempt is aborted.
        /// </summary>
        /// <returns>True if the lock was entered successfully, false otherwise.</returns>
        public bool TryEnterLock()
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
        }

        /// <summary>
        /// Updates the activity state of the solver updateable based on its members.
        /// </summary>
        public virtual void UpdateSolverActivity()
        {
            if (isActive)
            {
                //This is a simulation island connection.  We already know that all connected objects share the
                //same simulation island (or don't have one, in the case of kinematics).  All we have to do is test to see if that island is active!
                for (int i = 0; i < simulationIslandConnection.entries.Count; i++)
                {
                    var island = simulationIslandConnection.entries.Elements[i].Member.SimulationIsland;
                    if (island != null && island.isActive)
                    {
                        isActiveInSolver = true;
                        return;
                    }
                }
            }
            isActiveInSolver = false;
        }

        ///<summary>
        /// Performs the frame's configuration step.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        public abstract void Update(float dt);

        //Will be locked by the solver during multithreaded updates.
        /// <summary>
        /// Performs any pre-solve iteration work that needs exclusive
        /// access to the members of the solver updateable.
        /// Usually, this is used for applying warmstarting impulses.
        /// </summary>
        public abstract void ExclusiveUpdate();

        //Will be locked by the solver during multithreaded updates.
        /// <summary>
        /// Computes one iteration of the constraint to meet the solver updateable's goal.
        /// </summary>
        /// <returns>The rough applied impulse magnitude.</returns>
        public abstract float SolveIteration();


        /// <summary>
        /// Handle any bookkeeping needed when the entities involved in this SolverUpdateable change.
        /// </summary>
        protected internal virtual void OnInvolvedEntitiesChanged()
        {
            //First verify that something really changed.
            bool entitiesChanged = false;
            RawList<Entity> newInvolvedEntities = PhysicsResources.GetEntityRawList();
            CollectInvolvedEntities(newInvolvedEntities);
            if (newInvolvedEntities.Count == involvedEntities.Count)
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

            if (entitiesChanged)
            {
                //Probably need to wake things up given that such a significant change was made.

                for (int i = 0; i < involvedEntities.Count; i++)
                {
                    Entity e = involvedEntities.Elements[i];
                    if (e.isDynamic)
                    {
                        e.activityInformation.Activate();
                        break;//Don't bother activating other entities; they are all a part of the same simulation island.
                    }
                }

                //CollectInvolvedEntities will give the updateable a new simulationIslandConnection and get rid of the old one.
                CollectInvolvedEntities();



                if (SolverGroup != null)
                    SolverGroup.OnInvolvedEntitiesChanged();

                //We woke up the FORMER involved entities, now wake up the current involved entities.
                for (int i = 0; i < involvedEntities.Count; i++)
                {
                    Entity e = involvedEntities.Elements[i];
                    if (e.isDynamic)
                    {
                        e.activityInformation.Activate();
                        break; //Don't bother activating other entities; they are all a part of the same simulation island.
                    }
                }
            }
            PhysicsResources.GiveBack(newInvolvedEntities);
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
        /// Adds entities associated with the solver item to the involved entities list.
        /// This allows the non-batched multithreading system to lock properly.
        /// </summary>
        protected internal abstract void CollectInvolvedEntities(RawList<Entity> outputInvolvedEntities);

        /// <summary>
        /// Sorts the involved entities according to their hashcode to allow non-batched multithreading to avoid deadlocks.
        /// </summary>
        protected internal void SortInvolvedEntities()
        {
            numberOfInvolvedEntities = involvedEntities.Count;
            involvedEntities.Sort(comparer);
        }



        void UpdateConnectedMembers()
        {

            //Since we're about to change this updateable's connections, make sure the 
            //simulation islands hear about it.  This is NOT thread safe.
            var deactivationManager = simulationIslandConnection.DeactivationManager;

            //Orphan the simulation island connection since it's about to get replaced.
            //There's three possible situations here:
            //1) We belong to the DeactivationManager.
            //2) We don't belong to a DeactivationManager and the connection is slated for removal (we were in the deactivation manager before).
            //   This can happen when a solver updateable associated with a pair gets removed and cleaned up.
            //3) We don't belong to a DeactivationManager and the connection is not slated for removal (we weren't in a deactivation manager before).

            //In Case #1, all we have to do is orphan the connection and remove it from the manager. This performs any splits necessary. The replacement connection will force any necessary merges.
            //In Case #2, we were just removed but the connection is still considered to have an owner.
            //It won't get cleaned up by the removal, and doing it here would be premature: orphan the connection so the next deactivation manager splits flush cleans it up!
            //In Case #3, we have full control over the simulation island connection because there is no interaction with a deactivation manager. We can just get rid of it directly.
            simulationIslandConnection.Owner = null;
            if (deactivationManager != null)
            {
                deactivationManager.Remove(simulationIslandConnection);
            }
            else if (!simulationIslandConnection.SlatedForRemoval) //If it's already been removed, cleaning it ourselves would prevent proper simulation island splits in the deactivation manager split flush.
                PhysicsResources.GiveBack(simulationIslandConnection); //Well, since we're going to orphan the connection, we'll need to take care of its trash.


            //The SimulationIslandConnection is immutable.
            //So create a new one!
            //Assume we've already dealt with the old connection.
            simulationIslandConnection = PhysicsResources.GetSimulationIslandConnection();
            for (int i = 0; i < involvedEntities.Count; i++)
            {
                simulationIslandConnection.Add(involvedEntities.Elements[i].activityInformation);
            }
            simulationIslandConnection.Owner = this;


            //Add the new reference back.
            if (deactivationManager != null)
                deactivationManager.Add(simulationIslandConnection);

        }


        private static EntityComparer comparer = new EntityComparer();
        private class EntityComparer : IComparer<Entity>
        {
            #region IComparer<Entity> Members

            int IComparer<Entity>.Compare(Entity x, Entity y)
            {
                if (x.InstanceId > y.InstanceId)
                    return 1;
                if (x.InstanceId < y.InstanceId)
                    return -1;
                return 0;
            }

            #endregion
        }



        protected internal Space space;
        Space ISpaceObject.Space
        {
            get
            {
                return space;
            }
            set
            {
                space = value;
            }
        }


        /// <summary>
        /// Gets or sets the user data associated with this object.
        /// </summary>
        public object Tag { get; set; }

        /// <summary>
        /// Called after the object is added to a space.
        /// </summary>
        /// <param name="newSpace">Space to which this object was added.</param>
        public virtual void OnAdditionToSpace(Space newSpace)
        {
        }

        /// <summary>
        /// Called before an object is removed from its space.
        /// </summary>
        public virtual void OnRemovalFromSpace(Space oldSpace)
        {
        }

        ///<summary>
        /// Called when the updateable is added to a solver.
        ///</summary>
        ///<param name="newSolver">Solver to which the updateable was added.</param>
        public virtual void OnAdditionToSolver(Solver newSolver)
        {
        }

        /// <summary>
        /// Called when the updateable is removed from its solver.
        /// </summary>
        /// <param name="oldSolver">Solver from which the updateable was removed.</param>
        public virtual void OnRemovalFromSolver(Solver oldSolver)
        {
        }

    }
}