using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.DataStructures;
using BEPUphysics.Entities;
using System.Threading;

namespace BEPUphysics.DeactivationManagement
{
    /// <summary>
    /// Object owned by an entity which lives in a simulation island.
    /// Can be considered the entity's deactivation system proxy, just as the CollisionInformation property stores the collision pipeline proxy.
    /// </summary>
    public class SimulationIslandMember
    {
        //This system could be expanded to allow non-entity simulation island members.
        //However, there are no such objects on the near horizon, and it is unlikely that anyone will be interested in developing custom simulation island members.
        Entity owner;
        float previousVelocity;
        internal float velocityTimeBelowLimit;
        internal bool isSlowing;

        /// <summary>
        /// Gets the entity that owns this simulation island member.
        /// </summary>
        public Entity Owner
        {
            get
            {
                return owner;
            }
        }

        internal SimulationIslandMember(Entity owner)
        {
            this.owner = owner;
        }

        internal RawList<SimulationIslandConnection> connections = new RawList<SimulationIslandConnection>(8);
        ///<summary>
        /// Gets the connections associated with this member.
        ///</summary>
        public ReadOnlyList<SimulationIslandConnection> Connections
        {
            get
            {
                return new ReadOnlyList<SimulationIslandConnection>(connections);
            }
        }

        ///<summary>
        /// Updates the member's deactivation state.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        public void UpdateDeactivationCandidacy(float dt)
        {
            //Get total velocity, and see if the entity is losing energy.
            float velocity = owner.linearVelocity.LengthSquared() + owner.angularVelocity.LengthSquared();

            bool isActive = IsActive;
            if (isActive)
            {
                TryToCompressIslandHierarchy();
                isSlowing = velocity <= previousVelocity;
                if (IsDynamic)
                {

                    //Update time entity's been under the low-velocity limit, or reset if it's not
                    if (velocity < DeactivationManager.velocityLowerLimitSquared)
                        velocityTimeBelowLimit += dt;
                    else
                        velocityTimeBelowLimit = 0;

                    if (!IsAlwaysActive)
                    {
                        if (!isDeactivationCandidate)
                        {
                            //See if the velocity has been low long enough to make this object a deactivation candidate.
                            if (velocityTimeBelowLimit > DeactivationManager.lowVelocityTimeMinimum &&
                                isSlowing) //Only deactivate if it is NOT increasing in speed.
                            {
                                IsDeactivationCandidate = true;
                            }
                        }
                        else
                        {
                            //See if velocity is high enough to make this object not a deactivation candidate.
                            if (velocityTimeBelowLimit <= DeactivationManager.lowVelocityTimeMinimum)
                            {
                                IsDeactivationCandidate = false;
                            }
                        }


                    }
                    else
                        IsDeactivationCandidate = false;


                }
                else
                {
                    //Update the flagging system.
                    //If time <= 0, the entity is considered active.
                    //Forcing a kinematic active needs to allow the system to run for a whole frame.
                    //This means that in here, if it is < 0, we set it to zero.  It will still update for the rest of the frame.
                    //Then, next frame, when its == 0, set it to 1.  It will be considered inactive unless it was activated manually again.

                    if (velocityTimeBelowLimit == 0)
                        velocityTimeBelowLimit = 1;
                    else if (velocityTimeBelowLimit < 0)
                        velocityTimeBelowLimit = 0;

                    //If it's not dynamic, then deactivation candidacy is based entirely on whether or not the object has velocity.
                    if (velocity == 0)
                    {
                        IsDeactivationCandidate = true;
                    }
                    else
                    {
                        //Alright, so it's moving.
                        IsDeactivationCandidate = false;
                        //There's a single oddity we need to worry about in this case.
                        //An active kinematic object has no simulation island.  Without intervention,
                        //an active kinematic object will not keep an island awake.
                        //To solve this, when we encounter active kinematic objects,
                        //tell simulation islands associated with connected objects that they aren't allowed to deactivate.

                        for (int i = 0; i < connections.count; i++)
                        {
                            var connectedMembers = connections.Elements[i].members;
                            for (int j = connectedMembers.count - 1; j >= 0; j--)
                            {
                                //The change locker must be obtained before attempting to access the SimulationIsland.
                                //Path compression can force the simulation island to evaluate to null briefly.
                                //Do not permit the object to undergo path compression during this (brief) operation.
                                connectedMembers.Elements[j].simulationIslandChangeLocker.Enter();
                                if (connectedMembers.Elements[j].SimulationIsland != null)
                                    connectedMembers.Elements[j].SimulationIsland.allowDeactivation = false;
                                connectedMembers.Elements[j].simulationIslandChangeLocker.Exit();
                                //Note: Is it possible for an active kinematic to be involved with a simulation island that is currently inactive?
                                //Collision pair creation/merge should take care of the entry case, while this takes care of the maintenance.
                                //But, if it turns out that's wrong later, the simulation island could be forced active here.  It's a simple boolean operation, and would only ever be set to true during this stage.
                                //If there's a nasty special case that takes care of kinematics creating pairs, then adding in that force-activate would be a good idea to localize the effect
                                //of the deactivation system.
                            }
                        }


                    }
                }
            }
            previousVelocity = velocity;

            //These will be 'eventually right.'
            if (previouslyActive && !isActive)
                OnDeactivated();
            else if (!previouslyActive && isActive)
                OnActivated();
            previouslyActive = isActive;
        }

        bool isDeactivationCandidate;
        ///<summary>
        /// Gets or sets whether or not the object is a deactivation candidate.
        ///</summary>
        public bool IsDeactivationCandidate
        {
            get { return isDeactivationCandidate; }
            private set
            {
                if (value && !isDeactivationCandidate)
                {
                    isDeactivationCandidate = true;
                    OnBecameDeactivationCandidate();
                }
                else if (!value && isDeactivationCandidate)
                {
                    isDeactivationCandidate = false;
                    OnBecameNonDeactivationCandidate();
                }
                if (!value)
                {
                    velocityTimeBelowLimit = 0;
                }
            }
        }

        internal BEPUphysics.Threading.SpinLock simulationIslandChangeLocker = new BEPUphysics.Threading.SpinLock();
        void TryToCompressIslandHierarchy()
        {

            var currentSimulationIsland = simulationIsland;
            if (currentSimulationIsland != null)
            {
                if (currentSimulationIsland.immediateParent != currentSimulationIsland)
                {
                    //Only remove ourselves from the owning simulation island, not all the way up the chain.
                    //The change locker must be obtained first to prevent kinematic notifications in the candidacy update 
                    //from attempting to evaluate the SimulationIsland while we are reorganizing things.
                    simulationIslandChangeLocker.Enter();
                    lock (currentSimulationIsland)
                        currentSimulationIsland.Remove(this);
                    currentSimulationIsland = currentSimulationIsland.Parent;
                    //Add ourselves to the new owner.
                    lock (currentSimulationIsland)
                        currentSimulationIsland.Add(this);
                    simulationIslandChangeLocker.Exit();
                    //TODO: Should it activate the new island?  This might avoid a possible corner case.
                    //It could interfere with the activated event meaningfulness, since that is triggered
                    //at the end of the update candidacy loop..
                    //currentSimulationIsland.isActive = true;
                }
            }
        }

        bool previouslyActive = true;
        ///<summary>
        /// Gets whether or not the member is active.
        ///</summary>
        public bool IsActive
        {
            get
            {
                var currentSimulationIsland = SimulationIsland;
                if (currentSimulationIsland != null)
                {
                    return currentSimulationIsland.isActive;
                }
                else
                {
                    //If the simulation island is null,
                    //then this member has either not been added to a deactivation manager,
                    //or it is a kinematic entity.
                    //In either case, using the previous velocity is a reasonable approach.

                    //Kinematic objects use the time as a flag- if it's greater than zero,
                    //then the entity's been awake for at least a frame and go to sleep.
                    //If it is zero, though, then the kinematic needs to be considered awake.
                    return previousVelocity > 0 || velocityTimeBelowLimit <= 0;
                }
            }

        }

        /// <summary>
        /// Attempts to activate the entity.
        /// </summary>
        public void Activate()
        {
            //If we're trying to activate, always set the deactivation candidacy to false.  This resets the timer if necessary.
            IsDeactivationCandidate = false;
            var currentSimulationIsland = SimulationIsland;
            if (currentSimulationIsland != null)
            {
                //We can force-activate an island.
                //Note that this does nothing for objects not in a space
                //or kinematic objects that don't have an island.
                //"Activating" a kinematic object is meaningless- their activity state
                //is entirely defined by their velocity.
                currentSimulationIsland.IsActive = true;

            }
            else
            {
                //"Wake up" the kinematic entity.
                //The time is used as a flag.  If time <= 0, that means the object will be considered active until the subsequent update.
                velocityTimeBelowLimit = -1;
            }

        }

        /// <summary>
        /// Gets or sets whether or not this member is always active.
        /// </summary>
        public bool IsAlwaysActive { get; set; }

        internal bool allowStabilization = true;
        /// <summary>
        /// Gets or sets whether or not the entity can be stabilized by the deactivation system.  This allows systems of objects to go to sleep faster.
        /// Defaults to true.
        /// </summary>
        public bool AllowStabilization
        {
            get
            {
                return allowStabilization;
            }
            set
            {
                allowStabilization = value;
            }
        }

        //simulationisland should hook into the activated event.  If it is fired and the simulation island is inactive, the simulation island should activate.
        //Obviously only call event if it goes from inactive to active.
        ///<summary>
        /// Fired when the object activates.
        ///</summary>
        public event Action<SimulationIslandMember> Activated;
        ///<summary>
        /// Fired when the object becomes a deactivation candidate.
        ///</summary>
        public event Action<SimulationIslandMember> BecameDeactivationCandidate; //semi-horrible name
        ///<summary>
        /// Fired when the object is no longer a deactivation candidate.
        ///</summary>
        public event Action<SimulationIslandMember> BecameNonDeactivationCandidate; //horrible name
        ///<summary>
        /// Fired when the object deactivates.
        ///</summary>
        public event Action<SimulationIslandMember> Deactivated;

        protected internal void OnActivated()
        {
            if (Activated != null)
                Activated(this);
        }

        protected internal void OnBecameDeactivationCandidate()
        {
            if (BecameDeactivationCandidate != null)
                BecameDeactivationCandidate(this);
        }

        protected internal void OnBecameNonDeactivationCandidate()
        {
            if (BecameNonDeactivationCandidate != null)
                BecameNonDeactivationCandidate(this);
        }

        protected internal void OnDeactivated()
        {
            if (Deactivated != null)
                Deactivated(this);
        }


        internal SimulationIsland simulationIsland;
        ///<summary>
        /// Gets the simulation island that owns this member.
        ///</summary>
        public SimulationIsland SimulationIsland
        {
            get
            {
                return simulationIsland != null ? simulationIsland.Parent : null;
            }
            internal set
            {
                simulationIsland = value;
            }
        }

        /// <summary>
        /// Gets the deactivation manager that is managing this member.
        /// </summary>
        public DeactivationManager DeactivationManager { get; internal set; }

        //This is appropriate because it allows kinematic entities, while still technically members (they inherit ISimulationIslandMember), to act as dead-ends.
        ///<summary>
        /// Gets whether or not the object is dynamic.
        /// Non-dynamic members act as dead-ends in connection graphs.
        ///</summary>
        public bool IsDynamic
        {
            get
            {
                return owner.isDynamic;
            }
        }

        ///<summary>
        /// Gets or sets the current search state of the simulation island member.  This is used by the simulation island system
        /// to efficiently split islands.
        ///</summary>
        internal SimulationIslandSearchState searchState;

        ///<summary>
        /// Removes a connection reference from the member.
        ///</summary>
        ///<param name="connection">Reference to remove.</param>
        internal void RemoveConnectionReference(SimulationIslandConnection connection)
        {
            connections.FastRemove(connection);
        }
        ///<summary>
        /// Adds a connection reference to the member.
        ///</summary>
        ///<param name="connection">Reference to add.</param>
        internal void AddConnectionReference(SimulationIslandConnection connection)
        {
            connections.Add(connection);
        }


    }


    ///<summary>
    /// Defines the current state of a simulation island member in a split attempt.
    ///</summary>
    public enum SimulationIslandSearchState
    {
        Unclaimed,
        OwnedByFirst,
        OwnedBySecond
    }
}
