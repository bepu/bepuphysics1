using System;
using System.Collections.Generic;
using BEPUphysics.Threading;
using BEPUphysics.DataStructures;
using BEPUphysics.ResourceManagement;
using System.Collections.ObjectModel;

namespace BEPUphysics.DeactivationManagement
{
    ///<summary>
    /// Manages the sleeping states of objects.
    ///</summary>
    public class DeactivationManager : MultithreadedProcessingStage
    {
        private int maximumDeactivationsPerFrame = 100;
        private int deactivationIslandIndex;

        internal float velocityLowerLimitSquared = .07f;
        internal float lowVelocityTimeMinimum = 1f;

        ///<summary>
        /// Gets or sets the velocity under which the deactivation system will consider 
        /// objects to be deactivation candidates (if their velocity stays below the limit
        /// for the LowVelocityTimeMinimum).
        ///</summary>
        public float VelocityLowerLimit
        {
            get
            {
                return (float)Math.Sqrt(velocityLowerLimitSquared);
            }
            set
            {
                velocityLowerLimitSquared = value * value;
            }
        }

        /// <summary>
        /// Gets or sets the time limit above which the deactivation system will consider
        /// objects to be deactivation candidates (if their velocity stays below the VelocityLowerLimit for the duration).
        /// </summary>
        public float LowVelocityTimeMinimum
        {
            get
            {
                return lowVelocityTimeMinimum;
            }
            set
            {
                if (value <= 0)
                    throw new Exception("Must use a positive, non-zero value for deactivation time minimum.");
                lowVelocityTimeMinimum = value;
            }
        }

        internal bool useStabilization = true;
        ///<summary>
        /// Gets or sets whether or not to use a stabilization effect on nearly motionless objects.
        /// This removes a lot of energy from a system when things are settling down, allowing them to go 
        /// to sleep faster.  It also makes most simulations appear a lot more robust.
        ///</summary>
        public bool UseStabilization
        {
            get
            {
                return useStabilization;
            }
            set
            {
                useStabilization = value;
            }
        }



        //TryToSplit is NOT THREAD SAFE.  Only one TryToSplit should ever be run.
        Queue<ISimulationIslandMember> member1Friends = new Queue<ISimulationIslandMember>(), member2Friends = new Queue<ISimulationIslandMember>();
        List<ISimulationIslandMember> searchedMembers1 = new List<ISimulationIslandMember>(), searchedMembers2 = new List<ISimulationIslandMember>();

        ///<summary>
        /// Gets or sets the maximum number of objects to attempt to deactivate each frame.
        ///</summary>
        public int MaximumDeactivationsPerFrame { get { return maximumDeactivationsPerFrame; } set { maximumDeactivationsPerFrame = value; } }

        TimeStepSettings timeStepSettings;
        ///<summary>
        /// Gets or sets the time step settings used by the deactivation manager in determining which objects 
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
        /// Constructs a deactivation manager.
        ///</summary>
        ///<param name="timeStepSettings">The time step settings used by the manager.</param>
        public DeactivationManager(TimeStepSettings timeStepSettings)
        {
            Enabled = true;
            multithreadedCandidacyLoopDelegate = MultithreadedCandidacyLoop;
            this.timeStepSettings = timeStepSettings;
        }

        ///<summary>
        /// Constructs a deactivation manager.
        ///</summary>
        ///<param name="timeStepSettings">The time step settings used by the manager.</param>
        /// <param name="threadManager">Thread manager used by the manager.</param>
        public DeactivationManager(TimeStepSettings timeStepSettings, IThreadManager threadManager)
            : this(timeStepSettings)
        {
            ThreadManager = threadManager;
            AllowMultithreading = true;
        }
        //TODO: Deactivation Candidate Detection
        //-Could scan the entities of CURRENTLY ACTIVE simulation islands.
        //-Requires a List-format of active sim islands.
        //-Requires sim islands have a list-format entity set.
        //-Simulation islands of different sizes won't load-balance well on the xbox360; it would be fine on the pc though.
        //TODO: Simulation Island Deactivation

        RawList<ISimulationIslandMember> simulationIslandMembers = new RawList<ISimulationIslandMember>();
        RawList<SimulationIsland> simulationIslands = new RawList<SimulationIsland>();

        ///<summary>
        /// Gets the simulation islands currently in the manager.
        ///</summary>
        public ReadOnlyList<SimulationIsland> SimulationIslands
        {
            get
            {
                return new ReadOnlyList<SimulationIsland>(simulationIslands);
            }
        }

        UnsafeResourcePool<SimulationIsland> islandPool = new UnsafeResourcePool<SimulationIsland>();

        void GiveBackIsland(SimulationIsland island)
        {
            island.CleanUp();
            islandPool.GiveBack(island);
        }

        ///<summary>
        /// Adds a simulation island member to the manager.
        ///</summary>
        ///<param name="simulationIslandMember">Member to add.</param>
        ///<exception cref="Exception">Thrown if the member already belongs to a manager.</exception>
        public void Add(ISimulationIslandMember simulationIslandMember)
        {
            if (simulationIslandMember.DeactivationManager == null)
            {
                simulationIslandMember.IsActive = true;
                simulationIslandMember.DeactivationManager = this;
                simulationIslandMembers.Add(simulationIslandMember);
                if (simulationIslandMember.IsDynamic)
                {
                    AddSimulationIslandToMember(simulationIslandMember);
                }
                else
                {
                    RemoveSimulationIslandFromMember(simulationIslandMember);
                }
            }
            else
                throw new Exception("Cannot add that member to this DeactivationManager; it already belongs to a manager.");
        }

        /// <summary>
        /// Removes the member from this island.
        /// </summary>
        /// <param name="simulationIslandMember">Removes the member from the manager.</param>
        public void Remove(ISimulationIslandMember simulationIslandMember)
        {
            if (simulationIslandMember.DeactivationManager == this)
            {
                simulationIslandMember.IsActive = true;
                simulationIslandMember.DeactivationManager = null;
                simulationIslandMembers.Remove(simulationIslandMember);
                RemoveSimulationIslandFromMember(simulationIslandMember);

            }
            else
                throw new Exception("Cannot remove that member from this DeactivationManager; it belongs to a different or no manager.");
        }

        Action<int> multithreadedCandidacyLoopDelegate;
        void MultithreadedCandidacyLoop(int i)
        {
            if (simulationIslandMembers.Elements[i].IsActive)
                simulationIslandMembers.Elements[i].UpdateDeactivationCandidacy(timeStepSettings.TimeStepDuration);
        }

        protected override void UpdateMultithreaded()
        {
            ThreadManager.ForLoop(0, simulationIslandMembers.count, multithreadedCandidacyLoopDelegate);


            DeactivateObjects();
        }

        protected override void UpdateSingleThreaded()
        {
            for (int i = 0; i < simulationIslandMembers.count; i++)
                if (simulationIslandMembers.Elements[i].IsActive)
                    simulationIslandMembers.Elements[i].UpdateDeactivationCandidacy(timeStepSettings.TimeStepDuration);

            DeactivateObjects();

        }

        void DeactivateObjects()
        {
            //Deactivate only some objects each frame.
            int numberOfEntitiesDeactivated = 0;
            while (numberOfEntitiesDeactivated < maximumDeactivationsPerFrame && simulationIslands.count > 0)
            {
                deactivationIslandIndex = (deactivationIslandIndex + 1) % simulationIslands.count;
                var island = simulationIslands.Elements[deactivationIslandIndex];
                if (island.members.count == 0)
                {
                    //Found an orphan island left over from merge procedures or removal procedures.
                    //Shoo it on out.
                    simulationIslands.FastRemoveAt(deactivationIslandIndex);
                    GiveBackIsland(island);
                }
                else
                {
                    island.TryToDeactivate();
                    numberOfEntitiesDeactivated += island.members.count;
                }
            }
        }

        ///<summary>
        /// Adds a simulation island connection to the deactivation manager.
        ///</summary>
        ///<param name="connection">Connection to add.</param>
        ///<exception cref="ArgumentException">Thrown if the connection already belongs to a manager.</exception>
        public void Add(ISimulationIslandConnection connection)
        {
            //DO A MERGE IF NECESSARY
            if (connection.DeactivationManager == null)
            {
                connection.DeactivationManager = this;
                if (connection.ConnectedMembers.Count > 0)
                {
                    var island = connection.ConnectedMembers[0].SimulationIsland;
                    for (int i = 1; i < connection.ConnectedMembers.Count; i++)
                    {
                        SimulationIsland opposingIsland;
                        if (island != (opposingIsland = connection.ConnectedMembers[i].SimulationIsland))
                        {
                            //Need to do a merge between the two islands.
                            //Note that this merge may eliminate the need for a merge with subsequent connection if they belong to the same island.
                            island = Merge(island, opposingIsland);
                        }

                    }
                    connection.AddReferencesToConnectedMembers();
                }
            }
            else
            {
                throw new ArgumentException("Cannot add connection to deactivation manager; it already belongs to one.");
            }
        }

        private SimulationIsland Merge(SimulationIsland s1, SimulationIsland s2)
        {
            //Pull the smaller island into the larger island and set all members
            //of the smaller island to refer to the new island.

            //The simulation islands can be null; a connection can be a kinematic entity, which has no simulation island.
            //'Merging' a null island with an island simply gets back the island.
            if (s1 == null)
            {
                //Should still activate the island, though.
                s2.Activate();
                return s2;
            }
            if (s2 == null)
            {
                //Should still activate the island, though.
                s1.Activate();
                return s1;
            }

            if (s1.members.count < s2.members.count)
            {
                SimulationIsland biggerIsland;
                biggerIsland = s2;
                s2 = s1;
                s1 = biggerIsland;
            }

            if (s2.isActive && !s1.isActive)
                s1.Activate();
            if (s1.isActive && !s2.isActive)
                s2.Activate();

            for (int i = s2.members.count - 1; i >= 0; i--)
            {
                ISimulationIslandMember member = s2.members.Elements[i];
                s2.RemoveAt(i); //This is done instead of a clear because removing also modifies event hooks.
                s1.Add(member);
            }

            //By now, s2 is empty.  But we will not remove it!
            //Removing it here would require that we perform an O(n) index lookup on the whole simulationIslands list.
            //Instead, leave it floating in the list as an orphan.  The deactivation attempt loop will catch it eventually and
            //remove it using an extremely fast FastRemoveAt.
            //It sounds like a micro-optimization, but this code is running singlethreadedly- it needs to be as fast as possible because
            //we're wasting time that could be spent working all cores!


            //The larger one survives.
            return s1;
        }


        ///<summary>
        /// Removes a simulation island connection from the manager.
        ///</summary>
        ///<param name="connection">Connection to remove from the manager.</param>
        ///<exception cref="ArgumentException">Thrown if the connection does not belong to this manager.</exception>
        public void Remove(ISimulationIslandConnection connection)
        {
            if (connection.DeactivationManager == this)
            {
                connection.DeactivationManager = null;
                //Try to split by examining the connections and breadth-first searching outward.
                //If it is determined that a split is required, grab a new island and add it.
                //This is a little tricky because it's a theoretically N-way split.

                //For two members which have the same simulation island (they will initially), try to split.
                connection.RemoveReferencesFromConnectedMembers();
                for (int i = 0; i < connection.ConnectedMembers.Count; i++)
                {
                    for (int j = i + 1; j < connection.ConnectedMembers.Count; j++)
                    {
                        TryToSplit(connection.ConnectedMembers[i], connection.ConnectedMembers[j]);
                    }
                }
            }
            else
            {
                throw new ArgumentException("Cannot remove connection from activity manager; it is owned by a different or no activity manager.");
            }



        }



        private void TryToSplit(ISimulationIslandMember member1, ISimulationIslandMember member2)
        {
            //Can't split if they aren't even in the same island.
            //This also covers the case where the connection involves a kinematic entity that has no 
            //simulation island at all.
            if (member1.SimulationIsland != member2.SimulationIsland ||
                member1.SimulationIsland == null ||
                member2.SimulationIsland == null)
                return;

            //By now, we know the members belong to the same island and are not null.
            //Start a BFS starting from each member.
            //Two-way can complete the search quicker.

            member1Friends.Enqueue(member1);
            member2Friends.Enqueue(member2);
            searchedMembers1.Add(member1);
            searchedMembers2.Add(member2);
            member1.SearchState = SimulationIslandSearchState.OwnedByFirst;
            member2.SearchState = SimulationIslandSearchState.OwnedBySecond;

            while (member1Friends.Count > 0 && member2Friends.Count > 0)
            {


                ISimulationIslandMember currentNode = member1Friends.Dequeue();
                for (int i = 0; i < currentNode.Connections.Count; i++)
                {
                    for (int j = 0; j < currentNode.Connections[i].ConnectedMembers.Count; j++)
                    {
                        ISimulationIslandMember connectedNode;
                        if ((connectedNode = currentNode.Connections[i].ConnectedMembers[j]) != currentNode &&
                            connectedNode.SimulationIsland != null) //The connection could be connected to something that isn't in the Space and has no island, or it's not dynamic.
                        {
                            switch (connectedNode.SearchState)
                            {
                                case SimulationIslandSearchState.Unclaimed:
                                    //Found a new friend :)
                                    member1Friends.Enqueue(connectedNode);
                                    connectedNode.SearchState = SimulationIslandSearchState.OwnedByFirst;
                                    searchedMembers1.Add(connectedNode);
                                    break;
                                case SimulationIslandSearchState.OwnedBySecond:
                                    //Found our way to member2Friends set; cannot split!
                                    member1Friends.Clear();
                                    member2Friends.Clear();
                                    goto ResetSearchStates;
                            }

                        }
                    }
                }

                currentNode = member2Friends.Dequeue();
                for (int i = 0; i < currentNode.Connections.Count; i++)
                {
                    for (int j = 0; j < currentNode.Connections[i].ConnectedMembers.Count; j++)
                    {
                        ISimulationIslandMember connectedNode;
                        if ((connectedNode = currentNode.Connections[i].ConnectedMembers[j]) != currentNode &&
                            connectedNode.SimulationIsland != null) //The connection could be connected to something that isn't in the Space and has no island, or it's not dynamic.
                        {
                            switch (connectedNode.SearchState)
                            {
                                case SimulationIslandSearchState.Unclaimed:
                                    //Found a new friend :)
                                    member2Friends.Enqueue(connectedNode);
                                    connectedNode.SearchState = SimulationIslandSearchState.OwnedBySecond;
                                    searchedMembers2.Add(connectedNode);
                                    break;
                                case SimulationIslandSearchState.OwnedByFirst:
                                    //Found our way to member1Friends set; cannot split!
                                    member1Friends.Clear();
                                    member2Friends.Clear();
                                    goto ResetSearchStates;
                            }

                        }
                    }
                }
            }
            //If one of the queues empties out without finding anything, it means it's isolated.  The other one will never find it.
            //Now we can do a split.  Grab a new Island, fill it with the isolated search stuff.  Remove the isolated search stuff from the old Island.

            SimulationIsland newIsland = islandPool.Take();
            simulationIslands.Add(newIsland);
            if (member1Friends.Count == 0)
            {

                //Member 1 is isolated, give it its own simulation island!
                for (int i = 0; i < searchedMembers1.Count; i++)
                {
                    member2.SimulationIsland.Remove(searchedMembers1[i]);
                    newIsland.Add(searchedMembers1[i]);
                }
                member2Friends.Clear();
            }
            else if (member2Friends.Count == 0)
            {

                //Member 2 is isolated, give it its own simulation island!
                for (int i = 0; i < searchedMembers2.Count; i++)
                {
                    member1.SimulationIsland.Remove(searchedMembers2[i]);
                    newIsland.Add(searchedMembers2[i]);
                }
                member1Friends.Clear();
            }

        ResetSearchStates:
            for (int i = 0; i < searchedMembers1.Count; i++)
            {
                searchedMembers1[i].SearchState = SimulationIslandSearchState.Unclaimed;
            }
            for (int i = 0; i < searchedMembers2.Count; i++)
            {
                searchedMembers2[i].SearchState = SimulationIslandSearchState.Unclaimed;
            }
            searchedMembers1.Clear();
            searchedMembers2.Clear();


        }




        ///<summary>
        /// Strips a member of its simulation island.
        ///</summary>
        ///<param name="member">Member to be stripped.</param>
        public void RemoveSimulationIslandFromMember(ISimulationIslandMember member)
        {

            //Becoming kinematic eliminates the member as a possible path.
            //Splits must be attempted between its connected members.
            //Don't need to split same-connection members.  Splitting one non-null entry against a non null entry in each of the other connections will do the trick.
            if (member.SimulationIsland != null)
            {
                SimulationIsland island = member.SimulationIsland;
                island.Remove(member);
                if (island.members.count == 0)
                {
                    simulationIslands.Remove(island);
                    GiveBackIsland(island);
                    //Even though we appear to have connections, the island was only me!
                    //We can stop now.
                    //Note that we do NOT remove the island from the simulation islands list here.
                    //That would take an O(n) search.  Instead, orphan it and let the TryToDeactivate loop find it.
                    return;
                }
            }
            if (member.Connections.Count > 0)
            {
                for (int i = 0; i < member.Connections.Count; i++)
                {
                    //Find a member with a non-null island to represent connection i.
                    ISimulationIslandMember representativeA = null;
                    for (int j = 0; j < member.Connections[i].ConnectedMembers.Count; j++)
                    {
                        if (member.Connections[i].ConnectedMembers[j].SimulationIsland != null)
                        {
                            representativeA = member;
                            break;
                        }
                    }

                    if (representativeA == null)
                    {
                        //There was no representative!  That means it was a connection in which
                        //no member had a simulation island.  Consider removing a dynamic box from the space
                        //while it sits on a kinematic box.  Neither object has a simulation island.
                        //In this case, simply try the next connection.
                        continue;
                    }
                    //Split the representative against representatives from other connections.
                    for (int j = i + 1; j < member.Connections.Count; j++)
                    {
                        //Find a representative for another connection.
                        ISimulationIslandMember representativeB = null;
                        for (int k = 0; k < member.Connections[j].ConnectedMembers.Count; k++)
                        {
                            if (member.Connections[j].ConnectedMembers[k].SimulationIsland != null)
                            {
                                representativeB = member;
                                break;
                            }
                        }

                        if (representativeB == null)
                        {
                            //There was no representative!  Same idea as above.
                            //Try the next connection.
                            continue;
                        }

                        //Try to split the representatives.
                        TryToSplit(representativeA, representativeB);


                    }
                }
            }


        }

        ///<summary>
        /// Adds a simulation island to a member.
        ///</summary>
        ///<param name="member">Member to gain a simulation island.</param>
        ///<exception cref="Exception">Thrown if the member already has a simulation island.</exception>
        public void AddSimulationIslandToMember(ISimulationIslandMember member)
        {
            if (member.SimulationIsland != null)
            {
                throw new Exception("Cannot initialize member's simulation island; it already has one.");
            }
            if (member.Connections.Count > 0)
            {
                SimulationIsland island = null;
                //Find a simulation starting island to live in.
                for (int i = 0; i < member.Connections.Count; i++)
                {
                    for (int j = 0; j < member.Connections[i].ConnectedMembers.Count; j++)
                    {
                        island = member.Connections[i].ConnectedMembers[j].SimulationIsland;
                        if (island != null)
                        {
                            island.Add(member);
                            break;
                        }
                    }
                    if (island != null)
                        break;
                }
                if (member.SimulationIsland == null)
                {
                    //No non-null entries in any connections.  That's weird.
                    //Maybe it's connected to a bunch of kinematics, or maybe it's a vehicle-like situation
                    //where the body is associated with a 'vehicle' connection which sometimes contains only the body.

                    //No friends to merge with.
                    SimulationIsland newIsland = islandPool.Take();
                    simulationIslands.Add(newIsland);
                    newIsland.Add(member);
                    return;
                }


                //Becoming dynamic adds a new path.
                //Merges must be attempted between its connected members.
                for (int i = 0; i < member.Connections.Count; i++)
                {
                    for (int j = 0; j < member.Connections[i].ConnectedMembers.Count; j++)
                    {
                        if (member.Connections[i].ConnectedMembers[j] == member)
                            continue; //Don't bother trying to compare against ourselves.  That would cause an erroneous early-out sometimes.
                        SimulationIsland opposingIsland = member.Connections[i].ConnectedMembers[j].SimulationIsland;
                        if (opposingIsland != null)
                        {
                            if (island != opposingIsland)
                            {
                                island = Merge(island, opposingIsland);
                            }
                            //All non-null simulation islands in a single connection are guaranteed to be the same island due to previous merges.
                            //Once we find one, we can stop.
                            break;
                        }
                    }
                }
            }
            else
            {
                //No friends to merge with.
                SimulationIsland newIsland = islandPool.Take();
                simulationIslands.Add(newIsland);
                newIsland.Add(member);
            }

        }
    }
}
