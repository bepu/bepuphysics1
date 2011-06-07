using System;
using System.Collections.ObjectModel;
using BEPUphysics.DataStructures;

namespace BEPUphysics.DeactivationManagement
{
    ///<summary>
    /// Defines an object which can belong to a simulation island.
    ///</summary>
    public interface ISimulationIslandMember
    {
        //TODO: Investigate a three-stage deactivation process.
        //A deactivation candidate might be able to ignore some more stages given a certain ordering...
        //Having the deactivation candidacy update at the beginning of the frame may make this impossible.
        //If it was immediately before the update sequence?
        //Should it be before the update sequence anyway?
        //Don't worry about this too much until the rest of the system actually works.

        //TODO: Simulation island connections and/or connected ISimulationIslandMembers.
        //On deactivation candidate, notify owning simulation island that it happened.  It will do a thread safe decrement etc.
        //This also needs to allow Activity bools and stuff.
        //InteractingObjects are populated by a backing field by collision pairs and things.  The SimulationIsland system doesn't care how it happens.
        //Their source is from connecting more ISimulationIslandConnections.
        //TODO: This could be pretty annoying to do bookkeeping on.  Whenever things are added, it needs to check if it's already been added.  Not too bad...
        //But when something is removed, it has to check all other ISimulationIslandConnections on the member to see if any of them have it before removing.
        //That means the connections list must be available....
        ///<summary>
        /// Gets the connections associated with this member.
        ///</summary>
        ReadOnlyList<ISimulationIslandConnection> Connections { get; }

        //The system needs to be able to know if it's a deactivation candidate.
        //Possibly pass in the activity manager so that its settings can be observed from within the candidate
        ///<summary>
        /// Updates the member's deactivation state.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        void UpdateDeactivationCandidacy(float dt);
        ///<summary>
        /// Gets whether or not the object is a deactivation candidate.
        ///</summary>
        bool IsDeactivationCandidate { get; }

        //Getting activity is mainly useful to other systems.
        //The activity of the member can be separate from the island, if it is forced asleep when its island is still awake.
        //Cannot have an awake member in a sleeping sim island though.
        ///<summary>
        /// Gets whether or not the member is active.
        ///</summary>
        bool IsActive { get; set; }

        //simulationisland should hook into the activated event.  If it is fired and the simulation island is inactive, the simulation island should activate.
        //Obviously only call event if it goes from inactive to active.
        ///<summary>
        /// Fired when the object activates.
        ///</summary>
        event Action<ISimulationIslandMember> Activated;
        ///<summary>
        /// Fired when the object becomes a deactivation candidate.
        ///</summary>
        event Action<ISimulationIslandMember> BecameDeactivationCandidate; //semi-horrible name
        ///<summary>
        /// Fired when the object is no longer a deactivation candidate.
        ///</summary>
        event Action<ISimulationIslandMember> BecameNonDeactivationCandidate; //horrible name
        ///<summary>
        /// Fired when the object deactivates.
        ///</summary>
        event Action<ISimulationIslandMember> Deactivated;

        //Set needs to be implemented explicitly.  It could sometimes be a valid operation, but it's not something that needs to be exposed to the average user.
        ///<summary>
        /// Gets or sets the simulation island that owns this member.
        ///</summary>
        SimulationIsland SimulationIsland { get; set; }

        //This is appropriate because it allows kinematic entities, while still technically members (they inherit ISimulationIslandMember), to act as dead-ends.
        ///<summary>
        /// Gets whether or not the object is dynamic.
        /// Non-dynamic members act as dead-ends in connection graphs.
        ///</summary>
        bool IsDynamic { get; }

        //Implement this explicitly.  This isn't thread safe, but simulation islands are managed sequentially anyway.
        ///<summary>
        /// Gets or sets the current search state of the simulation island member.  This is used by the simulation island system
        /// to efficiently split islands.
        ///</summary>
        SimulationIslandSearchState SearchState { get; set; }

        ///<summary>
        /// Removes a connection reference from the member.
        ///</summary>
        ///<param name="connection">Reference to remove.</param>
        void RemoveConnectionReference(ISimulationIslandConnection connection);
        ///<summary>
        /// Adds a connection reference to the member.
        ///</summary>
        ///<param name="connection">Reference to add.</param>
        void AddConnectionReference(ISimulationIslandConnection connection);

        /// <summary>
        /// Gets or sets the deactivation manager that is managing this member.
        /// </summary>
        DeactivationManager DeactivationManager { get; set; }

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
