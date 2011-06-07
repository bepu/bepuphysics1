using BEPUphysics.BroadPhaseSystems;

namespace BEPUphysics.NarrowPhaseSystems.Pairs
{
    ///<summary>
    /// Defines an object which handles a collision between two broad phase entries.
    ///</summary>
    public interface INarrowPhasePair
    {
        ///<summary>
        /// Updates the collision between the broad phase entries.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        void UpdateCollision(float dt);

        ///<summary>
        /// Gets or sets whether or not the pair needs to be updated.
        /// Used by the NarrowPhase to manage pairs.
        ///</summary>
        bool NeedsUpdate { get; set; }

        ///<summary>
        /// Gets or sets the overlap used to create the pair.
        ///</summary>
        BroadPhaseOverlap BroadPhaseOverlap { get; set; }

        ///<summary>
        /// Gets or sets the factory that created the pair.
        ///</summary>
        NarrowPhasePairFactory Factory { get; set; }

        /// <summary>
        /// Gets or sets the narrow phase that owns this pair.
        /// </summary>
        NarrowPhase NarrowPhase { get; set; }

        ///<summary>
        /// Initializes the pair.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        ///<param name="entryB">Second entry in the pair.</param>
        void Initialize(BroadPhaseEntry entryA, BroadPhaseEntry entryB);
        /// <summary>
        /// Called when the pair is added to the narrow phase.
        /// </summary>
        void OnAddedToNarrowPhase();
        /// <summary>
        /// Cleans up the pair, preparing it to go inactive.
        /// </summary>
        void CleanUp();

    }
}
