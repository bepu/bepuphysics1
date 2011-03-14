using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Collidables;
using BEPUphysics.NarrowPhaseSystems.Factories;
using BEPUphysics.CollisionRuleManagement;

namespace BEPUphysics.NarrowPhaseSystems.Pairs
{
    ///<summary>
    /// Superclass of pairs between collidables that generate contact points.
    ///</summary>
    public abstract class CollidablePairHandler : INarrowPhasePair
    {
        protected CollidablePairHandler()
        {
            Contacts = new ContactCollection(this);
        }

        ///<summary>
        /// Updates the pair handler.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        public abstract void UpdateCollision(float dt);

        protected internal float timeOfImpact = 1;
        ///<summary>
        /// Gets the last computed time of impact of the pair handler.
        /// This is only computed when one of the members is a continuously
        /// updated object.
        ///</summary>
        public float TimeOfImpact
        {
            get
            {
                return timeOfImpact;
            }
        }

        ///<summary>
        /// Updates the time of impact for the pair.
        ///</summary>
        ///<param name="requester">Collidable requesting the update.</param>
        ///<param name="dt">Timestep duration.</param>
        public abstract void UpdateTimeOfImpact(Collidable requester, float dt);
        
        bool INarrowPhasePair.NeedsUpdate
        {
            get;
            set;
        }

        internal BroadPhaseOverlap broadPhaseOverlap;
        ///<summary>
        /// Gets the broad phase overlap associated with this pair handler.
        ///</summary>
        public BroadPhaseOverlap BroadPhaseOverlap
        {
            get
            {
                return broadPhaseOverlap;
            }
        }

        ///<summary>
        /// Gets or sets the collision rule governing this pair handler.
        ///</summary>
        public CollisionRule CollisionRule
        {
            get
            {
                return broadPhaseOverlap.collisionRule;
            }
            set
            {
                broadPhaseOverlap.collisionRule = value;
            }
        }

        BroadPhaseOverlap INarrowPhasePair.BroadPhaseOverlap
        {
            get
            {
                return broadPhaseOverlap;
            }
            set
            {
                broadPhaseOverlap = value;
                Initialize(value.entryA, value.entryB);
            }
        }

        NarrowPhasePairFactory INarrowPhasePair.Factory
        {
            get;
            set;
        }

        NarrowPhase narrowPhase;
        ///<summary>
        /// Gets the narrow phase that owns this pair handler.
        ///</summary>
        public NarrowPhase NarrowPhase
        {
            get
            {
                return narrowPhase;
            }
            set
            {
                narrowPhase = value;
            }
        }

        protected bool suppressEvents;
        ///<summary>
        /// Gets or sets whether or not to suppress events from this pair handler.
        ///</summary>
        public bool SuppressEvents
        {
            get
            {
                return suppressEvents;
            }
            set
            {
                suppressEvents = value;
            }
        }

        ///<summary>
        /// Gets or sets the parent of this pair handler.
        /// Pairs with parents report to their parents various
        /// changes in state.  This is mainly used to support
        /// hierarchies of pairs for compound collisions.
        ///</summary>
        public IPairHandlerParent Parent { get; set; }



        ///<summary>
        /// Initializes the pair handler.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        ///<param name="entryB">Second entry in the pair.</param>
        public abstract void Initialize(BroadPhaseEntry entryA, BroadPhaseEntry entryB);

        ///<summary>
        /// Called when the pair handler is added to the narrow phase.
        ///</summary>
        public abstract void OnAddedToNarrowPhase();

        ///<summary>
        /// Cleans up the pair handler.
        ///</summary>
        public virtual void CleanUp()
        {
            broadPhaseOverlap = new BroadPhaseOverlap();
            (this as INarrowPhasePair).NeedsUpdate = false;
            (this as INarrowPhasePair).NarrowPhase = null;
            suppressEvents = false;
            timeOfImpact = 1;
            Parent = null;
        }

        ///<summary>
        /// Forces an update of the pair's material properties.
        ///</summary>
        public abstract void UpdateMaterialProperties();


        internal abstract void GetContactInformation(int index, out ContactInformation info);

        internal abstract int ContactCount { get; }

        ///<summary>
        /// Gets a list of the contacts in the pair and their associated constraint information.
        ///</summary>
        public ContactCollection Contacts { get; private set; }
    }
}
