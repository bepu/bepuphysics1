using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.OtherSpaceStages;
using BEPUphysics.Threading;

namespace BEPUphysics.Collidables.Events
{

    ///<summary>
    /// Event manager for BroadPhaseEntries (all types that live in the broad phase).
    ///</summary>
    ///<typeparam name="T">Some BroadPhaseEntry subclass.</typeparam>
    public class EntryEventManager<T> : IDeferredEventCreator, IEntryEventTriggerer where T : BroadPhaseEntry
    {
        protected internal T owner;
        ///<summary>
        /// Owner of the event manager.
        ///</summary>
        public T Owner
        {
            get
            {
                return owner;
            }
        }

        ///<summary>
        /// Constructs a new event manager.
        ///</summary>
        ///<param name="owner">Owner of the event manager.</param>
        public EntryEventManager(T owner)
        {
            this.owner = owner;
        }

        #region Events
        /// <summary>
        /// Fires when this entity's bounding box newly overlaps another entity's bounding box.
        /// </summary>
        public event PairCreatedEventHandler<T> PairCreated
        {
            add
            {
                InternalPairCreated += value;
                AddToEventfuls();
            }
            remove
            {
                InternalPairCreated -= value;
                VerifyEventStatus();
            }
        }

        /// <summary>
        /// Fires when this entity's bounding box no longer overlaps another entity's bounding box.
        /// </summary>
        public event PairRemovedEventHandler<T> PairRemoved
        {
            add
            {
                InternalPairRemoved += value;
                AddToEventfuls();
            }
            remove
            {
                InternalPairRemoved -= value;
                VerifyEventStatus();
            }
        }

        /// <summary>
        /// Fires when a pair is updated.
        /// </summary>
        public event PairUpdatedEventHandler<T> PairUpdated
        {
            add
            {
                InternalPairUpdated += value;
                AddToEventfuls();
            }
            remove
            {
                InternalPairUpdated -= value;
                VerifyEventStatus();
            }
        }

        /// <summary>
        /// Fires when a pair is updated.
        /// Unlike the PairUpdated event, this event will run inline instead of at the end of the space's update.
        /// Some operations are unsupported while the engine is updating, and be especially careful if internal multithreading is enabled.
        /// </summary>
        public event PairUpdatingEventHandler<T> PairUpdating;

        /// <summary>
        /// Fires when this entity's bounding box newly overlaps another entity's bounding box.
        /// Unlike the PairCreated event, this event will run inline instead of at the end of the space's update.
        /// Some operations are unsupported while the engine is updating, and be especially careful if internal multithreading is enabled.
        /// </summary>
        public event CreatingPairEventHandler<T> CreatingPair;

        /// <summary>
        /// Fires when this entity's bounding box no longer overlaps another entity's bounding box.
        /// Unlike the PairRemoved event, this event will run inline instead of at the end of the space's update.
        /// Some operations are unsupported while the engine is updating, and be especially careful if internal multithreading is enabled.
        /// </summary>
        public event RemovingPairEventHandler<T> RemovingPair;


        private event PairCreatedEventHandler<T> InternalPairCreated;
        private event PairRemovedEventHandler<T> InternalPairRemoved;
        private event PairUpdatedEventHandler<T> InternalPairUpdated;

        #endregion


        #region Supporting members

        /// <summary>
        /// Removes the entity from the space's list of eventful entities if no events are active.
        /// </summary>
        protected void VerifyEventStatus()
        {
            if (EventsAreInactive())
            {
                ((IDeferredEventCreator)this).IsActive = false;
            }
        }

        protected virtual bool EventsAreInactive()
        {
            return InternalPairCreated == null &&
                   InternalPairRemoved == null &&
                   InternalPairUpdated == null;
        }


        protected void AddToEventfuls()
        {
            ((IDeferredEventCreator)this).IsActive = true;
        }

        
        private DeferredEventDispatcher deferredEventDispatcher;
        DeferredEventDispatcher IDeferredEventCreator.DeferredEventDispatcher
        {
            get
            {
                return deferredEventDispatcher;
            }
            set
            {
                deferredEventDispatcher = value;
            }
        }

        readonly ConcurrentDeque<EventStoragePairCreated> eventStoragePairCreated = new ConcurrentDeque<EventStoragePairCreated>(0);
        readonly ConcurrentDeque<EventStoragePairRemoved> eventStoragePairRemoved = new ConcurrentDeque<EventStoragePairRemoved>(0);
        readonly ConcurrentDeque<EventStoragePairUpdated> eventStoragePairUpdated = new ConcurrentDeque<EventStoragePairUpdated>(0);

        void IDeferredEventCreator.DispatchEvents()
        {
            DispatchEvents();
        }
        protected virtual void DispatchEvents()
        {
            //Note: Deferred event creation should be performed sequentially with dispatching.
            //This means a event creation from this creator cannot occur ASYNCHRONOUSLY while DispatchEvents is running.
            EventStoragePairCreated collisionPairCreated;
            while (eventStoragePairCreated.TryUnsafeDequeueFirst(out collisionPairCreated))
                InternalPairCreated(owner, collisionPairCreated.other, collisionPairCreated.pair);
            EventStoragePairRemoved collisionPairRemoved;
            while (eventStoragePairRemoved.TryUnsafeDequeueFirst(out collisionPairRemoved))
                InternalPairRemoved(owner, collisionPairRemoved.other);
            EventStoragePairUpdated collisionPairUpdated;
            while (eventStoragePairUpdated.TryUnsafeDequeueFirst(out collisionPairUpdated))
                InternalPairUpdated(owner, collisionPairUpdated.other, collisionPairUpdated.pair);


        }


        public void OnPairCreated(BroadPhaseEntry other, NarrowPhasePair collisionPair)
        {
            if (InternalPairCreated != null)
                eventStoragePairCreated.Enqueue(new EventStoragePairCreated(other, collisionPair));
            if (CreatingPair != null)
                CreatingPair(owner, other, collisionPair);
        }

        public void OnPairRemoved(BroadPhaseEntry other)
        {
            if (InternalPairRemoved != null)
            {
                eventStoragePairRemoved.Enqueue(new EventStoragePairRemoved(other));
            }
            if (RemovingPair != null)
            {
                RemovingPair(owner, other);
            }
        }

        public void OnPairUpdated(BroadPhaseEntry other, NarrowPhasePair collisionPair)
        {
            if (InternalPairUpdated != null)
                eventStoragePairUpdated.Enqueue(new EventStoragePairUpdated(other, collisionPair));
            if (PairUpdating != null)
                PairUpdating(owner, other, collisionPair);
        }

        
        private bool isActive;
        //IsActive is enabled whenever this collision information can dispatch events.
        bool IDeferredEventCreator.IsActive
        {
            get
            {
                return isActive;
            }
            set
            {
                if (!isActive && value)
                {
                    isActive = true;
                    if (deferredEventDispatcher != null)
                        deferredEventDispatcher.CreatorActivityChanged(this);
                }
                else if (isActive && !value)
                {
                    isActive = false;
                    if (deferredEventDispatcher != null)
                        deferredEventDispatcher.CreatorActivityChanged(this);
                }
            }
        }

        ///<summary>
        /// Removes all event hooks from the manager.
        ///</summary>
        public virtual void RemoveAllEvents()
        {
            PairUpdating = null;
            CreatingPair = null;
            RemovingPair = null;
            InternalPairCreated = null;
            InternalPairRemoved = null;
            InternalPairUpdated = null;

            VerifyEventStatus();
        }

        #endregion



    }




}
