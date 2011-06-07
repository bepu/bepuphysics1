using System;
using System.Collections.Generic;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Constraints;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.SolverSystems;
using BEPUphysics.Threading;
using BEPUphysics.CollisionRuleManagement;
using System.Collections.ObjectModel;
using BEPUphysics.DataStructures;

namespace BEPUphysics.NarrowPhaseSystems
{
    ///<summary>
    /// Pair of types.
    ///</summary>
    public struct TypePair : IEquatable<TypePair>
    {
        //Currently this requires some reflective labor.  If perhaps the broad phase entries had some sort of 'id'... and they simply return that int through the interface.
        //Could be 'faster' assuming the supporting logic that creates the ids to begin with isn't too obtuse.
        ///<summary>
        /// First type in the pair.
        ///</summary>
        public Type A;

        ///<summary>
        /// Second type in the pair.
        ///</summary>
        public Type B;

        ///<summary>
        /// Constructs a new type pair.
        ///</summary>
        ///<param name="a">First type in the pair.</param>
        ///<param name="b">Second type in the pair.</param>
        public TypePair(Type a, Type b)
        {
            A = a;
            B = b;
        }

        /// <summary>
        /// Returns the hash code for this instance.
        /// </summary>
        /// <returns>
        /// A 32-bit signed integer that is the hash code for this instance.
        /// </returns>
        /// <filterpriority>2</filterpriority>
        public override int GetHashCode()
        {
            //TODO: Use old hash code system?
            return A.GetHashCode() + B.GetHashCode();
        }


        #region IEquatable<TypePair> Members

        /// <summary>
        /// Indicates whether the current object is equal to another object of the same type.
        /// </summary>
        /// <returns>
        /// true if the current object is equal to the <paramref name="other"/> parameter; otherwise, false.
        /// </returns>
        /// <param name="other">An object to compare with this object.</param>
        public bool Equals(TypePair other)
        {
            return (other.A == A && other.B == B) || (other.B == A && other.A == B);
        }

        #endregion
    }
    ///<summary>
    /// Manages and constructs pair handlers from broad phase overlaps.
    ///</summary>
    public class NarrowPhase : MultithreadedProcessingStage
    {
        RawList<BroadPhaseOverlap> broadPhaseOverlaps; //Could be a reference passed into the system somehow too... use rawlist?
        ///<summary>
        /// Gets or sets the list of broad phase overlaps used by the narrow phase to manage pairs.
        ///</summary>
        public RawList<BroadPhaseOverlap> BroadPhaseOverlaps { get { return broadPhaseOverlaps; } set { broadPhaseOverlaps = value; } }

        Dictionary<BroadPhaseOverlap, INarrowPhasePair> overlapMapping = new Dictionary<BroadPhaseOverlap, INarrowPhasePair>();
        List<INarrowPhasePair> narrowPhasePairs = new List<INarrowPhasePair>();
        ///<summary>
        /// Gets the list of Pairs managed by the narrow phase.
        ///</summary>
        public ReadOnlyList<INarrowPhasePair> Pairs
        {
            get
            {
                return new ReadOnlyList<INarrowPhasePair>(narrowPhasePairs);
            }
        }

        ///<summary>
        /// Gets or sets the time step settings used by the narrow phase.
        ///</summary>
        public TimeStepSettings TimeStepSettings { get; set; }


        ConcurrentDeque<INarrowPhasePair> newNarrowPhasePairs = new ConcurrentDeque<INarrowPhasePair>();

        ///<summary>
        /// The required number of pairs in the narrow phase to use multithreading.
        ///</summary>
        public static int MultithreadedRemovalCutoff = 200;

        ///<summary>
        /// Constructs a new narrow phase.
        ///</summary>
        ///<param name="timeStepSettings">Time step settings used by the narrow phase.</param>
        public NarrowPhase(TimeStepSettings timeStepSettings)
        {
            TimeStepSettings = timeStepSettings;
            updateBroadPhaseOverlapDelegate = UpdateBroadPhaseOverlap;
            multithreadedRemovalLoopDelegate = MultithreadedRemovalLoopBody;
            Enabled = true;
        }

        ///<summary>
        /// Constructs a new narrow phase.
        ///</summary>
        ///<param name="timeStepSettings">Time step settings used by the narrow phase.</param>
        /// <param name="overlaps">Overlaps list used by the narrow phase to create pairs.</param>
        public NarrowPhase(TimeStepSettings timeStepSettings, RawList<BroadPhaseOverlap> overlaps)
            : this(timeStepSettings)
        {
            broadPhaseOverlaps = overlaps;
        }
        ///<summary>
        /// Constructs a new narrow phase.
        ///</summary>
        ///<param name="timeStepSettings">Time step settings used by the narrow phase.</param>
        /// <param name="overlaps">Overlaps list used by the narrow phase to create pairs.</param>
        /// <param name="threadManager">Thread manager used by the narrow phase.</param>
        public NarrowPhase(TimeStepSettings timeStepSettings, RawList<BroadPhaseOverlap> overlaps, IThreadManager threadManager)
            : this(timeStepSettings, overlaps)
        {
            ThreadManager = threadManager;
            AllowMultithreading = true;
        }

        Action<int> updateBroadPhaseOverlapDelegate;
        void UpdateBroadPhaseOverlap(int i)
        {
            BroadPhaseOverlap overlap = broadPhaseOverlaps.Elements[i];
            if (overlap.collisionRule < CollisionRule.NoNarrowPhasePair)
            {
                INarrowPhasePair narrowPhaseObject;
                //see if the overlap is already present in the narrow phase.
                if (!overlapMapping.TryGetValue(overlap, out narrowPhaseObject))
                {
                    //Create/enqueue based on collision table
                    narrowPhaseObject = NarrowPhaseHelper.GetPair(ref overlap);
                    if (narrowPhaseObject != null)
                    {
                        narrowPhaseObject.NarrowPhase = this;
                        //Add the new object to the 'todo' list.
                        //Technically, this doesn't need to be thread-safe when this is called from the sequential context.
                        //It's just bunched together for maintainability despite the slightly performance hit.
                        newNarrowPhasePairs.Enqueue(narrowPhaseObject);
                    }
                }
                if (narrowPhaseObject != null)
                {

                    if (narrowPhaseObject.BroadPhaseOverlap.collisionRule < CollisionRule.NoNarrowPhaseUpdate)
                        narrowPhaseObject.UpdateCollision(TimeStepSettings.TimeStepDuration);
                    narrowPhaseObject.NeedsUpdate = false;
                }


            }
        }



        //Only used in the multithreaded case where there are a lot of narrow phase objects.
        ConcurrentDeque<INarrowPhasePair> overlapsToRemove = new ConcurrentDeque<INarrowPhasePair>();
        Action<int> multithreadedRemovalLoopDelegate;
        void MultithreadedRemovalLoopBody(int i)
        {
            INarrowPhasePair narrowPhaseObject = narrowPhasePairs[i];
            if (narrowPhaseObject.NeedsUpdate &&
                //Overlap will not be refreshed if entries are inactive, but shouldn't remove narrow phase pair.
                (narrowPhaseObject.BroadPhaseOverlap.entryA.IsActive || narrowPhaseObject.BroadPhaseOverlap.entryB.IsActive))
            {
                overlapsToRemove.Enqueue(narrowPhaseObject);
            }
            else
                narrowPhaseObject.NeedsUpdate = true;
        }


        protected override void UpdateMultithreaded()
        {
            ThreadManager.ForLoop(0, broadPhaseOverlaps.Count, updateBroadPhaseOverlapDelegate);

            //Remove stale objects.
            //TODO: This could benefit from a custom data structure (a tiny amount).
            //TODO: This could possibly be done with a computation spreading approach.
            //Problem: Consider a collision pair that has contacts one frame, and in the next, no longer even has a broad phase overlap.
            //It will receive no update, and the collision pair will still have a contact in it.
            //Collision solver will operate on permanently out of date information.
            //One possible solution requires that the user of the narrow phase object checks its age, and if it's out of date, ignore it.
            //In a subsequent frame, the system will get rid of it.  This has an advantage of lowering the (somewhat tiny) per frame cost of removal management.
            //Additionally, in highly chaotic situations where collisions are constantly being created/destroyed, spreading out the computations
            //smooths the work out a bit.
            if (narrowPhasePairs.Count < MultithreadedRemovalCutoff) //TODO: Configurable cutoff?
            {
                RemoveStaleOverlaps();
            }
            else
            {
                ThreadManager.ForLoop(0, narrowPhasePairs.Count, multithreadedRemovalLoopDelegate);
                INarrowPhasePair overlapToRemove;
                while (overlapsToRemove.TryUnsafeDequeueFirst(out overlapToRemove))
                {
                    narrowPhasePairs[narrowPhasePairs.IndexOf(overlapToRemove)] = narrowPhasePairs[narrowPhasePairs.Count - 1];
                    narrowPhasePairs.RemoveAt(narrowPhasePairs.Count - 1);
                    OnRemovePair(overlapToRemove);
                }
            }


            AddNewNarrowPhaseObjects();
        }


        protected override void UpdateSingleThreaded()
        {
            int count = broadPhaseOverlaps.Count;
            for (int i = 0; i < count; i++)
            {
                UpdateBroadPhaseOverlap(i);
            }


            RemoveStaleOverlaps();

            AddNewNarrowPhaseObjects();

        }



        void RemoveStaleOverlaps()
        {
            //Remove stale objects.
            //TODO: This could benefit from a custom data structure (a tiny amount).
            //TODO: This could possibly be done with a computation spreading approach.
            //Problem: Consider a collision pair that has contacts one frame, and in the next, no longer even has a broad phase overlap.
            //It will receive no update, and the collision pair will still have a contact in it.
            //Collision solver will operate on permanently out of date information.
            //One possible solution requires that the user of the narrow phase object checks its age, and if it's out of date, ignore it.
            //In a subsequent frame, the system will get rid of it.  This has an advantage of lowering the (somewhat tiny) per frame cost of removal management.
            //Additionally, in highly chaotic situations where collisions are constantly being created/destroyed, spreading out the computations
            //smooths the work out a bit.
            for (int i = narrowPhasePairs.Count - 1; i >= 0; i--)
            {
                INarrowPhasePair narrowPhaseObject = narrowPhasePairs[i];
                if (narrowPhaseObject.NeedsUpdate &&
                    //Overlap will not be refreshed if entries are inactive, but shouldn't remove narrow phase pair.
                    (narrowPhaseObject.BroadPhaseOverlap.entryA.IsActive || narrowPhaseObject.BroadPhaseOverlap.entryB.IsActive))
                {
                    narrowPhasePairs[i] = narrowPhasePairs[narrowPhasePairs.Count - 1];
                    narrowPhasePairs.RemoveAt(narrowPhasePairs.Count - 1);

                    OnRemovePair(narrowPhaseObject);
                }
                else
                    narrowPhaseObject.NeedsUpdate = true;
            }
        }

        void AddNewNarrowPhaseObjects()
        {
            //Add new narrow phase objects.  This will typically be a very tiny phase.
            INarrowPhasePair narrowPhaseObject;

            while (newNarrowPhasePairs.TryUnsafeDequeueFirst(out narrowPhaseObject))
            {
                narrowPhasePairs.Add(narrowPhaseObject);
                OnCreatePair(narrowPhaseObject);
            }
        }

        ///<summary>
        /// Gets the pair between two broad phase entries, if any.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        ///<param name="entryB">Second entry in the pair.</param>
        ///<returns>The pair if it exists, null otherwise.</returns>
        public INarrowPhasePair GetPair(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {
            INarrowPhasePair toReturn;
            overlapMapping.TryGetValue(new BroadPhaseOverlap(entryA, entryB), out toReturn);
            return toReturn;
        }

        protected void OnCreatePair(INarrowPhasePair pair)
        {
            overlapMapping.Add(pair.BroadPhaseOverlap, pair);
            pair.OnAddedToNarrowPhase();
            if (CreatingPair != null)
                CreatingPair(pair);
        }
        protected void OnRemovePair(INarrowPhasePair pair)
        {
            overlapMapping.Remove(pair.BroadPhaseOverlap);
            pair.CleanUp();
            pair.Factory.GiveBack(pair);
            if (RemovingPair != null)
                RemovingPair(pair);
        }
        ///<summary>
        /// Fires when the narrow phase creates a pair.
        ///</summary>
        public event Action<INarrowPhasePair> CreatingPair;
        ///<summary>
        /// Fires when the narrow phase removes a pair.
        ///</summary>
        public event Action<INarrowPhasePair> RemovingPair;

        ConcurrentDeque<SolverUpdateableChange> solverUpdateableChanges = new ConcurrentDeque<SolverUpdateableChange>();
        ///<summary>
        /// Enqueues a solver updateable created by some pair for flushing into the solver later.
        ///</summary>
        ///<param name="addedItem">Solver updateable to add.</param>
        public void EnqueueGeneratedSolverUpdateable(EntitySolverUpdateable addedItem)
        {
            solverUpdateableChanges.Enqueue(new SolverUpdateableChange(true, addedItem));
        }
        ///<summary>
        /// Enqueues a solver updateable removed by some pair for flushing into the solver later.
        ///</summary>
        ///<param name="removedItem">Solver updateable to add.</param>
        public void EnqueueRemovedSolverUpdateable(EntitySolverUpdateable removedItem)
        {
            solverUpdateableChanges.Enqueue(new SolverUpdateableChange(false, removedItem));
        }

        /// <summary>
        /// Flushes the solver updateable changes.
        /// </summary>
        /// <param name="solver">Solver to flush the changes into.</param>
        public void FlushGeneratedSolverUpdateables(Solver solver)
        {
            SolverUpdateableChange change;
            while (solverUpdateableChanges.TryUnsafeDequeueFirst(out change))
            {
                if (change.ShouldAdd)
                    solver.Add(change.Item);
                else
                    solver.Remove(change.Item);
            }
        }




        //The following are methods which can be used to split the narrow phase pair creation
        //from the pair update.  This can be helpful for some update orders where they cannot
        //be done simultaneously.  For performance, these would also need to implement 
        //multithreaded options.
        //void TryToAddOverlap(int i)
        //{
        //    BroadPhaseOverlap overlap = broadPhaseOverlaps.Elements[i];
        //    if (overlap.collisionRule < CollisionRule.NoNarrowPhasePair)
        //    {
        //        INarrowPhasePair narrowPhaseObject;
        //        //see if the overlap is already present in the narrow phase.
        //        if (!overlapMapping.TryGetValue(overlap, out narrowPhaseObject))
        //        {
        //            //Create/enqueue based on collision table
        //            narrowPhaseObject = NarrowPhaseHelper.GetPair(ref overlap);
        //            if (narrowPhaseObject != null)
        //            {
        //                narrowPhaseObject.NarrowPhase = this;
        //                //Add the new object to the 'todo' list.
        //                //Technically, this doesn't need to be thread-safe when this is called from the sequential context.
        //                //It's just bunched together for maintainability despite the slightly performance hit.
        //                newNarrowPhasePairs.Enqueue(narrowPhaseObject);
        //            }
        //        }
        //        if (narrowPhaseObject != null)
        //        {
        //            narrowPhaseObject.NeedsUpdate = false;  //This is hacky.
        //        }
        //    }
        //}

        //public void UpdatePairList()
        //{
        //    for (int i = 0; i < broadPhaseOverlaps.count; i++)
        //        TryToAddOverlap(i);

        //    if (narrowPhasePairs.Count < MultithreadedRemovalCutoff)
        //    {
        //        RemoveStaleOverlaps();
        //    }
        //    else
        //    {
        //        ThreadManager.ForLoop(0, narrowPhasePairs.Count, multithreadedRemovalLoopDelegate);
        //        INarrowPhasePair overlapToRemove;
        //        while (overlapsToRemove.TryUnsafeDequeueFirst(out overlapToRemove))
        //        {
        //            narrowPhasePairs[narrowPhasePairs.IndexOf(overlapToRemove)] = narrowPhasePairs[narrowPhasePairs.Count - 1];
        //            narrowPhasePairs.RemoveAt(narrowPhasePairs.Count - 1);
        //            OnRemovePair(overlapToRemove);
        //        }
        //    }


        //    AddNewNarrowPhaseObjects();
        //}

        //public void UpdatePairs()
        //{
        //    for (int i = 0; i < narrowPhasePairs.Count; i++)
        //    {
        //        if (narrowPhasePairs[i].BroadPhaseOverlap.collisionRule < CollisionRule.NoNarrowPhaseUpdate &&
        //            (narrowPhasePairs[i].BroadPhaseOverlap.entryA.IsActive || narrowPhasePairs[i].BroadPhaseOverlap.entryB.IsActive))
        //            narrowPhasePairs[i].UpdateCollision(TimeStepSettings.TimeStepDuration);
        //    }
        //}
    }
}
