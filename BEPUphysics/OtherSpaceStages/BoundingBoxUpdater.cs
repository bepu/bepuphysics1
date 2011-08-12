using System;
using System.Collections.Generic;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.Threading;

namespace BEPUphysics.OtherSpaceStages
{
    ///<summary>
    /// Updates the bounding box of managed objects.
    ///</summary>
    public class BoundingBoxUpdater : MultithreadedProcessingStage
    {

        //TODO: should the Entries field be publicly accessible since there's not any custom add/remove logic?
        List<MobileCollidable> entries = new List<MobileCollidable>();
        TimeStepSettings timeStepSettings;
        ///<summary>
        /// Gets or sets the time step settings used by the updater.
        ///</summary>
        public TimeStepSettings TimeStepSettings { get; set; }
        ///<summary>
        /// Constructs the bounding box updater.
        ///</summary>
        ///<param name="timeStepSettings">Time step setttings to be used by the updater.</param>
        public BoundingBoxUpdater(TimeStepSettings timeStepSettings)
        {
            multithreadedLoopBodyDelegate = MultithreadedLoopBody;
            Enabled = true;
            this.timeStepSettings = timeStepSettings;
        }
        ///<summary>
        /// Constructs the bounding box updater.
        ///</summary>
        ///<param name="timeStepSettings">Time step setttings to be used by the updater.</param>
        /// <param name="threadManager">Thread manager to be used by the updater.</param>
        public BoundingBoxUpdater(TimeStepSettings timeStepSettings, IThreadManager threadManager)
            : this(timeStepSettings)
        {
            ThreadManager = threadManager;
            AllowMultithreading = true;

        }
        Action<int> multithreadedLoopBodyDelegate;
        void MultithreadedLoopBody(int i)
        {
            if (entries[i].IsActive)
                entries[i].UpdateBoundingBox(timeStepSettings.TimeStepDuration);
        }

        ///<summary>
        /// Adds an entry to the updater.
        ///</summary>
        ///<param name="entry">Entry to add.</param>
        public void Add(MobileCollidable entry)
        {
            //TODO: Contains check?
            entries.Add(entry);
        }
        ///<summary>
        /// Removes an entry from the updater.
        ///</summary>
        ///<param name="entry">Entry to remove.</param>
        public void Remove(MobileCollidable entry)
        {
            entries.Remove(entry);
        }
        protected override void UpdateMultithreaded()
        {
            ThreadManager.ForLoop(0, entries.Count, multithreadedLoopBodyDelegate);
        }

        protected override void UpdateSingleThreaded()
        {
            foreach (MobileCollidable entry in entries)
                if (entry.IsActive)
                    entry.UpdateBoundingBox(timeStepSettings.TimeStepDuration);
        }
    }
}
