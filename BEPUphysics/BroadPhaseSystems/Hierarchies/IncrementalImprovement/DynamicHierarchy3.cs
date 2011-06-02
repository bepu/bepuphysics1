using System;
using System.Collections.Generic;
using BEPUphysics.ResourceManagement;
using Microsoft.Xna.Framework;
using BEPUphysics.Threading;

namespace BEPUphysics.BroadPhaseSystems.Hierarchies
{
    ///<summary>
    /// Broadphase which uses a dynamically changing bounding box tree to calculate overlaps.
    /// It has good insertion and update speed.
    ///</summary>
    public class DynamicHierarchy3 : BroadPhase
    {
        private readonly LockingResourcePool<DynamicHierarchyNode3> dynamicHierarchyNodes = new LockingResourcePool<DynamicHierarchyNode3>(128);

        internal DynamicHierarchyNode3 GetNode()
        {
            //Debug.WriteLine("S: " + dynamicHierarchyNodes.sharedPool.Count);
            //Debug.WriteLine("TS: " + dynamicHierarchyNodes.threadSpecificPools.Length);

            var toReturn = dynamicHierarchyNodes.Take();

            toReturn.hierarchy = this;
            return toReturn;

        }

        internal void GiveBack(DynamicHierarchyNode3 node)
        {
            foreach (var childNode in node.children)
            {
                GiveBack(childNode);
            }
            node.entries.Clear();
            node.currentVolume = 0;
            node.maximumAllowedVolume = 0;
            node.children.Clear();

            dynamicHierarchyNodes.GiveBack(node);
        }

        /// <summary>
        /// When an internal node is revalidated, its volume is stored.  When a node's volume exceeds its stored volume multiplied by this factor, it is revalidated again.
        /// </summary>
        public float MaximumAllowedVolumeFactor = 1.4f;

        /// <summary>
        /// Maximum fraction of a parent's entities that a child can inherit.
        /// If a child has as much or more, the validation process is done over again to ensure a more even split.
        /// </summary>
        public float MaximumChildEntityLoad = .8f;

        /// <summary>
        /// The maximum number of entities present in the leaf nodes of the hierarchy.
        /// </summary>
        public int MaximumEntitiesInLeaves = 2;

        /// <summary>
        /// The number of entities needed in a particular node to use the multithreaded reconstruction method on it.
        /// If the node has less, the current thread does the remainder of the subtree itself.
        /// This only applies if multithreading is currently being used.
        /// </summary>
        public int MinimumNodeEntitiesRequiredToMultithread = 100;

        /// <summary>
        /// Highest parent in the hierarchy.
        /// </summary>
        public DynamicHierarchyNode3 Root;

        /// <summary>
        /// Constructs a new instance of the hierarchy and sets up the root node.
        /// </summary>
        public DynamicHierarchy3()
        {
            Root = new DynamicHierarchyNode3(this);
            QueryAccelerator = new DynamicHierarchyQueryAccelerator3(this);
        }

        /// <summary>
        /// Adds an entity to the hierarchy.  Called automatically by the Space owning this broad phase system when an entity is added.
        /// </summary>
        /// <param name="entry">Entry to add.</param>
        public override void Add(BroadPhaseEntry entry)
        {
            lock (Locker)
            {

                Root.Add(entry);

            }

        }


        /// <summary>
        /// Removes an entity from the hierarchy.  Called automatically by the Space owning this broad phase system when an entity is removed.
        /// </summary>
        /// <param name="entry">Entry to add.</param>
        public override void Remove(BroadPhaseEntry entry)
        {
            lock (Locker)
            {
                Root.Remove(entry, Root.entries.IndexOf(entry));

            }

        }

        internal void TryToAdd(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {
            bool intersects;
            entryA.boundingBox.Intersects(ref entryB.boundingBox, out intersects);
            if (intersects)
                TryToAddOverlap(entryA, entryB);
        }

        


        protected override void UpdateMultithreaded()
        {
            UpdateSingleThreaded();
        }

        protected override void UpdateSingleThreaded()
        {
            Overlaps.Clear();
            lock (Locker)
            {
                Root.BinaryUpdateNode();
                Root.BinaryCollideAgainst(Root);
            }
        }
    }
}
