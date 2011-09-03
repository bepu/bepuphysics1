using System.Collections.Generic;
using BEPUphysics.MathExtensions;
 

namespace BEPUphysics.BroadPhaseSystems.Hierarchies.Testing.Old
{
    ///<summary>
    /// Interface to the DynamicHierarchy's volume query systems.
    ///</summary>
    public class DynamicHierarchyQueryAcceleratorOld : IQueryAccelerator
    {
        private readonly DynamicHierarchyOld hierarchy;
        internal DynamicHierarchyQueryAcceleratorOld(DynamicHierarchyOld hierarchy)
        {
            this.hierarchy = hierarchy;
        }

        /// <summary>
        /// Collects all entities with bounding boxes which intersect the given bounding box.
        /// </summary>
        /// <param name="box">Bounding box to test against the world.</param>
        /// <param name="entries">Entries of the space which intersect the bounding box.</param>
        public void GetEntries(BoundingBox box, IList<BroadPhaseEntry> entries)
        {
            hierarchy.Root.GetEntities(ref box, entries);

        }

        ///// <summary>
        ///// Collects all entities with bounding boxes which intersect the given frustum.
        ///// </summary>
        ///// <param name="frustum">Frustum to test against the world.</param>
        ///// <param name="entries">Entries of the space which intersect the frustum.</param>
        //public void GetEntries(BoundingFrustum frustum, IList<BroadPhaseEntry> entries)
        //{
        //    hierarchy.Root.GetEntities(ref frustum, entries);

        //}

        /// <summary>
        /// Collects all entities with bounding boxes which intersect the given sphere.
        /// </summary>
        /// <param name="sphere">Sphere to test against the world.</param>
        /// <param name="entries">Entries of the space which intersect the sphere.</param>
        public void GetEntries(BoundingSphere sphere, IList<BroadPhaseEntry> entries)
        {
            hierarchy.Root.GetEntities(ref sphere, entries);

        }


        /// <summary>
        /// Finds all intersections between the ray and the entities of the space.
        /// </summary>
        /// <param name="ray">Ray to test against the structure.</param>
        /// <param name="maximumLength">Maximum length of the ray in units of the ray's direction's length.</param>
        /// <param name="entries">Entries which have bounding boxes that overlap the ray.</param>
        public bool RayCast(Ray ray, float maximumLength, IList<BroadPhaseEntry> entries)
        {
            hierarchy.Root.RayCast(ref ray, maximumLength, entries);

            return entries.Count > 0;
        }


        /// <summary>
        /// Finds all intersections between the ray and the entities of the space.
        /// </summary>
        /// <param name="ray">Ray to test against the structure.</param>
        /// <param name="entries">Entries which have bounding boxes that overlap the ray.</param>
        public bool RayCast(Ray ray, IList<BroadPhaseEntry> entries)
        {
            hierarchy.Root.RayCast(ref ray, entries);

            return entries.Count > 0;
        }


    }
}
