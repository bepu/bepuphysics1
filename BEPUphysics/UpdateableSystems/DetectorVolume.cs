using System.Collections.Generic;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.DataStructures;
using BEPUphysics.Entities;
using BEPUphysics.ResourceManagement;
 
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.NarrowPhaseSystems;
using BEPUphysics.MathExtensions;

namespace BEPUphysics.UpdateableSystems
{
    /// <summary>
    /// Stores flags regarding an object's degree of inclusion in a volume.
    /// </summary>
    public struct ContainmentState
    {
        /// <summary>
        /// Whether or not the object is fully contained.
        /// </summary>
        public bool IsContained;

        /// <summary>
        /// Whether or not the object is partially or fully contained.
        /// </summary>
        public bool IsTouching;

        /// <summary>
        /// Constructs a new ContainmentState.
        /// </summary>
        /// <param name="touching">Whether or not the object is partially or fully contained.</param>
        /// <param name="contained">Whether or not the object is fully contained.</param>
        public ContainmentState(bool touching, bool contained)
        {
            IsTouching = touching;
            IsContained = contained;
        }
    }

    /// <summary>
    /// Manages the detection of entities within an arbitrary closed triangle mesh.
    /// </summary>
    public class DetectorVolume : Updateable, IEndOfTimeStepUpdateable, ICollisionRulesOwner
    {
        private RawList<Entity> entities = new RawList<Entity>();
        private RawList<BroadPhaseEntry> entries = new RawList<BroadPhaseEntry>();
        private readonly Dictionary<Entity, ContainmentState> nearbyEntities = new Dictionary<Entity, ContainmentState>();

        private CollisionRules collisionRules;
        ///<summary>
        /// Gets or sets the collision rules of the detector volume.
        ///</summary>
        public CollisionRules CollisionRules
        {
            get
            {
                return collisionRules;
            }
            set
            {
                collisionRules = value;
            }
        }


        ///<summary>
        /// Gets or sets the query accelerator used to find nearby objects.
        ///</summary>
        public IQueryAccelerator QueryAccelerator
        {
            get;
            set;
        }

        /// <summary>
        /// Creates a detector volume.
        /// </summary>
        /// <param name="triangleMesh">Arbitrary closed triangle mesh representing the volume.</param>
        /// <param name="queryAccelerator">System used to find nearby objects.</param>
        public DetectorVolume(MeshBoundingBoxTreeData triangleMesh, IQueryAccelerator queryAccelerator)
        {
            TriangleMesh = new TriangleMesh(triangleMesh);
            QueryAccelerator = queryAccelerator;
        }

        /// <summary>
        /// Gets a mapping of nearby entities to their containment states as determined by the DetectorVolume update method.
        /// </summary>
        public ReadOnlyDictionary<Entity, ContainmentState> NearbyEntities
        {
            get { return new ReadOnlyDictionary<Entity, ContainmentState>(nearbyEntities); }
        }

        TriangleMesh triangleMesh;
        /// <summary>
        /// Gets or sets the acceleration structure associated with the triangle mesh data.
        /// </summary>
        public TriangleMesh TriangleMesh
        {
            get
            {
                return triangleMesh;
            }
            set
            {
                triangleMesh = value;
            }
        }


        /// <summary>
        /// Updates the listing of contained entities and their states after the end of a space update.
        /// </summary>
        /// <param name="dt">Time since last frame in simulation seconds.</param>
        void IEndOfTimeStepUpdateable.Update(float dt)
        {
            //Get rid of any entities that are no longer nearby.
            foreach (Entity e in nearbyEntities.Keys)
            {
                if (!(e.CollisionInformation.BoundingBox.Intersects(TriangleMesh.Tree.BoundingBox)))
                {
                    entities.Add(e);
                }
            }

            for (int i = 0; i < entities.count; i++)
            {
                var e = entities.Elements[i];
                if (nearbyEntities[e].IsContained)
                {
                    //STOPS CONTAINED event
                    if (VolumeStopsContainingEntity != null)
                        VolumeStopsContainingEntity(this, e);
                }
                if (nearbyEntities[e].IsTouching)
                {
                    //STOPS TOUCHING event
                    if (EntityStopsTouching != null)
                        EntityStopsTouching(e, this);
                }

                nearbyEntities.Remove(e);
            }
            entities.Clear();

            //Get an updated look at all included entities.
            QueryAccelerator.GetEntries(TriangleMesh.Tree.BoundingBox, entries);
            for (int i = 0; i < entries.count; i++)
            {
                var e = entries.Elements[i] as EntityCollidable;
                if (e != null)
                    entities.Add(e.Entity);
            }
            entries.Clear();
            ContainmentState containmentState;
            for (int i = 0; i < entities.count; i++)
            {
                var e = entities.Elements[i];
                bool isContained;
                bool isTouching = IsEntityIntersectingVolume(e, out isContained);
                if (nearbyEntities.TryGetValue(e, out containmentState))
                {
                    if (!containmentState.IsTouching && isTouching)
                    {
                        //BEGIN TOUCHING event
                        if (EntityBeginsTouching != null)
                            EntityBeginsTouching(e, this);
                    }
                    else if (containmentState.IsTouching && !isTouching)
                    {
                        //END TOUCHING event
                        if (EntityStopsTouching != null)
                            EntityStopsTouching(e, this);
                    }
                    if (!containmentState.IsContained && isContained)
                    {
                        //BEGIN CONTAINED event
                        if (VolumeBeginsContainingEntity != null)
                            VolumeBeginsContainingEntity(this, e);
                    }
                    else if (containmentState.IsContained && !isContained)
                    {
                        //END CONTAINED event
                        if (VolumeStopsContainingEntity != null)
                            VolumeStopsContainingEntity(this, e);
                    }
                    nearbyEntities[e] = new ContainmentState(isTouching, isContained);
                }
                else
                {
                    //Add new entry for the new entity.
                    nearbyEntities.Add(e, new ContainmentState(isTouching, isContained));
                    if (isTouching)
                    {
                        //BEGIN TOUCHING event
                        if (EntityBeginsTouching != null)
                            EntityBeginsTouching(e, this);
                    }
                    if (isContained)
                    {
                        //BEGIN CONTAINED event
                        if (VolumeBeginsContainingEntity != null)
                            VolumeBeginsContainingEntity(this, e);
                    }
                }

            }

            entities.Clear();
        }

        /// <summary>
        /// Determines whether or not an entity is partially or fully contained within the volume.
        /// </summary>
        /// <param name="entity">Entity to check for intersection.</param>
        /// <returns>Whether or not the entity is partially or fully contained with the volume.</returns>
        public bool IsEntityIntersectingVolume(Entity entity)
        {
            return IsPointInVolume(entity.Position) || IsEntityIntersectingShell(entity);
        }

        /// <summary>
        /// Determines whether or not an entity is partially or fully contained within the volume.
        /// </summary>
        /// <param name="entity">Entity to check for intersection.</param>
        /// <param name="isContained">Whether or not the entity is fully contained within the volume.</param>
        /// <returns>Whether or not the entity is partially or fully contained with the volume.</returns>
        public bool IsEntityIntersectingVolume(Entity entity, out bool isContained)
        {
            //If the shell is being intersected, then the entity is not fully contained.
            bool intersecting = IsEntityIntersectingShell(entity);

            if (IsPointInVolume(entity.Position))
            {
                isContained = !intersecting;
                return true;
            }
            isContained = false;
            return intersecting;
        }

        /// <summary>
        /// Determines whether or not an entity is partially or fully contained within the volume.
        /// Any surface triangles intersected by the entity are collected.
        /// </summary>
        /// <param name="entity">Entity to check for intersection.</param>
        /// <param name="intersectedTriangleIndices">First indices of intersected triangles in the index buffer.</param>
        /// <returns>Whether or not the entity is partially or fully contained with the volume.</returns>
        public bool IsEntityIntersectingVolume(Entity entity, List<int> intersectedTriangleIndices)
        {
            return IsEntityIntersectingShell(entity, intersectedTriangleIndices) || IsPointInVolume(entity.Position);
        }


        /// <summary>
        /// Determines whether or not an entity is fully contained within the volume.
        /// </summary>
        /// <param name="entity">Entity to check for containment.</param>
        /// <returns>Whether or not the entity is fully contained with the volume.</returns>
        public bool IsEntityWithinVolume(Entity entity)
        {
            return IsPointInVolume(entity.Position) && !IsEntityIntersectingShell(entity);
        }

        ///<summary>
        /// Determines whether or not an entity is intersecting the triangle shell of a detector volume.
        ///</summary>
        ///<param name="entity">Entity to test.</param>
        ///<returns>Whether or not the entity is intersecting the shell.</returns>
        public bool IsEntityIntersectingShell(Entity entity)
        {
            var triangleIndices = Resources.GetIntList();

            bool toReturn = IsEntityIntersectingShell(entity, triangleIndices);

            Resources.GiveBack(triangleIndices);
            return toReturn;
        }

        ///<summary>
        /// Determines whether or not an entity is intersecting the triangle shell of a detector volume.
        ///</summary>
        ///<param name="entity">Entity to test.</param>
        /// <param name="intersectedTriangleIndices">First indices of intersected triangles in the index buffer.</param>
        ///<returns>Whether or not the entity is intersecting the shell.</returns>
        public bool IsEntityIntersectingShell(Entity entity, IList<int> intersectedTriangleIndices)
        {

            TriangleMesh.Tree.GetOverlaps(entity.CollisionInformation.BoundingBox, intersectedTriangleIndices);
            foreach (int i in intersectedTriangleIndices)
            {
                Vector3 v1, v2, v3;
                triangleMesh.Data.GetTriangle(i, out v1, out v2, out v3);
                var triangle = Resources.GetTriangleCollidable(ref v1, ref v2, ref v3);
                //TODO: Test a TRIANGLE against ANYTHING ELSE.
                //Requires BOOLEAN tests initially for:
                //Triangle-Convex
                //Triangle-CompoundShape
                var pair = new CollidablePair(entity.CollisionInformation, triangle);
                if (NarrowPhaseHelper.Intersecting(ref pair))
                {
                    Resources.GiveBack(triangle);
                    return true;
                }
                Resources.GiveBack(triangle);
            }

            return false;
        }

        /// <summary>
        /// Determines whether the given point is within the triangle mesh.
        /// </summary>
        /// <param name="point">Point to check.</param>
        /// <returns>Whether or not the point is in the triangle mesh volume.</returns>
        public bool IsPointInVolume(Vector3 point)
        {
            int hitCount;

            TriangleMesh.RayCast(new Ray(point, Toolbox.UpVector), out hitCount);

            return hitCount % 2 == 1;
        }

        #region Events

        /// <summary>
        /// Fires when an entity comes into contact with the volume.
        /// </summary>
        public event EntityBeginsTouchingVolumeEventHandler EntityBeginsTouching;

        /// <summary>
        /// Fires when an entity ceases to intersect the volume.
        /// </summary>
        public event EntityStopsTouchingVolumeEventHandler EntityStopsTouching;

        /// <summary>
        /// Fires when an entity becomes fully engulfed by a volume.
        /// </summary>
        public event VolumeBeginsContainingEntityEventHandler VolumeBeginsContainingEntity;

        /// <summary>
        /// Fires when an entity ceases to be fully engulfed by a volume.
        /// </summary>
        public event VolumeStopsContainingEntityEventHandler VolumeStopsContainingEntity;

        #endregion


    }


    /// <summary>
    /// Handles any special logic to perform when an entry begins touching a detector volume.
    /// Runs within an update loop for updateables; modifying the updateable listing during the event is disallowed.
    /// </summary>
    /// <param name="toucher">Entry touching the volume.</param>
    /// <param name="volume">DetectorVolume being touched.</param>
    public delegate void EntityBeginsTouchingVolumeEventHandler(Entity toucher, DetectorVolume volume);

    /// <summary>
    /// Handles any special logic to perform when an entry stops touching a detector volume.
    /// Runs within an update loop for updateables; modifying the updateable listing during the event is disallowed.
    /// </summary>
    /// <param name="toucher">Entry no longer touching the volume.</param>
    /// <param name="volume">DetectorVolume no longer being touched.</param>
    public delegate void EntityStopsTouchingVolumeEventHandler(Entity toucher, DetectorVolume volume);

    /// <summary>
    /// Handles any special logic to perform when an entry begins being contained by a detector volume.
    /// Runs within an update loop for updateables; modifying the updateable listing during the event is disallowed.
    /// </summary>
    /// <param name="volume">DetectorVolume containing the entry.</param>
    /// <param name="entry">Entry contained by the volume.</param>
    public delegate void VolumeBeginsContainingEntityEventHandler(DetectorVolume volume, Entity entry);

    /// <summary>
    /// Handles any special logic to perform when an entry stops being contained by a detector volume.
    /// Runs within an update loop for updateables; modifying the updateable listing during the event is disallowed.
    /// </summary>
    /// <param name="volume">DetectorVolume no longer containing the entry.</param>
    /// <param name="entry">Entry no longer contained by the volume.</param>
    public delegate void VolumeStopsContainingEntityEventHandler(DetectorVolume volume, Entity entry);
}