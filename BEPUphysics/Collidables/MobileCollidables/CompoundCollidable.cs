using System.Collections.Generic;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Collidables.Events;
using BEPUphysics.CollisionShapes;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;
using BEPUphysics.ResourceManagement;
using BEPUphysics.DataStructures;
using BEPUphysics.Materials;
using System.Collections.ObjectModel;
using BEPUphysics.CollisionRuleManagement;

namespace BEPUphysics.Collidables.MobileCollidables
{
    ///<summary>
    /// Collidable used by compound shapes.
    ///</summary>
    public class CompoundCollidable : EntityCollidable
    {
        ///<summary>
        /// Gets the shape of the collidable.
        ///</summary>
        public new CompoundShape Shape
        {
            get
            {
                return base.Shape as CompoundShape;
            }
        }

        internal RawList<CompoundChild> children = new RawList<CompoundChild>();
        ///<summary>
        /// Gets a list of the children in the collidable.
        ///</summary>
        public ReadOnlyCollection<CompoundChild> Children
        {
            get;
            private set;
        }



        protected override void OnEntityChanged()
        {
            for (int i = 0; i < children.count; i++)
            {
                children.Elements[i].CollisionInformation.Entity = entity;
                if (children.Elements[i].Material == null)
                    children.Elements[i].Material = entity.material;
            }
            base.OnEntityChanged();
        }




        private CompoundChild GetChild(CompoundChildData data, int index)
        {
            var instance = data.Entry.Shape.GetCollidableInstance();
            if (data.Events != null)
                instance.events = data.Events;
            if (data.CollisionRules != null)
                instance.collisionRules = data.CollisionRules;
            if (data.Material == null)
                data.Material = new Material();
            return new CompoundChild(Shape, instance, data.Material, index);
        }

        private CompoundChild GetChild(CompoundShapeEntry entry, int index)
        {
            var instance = entry.Shape.GetCollidableInstance();
            return new CompoundChild(Shape, instance, index);
        }

        ///<summary>
        /// Constructs a compound collidable using additional information about the shapes in the compound.
        ///</summary>
        ///<param name="children">Data representing the children of the compound collidable.</param>
        public CompoundCollidable(IList<CompoundChildData> children)
        {
            var shapeList = new RawList<CompoundShapeEntry>();
            for (int i = 0; i < children.Count; i++)
            {
                CompoundChild child = GetChild(children[i], i);
                shapeList.Add(children[i].Entry);
                this.children.Add(child);
            }
            base.Shape = new CompoundShape(shapeList);
            hierarchy = new CompoundHierarchy(this);
            Children = new ReadOnlyCollection<CompoundChild>(this.children);
 
        }

        ///<summary>
        /// Constructs a compound collidable using additional information about the shapes in the compound.
        ///</summary>
        ///<param name="children">Data representing the children of the compound collidable.</param>
        ///<param name="center">Location computed to be the center of the compound object.</param>
        public CompoundCollidable(IList<CompoundChildData> children, out Vector3 center)
        {
            var shapeList = new RawList<CompoundShapeEntry>();
            for (int i = 0; i < children.Count; i++)
            {
                CompoundChild child = GetChild(children[i], i);
                shapeList.Add(children[i].Entry);
                this.children.Add(child);
            }
            base.Shape = new CompoundShape(shapeList, out center);
            hierarchy = new CompoundHierarchy(this);
            Children = new ReadOnlyCollection<CompoundChild>(this.children);

        }


        ///<summary>
        /// Constructs a new CompoundCollidable.
        ///</summary>
        ///<param name="compoundShape">Compound shape to use for the collidable.</param>
        public CompoundCollidable(CompoundShape compoundShape)
            : base(compoundShape)
        {
            for (int i = 0; i < compoundShape.shapes.count; i++)
            {
                CompoundChild child = GetChild(compoundShape.shapes.Elements[i], i);
                this.children.Add(child);
            }
            hierarchy = new CompoundHierarchy(this);
            Children = new ReadOnlyCollection<CompoundChild>(this.children);
 
        }






        internal CompoundHierarchy hierarchy;
        ///<summary>
        /// Gets the hierarchy of children used by the collidable.
        ///</summary>
        public CompoundHierarchy Hierarchy
        {
            get
            {
                return hierarchy;
            }
        }


        ///<summary>
        /// Updates the world transform of the collidable.
        ///</summary>
        ///<param name="position">Position to use for the calculation.</param>
        ///<param name="orientation">Orientation to use for the calculation.</param>
        public override void UpdateWorldTransform(ref Vector3 position, ref Quaternion orientation)
        {
            base.UpdateWorldTransform(ref position, ref orientation);
            var shapeList = Shape.shapes;
            for (int i = 0; i < children.count; i++)
            {
                RigidTransform transform;
                RigidTransform.Transform(ref shapeList.Elements[children.Elements[i].shapeIndex].LocalTransform, ref worldTransform, out transform);
                children.Elements[i].CollisionInformation.UpdateWorldTransform(ref transform.Position, ref transform.Orientation);
            }
        }

        protected internal override void UpdateBoundingBoxInternal(float dt)
        {
            for (int i = 0; i < children.count; i++)
            {
                children.Elements[i].CollisionInformation.UpdateBoundingBoxInternal(dt);
            }
            hierarchy.Tree.Refit();
            boundingBox = hierarchy.Tree.BoundingBox;

        }


        /// <summary>
        /// Tests a ray against the entry.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length, in units of the ray's direction's length, to test.</param>
        /// <param name="rayHit">Hit location of the ray on the entry, if any.</param>
        /// <returns>Whether or not the ray hit the entry.</returns>
        public override bool RayCast(Ray ray, float maximumLength, out RayHit rayHit)
        {
            rayHit = new RayHit();
            var hitElements = Resources.GetCompoundChildList();
            if (hierarchy.Tree.GetOverlaps(ray, maximumLength, hitElements))
            {
                rayHit.T = float.MaxValue;
                for (int i = 0; i < hitElements.count; i++)
                {
                    EntityCollidable candidate = hitElements.Elements[i].CollisionInformation;
                    RayHit tempHit;
                    if (candidate.RayCast(ray, maximumLength, out tempHit) && tempHit.T < rayHit.T)
                    {
                        rayHit = tempHit;
                    }
                }
                Resources.GiveBack(hitElements);
                return rayHit.T != float.MaxValue;
            }
            Resources.GiveBack(hitElements);
            return false;
        }

        /// <summary>
        /// Casts a convex shape against the collidable.
        /// </summary>
        /// <param name="castShape">Shape to cast.</param>
        /// <param name="startingTransform">Initial transform of the shape.</param>
        /// <param name="sweep">Sweep to apply to the shape.</param>
        /// <param name="hit">Hit data, if any.</param>
        /// <returns>Whether or not the cast hit anything.</returns>
        public override bool ConvexCast(CollisionShapes.ConvexShapes.ConvexShape castShape, ref RigidTransform startingTransform, ref Vector3 sweep, out RayHit hit)
        {
            hit = new RayHit();
            BoundingBox boundingBox;
            Toolbox.GetExpandedBoundingBox(ref castShape, ref startingTransform, ref sweep, out boundingBox);
            var hitElements = Resources.GetCompoundChildList();
            if (hierarchy.Tree.GetOverlaps(boundingBox, hitElements))
            {
                hit.T = float.MaxValue;
                for (int i = 0; i < hitElements.count; i++)
                {
                    var candidate = hitElements.Elements[i].CollisionInformation;
                    RayHit tempHit;
                    if (candidate.ConvexCast(castShape, ref startingTransform, ref sweep, out tempHit) && tempHit.T < hit.T)
                    {
                        hit = tempHit;
                    }
                }
                Resources.GiveBack(hitElements);
                return hit.T != float.MaxValue;
            }
            Resources.GiveBack(hitElements);
            return false;
        }

    }

    ///<summary>
    /// Data which can be used to create a CompoundChild.
    /// This data is not itself a child yet; another system
    /// will use it as input to construct the children.
    ///</summary>
    public struct CompoundChildData
    {
        ///<summary>
        /// Shape entry of the compound child.
        ///</summary>
        public CompoundShapeEntry Entry;
        ///<summary>
        /// Event manager for the new child.
        ///</summary>
        public ContactEventManager<EntityCollidable> Events;
        ///<summary>
        /// Collision rules for the new child.
        ///</summary>
        public CollisionRules CollisionRules;
        ///<summary>
        /// Material for the new child.
        ///</summary>
        public Material Material;


        ///<summary>
        /// Constructs data for a compound child.
        ///</summary>
        ///<param name="entry">Shape entry for the new child.</param>
        ///<param name="events">Event manager to use in the new child.</param>
        ///<param name="collisionRules">Collision rules to use in the new child.</param>
        ///<param name="material">Material to use in the new child.</param>
        public CompoundChildData(CompoundShapeEntry entry, ContactEventManager<EntityCollidable> events, CollisionRules collisionRules, Material material)
        {
            Entry = entry;
            Events = events;
            CollisionRules = collisionRules;
            Material = material;
        }

        ///<summary>
        /// Constructs data for a compound child.
        ///</summary>
        ///<param name="entry">Shape entry for the new child.</param>
        ///<param name="events">Event manager to use in the new child.</param>
        ///<param name="material">Material to use in the new child.</param>
        public CompoundChildData(CompoundShapeEntry entry, ContactEventManager<EntityCollidable> events, Material material)
        {
            Entry = entry;
            Events = events;
            CollisionRules = null;
            Material = material;
        }

        ///<summary>
        /// Constructs data for a compound child.
        ///</summary>
        ///<param name="entry">Shape entry for the new child.</param>
        ///<param name="events">Event manager to use in the new child.</param>
        public CompoundChildData(CompoundShapeEntry entry, ContactEventManager<EntityCollidable> events)
        {
            Entry = entry;
            Events = events;
            CollisionRules = null;
            Material = null;
        }

        ///<summary>
        /// Constructs data for a compound child.
        ///</summary>
        ///<param name="entry">Shape entry for the new child.</param>
        ///<param name="collisionRules">Collision rules to use in the new child.</param>
        public CompoundChildData(CompoundShapeEntry entry, CollisionRules collisionRules)
        {
            Entry = entry;
            Events = null;
            CollisionRules = collisionRules;
            Material = null;
        }

        ///<summary>
        /// Constructs data for a compound child.
        ///</summary>
        ///<param name="entry">Shape entry for the new child.</param>
        ///<param name="collisionRules">Collision rules to use in the new child.</param>
        ///<param name="material">Material to use in the new child.</param>
        public CompoundChildData(CompoundShapeEntry entry, CollisionRules collisionRules, Material material)
        {
            Entry = entry;
            Events = null;
            CollisionRules = collisionRules;
            Material = material;
        }

        ///<summary>
        /// Constructs data for a compound child.
        ///</summary>
        ///<param name="entry">Shape entry for the new child.</param>
        ///<param name="material">Material to use in the new child.</param>
        public CompoundChildData(CompoundShapeEntry entry, Material material)
        {
            Entry = entry;
            Events = null;
            CollisionRules = null;
            Material = material;
        }

        ///<summary>
        /// Constructs data for a compound child.
        ///</summary>
        ///<param name="entry">Shape entry for the new child.</param>
        public CompoundChildData(CompoundShapeEntry entry)
        {
            Entry = entry;
            Events = null;
            CollisionRules = null;
            Material = null;
        }
    }


    ///<summary>
    /// A collidable child of a compound.
    ///</summary>
    public class CompoundChild : IBoundingBoxOwner
    {
        CompoundShape shape;
        internal int shapeIndex;

        private EntityCollidable collisionInformation;
        ///<summary>
        /// Gets the Collidable associated with the child.
        ///</summary>
        public EntityCollidable CollisionInformation
        {
            get
            {
                return collisionInformation;
            }
        }

        ///<summary>
        /// Gets or sets the material associated with the child.
        ///</summary>
        public Material Material { get; set; }

        /// <summary>
        /// Gets the index of the shape associated with this child in the CompoundShape's shapes list.
        /// </summary>
        public CompoundShapeEntry Entry
        {
            get
            {
                return shape.shapes.Elements[shapeIndex];
            }

        }

        internal CompoundChild(CompoundShape shape, EntityCollidable collisionInformation, Material material, int index)
        {
            this.shape = shape;
            this.collisionInformation = collisionInformation;
            Material = material;
            this.shapeIndex = index;
        }

        internal CompoundChild(CompoundShape shape, EntityCollidable collisionInformation, int index)
        {
            this.shape = shape;
            this.collisionInformation = collisionInformation;
            this.shapeIndex = index;
        }

        /// <summary>
        /// Gets the bounding box of the child.
        /// </summary>
        public BoundingBox BoundingBox
        {
            get { return collisionInformation.boundingBox; }
        }
    }
}
