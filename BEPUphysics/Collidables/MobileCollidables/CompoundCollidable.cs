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
                if (children.Elements[i].CollisionInformation.collisionRules.group == null)
                    children.Elements[i].CollisionInformation.collisionRules.group = entity.CollisionInformation.collisionRules.group;
            }
            base.OnEntityChanged();
        }




        private static CompoundChild GetChild(CompoundChildData data)
        {
            var instance = data.Entry.Shape.GetMobileInstance();
            if (data.Events != null)
                instance.events = data.Events;
            if (data.CollisionRules != null)
                instance.collisionRules = data.CollisionRules;
            if (data.Material == null)
                data.Material = new Material();
            return new CompoundChild(instance, data.Entry.LocalTransform, data.Material);
        }

        ///<summary>
        /// Constructs a compound collidable.
        ///</summary>
        ///<param name="children">Data representing the children of the compound collidable.</param>
        ///<param name="center">Center of the compound collidable.</param>
        public CompoundCollidable(IList<CompoundChildData> children, Vector3 center)
        {
            var shapeList = new RawList<CompoundShapeEntry>();
            for (int i = 0; i < children.Count; i++)
            {
                CompoundChild child = GetChild(children[i]);
                child.localTransform.Position -= center; //Recenter.
                var shapeEntry = new CompoundShapeEntry(child.CollisionInformation.Shape, child.LocalTransform);
                shapeList.Add(shapeEntry);
                this.children.Add(child);
            }
            shape = new CompoundShape(shapeList);
            hierarchy = new CompoundHierarchy(this);
            Children = new ReadOnlyCollection<CompoundChild>(this.children);
            Shape.ShapeChanged += OnShapeChanged;
        }

        ///<summary>
        /// Constructs a compound collidable.
        ///</summary>
        ///<param name="children">Children of the compound collidable.</param>
        ///<param name="center">Center of the compound collidable.</param>
        public CompoundCollidable(IList<CompoundChild> children, Vector3 center)
        {
            var shapeList = new RawList<CompoundShapeEntry>();
            for (int i = 0; i < children.Count; i++)
            {
                children[i].localTransform.Position -= center; //Recenter.
                var shapeEntry = new CompoundShapeEntry(children[i].CollisionInformation.Shape, children[i].LocalTransform);
                shapeList.Add(shapeEntry);
                this.children.Add(children[i]);
            }
            shape = new CompoundShape(shapeList);
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
            Children = new ReadOnlyCollection<CompoundChild>(children);
            Initialize();

            
        }

        protected override void OnShapeChanged(CollisionShape collisionShape)
        {
            //TODO: Some oddities when changing compound bodies after being constructed.
            //The center computation performed with two of the above constructors
            //is not performed with Initialize, since it would require a modification
            //of the shape...
            //Additionally, it kills off any event/material/collision rules stuff previously in the shape.
            Initialize();
        }



        void Initialize()
        {
            children.Clear();
            RawList<CompoundShapeEntry> list = Shape.Shapes.list;
            for (int i = 0; i < list.count; i++)
            {
                var child = new CompoundChild(list.Elements[i].Shape.GetMobileInstance(), list.Elements[i].LocalTransform);
                children.Add(child);
            }
            hierarchy = new CompoundHierarchy(this);
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
            for (int i = 0; i < children.count; i++)
            {
                RigidTransform transform;
                RigidTransform.Transform(ref children.Elements[i].localTransform, ref worldTransform, out transform);
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
            if (hierarchy.Tree.RayCast(ray, maximumLength, hitElements))
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
    /// Child data for a new dynamic child.
    /// This data is not itself a child yet; another system
    /// will use it as input to construct the children.
    ///</summary>
    public struct DynamicCompoundChildData
    {
        ///<summary>
        /// Base data for the child.
        ///</summary>
        public CompoundChildData ChildData;
        ///<summary>
        /// Mass of the child.
        ///</summary>
        public float Mass;

        ///<summary>
        /// Constructs data for a new dynamic child.
        ///</summary>
        ///<param name="data">Base data to use to construct the new child.</param>
        ///<param name="mass">Mass to use to construct the new child.</param>
        public DynamicCompoundChildData(CompoundChildData data, float mass)
        {
            ChildData = data;
            Mass = mass;
        }


    }

    ///<summary>
    /// A collidable child of a compound.
    ///</summary>
    public class CompoundChild : IBoundingBoxOwner
    {
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

        internal RigidTransform localTransform;
        //This can't be changed because it is a property of the shape, stored here for convenience.
        ///<summary>
        /// Gets the local transform of the child.
        ///</summary>
        public RigidTransform LocalTransform
        {
            get
            {
                return localTransform;
            }
        }

        ///<summary>
        /// Constructs a new compound child.
        ///</summary>
        ///<param name="collisionInformation">Collidable to use for the child.</param>
        ///<param name="localTransform">Local transform to use to position the child.</param>
        ///<param name="material">Material of the child.</param>
        public CompoundChild(EntityCollidable collisionInformation, RigidTransform localTransform, Material material)
        {
            this.collisionInformation = collisionInformation;
            this.localTransform = localTransform;
            Material = material;
        }

        ///<summary>
        /// Constructs a new compound child.
        ///</summary>
        ///<param name="collisionInformation">Collidable to use for the child.</param>
        ///<param name="localTransform">Local transform to use to position the child.</param>\
        public CompoundChild(EntityCollidable collisionInformation, RigidTransform localTransform)
        {
            this.collisionInformation = collisionInformation;
            this.localTransform = localTransform;
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
