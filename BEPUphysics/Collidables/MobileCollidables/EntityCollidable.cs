using BEPUphysics.Collidables.Events;
using BEPUphysics.CollisionShapes;
using BEPUphysics.Entities;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;

namespace BEPUphysics.Collidables.MobileCollidables
{
    ///<summary>
    /// Mobile collidable acting as a collision proxy for an entity.
    ///</summary>
    public abstract class EntityCollidable : MobileCollidable
    {
        protected EntityCollidable()
        {
            //This constructor is used when the subclass is going to set the shape after doing some extra initialization.
            events = new ContactEventManager<EntityCollidable>(this);
        }

        protected EntityCollidable(EntityShape shape)
            : this()
        {
            base.Shape = shape;
        }





        /// <summary>
        /// Gets the shape of the collidable.
        /// </summary>
        public new EntityShape Shape
        {
            get
            {
                return base.Shape as EntityShape;
            }
            protected set
            {
                base.Shape = value;
            }
        }

        protected internal Entity entity;
        ///<summary>
        /// Gets the entity owning the collidable.
        ///</summary>
        public Entity Entity
        {
            get
            {
                return entity;
            }
            protected internal set
            {
                entity = value;
                OnEntityChanged();
            }
        }

   
        protected virtual void OnEntityChanged()
        {
        }

        protected internal RigidTransform worldTransform;
        ///<summary>
        /// Gets the world transform of the collidable.
        ///</summary>
        public RigidTransform WorldTransform
        {
            get
            {
                return worldTransform;
            }
        }

        protected internal override bool IsActive
        {
            get
            {

                return entity.IsActive;
            }
        }

        protected internal Vector3 localPosition;
        ///<summary>
        /// Gets or sets the local position of the collidable.
        /// The local position can be used to offset the collision geometry
        /// from an entity's center of mass.
        ///</summary>
        public Vector3 LocalPosition
        {
            get
            {
                return localPosition;
            }
            set
            {
                localPosition = value;
            }
        }


        ///<summary>
        /// Updates the bounding box of the mobile collidable.
        ///</summary>
        ///<param name="dt">Timestep with which to update the bounding box.</param>
        public override void UpdateBoundingBox(float dt)
        {
            //The world transform update isn't strictly required for uninterrupted simulation.
            //The entity update method manages the world transforms.
            //However, the redundancy allows a user to change the position in between frames.
            //If the order of the space update changes to position-update-first, this is completely unnecessary.
            UpdateWorldTransform(ref entity.position, ref entity.orientation);
            UpdateBoundingBoxInternal(dt);
        }

        ///<summary>
        /// Updates the world transform of the collidable.
        ///</summary>
        ///<param name="position">Position to use for the calculation.</param>
        ///<param name="orientation">Orientation to use for the calculation.</param>
        public virtual void UpdateWorldTransform(ref Vector3 position, ref Quaternion orientation)
        {
            Vector3.Transform(ref localPosition, ref orientation, out worldTransform.Position);
            Vector3.Add(ref worldTransform.Position, ref position, out worldTransform.Position);
            worldTransform.Orientation = orientation;
        }




        protected internal abstract void UpdateBoundingBoxInternal(float dt);



        protected override void CollisionRulesUpdated()
        {
            //Try to activate the entity since our collision rules just changed; broadphase might need to update some stuff.
            entity.IsActive = true;
        }


        protected internal ContactEventManager<EntityCollidable> events;
        ///<summary>
        /// Gets or sets the event manager of the collidable.
        ///</summary>
        public ContactEventManager<EntityCollidable> Events
        {
            get
            {
                return events;
            }
            set
            {
                events = value;
            }
        }


        ///<summary>
        /// Gets an enumerable collection of all entities overlapping this collidable.
        ///</summary>
        public EntityCollidableCollection OverlappedEntities
        {
            get
            {
                return new EntityCollidableCollection(this);
            }
        }


    }
}
