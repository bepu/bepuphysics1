using System;
using System.Collections.Generic;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Constraints.TwoEntity;
using BEPUphysics.DeactivationManagement;
using BEPUphysics.EntityStateManagement;
using BEPUphysics.OtherSpaceStages;
using BEPUphysics.PositionUpdating;
using BEPUphysics.Settings;
using Microsoft.Xna.Framework;
using BEPUphysics.MathExtensions;
using BEPUphysics.Materials;
using BEPUphysics.Constraints;
using System.Collections.ObjectModel;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.DataStructures;
using BEPUphysics.Threading;

namespace BEPUphysics.Entities
{
    ///<summary>
    /// Superclass of movable rigid bodies.  Contains information for
    /// both dynamic and kinematic simulation.
    ///</summary>
    public class Entity :
        IBroadPhaseEntryOwner,
        IDeferredEventCreatorOwner,
        ISimulationIslandMember,
        ICCDPositionUpdateable,
        IForceUpdateable,
        ISpaceObject,
        IMaterialOwner,
        ICollisionRulesOwner
    {
        internal Vector3 position;
        internal Quaternion orientation = Quaternion.Identity;
        internal Matrix3X3 orientationMatrix = Matrix3X3.Identity;
        internal Vector3 linearVelocity;
        internal Vector3 linearMomentum;
        internal Vector3 angularVelocity;
        internal Vector3 angularMomentum;
        internal bool isDynamic;
        internal bool isActive = true;
        internal bool isAlwaysActive;


        ///<summary>
        /// Gets or sets the position of the Entity.  This Position acts
        /// as the center of mass for dynamic entities.
        ///</summary>
        public Vector3 Position
        {
            get
            {
                return position;
            }
            set
            {
                position = value;
                IsActive = true;
            }
        }
        ///<summary>
        /// Gets or sets the orientation quaternion of the entity.
        ///</summary>
        public Quaternion Orientation
        {
            get
            {
                return orientation;
            }
            set
            {
                Quaternion.Normalize(ref value, out orientation);
                Matrix3X3.CreateFromQuaternion(ref orientation, out orientationMatrix);
                IsActive = true;
            }
        }
        /// <summary>
        /// Gets or sets the orientation matrix of the entity.
        /// </summary>
        public Matrix3X3 OrientationMatrix
        {
            get
            {
                return orientationMatrix;
            }
            set
            {
                Matrix3X3.CreateQuaternion(ref value, out orientation);
                Orientation = orientation; //normalizes and sets.
            }
        }
        ///<summary>
        /// Gets or sets the world transform of the entity.
        /// The upper left 3x3 part is the Orientation, and the translation is the Position.
        /// When setting this property, ensure that the rotation matrix component does not include
        /// any scaling or shearing.
        ///</summary>
        public Matrix WorldTransform
        {
            get
            {
                Matrix worldTransform;
                Matrix3X3.ToMatrix4X4(ref orientationMatrix, out worldTransform);
                worldTransform.Translation = position;
                return worldTransform;
            }
            set
            {
                Quaternion.CreateFromRotationMatrix(ref value, out orientation);
                Orientation = orientation; //normalizes and sets.
                position = value.Translation;
                IsActive = true;
            }

        }
        /// <summary>
        /// Gets or sets the angular velocity of the entity.
        /// </summary>
        public Vector3 AngularVelocity
        {
            get
            {
                return angularVelocity;
            }
            set
            {
                angularVelocity = value;
                //TODO: infinite inertia tensor will cause annoyances.
                Matrix3X3.Transform(ref value, ref inertiaTensor, out angularMomentum);
                IsActive = true;
            }
        }
        /// <summary>
        /// Gets or sets the angular momentum of the entity.
        /// </summary>
        public Vector3 AngularMomentum
        {
            get
            {
                return angularMomentum;
            }
            set
            {
                angularMomentum = value;
                Matrix3X3.Transform(ref value, ref inertiaTensorInverse, out angularVelocity);
                IsActive = true;
            }
        }
        /// <summary>
        /// Gets or sets the linear velocity of the entity.
        /// </summary>
        public Vector3 LinearVelocity
        {
            get
            {
                return linearVelocity;
            }
            set
            {
                linearVelocity = value;
                Vector3.Multiply(ref linearVelocity, mass, out linearMomentum);
                IsActive = true;
            }
        }
        /// <summary>
        /// Gets or sets the linear momentum of the entity.
        /// </summary>
        public Vector3 LinearMomentum
        {
            get
            {
                return linearMomentum;
            }
            set
            {
                linearMomentum = value;
                Vector3.Divide(ref linearMomentum, mass, out linearVelocity);
                IsActive = true;
            }
        }
        /// <summary>
        /// Gets or sets the position, orientation, linear velocity, and angular velocity of the entity.
        /// </summary>
        public MotionState MotionState
        {
            get
            {
                MotionState toReturn;
                toReturn.Position = position;
                toReturn.Orientation = orientation;
                toReturn.LinearVelocity = linearVelocity;
                toReturn.AngularVelocity = angularVelocity;
                return toReturn;
            }
            set
            {
                Position = value.Position;
                Orientation = value.Orientation;
                LinearVelocity = value.LinearVelocity;
                AngularVelocity = value.AngularVelocity;
            }
        }

        /// <summary>
        /// Gets whether or not the entity is dynamic.
        /// Dynamic entities have finite mass and respond
        /// to collisions.  Kinematic (non-dynamic) entities
        /// have infinite mass and inertia and will plow through anything.
        /// </summary>
        public bool IsDynamic
        {
            get
            {
                return isDynamic;
            }
        }

        ///<summary>
        /// Gets whether or not the member is active.
        ///</summary>
        public bool IsActive
        {
            get
            {
                return isActive;
            }
            set
            {
                if (isActive && !value)
                {
                    //Deactivating.
                    IsDeactivationCandidate = true;
                    isActive = false;
                    OnDeactivated();
                }
                else if (!isActive && value)
                {
                    //Activating.
                    IsDeactivationCandidate = false;
                    isActive = true;
                    OnActivated();
                }

            }
        }

        ///<summary>
        /// Gets or sets whether or not the entity is always active.
        /// If set to true, the object cannot deactivate.
        ///</summary>
        public bool IsAlwaysActive
        {
            get
            {
                return isAlwaysActive;
            }
            set
            {
                if (!isAlwaysActive && value)
                    IsActive = true;
                isAlwaysActive = value;

            }
        }

        bool isAffectedByGravity = true;
        ///<summary>
        /// Gets or sets whether or not the entity can be affected by gravity applied by the ForceUpdater.
        ///</summary>
        public bool IsAffectedByGravity
        {
            get
            {
                return isAffectedByGravity;
            }
            set
            {
                isAffectedByGravity = value;
            }
        }

        ///<summary>
        /// Gets the buffered states of the entity.  If the Space.BufferedStates manager is enabled,
        /// this property provides access to the buffered and interpolated states of the entity.
        /// Buffered states are the most recent completed update values, while interpolated states are the previous values blended
        /// with the current frame's values.  Interpolated states are helpful when updating the engine with internal time stepping, 
        /// giving entity motion a smooth appearance even when updates aren't occurring consistently every frame.  
        /// Both are buffered for asynchronous access.
        ///</summary>
        public EntityBufferedStates BufferedStates { get; private set; }

        internal Matrix3X3 inertiaTensorInverse;
        ///<summary>
        /// Gets the world space inertia tensor inverse of the entity.
        ///</summary>
        public Matrix3X3 InertiaTensorInverse
        {
            get
            {
                return inertiaTensorInverse;
            }
        }
        internal Matrix3X3 inertiaTensor;
        ///<summary>
        /// Gets the world space inertia tensor of the entity.
        ///</summary>
        public Matrix3X3 InertiaTensor
        {
            get { return inertiaTensor; }
            set
            {
                //Settable, but only for extensibility.  It will be computed...
                inertiaTensor = value;
            }
        }

        internal Matrix3X3 localInertiaTensor;
        ///<summary>
        /// Gets or sets the local inertia tensor of the entity.
        ///</summary>
        public Matrix3X3 LocalInertiaTensor
        {
            get
            {
                return localInertiaTensor;
            }
            set
            {
                localInertiaTensor = value;
                Matrix3X3.Invert(ref localInertiaTensor, out localInertiaTensorInverse);
                //TODO: Deal with infinities.
            }
        }
        internal Matrix3X3 localInertiaTensorInverse;
        /// <summary>
        /// Gets or sets the local inertia tensor inverse of the entity.
        /// </summary>
        public Matrix3X3 LocalInertiaTensorInverse
        {
            get
            {
                return localInertiaTensorInverse;
            }
            set
            {
                localInertiaTensorInverse = value;
                Matrix3X3.Invert(ref localInertiaTensorInverse, out localInertiaTensor);
                //TODO: Deal with infinities.
            }
        }

        internal float mass;
        ///<summary>
        /// Gets or sets the mass of the entity.
        ///</summary>
        public float Mass
        {
            get
            {
                return mass;
            }
            set
            {
                if (value <= 0 || float.IsNaN(value) || float.IsInfinity(value))
                    BecomeKinematic();
                else
                {
                    if (isDynamic)
                    {
                        //If it's already dynamic, then we don't need to recompute the inertia tensor.
                        //Instead, scale the one we have already.
                        Matrix3X3 newInertia;
                        Matrix3X3.Multiply(ref localInertiaTensor, value / mass, out newInertia);
                        BecomeDynamic(value, newInertia);
                    }
                    else
                    {
                        BecomeDynamic(value);
                    }
                }
            }
        }


        internal float volume;
        /// <summary>
        /// Gets or sets the volume of the entity.
        /// This is computed along with other physical properties at initialization,
        /// but it's only used for auxiliary systems like the FluidVolume.
        /// Changing this can tune behavior of those systems.
        /// </summary>
        public float Volume { get { return volume; } set { volume = value; } }



        ///<summary>
        /// Fires when the entity's position is updated.
        ///</summary>
        public event Action<Entity> PositionUpdated;



        private EntityCollidable collisionInformation;
        ///<summary>
        /// Gets the collidable used by the entity.
        ///</summary>
        public EntityCollidable CollisionInformation
        {
            get { return collisionInformation; }
            protected set
            {
                if (collisionInformation != null)
                    collisionInformation.Shape.ShapeChanged -= shapeChangedDelegate;
                collisionInformation = value;
                if (collisionInformation != null)
                    collisionInformation.Shape.ShapeChanged += shapeChangedDelegate;
                //Entity constructors do their own initialization when the collision information changes.
                //Might be able to condense it up here, but don't really need it right now.
                //ShapeChangedHandler(collisionInformation.shape);
            }
        }

        //protected internal object locker = new object();
        /////<summary>
        ///// Gets the synchronization object used by systems that need
        ///// exclusive access to the entity's properties.
        /////</summary>
        //public object Locker
        //{
        //    get
        //    {
        //        return locker;
        //    }
        //}

        protected internal SpinLock locker = new SpinLock();
        ///<summary>
        /// Gets the synchronization object used by systems that need
        /// exclusive access to the entity's properties.
        ///</summary>
        public SpinLock Locker
        {
            get
            {
                return locker;
            }
        }

        internal Material material;
        //NOT thread safe due to material change pair update.
        ///<summary>
        /// Gets or sets the material used by the entity.
        ///</summary>
        public Material Material
        {
            get
            {
                return material;
            }
            set
            {
                if (material != null)
                    material.MaterialChanged -= materialChangedDelegate;
                material = value;
                if (material != null)
                    material.MaterialChanged += materialChangedDelegate;
                OnMaterialChanged(material);
            }
        }

        Action<Material> materialChangedDelegate;
        void OnMaterialChanged(Material newMaterial)
        {
            for (int i = 0; i < collisionInformation.pairs.Count; i++)
            {
                collisionInformation.pairs[i].UpdateMaterialProperties();
            }
        }


        ///<summary>
        /// Gets all the EntitySolverUpdateables associated with this entity.
        ///</summary>
        public EntitySolverUpdateableCollection SolverUpdateables
        {
            get
            {
                return new EntitySolverUpdateableCollection(connections);
            }
        }

        ///<summary>
        /// Gets the two-entity constraints associated with this entity (a subset of the solver updateables).
        ///</summary>
        public EntityConstraintCollection Constraints
        {
            get
            {
                return new EntityConstraintCollection(connections);
            }
        }

        #region Construction

        protected Entity()
        {
            BufferedStates = new EntityBufferedStates(this);

            material = new Material();
            materialChangedDelegate = OnMaterialChanged;
            material.MaterialChanged += materialChangedDelegate;

            shapeChangedDelegate = OnShapeChanged;

        }

        ///<summary>
        /// Constructs a new entity.
        ///</summary>
        ///<param name="collisionInformation">Collidable to use with the entity.</param>
        public Entity(EntityCollidable collisionInformation)
            :this()
        {
            Initialize(collisionInformation);
        }

        ///<summary>
        /// Constructs a new entity.
        ///</summary>
        ///<param name="collisionInformation">Collidable to use with the entity.</param>
        ///<param name="mass">Mass of the entity.</param>
        public Entity(EntityCollidable collisionInformation, float mass)
            : this()
        {
            Initialize(collisionInformation, mass);
        }

        ///<summary>
        /// Constructs a new entity.
        ///</summary>
        ///<param name="collisionInformation">Collidable to use with the entity.</param>
        ///<param name="mass">Mass of the entity.</param>
        /// <param name="inertiaTensor">Inertia tensor of the entity.</param>
        public Entity(EntityCollidable collisionInformation, float mass, Matrix3X3 inertiaTensor)
            : this()
        {
            Initialize(collisionInformation, mass, inertiaTensor);
        }
        ///<summary>
        /// Constructs a new entity.
        ///</summary>
        ///<param name="collisionInformation">Collidable to use with the entity.</param>
        ///<param name="mass">Mass of the entity.</param>
        /// <param name="inertiaTensor">Inertia tensor of the entity.</param>
        /// <param name="volume">Volume of the entity.</param>
        public Entity(EntityCollidable collisionInformation, float mass, Matrix3X3 inertiaTensor, float volume)
            : this()
        {
            Initialize(collisionInformation, mass, inertiaTensor, volume);
        }

        ///<summary>
        /// Constructs a new entity.
        ///</summary>
        ///<param name="shape">Shape to use with the entity.</param>
        public Entity(EntityShape shape)
            : this()
        {
            Initialize(shape.GetCollidableInstance());
        }

        ///<summary>
        /// Constructs a new entity.
        ///</summary>
        ///<param name="shape">Shape to use with the entity.</param>
        ///<param name="mass">Mass of the entity.</param>
        public Entity(EntityShape shape, float mass)
            : this()
        {
            Initialize(shape.GetCollidableInstance(), mass);
        }

        ///<summary>
        /// Constructs a new entity.
        ///</summary>
        ///<param name="shape">Shape to use with the entity.</param>
        ///<param name="mass">Mass of the entity.</param>
        /// <param name="inertiaTensor">Inertia tensor of the entity.</param>
        public Entity(EntityShape shape, float mass, Matrix3X3 inertiaTensor)
            : this()
        {
            Initialize(shape.GetCollidableInstance(), mass, inertiaTensor);
        }
        ///<summary>
        /// Constructs a new entity.
        ///</summary>
        ///<param name="shape">Shape to use with the entity.</param>
        ///<param name="mass">Mass of the entity.</param>
        /// <param name="inertiaTensor">Inertia tensor of the entity.</param>
        /// <param name="volume">Volume of the entity.</param>
        public Entity(EntityShape shape, float mass, Matrix3X3 inertiaTensor, float volume)
            : this()
        {
            Initialize(shape.GetCollidableInstance(), mass, inertiaTensor, volume);
        }


        //These initialize methods make it easier to construct some Entity prefab types.
        protected internal void Initialize(EntityCollidable collisionInformation)
        {
            CollisionInformation = collisionInformation;
            BecomeKinematic();
            collisionInformation.Entity = this;
        }

        protected internal void Initialize(EntityCollidable collisionInformation, float mass)
        {
            CollisionInformation = collisionInformation;

            ShapeDistributionInformation shapeInfo;
            collisionInformation.Shape.ComputeDistributionInformation(out shapeInfo);
            Matrix3X3.Multiply(ref shapeInfo.VolumeDistribution, mass * InertiaHelper.InertiaTensorScale, out shapeInfo.VolumeDistribution);

            volume = shapeInfo.Volume;

            BecomeDynamic(mass, shapeInfo.VolumeDistribution);

            collisionInformation.Entity = this;
        }

        protected internal void Initialize(EntityCollidable collisionInformation, float mass, Matrix3X3 inertiaTensor)
        {
            CollisionInformation = collisionInformation;

            volume = collisionInformation.Shape.ComputeVolume();

            BecomeDynamic(mass, inertiaTensor);

            collisionInformation.Entity = this;
        }

        protected internal void Initialize(EntityCollidable collisionInformation, float mass, Matrix3X3 inertiaTensor, float volume)
        {
            CollisionInformation = collisionInformation;
            this.volume = volume;
            BecomeDynamic(mass, inertiaTensor);

            collisionInformation.Entity = this;
        }

        #endregion

        #region IDeferredEventCreatorOwner Members

        IDeferredEventCreator IDeferredEventCreatorOwner.EventCreator
        {
            get { return CollisionInformation.Events; }
        }

        #endregion




        #region ISimulationIslandMember Members

        void ISimulationIslandMember.AddConnectionReference(ISimulationIslandConnection connection)
        {
            connections.Add(connection);
        }

        void ISimulationIslandMember.RemoveConnectionReference(ISimulationIslandConnection connection)
        {
            connections.Remove(connection);
        }

        protected internal RawList<ISimulationIslandConnection> connections = new RawList<ISimulationIslandConnection>();

        ///<summary>
        /// Gets the connections associated with this member.
        ///</summary>
        public ReadOnlyList<ISimulationIslandConnection> Connections
        {
            get
            {
                return new ReadOnlyList<ISimulationIslandConnection>(connections);
            }
        }

        float velocityTimeBelowLimit;
        float previousVelocity;
        bool isSlowing;

        void ISimulationIslandMember.UpdateDeactivationCandidacy(float dt)
        {
            float velocity = linearVelocity.LengthSquared() + angularVelocity.LengthSquared();
            isSlowing = velocity <= previousVelocity;
            if (isDynamic)
            {

                if (velocity < deactivationManager.velocityLowerLimitSquared)
                    velocityTimeBelowLimit += dt;
                else
                    velocityTimeBelowLimit = 0;

                if (!isAlwaysActive)
                {
                    if (!isDeactivationCandidate)
                    {
                        //See if the velocity is low enough to make this object a deactivation candidate.
                        if (velocityTimeBelowLimit > deactivationManager.lowVelocityTimeMinimum &&
                            isSlowing) //Only deactivate if it is NOT increasing in speed.
                        {
                            IsDeactivationCandidate = true;
                        }
                    }
                    else
                    {
                        //See if velocity is high enough to make this object not a deactivation candidate.
                        if (velocityTimeBelowLimit <= deactivationManager.lowVelocityTimeMinimum)
                        {
                            IsDeactivationCandidate = false;
                        }
                    }

                    //This prevents the entity from going to sleep when interacting with an active kinematic.
                    //It's not very elegant or fast, though.
                    //TODO: Work out a better way that is still thread safe.
                    for (int i = 0; i < connections.Count; i++)
                    {
                        for (int j = 0; j < connections[i].ConnectedMembers.Count; j++)
                        {
                            if (connections[i].ConnectedMembers[j].IsActive && !connections[i].ConnectedMembers[j].IsDynamic)
                            {
                                isDeactivationCandidate = false;
                                velocityTimeBelowLimit = 0;
                            }
                        }
                    }

                }
                else
                    isDeactivationCandidate = false;


            }
            else
            {
                //If it's not dynamic, then deactivation is based entirely on whether or not the object has velocity.
                //Note the slight weirdness with angularVelocityTimeBelowLimit.  It's used as a flag;
                //the engine needs one frame worth of activity to reinitialize stuff before a kinematic goes to sleep.
                if (velocity == 0)
                {
                    if (velocityTimeBelowLimit > 0)
                    {
                        isActive = false;
                        isDeactivationCandidate = true;
                    }
                    else
                        velocityTimeBelowLimit = .01f;
                }
                else
                {
                    isActive = true;
                    isDeactivationCandidate = false;
                    velocityTimeBelowLimit = 0;
                }
            }
            previousVelocity = velocity;
        }

        protected internal bool isDeactivationCandidate;

        ///<summary>
        /// Gets whether or not the object is a deactivation candidate.
        ///</summary>
        public bool IsDeactivationCandidate
        {
            get { return isDeactivationCandidate; }
            private set
            {
                if (value && !isDeactivationCandidate)
                {
                    isDeactivationCandidate = true;
                    OnBecameDeactivationCandidate();
                }
                else if (!value && isDeactivationCandidate)
                {
                    //Even though we the deactivation candidacy is becoming false, 
                    //the timers don't need to be reset.
                    //They will automatically reset if the velocity is found to require it.
                    velocityTimeBelowLimit = 0;
                    isDeactivationCandidate = false;
                    OnBecameNonDeactivationCandidate();
                }
            }
        }

        ///<summary>
        /// Fires when the entity activates.
        ///</summary>
        public event Action<ISimulationIslandMember> Activated;

        ///<summary>
        /// Fires when the entity becomes a deactivation candidate.
        ///</summary>
        public event Action<ISimulationIslandMember> BecameDeactivationCandidate;

        /// <summary>
        /// Fires when the entity is no longer a deactivation candidate.
        /// </summary>
        public event Action<ISimulationIslandMember> BecameNonDeactivationCandidate;

        ///<summary>
        /// Fires when the entity deactivates.
        ///</summary>
        public event Action<ISimulationIslandMember> Deactivated;

        protected internal void OnActivated()
        {
            if (Activated != null)
                Activated(this);
        }

        protected internal void OnBecameDeactivationCandidate()
        {
            if (BecameDeactivationCandidate != null)
                BecameDeactivationCandidate(this);
        }

        protected internal void OnBecameNonDeactivationCandidate()
        {
            if (BecameNonDeactivationCandidate != null)
                BecameNonDeactivationCandidate(this);
        }

        protected internal void OnDeactivated()
        {
            if (Deactivated != null)
                Deactivated(this);
        }

        SimulationIsland ISimulationIslandMember.SimulationIsland { get; set; }
        ///<summary>
        /// Gets the simulation island that owns this entity.
        ///</summary>
        public SimulationIsland SimulationIsland
        {
            get
            {
                return (this as ISimulationIslandMember).SimulationIsland;
            }
        }
        SimulationIslandSearchState ISimulationIslandMember.SearchState { get; set; }

        DeactivationManager deactivationManager;
        DeactivationManager ISimulationIslandMember.DeactivationManager
        {
            get
            {
                return deactivationManager;
            }
            set
            {
                deactivationManager = value;
            }
        }

        #endregion

        ///<summary>
        /// Applies an impulse to the entity.
        ///</summary>
        ///<param name="location">Location to apply the impulse.</param>
        ///<param name="impulse">Impulse to apply.</param>
        public void ApplyImpulse(Vector3 location, Vector3 impulse)
        {
            ApplyImpulse(ref location, ref impulse);
        }

        ///<summary>
        /// Applies an impulse to the entity.
        ///</summary>
        ///<param name="location">Location to apply the impulse.</param>
        ///<param name="impulse">Impulse to apply.</param>
        public void ApplyImpulse(ref Vector3 location, ref Vector3 impulse)
        {
            if (isDynamic)
            {
                ApplyLinearImpulse(ref impulse);
#if WINDOWS
                Vector3 positionDifference;
#else
                Vector3 positionDifference = new Vector3();
#endif
                positionDifference.X = location.X - position.X;
                positionDifference.Y = location.Y - position.Y;
                positionDifference.Z = location.Z - position.Z;

                Vector3 cross;
                Vector3.Cross(ref positionDifference, ref impulse, out cross);
                ApplyAngularImpulse(ref cross);

                if (!isActive)
                    SimulationIsland.Activate();
            }
        }

        //These methods are very direct and quick.  They don't activate the object or anything.
        internal void ApplyLinearImpulse(ref Vector3 impulse)
        {
            linearMomentum.X += impulse.X;
            linearMomentum.Y += impulse.Y;
            linearMomentum.Z += impulse.Z;
            float invMass = 1 / mass;
            linearVelocity.X = linearMomentum.X * invMass;
            linearVelocity.Y = linearMomentum.Y * invMass;
            linearVelocity.Z = linearMomentum.Z * invMass;
        }

        internal void ApplyAngularImpulse(ref Vector3 impulse)
        {

            angularMomentum.X += impulse.X;
            angularMomentum.Y += impulse.Y;
            angularMomentum.Z += impulse.Z;
            if (MotionSettings.ConserveAngularMomentum)
            {
                angularVelocity.X = angularMomentum.X * inertiaTensorInverse.M11 + angularMomentum.Y * inertiaTensorInverse.M21 + angularMomentum.Z * inertiaTensorInverse.M31;
                angularVelocity.Y = angularMomentum.X * inertiaTensorInverse.M12 + angularMomentum.Y * inertiaTensorInverse.M22 + angularMomentum.Z * inertiaTensorInverse.M32;
                angularVelocity.Z = angularMomentum.X * inertiaTensorInverse.M13 + angularMomentum.Y * inertiaTensorInverse.M23 + angularMomentum.Z * inertiaTensorInverse.M33;
            }
            else
            {
                angularVelocity.X += impulse.X * inertiaTensorInverse.M11 + impulse.Y * inertiaTensorInverse.M21 + impulse.Z * inertiaTensorInverse.M31;
                angularVelocity.Y += impulse.X * inertiaTensorInverse.M12 + impulse.Y * inertiaTensorInverse.M22 + impulse.Z * inertiaTensorInverse.M32;
                angularVelocity.Z += impulse.X * inertiaTensorInverse.M13 + impulse.Y * inertiaTensorInverse.M23 + impulse.Z * inertiaTensorInverse.M33;
            }
        }

        /// <summary>
        /// Gets or sets whether or not to ignore shape changes.  When true, changing the entity's collision shape will not update the maximum radius, volume, density, or inertia tensor. 
        /// </summary>
        public bool IgnoreShapeChanges { get; set; }

        Action<CollisionShape> shapeChangedDelegate;
        protected void OnShapeChanged(CollisionShape shape)
        {
            if (!IgnoreShapeChanges)
            {
                ShapeDistributionInformation shapeInfo;
                collisionInformation.Shape.ComputeDistributionInformation(out shapeInfo);
                volume = shapeInfo.Volume;
                if (isDynamic)
                {
                    Matrix3X3.Multiply(ref shapeInfo.VolumeDistribution, InertiaHelper.InertiaTensorScale * mass, out shapeInfo.VolumeDistribution);
                    LocalInertiaTensor = shapeInfo.VolumeDistribution;
                }
                else
                {
                    LocalInertiaTensorInverse = new Matrix3X3();
                }
            }
        }


        //TODO: Include warnings about multithreading.  These modify things outside of the entity and use single-thread-only helpers.
        ///<summary>
        /// Forces the entity to become kinematic.  Kinematic entities have infinite mass and inertia.
        ///</summary>
        public void BecomeKinematic()
        {
            bool previousState = isDynamic;
            isDynamic = false;
            LocalInertiaTensorInverse = new Matrix3X3();
            mass = float.MaxValue;

            //Notify simulation island of the change.
            if (previousState)
            {
                if ((this as ISimulationIslandMember).DeactivationManager != null)
                    (this as ISimulationIslandMember).DeactivationManager.RemoveSimulationIslandFromMember(this);

                if ((this as IForceUpdateable).ForceUpdater != null)
                    (this as IForceUpdateable).ForceUpdater.ForceUpdateableBecomingKinematic(this);
            }
            //Change the collision group if it was using the default.
            if (collisionInformation.CollisionRules.Group == CollisionRules.DefaultDynamicCollisionGroup ||
                collisionInformation.CollisionRules.Group == null)
                collisionInformation.CollisionRules.Group = CollisionRules.DefaultKinematicCollisionGroup;

            IsActive = true;

            //Preserve velocity and reinitialize momentum for new state.
            LinearVelocity = linearVelocity;
            AngularVelocity = angularVelocity;
        }


        ///<summary>
        /// Forces the entity to become dynamic.  Dynamic entities respond to collisions and have finite mass and inertia.
        ///</summary>
        ///<param name="mass">Mass to use for the entity.</param>
        public void BecomeDynamic(float mass)
        {
            Matrix3X3 inertiaTensor = collisionInformation.Shape.ComputeVolumeDistribution();
            Matrix3X3.Multiply(ref inertiaTensor, mass * InertiaHelper.InertiaTensorScale, out inertiaTensor);
            BecomeDynamic(mass, inertiaTensor);
        }

        ///<summary>
        /// Forces the entity to become dynamic.  Dynamic entities respond to collisions and have finite mass and inertia.
        ///</summary>
        ///<param name="mass">Mass to use for the entity.</param>
        /// <param name="localInertiaTensor">Inertia tensor to use for the entity.</param>
        public void BecomeDynamic(float mass, Matrix3X3 localInertiaTensor)
        {
            if (mass <= 0 || float.IsInfinity(mass) || float.IsNaN(mass))
                throw new InvalidOperationException("Cannot use a mass of " + mass + " for a dynamic entity.  Consider using a kinematic entity instead.");
            bool previousState = isDynamic;
            isDynamic = true;
            LocalInertiaTensor = localInertiaTensor;
            this.mass = mass;

            //Notify simulation island system of the change.
            if (!previousState)
            {
                if ((this as ISimulationIslandMember).DeactivationManager != null)
                    (this as ISimulationIslandMember).DeactivationManager.AddSimulationIslandToMember(this);

                if ((this as IForceUpdateable).ForceUpdater != null)
                    (this as IForceUpdateable).ForceUpdater.ForceUpdateableBecomingDynamic(this);
            }
            //Change the group if it was using the defaults.
            if (collisionInformation.CollisionRules.Group == CollisionRules.DefaultKinematicCollisionGroup ||
                collisionInformation.CollisionRules.Group == null)
                collisionInformation.CollisionRules.Group = CollisionRules.DefaultDynamicCollisionGroup;

            IsActive = true;


            //Preserve velocity and reinitialize momentum for new state.
            LinearVelocity = linearVelocity;
            AngularVelocity = angularVelocity;

        }


        void IForceUpdateable.UpdateForForces(float dt)
        {


            //Linear velocity
            if (IsAffectedByGravity)
            {
                Vector3.Add(ref forceUpdater.gravityDt, ref linearVelocity, out linearVelocity);
            }

            //Boost damping at very low velocities.  This is a strong stabilizer; removes a ton of energy from the system.
            if (deactivationManager.useStabilization && (isSlowing || velocityTimeBelowLimit > deactivationManager.lowVelocityTimeMinimum))
            {
                float energy = linearVelocity.LengthSquared() + angularVelocity.LengthSquared();
                if (energy < deactivationManager.velocityLowerLimitSquared)
                {
                    float boost = 1 - energy / (2f * deactivationManager.velocityLowerLimitSquared);
                    ModifyAngularDamping(boost);
                    ModifyLinearDamping(boost);
                }
            }

            //Damping
            float linear = LinearDamping + linearDampingBoost;
            if (linear > 0)
            {
                Vector3.Multiply(ref linearVelocity, (float)Math.Pow(MathHelper.Clamp(1 - linear, 0, 1), dt), out linearVelocity);
            }
            //When applying angular damping, the momentum or velocity is damped depending on the conservation setting.
            float angular = AngularDamping + angularDampingBoost;
            if (angular > 0 && MotionSettings.ConserveAngularMomentum)
            {
                Vector3.Multiply(ref angularMomentum, (float)Math.Pow(MathHelper.Clamp(1 - angular, 0, 1), dt), out angularMomentum);
            }
            else if (angular > 0)
            {
                Vector3.Multiply(ref angularVelocity, (float)Math.Pow(MathHelper.Clamp(1 - angular, 0, 1), dt), out angularVelocity);
            }

            linearDampingBoost = 0;
            angularDampingBoost = 0;

            //Linear momentum
            Vector3.Multiply(ref linearVelocity, mass, out linearMomentum);


            //Update world inertia tensors.
            Matrix3X3 multiplied;
            Matrix3X3.MultiplyTransposed(ref orientationMatrix, ref localInertiaTensorInverse, out multiplied);
            Matrix3X3.Multiply(ref multiplied, ref orientationMatrix, out inertiaTensorInverse);
            Matrix3X3.MultiplyTransposed(ref orientationMatrix, ref localInertiaTensor, out multiplied);
            Matrix3X3.Multiply(ref multiplied, ref orientationMatrix, out inertiaTensor);

            //Update angular velocity or angular momentum.
            if (MotionSettings.ConserveAngularMomentum)
            {
                Matrix3X3.Transform(ref angularMomentum, ref inertiaTensorInverse, out angularVelocity);
            }
            else
            {
                Matrix3X3.Transform(ref angularVelocity, ref inertiaTensor, out angularMomentum);
            }


        }

        private ForceUpdater forceUpdater;
        ForceUpdater IForceUpdateable.ForceUpdater
        {
            get
            {
                return forceUpdater;
            }
            set
            {
                forceUpdater = value;
            }
        }

        #region ISpaceObject

        ISpace space;
        ISpace ISpaceObject.Space
        {
            get
            {
                return space;
            }
            set
            {
                space = value;
            }
        }
        ///<summary>
        /// Gets the space that owns the entity.
        ///</summary>
        public ISpace Space
        {
            get
            {
                return space;
            }
        }


        void ISpaceObject.OnAdditionToSpace(ISpace newSpace)
        {
            OnAdditionToSpace(newSpace);
        }

        protected virtual void OnAdditionToSpace(ISpace newSpace)
        {
        }

        void ISpaceObject.OnRemovalFromSpace(ISpace oldSpace)
        {
            OnRemovalFromSpace(oldSpace);
        }

        protected virtual void OnRemovalFromSpace(ISpace oldSpace)
        {
        }
        #endregion


        #region ICCDPositionUpdateable

        PositionUpdater IPositionUpdateable.PositionUpdater
        {
            get;
            set;
        }

        PositionUpdateMode positionUpdateMode = MotionSettings.DefaultPositionUpdateMode;
        ///<summary>
        /// Gets the position update mode of the entity.
        ///</summary>
        public PositionUpdateMode PositionUpdateMode
        {
            get
            {
                return positionUpdateMode;
            }
            set
            {
                var previous = positionUpdateMode;
                positionUpdateMode = value;
                //Notify our owner of the change, if needed.
                if (positionUpdateMode != previous &&
                    (this as IPositionUpdateable).PositionUpdater != null &&
                    ((this as IPositionUpdateable).PositionUpdater as ContinuousPositionUpdater) != null)
                {
                    ((this as IPositionUpdateable).PositionUpdater as ContinuousPositionUpdater).UpdateableModeChanged(this, previous);
                }

            }
        }

        void ICCDPositionUpdateable.UpdateTimeOfImpacts(float dt)
        {
            //I am a continuous object.  If I am in a pair with another object, even if I am inactive,
            //I must order the pairs to compute a time of impact.

            //The pair method works in such a way that, when this method is run asynchronously, there will be no race conditions.
            for (int i = 0; i < collisionInformation.pairs.Count; i++)
            {
                collisionInformation.pairs.Elements[i].UpdateTimeOfImpact(collisionInformation, dt);
            }
        }

        void ICCDPositionUpdateable.UpdatePositionContinuously(float dt)
        {
            float minimumToi = 1;
            for (int i = 0; i < collisionInformation.pairs.Count; i++)
            {
                if (collisionInformation.pairs.Elements[i].timeOfImpact < minimumToi)
                    minimumToi = collisionInformation.pairs.Elements[i].timeOfImpact;
            }

            //The orientation was already updated by the PreUpdatePosition.
            //However, to be here, this object is not a discretely updated object.
            //That means we still need to update the linear motion.

            Vector3 increment;
            Vector3.Multiply(ref linearVelocity, dt * minimumToi, out increment);
            Vector3.Add(ref position, ref increment, out position);

            collisionInformation.UpdateWorldTransform(ref position, ref orientation);

            if (PositionUpdated != null)
                PositionUpdated(this);
        }

        void IPositionUpdateable.PreUpdatePosition(float dt)
        {
            Vector3 increment;
            if (MotionSettings.UseRk4AngularIntegration && isDynamic)
            {
                Toolbox.UpdateOrientationRK4(ref orientation, ref localInertiaTensorInverse, ref angularMomentum, dt, out orientation);
            }
            else
            {
                Vector3.Multiply(ref angularVelocity, dt * .5f, out increment);
                var multiplier = new Quaternion(increment.X, increment.Y, increment.Z, 0);
                Quaternion.Multiply(ref multiplier, ref orientation, out multiplier);
                Quaternion.Add(ref orientation, ref multiplier, out orientation);
                orientation.Normalize();
            }
            Matrix3X3.CreateFromQuaternion(ref orientation, out orientationMatrix);

            //Only do the linear motion if this object doesn't obey CCD.
            if (PositionUpdateMode == PositionUpdateMode.Discrete)
            {
                Vector3.Multiply(ref linearVelocity, dt, out increment);
                Vector3.Add(ref position, ref increment, out position);

                collisionInformation.UpdateWorldTransform(ref position, ref orientation);
                //The position update is complete if this is a discretely updated object.
                if (PositionUpdated != null)
                    PositionUpdated(this);
            }
            collisionInformation.UpdateWorldTransform(ref position, ref orientation);

        }



        #endregion



        float linearDampingBoost, angularDampingBoost;
        float angularDamping = .15f;
        float linearDamping = .03f;
        ///<summary>
        /// Gets or sets the angular damping of the entity.
        /// Values range from 0 to 1, correspondong to a fraction of angular momentum removed
        /// from the entity over a unit of time.
        ///</summary>
        public float AngularDamping
        {
            get
            {
                return angularDamping;
            }
            set
            {
                angularDamping = MathHelper.Clamp(value, 0, 1);
            }
        }
        ///<summary>
        /// Gets or sets the linear damping of the entity.
        /// Values range from 0 to 1, correspondong to a fraction of linear momentum removed
        /// from the entity over a unit of time.
        ///</summary>
        public float LinearDamping
        {
            get
            {
                return linearDamping;
            }

            set
            {
                linearDamping = value;
            }
        }

        /// <summary>
        /// Temporarily adjusts the linear damping by an amount.  After the value is used, the
        /// damping returns to the base value.
        /// </summary>
        /// <param name="damping">Damping to add.</param>
        public void ModifyLinearDamping(float damping)
        {
            float totalDamping = LinearDamping + linearDampingBoost;
            float remainder = 1 - totalDamping;
            linearDampingBoost += damping * remainder;
        }
        /// <summary>
        /// Temporarily adjusts the angular damping by an amount.  After the value is used, the
        /// damping returns to the base value.
        /// </summary>
        /// <param name="damping">Damping to add.</param>
        public void ModifyAngularDamping(float damping)
        {
            float totalDamping = AngularDamping + angularDampingBoost;
            float remainder = 1 - totalDamping;
            angularDampingBoost += damping * remainder;
        }

        /// <summary>
        /// Gets or sets the user data associated with the entity.
        /// This is separate from the entity's collidable's tag.
        /// If a tag needs to be accessed from within the collision
        /// detection pipeline, consider using the entity.CollisionInformation.Tag.
        /// </summary>
        public object Tag { get; set; }






        CollisionRules ICollisionRulesOwner.CollisionRules
        {
            get
            {
                return collisionInformation.collisionRules;
            }
            set
            {
                collisionInformation.CollisionRules = value;
            }
        }

        BroadPhaseEntry IBroadPhaseEntryOwner.Entry
        {
            get { return collisionInformation; }
        }


    }
}
