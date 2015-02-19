using System;
using System.Collections.Generic;
using System.Diagnostics;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.UpdateableSystems;
using BEPUutilities;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.Materials;
using BEPUphysics.PositionUpdating;
using System.Threading;

namespace BEPUphysics.Character
{
    /// <summary>
    /// Gives a physical object simple and cheap FPS-like control.
    /// This character has less features than the full CharacterController but is a little bit faster.
    /// </summary>
    public class SphereCharacterController : Updateable, IBeforeSolverUpdateable
    {
        /// <summary>
        /// Gets the physical body of the character.
        /// </summary>
        public Sphere Body { get; private set; }

        /// <summary>
        /// Gets the contact categorizer used by the character to determine how contacts affect the character's movement.
        /// </summary>
        public CharacterContactCategorizer ContactCategorizer { get; private set; }

        /// <summary>
        /// Gets the support system which other systems use to perform local ray casts and contact queries.
        /// </summary>
        public QueryManager QueryManager { get; private set; }

        /// <summary>
        /// Gets the constraint used by the character to handle horizontal motion.  This includes acceleration due to player input and deceleration when the relative velocity
        /// between the support and the character exceeds specified maximums.
        /// </summary>
        public HorizontalMotionConstraint HorizontalMotionConstraint { get; private set; }

        /// <summary>
        /// Gets the constraint used by the character to stay glued to surfaces it stands on.
        /// </summary>
        public VerticalMotionConstraint VerticalMotionConstraint { get; private set; }

        /// <summary>
        /// Gets or sets the pair locker used by the character controller to avoid interfering with the behavior of other characters.
        /// </summary>
        private CharacterPairLocker PairLocker { get; set; }

        private Vector3 down = new Vector3(0, -1, 0);
        /// <summary>
        /// Gets or sets the down direction of the character. Controls the interpretation of movement and support finding.
        /// </summary>
        public Vector3 Down
        {
            get
            {
                return down;
            }
            set
            {
                float lengthSquared = value.LengthSquared();
                if (lengthSquared < Toolbox.Epsilon)
                    return; //Silently fail. Assuming here that a dynamic process is setting this property; don't need to make a stink about it.
                Vector3.Divide(ref value, (float)Math.Sqrt(lengthSquared), out value);
                down = value;
            }
        }

        Vector3 viewDirection = new Vector3(0, 0, -1);

        /// <summary>
        /// Gets or sets the view direction associated with the character.
        /// Also sets the horizontal view direction internally based on the current down vector.
        /// This is used to interpret the movement directions.
        /// </summary>
        public Vector3 ViewDirection
        {
            get
            {
                return viewDirection;
            }
            set
            {
                viewDirection = value;
            }
        }

        private float jumpSpeed;
        /// <summary>
        /// Gets or sets the speed at which the character leaves the ground when it jumps.
        /// </summary>
        public float JumpSpeed
        {
            get
            {
                return jumpSpeed;
            }
            set
            {
                if (value < 0)
                    throw new ArgumentException("Value must be nonnegative.");
                jumpSpeed = value;
            }
        }
        float slidingJumpSpeed;
        /// <summary>
        /// Gets or sets the speed at which the character leaves the ground when it jumps without traction.
        /// </summary>
        public float SlidingJumpSpeed
        {
            get
            {
                return slidingJumpSpeed;
            }
            set
            {
                if (value < 0)
                    throw new ArgumentException("Value must be nonnegative.");
                slidingJumpSpeed = value;
            }
        }
        float jumpForceFactor = 1f;
        /// <summary>
        /// Gets or sets the amount of force to apply to supporting dynamic entities as a fraction of the force used to reach the jump speed.
        /// </summary>
        public float JumpForceFactor
        {
            get
            {
                return jumpForceFactor;
            }
            set
            {
                if (value < 0)
                    throw new ArgumentException("Value must be nonnegative.");
                jumpForceFactor = value;
            }
        }

        float speed;
        /// <summary>
        /// Gets or sets the speed at which the character will try to move while standing with a support that provides traction.
        /// Relative velocities with a greater magnitude will be decelerated.
        /// </summary>
        public float Speed
        {
            get
            {
                return speed;
            }
            set
            {
                if (value < 0)
                    throw new ArgumentException("Value must be nonnegative.");
                speed = value;
            }
        }
        float tractionForce;
        /// <summary>
        /// Gets or sets the maximum force that the character can apply while on a support which provides traction.
        /// </summary>
        public float TractionForce
        {
            get
            {
                return tractionForce;
            }
            set
            {
                if (value < 0)
                    throw new ArgumentException("Value must be nonnegative.");
                tractionForce = value;
            }
        }

        float slidingSpeed;
        /// <summary>
        /// Gets or sets the speed at which the character will try to move while on a support that does not provide traction.
        /// Relative velocities with a greater magnitude will be decelerated.
        /// </summary>
        public float SlidingSpeed
        {
            get
            {
                return slidingSpeed;
            }
            set
            {
                if (value < 0)
                    throw new ArgumentException("Value must be nonnegative.");
                slidingSpeed = value;
            }
        }
        float slidingForce;
        /// <summary>
        /// Gets or sets the maximum force that the character can apply while on a support which does not provide traction.
        /// </summary>
        public float SlidingForce
        {
            get
            {
                return slidingForce;
            }
            set
            {
                if (value < 0)
                    throw new ArgumentException("Value must be nonnegative.");
                slidingForce = value;
            }
        }

        float airSpeed;
        /// <summary>
        /// Gets or sets the speed at which the character will try to move with no support.
        /// The character will not be decelerated while airborne.
        /// </summary>
        public float AirSpeed
        {
            get
            {
                return airSpeed;
            }
            set
            {
                if (value < 0)
                    throw new ArgumentException("Value must be nonnegative.");
                airSpeed = value;
            }
        }
        float airForce;
        /// <summary>
        /// Gets or sets the maximum force that the character can apply with no support.
        /// </summary>
        public float AirForce
        {
            get
            {
                return airForce;
            }
            set
            {
                if (value < 0)
                    throw new ArgumentException("Value must be nonnegative.");
                airForce = value;
            }
        }

        private float speedScale = 1;
        /// <summary>
        /// Gets or sets a scaling factor to apply to the maximum speed of the character.
        /// This is useful when a character does not have 0 or MaximumSpeed target speed, but rather
        /// intermediate values. A common use case is analog controller sticks.
        /// </summary>
        public float SpeedScale
        {
            get { return speedScale; }
            set { speedScale = value; }
        }


        /// <summary>
        /// Gets the support finder used by the character.
        /// The support finder analyzes the character's contacts to see if any of them provide support and/or traction.
        /// </summary>
        public SupportFinder SupportFinder { get; private set; }



        /// <summary>
        /// Constructs a new character controller.
        /// </summary>
        /// <param name="position">Initial position of the character.</param>
        /// <param name="radius">Radius of the character body.</param>
        /// <param name="mass">Mass of the character body.</param>
        /// <param name="maximumTractionSlope">Steepest slope, in radians, that the character can maintain traction on.</param>
        /// <param name="maximumSupportSlope">Steepest slope, in radians, that the character can consider a support.</param>
        /// <param name="speed">Speed at which the character will try to move while crouching with a support that provides traction.
        /// Relative velocities with a greater magnitude will be decelerated.</param>
        /// <param name="tractionForce">Maximum force that the character can apply while on a support which provides traction.</param>
        /// <param name="slidingSpeed">Speed at which the character will try to move while on a support that does not provide traction.
        /// Relative velocities with a greater magnitude will be decelerated.</param>
        /// <param name="slidingForce">Maximum force that the character can apply while on a support which does not provide traction</param>
        /// <param name="airSpeed">Speed at which the character will try to move with no support.
        /// The character will not be decelerated while airborne.</param>
        /// <param name="airForce">Maximum force that the character can apply with no support.</param>
        /// <param name="jumpSpeed">Speed at which the character leaves the ground when it jumps</param>
        /// <param name="slidingJumpSpeed">Speed at which the character leaves the ground when it jumps without traction</param>
        /// <param name="maximumGlueForce">Maximum force the vertical motion constraint is allowed to apply in an attempt to keep the character on the ground.</param>
        public SphereCharacterController(
            Vector3 position = new Vector3(),
            float radius = .85f, float mass = 10f,
            float maximumTractionSlope = 0.8f, float maximumSupportSlope = 1.3f,
            float speed = 8f, float tractionForce = 1000, float slidingSpeed = 6, float slidingForce = 50, float airSpeed = 1, float airForce = 250,
            float jumpSpeed = 4.5f, float slidingJumpSpeed = 3,
            float maximumGlueForce = 5000)
        {
            Body = new Sphere(position, radius, mass);
            Body.IgnoreShapeChanges = true; //Wouldn't want inertia tensor recomputations to occur if the shape changes.
            //Making the character a continuous object prevents it from flying through walls which would be pretty jarring from a player's perspective.
            Body.PositionUpdateMode = PositionUpdateMode.Continuous;
            Body.LocalInertiaTensorInverse = new Matrix3x3();
            //TODO: In v0.16.2, compound bodies would override the material properties that get set in the CreatingPair event handler.
            //In a future version where this is changed, change this to conceptually minimally required CreatingPair.
            Body.CollisionInformation.Events.DetectingInitialCollision += RemoveFriction;
            Body.LinearDamping = 0;
            ContactCategorizer = new CharacterContactCategorizer(maximumTractionSlope, maximumSupportSlope);
            QueryManager = new QueryManager(Body, ContactCategorizer);
            SupportFinder = new SupportFinder(Body, QueryManager, ContactCategorizer);
            HorizontalMotionConstraint = new HorizontalMotionConstraint(Body, SupportFinder);
            HorizontalMotionConstraint.PositionAnchorDistanceThreshold = (3f / 17f) * radius;
            VerticalMotionConstraint = new VerticalMotionConstraint(Body, SupportFinder, maximumGlueForce);
            PairLocker = new CharacterPairLocker(Body);

            Speed = speed;
            TractionForce = tractionForce;
            SlidingSpeed = slidingSpeed;
            SlidingForce = slidingForce;
            AirSpeed = airSpeed;
            AirForce = airForce;
            JumpSpeed = jumpSpeed;
            SlidingJumpSpeed = slidingJumpSpeed;

            //Enable multithreading for the sphere characters.  
            //See the bottom of the Update method for more information about using multithreading with this character.
            IsUpdatedSequentially = false;

            //Link the character body to the character controller so that it can be identified by the locker.
            //Any object which replaces this must implement the ICharacterTag for locking to work properly.
            Body.CollisionInformation.Tag = new CharacterSynchronizer(Body);



        }

        void RemoveFriction(EntityCollidable sender, BroadPhaseEntry other, NarrowPhasePair pair)
        {
            var collidablePair = pair as CollidablePairHandler;
            if (collidablePair != null)
            {
                //The default values for InteractionProperties is all zeroes- zero friction, zero bounciness.
                //That's exactly how we want the character to behave when hitting objects.
                collidablePair.UpdateMaterialProperties(new InteractionProperties());
            }
        }

        void ExpandBoundingBox()
        {
            if (Body.ActivityInformation.IsActive)
            {
                //This runs after the bounding box updater is run, but before the broad phase.
                //The expansion allows the downward pointing raycast to collect hit points.
                Vector3 expansion = SupportFinder.MaximumAssistedDownStepHeight * down;
                BoundingBox box = Body.CollisionInformation.BoundingBox;
                if (down.X < 0)
                    box.Min.X += expansion.X;
                else
                    box.Max.X += expansion.X;
                if (down.Y < 0)
                    box.Min.Y += expansion.Y;
                else
                    box.Max.Y += expansion.Y;
                if (down.Z < 0)
                    box.Min.Z += expansion.Z;
                else
                    box.Max.Z += expansion.Z;
                Body.CollisionInformation.BoundingBox = box;
            }


        }

        void IBeforeSolverUpdateable.Update(float dt)
        {
            //Someone may want to use the Body.CollisionInformation.Tag for their own purposes.
            //That could screw up the locking mechanism above and would be tricky to track down.
            //Consider using the making the custom tag implement ICharacterTag, modifying LockCharacterPairs to analyze the different Tag type, or using the Entity.Tag for the custom data instead.
            Debug.Assert(Body.CollisionInformation.Tag is ICharacterTag, "The character.Body.CollisionInformation.Tag must implement ICharacterTag to link the SphereCharacterController and its body together for character-related locking to work in multithreaded simulations.");

            SupportData supportData;

            HorizontalMotionConstraint.UpdateMovementBasis(ref viewDirection);
            //We can't let multiple characters manage the same pairs simultaneously.  Lock it up!
            PairLocker.LockCharacterPairs();
            try
            {
                bool hadSupport = SupportFinder.HasSupport;

                SupportFinder.UpdateSupports(ref HorizontalMotionConstraint.movementDirection3d);
                supportData = SupportFinder.SupportData;

                //Compute the initial velocities relative to the support.
                Vector3 relativeVelocity;
                ComputeRelativeVelocity(ref supportData, out relativeVelocity);
                float verticalVelocity = Vector3.Dot(supportData.Normal, relativeVelocity);



                //Don't attempt to use an object as support if we are flying away from it (and we were never standing on it to begin with).
                if (SupportFinder.HasSupport && !hadSupport && verticalVelocity < 0)
                {
                    SupportFinder.ClearSupportData();
                    supportData = new SupportData();
                }



                //Attempt to jump.
                if (tryToJump) //Jumping while crouching would be a bit silly.
                {
                    //In the following, note that the jumping velocity changes are computed such that the separating velocity is specifically achieved,
                    //rather than just adding some speed along an arbitrary direction.  This avoids some cases where the character could otherwise increase
                    //the jump speed, which may not be desired.
                    if (SupportFinder.HasTraction)
                    {
                        //The character has traction, so jump straight up.
                        float currentDownVelocity;
                        Vector3.Dot(ref down, ref relativeVelocity, out currentDownVelocity);
                        //Target velocity is JumpSpeed.
                        float velocityChange = Math.Max(jumpSpeed + currentDownVelocity, 0);
                        ApplyJumpVelocity(ref supportData, down * -velocityChange, ref relativeVelocity);


                        //Prevent any old contacts from hanging around and coming back with a negative depth.
                        foreach (var pair in Body.CollisionInformation.Pairs)
                            pair.ClearContacts();
                        SupportFinder.ClearSupportData();
                        supportData = new SupportData();
                    }
                    else if (SupportFinder.HasSupport)
                    {
                        //The character does not have traction, so jump along the surface normal instead.
                        float currentNormalVelocity = Vector3.Dot(supportData.Normal, relativeVelocity);
                        //Target velocity is JumpSpeed.
                        float velocityChange = Math.Max(slidingJumpSpeed - currentNormalVelocity, 0);
                        ApplyJumpVelocity(ref supportData, supportData.Normal * -velocityChange, ref relativeVelocity);

                        //Prevent any old contacts from hanging around and coming back with a negative depth.
                        foreach (var pair in Body.CollisionInformation.Pairs)
                            pair.ClearContacts();
                        SupportFinder.ClearSupportData();
                        supportData = new SupportData();
                    }
                }
                tryToJump = false;
            }
            finally
            {
                PairLocker.UnlockCharacterPairs();
            }


            //Tell the constraints to get ready to solve.
            HorizontalMotionConstraint.UpdateSupportData();
            VerticalMotionConstraint.UpdateSupportData();



            //Update the horizontal motion constraint's state.
            if (supportData.SupportObject != null)
            {
                if (SupportFinder.HasTraction)
                {
                    HorizontalMotionConstraint.MovementMode = MovementMode.Traction;
                    HorizontalMotionConstraint.TargetSpeed = speed;
                    HorizontalMotionConstraint.MaximumForce = tractionForce;
                }
                else
                {
                    HorizontalMotionConstraint.MovementMode = MovementMode.Sliding;
                    HorizontalMotionConstraint.TargetSpeed = slidingSpeed;
                    HorizontalMotionConstraint.MaximumForce = slidingForce;
                }
            }
            else
            {
                HorizontalMotionConstraint.MovementMode = MovementMode.Floating;
                HorizontalMotionConstraint.TargetSpeed = airSpeed;
                HorizontalMotionConstraint.MaximumForce = airForce;
            }
            HorizontalMotionConstraint.TargetSpeed *= SpeedScale;



        }

        void ComputeRelativeVelocity(ref SupportData supportData, out Vector3 relativeVelocity)
        {

            //Compute the relative velocity between the body and its support, if any.
            //The relative velocity will be updated as impulses are applied.
            relativeVelocity = Body.LinearVelocity;
            if (SupportFinder.HasSupport)
            {
                //Only entities have velocity.
                var entityCollidable = supportData.SupportObject as EntityCollidable;
                if (entityCollidable != null)
                {
                    //It's possible for the support's velocity to change due to another character jumping if the support is dynamic.
                    //Don't let that happen while the character is computing a relative velocity!
                    Vector3 entityVelocity;
                    bool locked;
                    if (locked = entityCollidable.Entity.IsDynamic)
                        entityCollidable.Entity.Locker.Enter();
                    try
                    {
                        entityVelocity = Toolbox.GetVelocityOfPoint(supportData.Position, entityCollidable.Entity.Position, entityCollidable.Entity.LinearVelocity, entityCollidable.Entity.AngularVelocity);
                    }
                    finally
                    {
                        if (locked)
                            entityCollidable.Entity.Locker.Exit();
                    }
                    Vector3.Subtract(ref relativeVelocity, ref entityVelocity, out relativeVelocity);
                }
            }

        }

        /// <summary>
        /// Changes the relative velocity between the character and its support.
        /// </summary>
        /// <param name="supportData">Support data to use to jump.</param>
        /// <param name="velocityChange">Change to apply to the character and support relative velocity.</param>
        /// <param name="relativeVelocity">Relative velocity to update.</param>
        void ApplyJumpVelocity(ref SupportData supportData, Vector3 velocityChange, ref Vector3 relativeVelocity)
        {
            Body.LinearVelocity += velocityChange;
            var entityCollidable = supportData.SupportObject as EntityCollidable;
            if (entityCollidable != null)
            {
                if (entityCollidable.Entity.IsDynamic)
                {
                    Vector3 change = velocityChange * jumpForceFactor;
                    //Multiple characters cannot attempt to modify another entity's velocity at the same time.
                    entityCollidable.Entity.Locker.Enter();
                    try
                    {
                        entityCollidable.Entity.LinearMomentum += change * -Body.Mass;
                    }
                    finally
                    {
                        entityCollidable.Entity.Locker.Exit();
                    }
                    velocityChange += change;
                }
            }

            //Update the relative velocity as well.  It's a ref parameter, so this update will be reflected in the calling scope.
            Vector3.Add(ref relativeVelocity, ref velocityChange, out relativeVelocity);

        }



        bool tryToJump;
        /// <summary>
        /// Jumps the character off of whatever it's currently standing on.  If it has traction, it will go straight up.
        /// If it doesn't have traction, but is still supported by something, it will jump in the direction of the surface normal.
        /// </summary>
        public void Jump()
        {
            //The actual jump velocities are applied next frame.  This ensures that gravity doesn't pre-emptively slow the jump, and uses more
            //up-to-date support data.
            tryToJump = true;
        }

        public override void OnAdditionToSpace(Space newSpace)
        {
            //Add any supplements to the space too.
            newSpace.Add(Body);
            newSpace.Add(HorizontalMotionConstraint);
            newSpace.Add(VerticalMotionConstraint);
            //This character controller requires the standard implementation of Space.
            newSpace.BoundingBoxUpdater.Finishing += ExpandBoundingBox;

            Body.AngularVelocity = new Vector3();
            Body.LinearVelocity = new Vector3();
        }
        public override void OnRemovalFromSpace(Space oldSpace)
        {
            //Remove any supplements from the space too.
            oldSpace.Remove(Body);
            oldSpace.Remove(HorizontalMotionConstraint);
            oldSpace.Remove(VerticalMotionConstraint);
            //This character controller requires the standard implementation of Space.
            oldSpace.BoundingBoxUpdater.Finishing -= ExpandBoundingBox;
            SupportFinder.ClearSupportData();
            Body.AngularVelocity = new Vector3();
            Body.LinearVelocity = new Vector3();
        }


    }
}

