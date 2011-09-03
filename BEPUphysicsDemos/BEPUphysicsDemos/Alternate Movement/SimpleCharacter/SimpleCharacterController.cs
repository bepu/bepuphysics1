using System;
using BEPUphysics;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.UpdateableSystems;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.AlternateMovement.SimpleCharacter
{
    public class SimpleCharacterController : Updateable, IEndOfTimeStepUpdateable
    {
        /// <summary>
        /// A box positioned relative to the character's body used to identify collision pairs with nearby objects that could be possibly stood upon.
        /// </summary>
        private Box collisionPairCollector;

        /// <summary>
        /// The displacement vector from the center of the character body capsule to the center of the collision pair collector box entity.
        /// </summary>
        private Vector3 collisionPairCollectorPositionOffset;

        /// <summary>
        /// The displacement vector from the center of the character body capsule to the origin of the ray used to find supports.
        /// </summary>
        private Vector3 rayOriginOffset;

        /// <summary>
        /// The distance above the ground that the bottom of the character's body floats.
        /// </summary>
        private float supportHeight;

        /// <summary>
        /// Rate of increase in the character's speed in the movementDirection.
        /// </summary>
        public float Acceleration = 50;

        /// <summary>
        /// The character's physical representation that handles iteractions with the environment.
        /// </summary>
        public Capsule Body;

        /// <summary>
        /// Whether or not the character is currently standing on anything that can be walked upon.
        /// False if there exists no support or the support is too heavily sloped, otherwise true.
        /// </summary>
        public bool HasTraction;

        /// <summary>
        /// Whether or not the character is currently standing on anything.
        /// </summary>
        public bool IsSupported;

        /// <summary>
        /// Initial vertical speed when jumping.
        /// </summary>
        public float JumpSpeed = 5;

        /// <summary>
        /// The maximum slope under which walking forces can be applied.
        /// </summary>
        public float MaxSlope = MathHelper.PiOver4;

        /// <summary>
        /// Maximum speed in the movementDirection that can be attained.
        /// </summary>
        public float MaxSpeed = 8;

        /// <summary>
        /// Normalized direction which the character tries to move.
        /// </summary>
        public Vector2 MovementDirection = Vector2.Zero;

        /// <summary>
        /// Deceleration applied to oppose horizontal movement when the character does not have a steady foothold on the ground (hasTraction == false).
        /// </summary>
        public float SlidingDeceleration = .3f;

        /// <summary>
        /// Deceleration applied to oppose uncontrolled horizontal movement when the character has a steady foothold on the ground (hasTraction == true).
        /// </summary>
        public float TractionDeceleration = 90f;

        /// <summary>
        /// Constructs a simple character controller.
        /// </summary>
        /// <param name="position">Location to initially place the character.</param>
        /// <param name="characterHeight">The height of the character.</param>
        /// <param name="characterWidth">The diameter of the character.</param>
        /// <param name="supportHeight">The distance above the ground that the bottom of the character's body floats.</param>
        /// <param name="mass">Total mass of the character.</param>
        public SimpleCharacterController(Vector3 position, float characterHeight, float characterWidth, float supportHeight, float mass)
        {
            Body = new Capsule(position, characterHeight - characterWidth, characterWidth / 2, mass);
            collisionPairCollectorPositionOffset = new Vector3(0, -characterHeight / 2 - supportHeight, 0);
            collisionPairCollector = new Box(position + collisionPairCollectorPositionOffset, characterWidth, supportHeight * 2, characterWidth, 1);
            collisionPairCollector.CollisionInformation.CollisionRules.Personal = CollisionRule.NoNarrowPhaseUpdate; //Prevents collision detection/contact generation from being run.
            collisionPairCollector.IsAffectedByGravity = false;
            CollisionRules.AddRule(collisionPairCollector, Body, CollisionRule.NoBroadPhase);//Prevents the creation of any collision pairs between the body and the collector.
            rayOriginOffset = new Vector3(0, -characterHeight / 2, 0);
            this.supportHeight = supportHeight;

            Body.LocalInertiaTensorInverse = new Matrix3X3();
            collisionPairCollector.LocalInertiaTensorInverse = new Matrix3X3();
            //Make the body slippery.
            //Note that this will not make all collisions have zero friction;
            //the friction coefficient between a pair of objects is based
            //on a blending of the two objects' materials.
            Body.Material.KineticFriction = 0;
            Body.Material.StaticFriction = 0;
        }

        /// <summary>
        /// Handles the updating of the character.  Called by the owning space object when necessary.
        /// </summary>
        /// <param name="dt">Simulation seconds since the last update.</param>
        void IEndOfTimeStepUpdateable.Update(float dt)
        {
            Entity supportEntity;
            Vector3 supportLocation, supportNormal;
            float supportDistance;

            if (FindSupport(out supportEntity, out supportLocation, out supportNormal, out supportDistance))
            {
                IsSupported = true;
                //Support location only has velocity if we're actually sitting on an entity, as opposed to some static geometry.
                Vector3 supportLocationVelocity;
                if (supportEntity != null)
                {
                    supportLocationVelocity = supportEntity.LinearVelocity + //linear component
                                              Vector3.Cross(supportEntity.AngularVelocity, supportLocation - supportEntity.Position);
                    supportEntity.ActivityInformation.Activate();
                }
                else
                    supportLocationVelocity = new Vector3();

                Support(supportLocationVelocity, supportNormal, supportDistance);
                HasTraction = IsSupportSlopeWalkable(supportNormal);
                HandleHorizontalMotion(supportLocationVelocity, supportNormal, dt);
            }
            else
            {
                IsSupported = false;
                HasTraction = false;
            }

            collisionPairCollector.LinearVelocity = Body.LinearVelocity;
            collisionPairCollector.Position = (Body.Position + collisionPairCollectorPositionOffset);
        }

        /// <summary>
        /// Locates the closest support entity by performing a raycast at collected candidates.
        /// </summary>
        /// <param name="supportEntity">The closest supporting entity.</param>
        /// <param name="supportLocation">The support location where the ray hit the entity.</param>
        /// <param name="supportNormal">The normal at the surface where the ray hit the entity.</param>
        /// <param name="supportDistance">Distance from the character to the support location.</param>
        /// <returns>Whether or not a support was located.</returns>
        private bool FindSupport(out Entity supportEntity, out Vector3 supportLocation, out Vector3 supportNormal, out float supportDistance)
        {
            supportEntity = null;
            supportLocation = Toolbox.NoVector;
            supportNormal = Toolbox.NoVector;
            supportDistance = float.MaxValue;

            Vector3 rayOrigin = Body.Position + rayOriginOffset;
            for (int i = 0; i < collisionPairCollector.CollisionInformation.Pairs.Count; i++)
            {
                var pair = collisionPairCollector.CollisionInformation.Pairs[i];
                //Determine which member of the collision pair is the possible support.
                Collidable candidate = (pair.BroadPhaseOverlap.EntryA == collisionPairCollector.CollisionInformation ? pair.BroadPhaseOverlap.EntryB : pair.BroadPhaseOverlap.EntryA) as Collidable;
                //Ensure that the candidate is a valid supporting entity.
                if (candidate.CollisionRules.Personal >= CollisionRule.NoSolver)
                    continue; //It is invalid!

                //The maximum length is supportHeight * 2 instead of supportHeight alone because the character should be able to step downwards.
                //This acts like a sort of 'glue' to help the character stick on the ground in general.
                float maximumDistance;
                //The 'glue' effect should only occur if the character has a solid hold on the ground though.
                //Otherwise, the character is falling or sliding around uncontrollably.
                if (HasTraction)
                    maximumDistance = supportHeight * 2;
                else
                    maximumDistance = supportHeight;

                RayHit rayHit;
                //Fire a ray at the candidate and determine some details! 
                if (candidate.RayCast(new Ray(rayOrigin, Vector3.Down), maximumDistance, out rayHit))
                {
                    //We want to find the closest support, so compare it against the last closest support.
                    if (rayHit.T < supportDistance)
                    {
                        supportDistance = rayHit.T;
                        supportLocation = rayHit.Location;
                        supportNormal = rayHit.T > 0 ? rayHit.Normal : Vector3.Up;

                        var entityInfo = candidate as EntityCollidable;
                        if (entityInfo != null)
                            supportEntity = entityInfo.Entity;
                        else
                            supportEntity = null;
                    }
                }
            }
            supportNormal.Normalize();
            return supportDistance < float.MaxValue;
        }

        /// <summary>
        /// Determines if the ground supporting the character is sloped gently enough to allow for normal walking.
        /// </summary>
        /// <param name="supportNormal">Normal of the surface being stood upon.</param>
        /// <returns>Whether or not the slope is walkable.</returns>
        private bool IsSupportSlopeWalkable(Vector3 supportNormal)
        {
            //The following operation is equivalent to performing a dot product between the support normal and Vector3.Down and finding the angle it represents using Acos.
            return Math.Acos(Math.Abs(Math.Min(supportNormal.Y, 1))) <= MaxSlope;
        }

        /// <summary>
        /// Maintains the position of the character's body above the ground.
        /// </summary>
        /// <param name="supportLocationVelocity">Velocity of the support point connected to the supportEntity.</param>
        /// <param name="supportNormal">The normal at the surface where the ray hit the entity.</param>
        /// <param name="supportDistance">Distance from the character to the support location.</param>
        private void Support(Vector3 supportLocationVelocity, Vector3 supportNormal, float supportDistance)
        {
            //Put the character at the right distance from the ground.
            float heightDifference = supportHeight - supportDistance;
            Body.Position += (new Vector3(0, heightDifference, 0));

            //Remove from the character velocity which would push it toward or away from the surface.
            //This is a relative velocity, so the velocity of the body and the velocity of a point on the support entity must be found.
            float bodyNormalVelocity = Vector3.Dot(Body.LinearVelocity, supportNormal);
            float supportEntityNormalVelocity = Vector3.Dot(supportLocationVelocity, supportNormal);
            Body.LinearVelocity -= (bodyNormalVelocity - supportEntityNormalVelocity) * supportNormal;
        }

        /// <summary>
        /// Manages movement acceleration, deceleration, and sliding.
        /// </summary>
        /// <param name="supportLocationVelocity">Velocity of the support point connected to the supportEntity.</param>
        /// <param name="supportNormal">The normal at the surface where the ray hit the entity.</param>
        /// <param name="dt">Timestep of the simulation.</param>
        private void HandleHorizontalMotion(Vector3 supportLocationVelocity, Vector3 supportNormal, float dt)
        {
            if (HasTraction && MovementDirection != Vector2.Zero)
            {
                //Identify a coordinate system that uses the support normal as Y.
                //X is the axis point along the left (negative) and right (positive) relative to the movement direction.
                //Z points forward (positive) and backward (negative) in the movement direction modified to be parallel to the surface.
                Vector3 x = Vector3.Cross(new Vector3(MovementDirection.X, 0, MovementDirection.Y), supportNormal);
                Vector3 z = Vector3.Cross(supportNormal, x);

                //Remove from the character a portion of velocity which pushes it horizontally off the desired movement track defined by the movementDirection.
                float bodyXVelocity = Vector3.Dot(Body.LinearVelocity, x);
                float supportEntityXVelocity = Vector3.Dot(supportLocationVelocity, x);
                float velocityChange = MathHelper.Clamp(bodyXVelocity - supportEntityXVelocity, -dt * TractionDeceleration, dt * TractionDeceleration);
                Body.LinearVelocity -= velocityChange * x;


                float bodyZVelocity = Vector3.Dot(Body.LinearVelocity, z);
                float supportEntityZVelocity = Vector3.Dot(supportLocationVelocity, z);
                float netZVelocity = bodyZVelocity - supportEntityZVelocity;
                //The velocity difference along the Z axis should accelerate/decelerate to match the goal velocity (max speed).
                if (netZVelocity > MaxSpeed)
                {
                    //Decelerate
                    velocityChange = Math.Min(dt * TractionDeceleration, netZVelocity - MaxSpeed);
                    Body.LinearVelocity -= velocityChange * z;
                }
                else
                {
                    //Accelerate
                    velocityChange = Math.Min(dt * Acceleration, MaxSpeed - netZVelocity);
                    Body.LinearVelocity += velocityChange * z;
                }
            }
            else
            {
                float deceleration;
                if (HasTraction)
                    deceleration = dt * TractionDeceleration;
                else
                    deceleration = dt * SlidingDeceleration;
                //Remove from the character a portion of velocity defined by the deceleration.
                Vector3 bodyHorizontalVelocity = Body.LinearVelocity - Vector3.Dot(Body.LinearVelocity, supportNormal) * supportNormal;
                Vector3 supportHorizontalVelocity = supportLocationVelocity - Vector3.Dot(supportLocationVelocity, supportNormal) * supportNormal;
                Vector3 relativeVelocity = bodyHorizontalVelocity - supportHorizontalVelocity;
                float speed = relativeVelocity.Length();
                if (speed > 0)
                {
                    Vector3 horizontalDirection = relativeVelocity / speed;
                    float velocityChange = Math.Min(speed, deceleration);
                    Body.LinearVelocity -= velocityChange * horizontalDirection;
                }


                ////Identify a coordinate system that uses the support normal as Y.
                ////Pick the X and Z axes arbitrarily so that the result is an orthonormal basis.
                //Vector3 x = Vector3.Cross(supportNormal, Vector3.Right);
                //Vector3 z = Vector3.Cross(supportNormal, x);

                //float frameDeceleration = dt * slidingDeceleration;
                ////Remove from the character a portion of velocity to slow down the sliding.
                ////This is a relative velocity, so the velocity of the body and the velocity of a point on the support entity must be found.
                //float bodyXVelocity = Vector3.Dot(body.LinearVelocity, x);
                //float supportEntityXVelocity = Vector3.Dot(supportLocationVelocity, x);
                //float velocityChange = MathHelper.Clamp(bodyXVelocity - supportEntityXVelocity, -frameDeceleration, frameDeceleration);
                //body.LinearVelocity -= velocityChange * x;

                //float bodyZVelocity = Vector3.Dot(body.LinearVelocity, z);
                //float supportEntityZVelocity = Vector3.Dot(supportLocationVelocity, z);
                //velocityChange = MathHelper.Clamp(bodyZVelocity - supportEntityZVelocity, -frameDeceleration, frameDeceleration);
                //body.LinearVelocity -= velocityChange * z;
            }
        }

        /// <summary>
        /// If the character has a support, it leaps into the air based on its jumpSpeed.
        /// </summary>
        public void Jump()
        {
            if (IsSupported)
            {
                IsSupported = false;
                HasTraction = false;
                Body.LinearVelocity += new Vector3(0, JumpSpeed, 0);
            }
        }

        /// <summary>
        /// Activates the character, adding its components to the space. 
        /// </summary>
        public void Activate()
        {
            if (!IsUpdating)
            {
                IsUpdating = true;
                if (Body.Space == null)
                {
                    Space.Add(Body);
                    Space.Add(collisionPairCollector);
                }
                HasTraction = false;
                IsSupported = false;
                Body.LinearVelocity = Vector3.Zero;
            }
        }

        /// <summary>
        /// Deactivates the character, removing its components from the space.
        /// </summary>
        public void Deactivate()
        {
            if (IsUpdating)
            {
                IsUpdating = false;
                Body.Position = new Vector3(10000, 0, 0);
                if (Body.Space != null)
                {
                    Space.Remove(Body);
                    Space.Remove(collisionPairCollector);
                }
            }
        }

        /// <summary>
        /// Called by the engine when the character is added to the space.
        /// Activates the character.
        /// </summary>
        /// <param name="newSpace">Space to which the character was added.</param>
        public override void OnAdditionToSpace(ISpace newSpace)
        {
            base.OnAdditionToSpace(newSpace); //sets this object's space to the newSpace.
            Activate();
        }

        /// <summary>
        /// Called by the engine when the character is removed from the space.
        /// Deactivates the character.
        /// </summary>
        public override void OnRemovalFromSpace(ISpace oldSpace)
        {
            Deactivate();
            base.OnRemovalFromSpace(oldSpace); //Sets this object's space to null.
        }
    }
}