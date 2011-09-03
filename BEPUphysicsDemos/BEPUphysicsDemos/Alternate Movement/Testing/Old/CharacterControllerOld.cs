using System;
using BEPUphysics;
using BEPUphysics.Entities;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.MathExtensions;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.UpdateableSystems;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;

namespace BEPUphysicsDemos.AlternateMovement.Testing.Old
{
    public class CharacterControllerOld : Updateable, IEndOfTimeStepUpdateable
    {
        /// <summary>
        /// Shape that will be used in a convex cast that finds supporting entities for the character or any entities blocking a step-up.
        /// When the character is supported and looking for something to step on, this shape will be cast downward.
        /// To verify that it is valid, it will then be cast upward from the character's head.
        /// </summary>
        private CylinderShape castingShape;

        /// <summary>
        /// A box positioned relative to the character's body used to identify collision pairs with nearby objects that could be possibly stood upon.
        /// </summary>
        private Box feetCollisionPairCollector;

        /// <summary>
        /// The displacement vector from the center of the character body cylinder to the center of the collision pair collector box entity.
        /// </summary>
        private Vector3 feetCollisionPairCollectorPositionOffset;

        /// <summary>
        /// The displacement vector from the center of the character body cylinder to the starting position of the convex cast used to find entity supports below the character.
        /// </summary>
        private Vector3 feetSupportFinderOffset;

        /// <summary>
        /// The displacement vector from the center of the character body cylinder to the starting position of the convex cast used to find blockages above the character's head when stepping.
        /// </summary>
        private Vector3 headBlockageFinderOffset;

        /// <summary>
        /// A box positioned relative to the character's body used to identify collision pairs with nearby objects that could hit the character's head if it stepped up on something.
        /// </summary>
        private Box headCollisionPairCollector;

        /// <summary>
        /// The displacement vector from the center of the character body cylinder to the center of the collision pair collector box entity.
        /// </summary>
        private Vector3 headCollisionPairCollectorPositionOffset;

        /// <summary>
        /// The maximum distance up or down which the character is able to move without jumping or falling.
        /// </summary>
        private float maximumStepHeight;

        /// <summary>
        /// The vertical distance between the ground and the outer collision margin of the character cylinder to maintain.
        /// Should be some relatively small value; if the character cylinder (and its margin) are allowed to hit the ground before the support finder, 
        /// the character may either lose support or become affected by extra frictional forces.
        /// </summary>
        private float supportMargin;

        /// <summary>
        /// Rate of increase in the character's speed in the movementDirection.
        /// </summary>
        public float Acceleration = 50f;

        /// <summary>
        /// Change in airborne speed per second.
        /// </summary>
        public float AirborneAcceleration = 30;


        /// <summary>
        /// The character's physical representation that handles iteractions with the environment.
        /// </summary>
        public Cylinder Body;

        /// <summary>
        /// If objects are moving away from each other vertically above this speed, the character will lose its grip on the ground.
        /// </summary>
        private float glueSpeed = 5;

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
        public float JumpSpeed = 6;

        /// <summary>
        /// The maximum slope under which walking forces can be applied.
        /// </summary>
        public float MaxSlope = MathHelper.PiOver4;

        /// <summary>
        /// Maximum speed in the movementDirection that can be attained.
        /// </summary>
        public float MaxSpeed = 8;


        /// <summary>
        /// The maximum speed that the character can achieve by itself while airborne.
        /// A character can exceed this by launching off the ground at a higher speed, but cannot force itself to accelerate faster than this using air control while airborne.
        /// </summary>
        public float MaximumAirborneSpeed = 2;


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
        /// <param name="mass">Total mass of the character.</param>
        /// <param name="maximumStepHeight">Height that the character can climb up.</param>
        public CharacterControllerOld(Vector3 position, float characterHeight, float characterWidth, float mass, float maximumStepHeight)
        {
            //Create the physical body of the character. 
            //The character's cylinder height and radius must be shrunk down marginally
            //to take into account the collision margin and support margin while still fitting in the defined character height/width.
            var bodyPosition = new Vector3(position.X, position.Y + supportMargin / 2, position.Z);
            float collisionMargin = .04f;
            Body = new Cylinder(bodyPosition,
                                characterHeight - 2 * collisionMargin - supportMargin,
                                characterWidth / 2 - collisionMargin,
                                mass);
            Body.CollisionInformation.Shape.CollisionMargin = collisionMargin;

            feetCollisionPairCollectorPositionOffset = new Vector3(0, -Body.Height / 2 - supportMargin - collisionMargin, 0);
            feetCollisionPairCollector = new Box(bodyPosition + feetCollisionPairCollectorPositionOffset, characterWidth, maximumStepHeight * 2, characterWidth, 1);
            feetCollisionPairCollector.CollisionInformation.CollisionRules.Personal = CollisionRule.NoNarrowPhaseUpdate; //Prevents collision detection/contact generation from being run.
            feetCollisionPairCollector.IsAffectedByGravity = false;
            CollisionRules.AddRule(feetCollisionPairCollector, Body, CollisionRule.NoBroadPhase);//Prevents the creation of any collision pairs between the body and the collector.
            feetSupportFinderOffset = new Vector3(0, feetCollisionPairCollectorPositionOffset.Y + maximumStepHeight, 0);

            headCollisionPairCollectorPositionOffset = new Vector3(0, (Body.Height + maximumStepHeight) / 2 + collisionMargin, 0);
            headCollisionPairCollector = new Box(bodyPosition + headCollisionPairCollectorPositionOffset, characterWidth, maximumStepHeight + collisionMargin, characterWidth, 1);
            headCollisionPairCollector.CollisionInformation.CollisionRules.Personal = CollisionRule.NoNarrowPhaseUpdate; //Prevents collision detection/contact generation from being run.
            headCollisionPairCollector.IsAffectedByGravity = false;
            CollisionRules.AddRule(headCollisionPairCollector, Body, CollisionRule.NoBroadPhase); //Prevents the creation of any collision pairs between the body and the collector.
            headBlockageFinderOffset = new Vector3(0, headCollisionPairCollectorPositionOffset.Y - maximumStepHeight / 2 - collisionMargin, 0);

            castingShape = new CylinderShape(0, Body.Radius + collisionMargin);
            castingShape.CollisionMargin = 0;

            this.maximumStepHeight = maximumStepHeight;
            this.supportMargin = .01f;

            Body.LocalInertiaTensorInverse = new Matrix3X3();
            feetCollisionPairCollector.LocalInertiaTensorInverse = new Matrix3X3();
            headCollisionPairCollector.LocalInertiaTensorInverse = new Matrix3X3();
        }

        /// <summary>
        /// Gets and sets the position of the character.
        /// </summary>
        public Vector3 Position
        {
            get
            {
                //Since the position of the character is not the position of the character's physical cylinder (due to the support margin), it needs to be offset.
                Vector3 bodyPosition = Body.Position;
                return new Vector3(bodyPosition.X, bodyPosition.Y + supportMargin / 2, bodyPosition.Z);
            }
            set
            {
                Vector3 offset = Position - value;
                Body.Position += offset;
                //While the collision pair collectors will follow the body in the following frame, it might be a bit messy just to leave them hanging around until then.
                feetCollisionPairCollector.Position += offset;
                headCollisionPairCollector.Position += offset;
            }
        }

        /// <summary>
        /// Handles the updating of the character.  Called by the owning space object when necessary.
        /// </summary>
        /// <param name="dt">Simulation seconds since the last update.</param>
        void IEndOfTimeStepUpdateable.Update(float dt)
        {
            Collidable supportCollidable;
            Vector3 supportLocation, supportNormal, supportLocationVelocity;
            float supportDistance;

            if (FindSupport(out supportCollidable, out supportLocation, out supportNormal, out supportDistance, out supportLocationVelocity))
            {
                IsSupported = true;
                Support(supportLocationVelocity, supportNormal, supportDistance);
                HasTraction = IsSupportSlopeWalkable(supportNormal);
                HandleHorizontalMotion(supportLocationVelocity, supportNormal, dt);
            }
            else
            {
                IsSupported = false;
                HasTraction = false;
                HandleAirborneMotion(dt);
            }

            feetCollisionPairCollector.LinearVelocity = Body.LinearVelocity;
            feetCollisionPairCollector.Position = (Body.Position + feetCollisionPairCollectorPositionOffset);
            headCollisionPairCollector.LinearVelocity = Body.LinearVelocity;
            headCollisionPairCollector.Position = (Body.Position + headCollisionPairCollectorPositionOffset);
        }

        /// <summary>
        /// Locates the closest support entity by performing a convex cast at collected candidates.
        /// </summary>
        /// <param name="supportEntity">The closest supporting entity.</param>
        /// <param name="supportLocation">The support location which the character is standing on.</param>
        /// <param name="supportNormal">The normal at the surface where the character is standing..</param>
        /// <param name="supportDistance">Distance from the maximum step height on the character down to where the latest support location was found.</param>
        /// <param name="supportLocationVelocity">Velocity of the support location on the support entity.</param>
        /// <returns>Whether or not a support was located.</returns>
        private bool FindSupport(out Collidable supportCollidable, out Vector3 supportLocation, out Vector3 supportNormal, out float supportDistance, out Vector3 supportLocationVelocity)
        {
            //If there's no traction, it shouldn't try to 'step down' anything so there's no need to extend the cast.
            float maximumDistance;
            if (HasTraction)
                maximumDistance = maximumStepHeight * 2;
            else
                maximumDistance = maximumStepHeight;
            var sweep = new Vector3(0, -maximumDistance, 0);
            Vector3 startingLocation = Body.Position + feetSupportFinderOffset;

            //There are two 'minimums' to keep track of.  
            //  The first is out of all collisions.
            //  This first collision is used to determine what to 'step' to, which means it requires a significant up or down jump to reach.
            //Since the first collision can be invalidated by various factors,
            //  there is also a defined range of support heights right around the character's feet where no validation is necessary.
            //  This allows the character keep standing on whatever it was that it was standing on before the "step" failed.
            Vector3 hitLocation = new Vector3(), stepHitLocation = new Vector3();
            Vector3 hitNormal = new Vector3(), stepHitNormal = new Vector3();
            Collidable hitCollidable = null, stepHitCollidable = null;
            float distance = float.MaxValue, stepDistance = float.MaxValue;

            foreach (var candidate in feetCollisionPairCollector.CollisionInformation.OverlappedCollidables)
            {

                //for (int i = 0; i < feetCollisionPairCollector.CollisionInformation.Pairs.Count; i++)
                //{
                //var pair = feetCollisionPairCollector.CollisionInformation.Pairs[i];

                //Determine which member of the collision pair is the possible support.
                //The comparisons are all kept on a "parent" as opposed to "collider" level so that interaction with compound shapes is simpler.
                //Entity candidate = pair.ColliderA == feetCollisionPairCollector ? pair.ColliderB : pair.ColliderA;
                //Ensure that the candidate is a valid supporting entity.
                if (candidate.CollisionRules.Personal > CollisionRule.Normal)
                    continue; //It is invalid!

                //Fire a convex cast at the candidate and determine some details! 

                RigidTransform sweepTransform = new RigidTransform(startingLocation);
                RayHit rayHit;
                //if (GJKToolbox.ConvexCast(castingShape, targetShape, ref sweep,
                //                          ref sweepTransform, ref targetTransform,
                //                          out rayHit))
                if (candidate.ConvexCast(castingShape, ref sweepTransform, ref sweep, out rayHit))
                {
                    //tempHitNormal *= -1;
                    rayHit.T *= maximumDistance;
                    if (rayHit.T < stepDistance)
                    {
                        stepDistance = rayHit.T;
                        stepHitLocation = rayHit.Location;
                        stepHitNormal = rayHit.Normal;
                        stepHitCollidable = candidate;
                    }
                    if (rayHit.T < distance &&
                        //If the hit is within a small margin range at the base of the character...
                        rayHit.T >= maximumStepHeight - Body.CollisionInformation.Shape.CollisionMargin - supportMargin && rayHit.T <= maximumStepHeight)
                    {
                        //Then this could be one of the non-step supports.
                        distance = rayHit.T;
                        hitLocation = rayHit.Location;
                        hitNormal = rayHit.Normal;
                        hitCollidable = candidate;
                    }

                }
            }
            Vector3 candidateLocationVelocity = new Vector3();
            if (stepDistance != float.MaxValue)
            {
                stepHitNormal.Normalize();
                var entityCollidable = (stepHitCollidable as EntityCollidable);
                if (entityCollidable != null)
                    candidateLocationVelocity = entityCollidable.Entity.LinearVelocity + //linear component
                                                Vector3.Cross(entityCollidable.Entity.AngularVelocity, stepHitLocation - entityCollidable.Entity.Position);
                //linear velocity of point on body relative to center

                //Analyze the hits.
                //Convert the 'times of impact' along the cast into distance by multiplying by the length of the cast.
                //Verify the "stepped" hit.  If it's invalid, go with the "non step" hit.
                if (stepDistance > 0 && //If hit distance was zero, it would probably mean that the outer convex cast found a wall (or something not steppable).
                    Body.LinearVelocity.Y - candidateLocationVelocity.Y < glueSpeed && //Don't try to 'glue' to the ground if we're just flying away from it.
                    IsSupportSlopeWalkable(stepHitNormal) && //In order for this to be stepped on, it should be walkable.
                    (stepDistance > maximumStepHeight || IsStepSafe(stepDistance))) //If its planning to step up, make sure its safe to do so.
                {
                    //Successfully found a stepping location.
                    supportCollidable = stepHitCollidable;
                    supportLocation = stepHitLocation;
                    supportNormal = stepHitNormal;
                    supportDistance = stepDistance;
                    supportLocationVelocity = candidateLocationVelocity;
                    return true;
                }
                if (distance != float.MaxValue)
                {
                    //Once it makes it here, it means there exists a support around the feet.
                    //Compute a new velocity for the feet support.
                    entityCollidable = (hitCollidable as EntityCollidable);
                    if (entityCollidable != null)
                        candidateLocationVelocity = entityCollidable.Entity.LinearVelocity + //linear component
                                                Vector3.Cross(entityCollidable.Entity.AngularVelocity, hitLocation - entityCollidable.Entity.Position);
                    //linear velocity of point on body relative to center

                    //The only condition that matters for a near-feet support is that the character is approaching the support rather than flying up away.
                    if (Body.LinearVelocity.Y - candidateLocationVelocity.Y < glueSpeed)
                    {
                        //While the step failed, there's still the backup of looking at the places right around the feet.
                        supportCollidable = hitCollidable;
                        supportLocation = hitLocation;
                        supportNormal = Vector3.Normalize(hitNormal);
                        supportDistance = distance;
                        supportLocationVelocity = candidateLocationVelocity;
                        return true;
                    }
                }
            }
            supportCollidable = null;
            supportLocation = Toolbox.NoVector;
            supportNormal = Toolbox.NoVector;
            supportDistance = float.MaxValue;
            supportLocationVelocity = Toolbox.NoVector;
            return false;
        }

        /// <summary>
        /// Performs a convex cast to determine if a step of a given height is valid.
        /// This means that stepping up onto the new support wouldn't shove the character's head into a ceiling or other obstacle.
        /// </summary>
        /// <param name="hitDistance">Vertical distance from the convex cast start to the hit location.</param>
        /// <returns>Whether or not the step is safe.</returns>
        private bool IsStepSafe(float hitDistance)
        {
            float stepHeight = maximumStepHeight - hitDistance;
            var sweep = new Vector3(0, stepHeight + Body.CollisionInformation.Shape.CollisionMargin, 0);
            Vector3 startingLocation = headBlockageFinderOffset + Body.Position;

            foreach (Entity candidate in headCollisionPairCollector.CollisionInformation.OverlappedEntities)
            {
                //foreach (CollisionPair pair in headCollisionPairCollector.CollisionPairs)
                //{
                //    //Determine which member of the collision pair is the possible blockage.
                //    //The comparisons are all kept on a "parent" as opposed to "collider" level so that interaction with compound shapes is simpler.

                //    Entity candidate = pair.ParentA == headCollisionPairCollector ? pair.ParentB : pair.ParentA;
                //Ensure that the candidate is a valid blocking entity.
                if (candidate.CollisionInformation.CollisionRules.Personal > CollisionRule.Normal)
                    continue; //It is invalid!


                //Fire a convex cast at the candidate and determine some details!  
                ConvexShape targetShape = candidate.CollisionInformation.Shape as ConvexShape;
                if (targetShape != null)
                {
                    RigidTransform sweepTransform = new RigidTransform(startingLocation);
                    RigidTransform targetTransform = new RigidTransform(candidate.Position, candidate.Orientation);
                    RayHit rayHit;
                    if (GJKToolbox.ConvexCast(castingShape, targetShape, ref sweep,
                                              ref sweepTransform, ref targetTransform,
                                              out rayHit))
                    {
                        return false;
                    }
                }
            }
            return true;
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
            float heightDifference = maximumStepHeight - supportDistance;
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
            }
        }

        /// <summary>
        /// Manages the character's air control.
        /// </summary>
        /// <param name="dt">Timestep of the simulation.</param>
        private void HandleAirborneMotion(float dt)
        {
            //Compute the current horizontal speed, two dimensional dot product.
            float speed = Body.LinearVelocity.X * MovementDirection.X + Body.LinearVelocity.Z * MovementDirection.Y;

            float previousSpeed = speed;
            speed = Math.Min(MaximumAirborneSpeed, speed + AirborneAcceleration * dt);
            float changeInSpeed = MathHelper.Max(0, speed - previousSpeed);

            Body.LinearVelocity += new Vector3(changeInSpeed * MovementDirection.X, 0, changeInSpeed * MovementDirection.Y);
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
                    Space.Add(feetCollisionPairCollector);
                    Space.Add(headCollisionPairCollector);
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
                    Space.Remove(feetCollisionPairCollector);
                    Space.Remove(headCollisionPairCollector);
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
            Activate();
        }

        /// <summary>
        /// Called by the engine when the character is removed from the space.
        /// Deactivates the character.
        /// </summary>
        public override void OnRemovalFromSpace(ISpace space)
        {
            Deactivate();
        }
    }
}