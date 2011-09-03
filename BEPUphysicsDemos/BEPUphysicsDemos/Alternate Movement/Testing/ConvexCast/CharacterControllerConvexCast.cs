using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.UpdateableSystems;
using BEPUphysics;
using BEPUphysics.MathExtensions;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.Materials;
using BEPUphysics.PositionUpdating;
using BEPUphysics.DataStructures;
using System.Diagnostics;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Collidables;

namespace BEPUphysicsDemos.AlternateMovement.Testing.ConvexCast
{
    public class CharacterControllerConvexCast : Updateable, IBeforeNarrowPhaseUpdateable, IEndOfTimeStepUpdateable
    {
        public Capsule Body { get; private set; }


        public float StepHeight { get; set; }

        public Vector2 MovementDirection;
        public float Speed = 8;
        public float SlidingSpeed = 6;
        public float AirSpeed = 4;
        public float Acceleration = 50;
        public float SlidingAcceleration = 0;
        public float AirAcceleration = 5;
        public float Deceleration = 80;
        public float SlidingDeceleration = 1;
        public float JumpSpeed = 6;
        public float SlidingJumpSpeed = 4;
        float cosMaximumTractionSlope = (float)Math.Cos(MathHelper.PiOver4);
        /// <summary>
        /// Gets or sets the maximum slope on which the character will have traction.
        /// </summary>
        public float MaximumTractionSlope
        {
            get
            {
                return (float)Math.Acos(MathHelper.Clamp(cosMaximumTractionSlope, -1, 1));
            }
            set
            {
                cosMaximumTractionSlope = (float)Math.Cos(value);
            }
        }

        /// <summary>
        /// Gets or sets the maximum change in speed that the character will apply in order to stay connected to the ground.
        /// </summary>
        public float GlueSpeed { get; set; }

        bool hasTraction;
        /// <summary>
        /// Gets whether or not the character currently has traction on the ground.
        /// </summary>
        public bool HasTraction
        {
            get
            {
                return hasTraction;
            }
        }

        /// <summary>
        /// Gets whether or not the character currently has something supporting it.
        /// </summary>
        public bool IsSupported
        {
            get
            {
                return support != null;
            }
        }


        BoundingBox convexCastUpVolume;
        BoundingBox convexCastDownVolume;
        CylinderShape castShape;
        float supportMargin = .01f;
        float sweepLength;
        float goalSupportT;
        Collidable support;
        RayHit supportData;
        Vector3 sweep;

        public CharacterControllerConvexCast()
        {
            Body = new Capsule(Vector3.Zero, 1.7f, .3f, 10);
            //Making the character a continuous object prevents it from flying through walls which would be pretty jarring from a player's perspective.
            Body.PositionUpdateMode = PositionUpdateMode.Continuous;
            Body.LocalInertiaTensorInverse = new Matrix3X3();
            Body.CollisionInformation.Events.CreatingPair += RemoveFriction;
            GlueSpeed = 20;
            StepHeight = 1;
            //construct the casting shape.  It should be a little smaller than the character's radius.
            castShape = new CylinderShape(0, Body.Radius * .8f);
            castShape.CollisionMargin = 0;
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
            //This runs after the bounding box updater is run, but before the broad phase.
            //Expanding the character's bounding box ensures that minor variations in velocity will not cause
            //any missed information.
            //For a character which is not bound to Vector3.Up (such as a character that needs to run around a spherical planet),
            //the bounding box expansion needs to be changed such that it includes the full convex cast at the bottom and top of the character under any orientation.
            float radius = Body.CollisionInformation.Shape.Radius;
            Vector3 offset = new Vector3();
            offset.X = radius;
            offset.Y = StepHeight;
            offset.Z = radius;
            BoundingBox box = Body.CollisionInformation.BoundingBox;
            Vector3.Add(ref box.Max, ref offset, out box.Max);
            offset.Y += StepHeight + supportMargin; //Bottom is expanded by 2x step height for down stepping support.
            Vector3.Subtract(ref box.Min, ref offset, out box.Min);
            Body.CollisionInformation.BoundingBox = box;

            //Compute the individual bounding boxes for the upper convex cast and lower convex cast.
            //TODO: Update for non-pure capsule shape center offset.
            //TODO: Could use swept shape bounding boxes to make this a bit easier, especially for 6DOF character controllers.
            convexCastDownVolume = new BoundingBox()
                {
                    Min = Body.Position + new Vector3(-radius, -Body.Length * .5f - radius - StepHeight * 2 - supportMargin, -radius),
                    Max = Body.Position + new Vector3(radius, -Body.Length * .5f, radius)
                };
            convexCastUpVolume = new BoundingBox()
            {
                Min = Body.Position + new Vector3(-radius, Body.Length * .5f, -radius),
                Max = Body.Position + new Vector3(radius, Body.Length * .5f + radius + StepHeight, radius)
            };
        }

        void IBeforeNarrowPhaseUpdateable.Update(float dt)
        {
            //Cast down to find the support.

            //In a 6DOF character, the starting transform will vary.
            //For now, just assume it's locked to Up.
            RigidTransform startingTransform = new RigidTransform();
            startingTransform.Position = Body.Position;
            startingTransform.Position.Y -= Body.Length * .5f;
            startingTransform.Orientation = Quaternion.Identity;

            //Compute the sweep direction.  This too will change in a 6DOF character (Body.OrientationMatrix.Down instead of Vector3.Down).
            //If the body had traction last frame, then do a full-length sweep; this will detect step-downs.
            //If it didn't have traction, just cast down by one step height to try and find a support.
            //The reason why Body.Radius is added is that the starting transform is embedded in the capsule.
            sweepLength = hasTraction ? StepHeight * 2 + Body.Radius : StepHeight + Body.Radius + supportMargin;
            sweep = new Vector3(0, -sweepLength, 0);

            //Cycle through every collidable and find the first time of impact for the convex cast.

            support = null;
            supportData = new RayHit() { T = float.MaxValue }; //Set the T value to an extremely high value to start with so that any result will be lower.
            foreach (var collidable in Body.CollisionInformation.OverlappedCollidables)
            {
                if (convexCastDownVolume.Intersects(collidable.BoundingBox))
                {
                    RayHit hit;
                    if (collidable.ConvexCast(castShape, ref startingTransform, ref sweep, out hit) && hit.T < supportData.T)
                    {
                        supportData = hit;
                        support = collidable;
                    }

                }
            }

            goalSupportT = (Body.Radius + StepHeight) / sweepLength;


            //If there's any support, analyze it.
            //Check the slope of the hit against the maximum.
            //This is a dot product of the normal against the body up vector (in this implementation, always {0, 1, 0}).
            //For a fixed up vector of {0,1,0}, the dot product result is just the Y component of the normal.
            //The following slope test is equivalent to MaximumTractionSlope > Math.Acos(Vector3.Dot(-firstHitData.Normal, Vector3.Up))
            hasTraction = support != null & cosMaximumTractionSlope < -supportData.Normal.Y;


            //Compute the relative velocity between the capsule and its support, if any.
            //The relative velocity will be updated as impulses are applied.
            Vector3 relativeVelocity = Body.LinearVelocity;
            if (IsSupported)
            {
                //Only entities has velocity.
                var entityCollidable = support as EntityCollidable;
                if (entityCollidable != null)
                {
                    Vector3 entityVelocity = Toolbox.GetVelocityOfPoint(supportData.Location, entityCollidable.Entity);
                    Vector3.Subtract(ref relativeVelocity, ref entityVelocity, out relativeVelocity);
                    //TODO: Multithreaded safety.  If characters are running in parallel, ensure that this operation will not corrupt anything.
                    if (entityCollidable.Entity.IsDynamic)
                        entityCollidable.Entity.ActivityInformation.Activate();
                }
            }

            //Attempt to jump.
            if (tryToJump)
            {
                //In the following, note that the jumping velocity changes are computed such that the separating velocity is specifically achieved,
                //rather than just adding some speed along an arbitrary direction.  This avoids some cases where the character could otherwise increase
                //the jump speed, which may not be desired.
                if (hasTraction)
                {
                    //The character has traction, so jump straight up.
                    float currentUpVelocity = Vector3.Dot(Body.OrientationMatrix.Up, relativeVelocity);
                    //Target velocity is JumpSpeed.
                    float velocityChange = JumpSpeed - currentUpVelocity;
                    ChangeVelocity(Body.OrientationMatrix.Up * velocityChange, ref relativeVelocity);
                }
                else if (IsSupported)
                {
                    //The character does not have traction, so jump along the surface normal instead.
                    float currentNormalVelocity = Vector3.Dot(supportData.Normal, relativeVelocity);
                    //Target velocity is JumpSpeed.
                    float velocityChange = SlidingJumpSpeed - currentNormalVelocity;
                    ChangeVelocity(supportData.Normal * -velocityChange, ref relativeVelocity);
                }
                hasTraction = false;
                support = null;
                tryToJump = false;
            }


            //Update the vertical motion of the character.
            if (hasTraction)
            {
                //Remove all velocity, penetrating or separating, along the contact normal if it is below a threshold.
                float separatingVelocity = -Vector3.Dot(relativeVelocity, supportData.Normal);
                if (separatingVelocity < GlueSpeed)
                {
                    ChangeVelocity(supportData.Normal * separatingVelocity, ref relativeVelocity);
                }
                else
                    hasTraction = false;


            }
            else if (IsSupported)
            {
                //The character has no traction, so just stop penetrating velocity.
                float dot = -Vector3.Dot(relativeVelocity, supportData.Normal);
                if (dot < 0)
                    ChangeVelocity(supportData.Normal * dot, ref relativeVelocity);

            }

            //Update the horizontal motion of the character.
            if (IsSupported)
            {
                //If the object has traction, it has a lot more control over its motion.  If it is sliding, then use the sliding coefficients.
                float accelerationToUse = hasTraction ? Acceleration : SlidingAcceleration;
                float decelerationToUse = hasTraction ? Deceleration : SlidingDeceleration;
                Vector3 velocityDirection;
                Vector3 violatingVelocity;
                if (MovementDirection.LengthSquared() > 0)
                {
                    //Project the movement direction onto the support plane defined by the support normal.
                    //This projection is NOT along the support normal to the plane; that would cause the character to veer off course when moving on slopes.
                    //Instead, project along the sweep direction to the plane.
                    //For a 6DOF character controller, the lineStart would be different; it must be perpendicular to the local up.
                    Vector3 lineStart = new Vector3(MovementDirection.X, 0, MovementDirection.Y);
                    Vector3 lineEnd;
                    Vector3.Add(ref lineStart, ref sweep, out lineEnd);
                    Plane plane = new Plane(supportData.Normal, 0);
                    float t;
                    //This method can return false when the line is parallel to the plane, but previous tests and the slope limit guarantee that it won't happen.
                    Toolbox.GetLinePlaneIntersection(ref lineStart, ref lineEnd, ref plane, out t, out velocityDirection);

                    //The origin->intersection line direction defines the horizontal velocity direction in 3d space.
                    velocityDirection.Normalize();

                    //Compare the current velocity to the goal velocity.
                    float currentVelocity;
                    Vector3.Dot(ref velocityDirection, ref relativeVelocity, out currentVelocity);

                    //Violating velocity is velocity which is not in the direction of the goal direction.
                    violatingVelocity = relativeVelocity - velocityDirection * currentVelocity;

                    //Compute the acceleration component.
                    float speedUpNecessary = Speed - currentVelocity;
                    float velocityChange = MathHelper.Clamp(speedUpNecessary, 0, accelerationToUse * dt);

                    //Apply the change.

                    ChangeVelocity(velocityDirection * velocityChange, ref relativeVelocity);

                }
                else
                {
                    velocityDirection = new Vector3();
                    violatingVelocity = relativeVelocity;
                }

                //Compute the deceleration component.
                float lengthSquared = violatingVelocity.LengthSquared();
                if (lengthSquared > 0)
                {
                    Vector3 violatingVelocityDirection;
                    float violatingVelocityMagnitude = (float)Math.Sqrt(lengthSquared);
                    Vector3.Divide(ref violatingVelocity, violatingVelocityMagnitude, out violatingVelocityDirection);

                    //We need to get rid of the violating velocity magnitude, but don't go under zero (that would cause nasty oscillations).
                    float velocityChange = -Math.Min(decelerationToUse * dt, violatingVelocityMagnitude);
                    //Apply the change.
                    ChangeVelocity(violatingVelocityDirection * velocityChange, ref relativeVelocity);
                }

            }
            else
            {
                //The character is in the air.  Still allow a little air control!
                //6DOF character controllers will likely completely replace this; if it doesn't,
                //use an oriented velocity direction instead of 2d movement direction.
                var velocityDirection = new Vector3(MovementDirection.X, 0, MovementDirection.Y);

                //Compare the current velocity to the goal velocity.
                float currentVelocity;
                Vector3.Dot(ref velocityDirection, ref relativeVelocity, out currentVelocity);

                //Compute the acceleration component.
                float speedUpNecessary = AirSpeed - currentVelocity;
                float velocityChange = MathHelper.Clamp(speedUpNecessary, 0, AirAcceleration * dt);

                //Apply the change.
                ChangeVelocity(velocityDirection * velocityChange, ref relativeVelocity);
            }


        }

        void ChangeVelocity(Vector3 velocityChange, ref Vector3 relativeVelocity)
        {
            Body.LinearVelocity += velocityChange;
            //Update the relative velocity as well.  It's a ref parameter, so this update will be reflected in the calling scope.
            Vector3.Add(ref relativeVelocity, ref velocityChange, out relativeVelocity);
            
        }




        void IEndOfTimeStepUpdateable.Update(float dt)
        {
            //Teleport the object to the first hit surface.
            //This has to be done after the position update to ensure that no other systems get a chance to make an invalid state visible to the user, which would be corrected
            //jerkily in a subsequent frame.
            //Consider using forces instead.

            if (IsSupported)
                Body.Position += -(goalSupportT - supportData.T) * sweep;
        }

        bool tryToJump = false;
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

        public override void OnAdditionToSpace(ISpace newSpace)
        {
            //Add any supplements to the space too.
            newSpace.Add(Body);
            //This character controller requires the standard implementation of Space.
            ((Space)newSpace).BoundingBoxUpdater.Finishing += ExpandBoundingBox;
        }
        public override void OnRemovalFromSpace(ISpace oldSpace)
        {
            //Remove any supplements from the space too.
            oldSpace.Remove(Body);
            //This character controller requires the standard implementation of Space.
            ((Space)oldSpace).BoundingBoxUpdater.Finishing -= ExpandBoundingBox;
            support = null;
            supportData = new RayHit();
            hasTraction = false;
            Body.AngularVelocity = new Vector3();
            Body.LinearVelocity = new Vector3();
        }

    }
}
