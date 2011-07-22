using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.UpdateableSystems;
using BEPUphysics;
using Microsoft.Xna.Framework;
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
using Microsoft.Xna.Framework.Input;

namespace BEPUphysicsDemos.AlternateMovement.Testing.New
{
    public class CharacterController : Updateable, IBeforeNarrowPhaseUpdateable, IBeforePositionUpdateUpdateable, IEndOfTimeStepUpdateable
    {
        public Cylinder Body { get; private set; }


        public float StepHeight { get; set; }

        public HorizontalMotionConstraint HorizontalMotionConstraint { get; private set; }
        
        public float JumpSpeed = 4.5f;
        public float SlidingJumpSpeed = 3;

        /// <summary>
        /// Gets the support finder used by the character.
        /// The support finder analyzes the character's contacts to see if any of them provide support and/or traction.
        /// </summary>
        public SupportFinder SupportFinder { get; private set; }


        /// <summary>
        /// Gets or sets the maximum change in speed that the character will apply in order to stay connected to the ground.
        /// </summary>
        public float GlueSpeed { get; set; }

        ///// <summary>
        ///// Gets or sets the multiplier of horizontal force to apply to support objects when standing on top of dynamic entities.
        ///// </summary>
        //public float HorizontalForceFactor { get; set; }


        SupportData supportData;

        public CharacterController()
        {
            Body = new Cylinder(Vector3.Zero, 1.7f, .3f, 10);
            Body.CollisionInformation.Shape.CollisionMargin = .1f;
            //Making the character a continuous object prevents it from flying through walls which would be pretty jarring from a player's perspective.
            Body.PositionUpdateMode = PositionUpdateMode.Continuous;
            Body.LocalInertiaTensorInverse = new Matrix3X3();
            Body.CollisionInformation.Events.CreatingPair += RemoveFriction;
            GlueSpeed = 20;
            StepHeight = 1;
            //HorizontalForceFactor = 0;
            SupportFinder = new SupportFinder(this);
            HorizontalMotionConstraint = new HorizontalMotionConstraint(this);
        }

        void RemoveFriction(EntityCollidable sender, BroadPhaseEntry other, INarrowPhasePair pair)
        {
            var collidablePair = pair as CollidablePairHandler;
            if (collidablePair != null)
            {
                //The default values for InteractionProperties is all zeroes- zero friction, zero bounciness.
                //That's exactly how we want the character to behave when hitting objects.
                collidablePair.UpdateMaterialProperties(new InteractionProperties() );
            }
        }

        void ExpandBoundingBox()
        {
            if (Body.IsActive)
            {
                //This runs after the bounding box updater is run, but before the broad phase.
                //Expanding the character's bounding box ensures that minor variations in velocity will not cause
                //any missed information.
                //For a character which is not bound to Vector3.Up (such as a character that needs to run around a spherical planet),
                //the bounding box expansion needs to be changed such that it includes the full convex cast at the bottom and top of the character under any orientation.
                float radius = Body.CollisionInformation.Shape.Radius;
#if WINDOWS
                Vector3 offset;
#else
            Vector3 offset = new Vector3();
#endif
                offset.X = radius;
                offset.Y = StepHeight;
                offset.Z = radius;
                BoundingBox box = Body.CollisionInformation.BoundingBox;
                Vector3.Add(ref box.Max, ref offset, out box.Max);
                Vector3.Subtract(ref box.Min, ref offset, out box.Min);
                Body.CollisionInformation.BoundingBox = box;
            }


        }

        void IBeforeNarrowPhaseUpdateable.Update(float dt)
        {

            bool hadTraction = SupportFinder.HasTraction;
            //Identify supports.
            SupportFinder.UpdateSupports();

            //Collect the support data from the support, if any.
            if (SupportFinder.HasSupport)
            {
                if (SupportFinder.HasTraction)
                {
                    supportData = SupportFinder.TractionData.Value;
                }
                else
                {
                    supportData = SupportFinder.SupportData.Value;
                }
            }
            else
            {
                supportData = new SupportData();
            }

            HorizontalMotionConstraint.SupportData = supportData;

            //Compute the initial velocities relative to the support.
            Vector3 relativeVelocity;
            ComputeRelativeVelocity(out relativeVelocity);
            float verticalVelocity = Vector3.Dot(supportData.Normal, relativeVelocity);
            Vector3 horizontalVelocity = relativeVelocity - supportData.Normal * verticalVelocity;



            //Don't attempt to use an object as support if we are flying away from it (and we were never standing on it to begin with).
            if (SupportFinder.HasTraction && !hadTraction && verticalVelocity < 0)
            {
                SupportFinder.ClearSupportData();
                HorizontalMotionConstraint.SupportData = new SupportData();
            }

            ////The effective mass matrix describes how velocities are applied to supporting objects.
            //if (SupportFinder.HasSupport)
            //{
            //    ComputeEffectiveMassMatrix();
            //}


            ////Apply velocity control.
            //if (SupportFinder.HasSupport)
            //{
            //    //If the character has support then it can move around.
            //    //Apply horizontal velocities.
            //    Vector3 downDirection = Body.OrientationMatrix.Down;
            //    //If the object has traction, it has a lot more control over its motion.  If it is sliding, then use the sliding coefficients.
            //    float accelerationToUse = SupportFinder.HasTraction ? Acceleration : SlidingAcceleration;
            //    float decelerationToUse = SupportFinder.HasTraction ? Deceleration : SlidingDeceleration;
            //    Vector3 velocityDirection;
            //    Vector3 violatingVelocity;
            //    if (MovementDirection.LengthSquared() > 0)
            //    {
            //        //Project the movement direction onto the support plane defined by the support normal.
            //        //This projection is NOT along the support normal to the plane; that would cause the character to veer off course when moving on slopes.
            //        //Instead, project along the sweep direction to the plane.
            //        //For a 6DOF character controller, the lineStart would be different; it must be perpendicular to the local up.
            //        Vector3 lineStart = new Vector3(MovementDirection.X, 0, MovementDirection.Y);
            //        Vector3 lineEnd;
            //        Vector3.Add(ref lineStart, ref downDirection, out lineEnd);
            //        Plane plane = new Plane(SupportFinder.HasTraction ? SupportFinder.TractionData.Value.Normal : SupportFinder.SupportData.Value.Normal, 0);
            //        float t;
            //        //This method can return false when the line is parallel to the plane, but previous tests and the slope limit guarantee that it won't happen.
            //        Toolbox.GetLinePlaneIntersection(ref lineStart, ref lineEnd, ref plane, out t, out velocityDirection);

            //        //The origin->intersection line direction defines the horizontal velocity direction in 3d space.
            //        velocityDirection.Normalize();

            //        //Compare the current velocity to the goal velocity.
            //        float currentVelocity;
            //        Vector3.Dot(ref velocityDirection, ref horizontalVelocity, out currentVelocity);

            //        //Violating velocity is velocity which is not in the direction of the goal direction.
            //        //Also, it includes any velocity beyond the max speed or below zero.
            //        float violatingMagnitude;
            //        if (SupportFinder.HasTraction && currentVelocity > Speed)
            //            violatingMagnitude = currentVelocity - Speed;
            //        else if (SupportFinder.HasSupport && currentVelocity > SlidingSpeed)
            //            violatingMagnitude = currentVelocity - SlidingSpeed;
            //        else if (currentVelocity < 0)
            //            violatingMagnitude = currentVelocity;
            //        else
            //            violatingMagnitude = 0;
            //        violatingVelocity = horizontalVelocity - velocityDirection * (currentVelocity - violatingMagnitude);

            //        //Compute the acceleration component.
            //        float speedUpNecessary = Speed - currentVelocity;
            //        float velocityChange = MathHelper.Clamp(speedUpNecessary, 0, accelerationToUse * dt);

            //        //Apply the change.

            //        ChangeVelocity(velocityDirection * velocityChange, ref relativeVelocity);

            //    }
            //    else
            //    {
            //        velocityDirection = new Vector3();
            //        violatingVelocity = horizontalVelocity;
            //    }

            //    //Compute the deceleration component.
            //    float lengthSquared = violatingVelocity.LengthSquared();
            //    if (lengthSquared > 0)
            //    {
            //        Vector3 violatingVelocityDirection;
            //        float violatingVelocityMagnitude = (float)Math.Sqrt(lengthSquared);
            //        Vector3.Divide(ref violatingVelocity, violatingVelocityMagnitude, out violatingVelocityDirection);

            //        //We need to get rid of the violating velocity magnitude, but don't go under zero (that would cause nasty oscillations).
            //        float velocityChange = -Math.Min(decelerationToUse * dt, violatingVelocityMagnitude);
            //        //Apply the change.
            //        ChangeVelocity(violatingVelocityDirection * velocityChange, ref relativeVelocity);
            //    }

            //    //Also manage the vertical velocity of the character;
            //    //don't let it separate from the ground.
            //    if (SupportFinder.HasTraction)
            //    {
            //        verticalVelocity += Math.Max(supportData.Depth / dt, 0);
            //        if (verticalVelocity < 0 && verticalVelocity > -GlueSpeed)
            //        {
            //            ChangeVelocityUnilaterally(-supportData.Normal * verticalVelocity, ref relativeVelocity);
            //        }

            //    }
            //}
            //else
            //{
            //    //The character is in the air.  Still allow a little air control!
            //    //6DOF character controllers will likely completely replace this; if it doesn't,
            //    //use an oriented velocity direction instead of 2d movement direction.
            //    var velocityDirection = new Vector3(MovementDirection.X, 0, MovementDirection.Y);

            //    //Compare the current velocity to the goal velocity.
            //    float currentVelocity;
            //    Vector3.Dot(ref velocityDirection, ref relativeVelocity, out currentVelocity);

            //    //Compute the acceleration component.
            //    float speedUpNecessary = AirSpeed - currentVelocity;
            //    float velocityChange = MathHelper.Clamp(speedUpNecessary, 0, AirAcceleration * dt);

            //    //Apply the change.
            //    ChangeVelocityUnilaterally(velocityDirection * velocityChange, ref relativeVelocity);
            //}

            //Also manage the vertical velocity of the character;
            //don't let it separate from the ground.
            if (SupportFinder.HasTraction)
            {
                verticalVelocity += Math.Max(supportData.Depth / dt, 0);
                if (verticalVelocity < 0 && verticalVelocity > -GlueSpeed)
                {
                    ChangeVelocityUnilaterally(-supportData.Normal * verticalVelocity, ref relativeVelocity);
                }

            }

            //Attempt to jump.
            if (tryToJump)
            {
                //In the following, note that the jumping velocity changes are computed such that the separating velocity is specifically achieved,
                //rather than just adding some speed along an arbitrary direction.  This avoids some cases where the character could otherwise increase
                //the jump speed, which may not be desired.
                if (SupportFinder.HasTraction)
                {
                    //The character has traction, so jump straight up.
                    float currentUpVelocity = Vector3.Dot(Body.OrientationMatrix.Up, relativeVelocity);
                    //Target velocity is JumpSpeed.
                    float velocityChange = JumpSpeed - currentUpVelocity;
                    ChangeVelocityUnilaterally(Body.OrientationMatrix.Up * velocityChange, ref relativeVelocity);
                }
                else if (SupportFinder.HasSupport)
                {
                    //The character does not have traction, so jump along the surface normal instead.
                    float currentNormalVelocity = Vector3.Dot(supportData.Normal, relativeVelocity);
                    //Target velocity is JumpSpeed.
                    float velocityChange = SlidingJumpSpeed - currentNormalVelocity;
                    ChangeVelocityUnilaterally(supportData.Normal * -velocityChange, ref relativeVelocity);
                }
                SupportFinder.ClearSupportData();
                tryToJump = false;
                HorizontalMotionConstraint.SupportData = new SupportData();

                //TODO: Apply an opposite force to the support if it's dynamic.
                //Don't bother using a solver-based, correct 'jump' with effective mass matrix...
                //Guaranteeing the unilateral speed while applying a reasonable jump force will be fine.
            }

        }

        void ComputeRelativeVelocity(out Vector3 relativeVelocity)
        {

            //Compute the relative velocity between the body and its support, if any.
            //The relative velocity will be updated as impulses are applied.
            relativeVelocity = Body.LinearVelocity;
            if (SupportFinder.HasSupport)
            {
                //Only entities has velocity.
                var entityCollidable = supportData.SupportObject as EntityCollidable;
                if (entityCollidable != null)
                {
                    Vector3 entityVelocity = Toolbox.GetVelocityOfPoint(supportData.Position, entityCollidable.Entity);
                    Vector3.Subtract(ref relativeVelocity, ref entityVelocity, out relativeVelocity);
                }
            }

        }

        /// <summary>
        /// Changes the relative velocity between the character and its support.
        /// </summary>
        /// <param name="velocityChange">Change to apply to the character and support relative velocity.</param>
        /// <param name="relativeVelocity">Relative velocity to update.</param>
        void ChangeVelocity(Vector3 velocityChange, ref Vector3 relativeVelocity)
        {
            ////Instead of only applying the velocity to the character, see if we can apply any force to our support.
            ////The effective mass matrix defines how much impulse to apply to the support, and technically, to the character as well.
            ////However, we will ignore the suggestion it provides in favor of having more direct control over the character velocity.
            ////This isn't strictly correct, but this approach isn't entirely physical anyway.

            ////Only do the fancy calculations if we're standing on a dynamic entity.  Other types of supports just result in a unilateral velocity change.
            //var entityCollidable = supportData.SupportObject as EntityCollidable;
            //if (entityCollidable != null && entityCollidable.Entity.IsDynamic)
            //{
            //    Vector3 impulse;
            //    Matrix3X3.Transform(ref velocityChange, ref massMatrix, out impulse);
            //    //Body.LinearMomentum += impulse;
            //    Body.LinearVelocity += velocityChange;
            //    //entityCollidable.Entity.ApplyImpulse(supportData.Position, -(impulse * HorizontalForceFactor));
            //}
            //else
            {
                Body.LinearVelocity += velocityChange;
            }

            //Update the relative velocity as well.  It's a ref parameter, so this update will be reflected in the calling scope.
            Vector3.Add(ref relativeVelocity, ref velocityChange, out relativeVelocity);

        }

        /// <summary>
        /// In some cases, an applied velocity should only modify the character.
        /// This allows partially non-physical behaviors, like gluing the character to the ground.
        /// </summary>
        /// <param name="velocityChange">Change to apply to the character.</param>
        /// <param name="relativeVelocity">Relative velocity to update.</param>
        void ChangeVelocityUnilaterally(Vector3 velocityChange, ref Vector3 relativeVelocity)
        {
            Body.LinearVelocity += velocityChange;
            //Update the relative velocity as well.  It's a ref parameter, so this update will be reflected in the calling scope.
            Vector3.Add(ref relativeVelocity, ref velocityChange, out relativeVelocity);

        }




        void IBeforePositionUpdateUpdateable.Update(float dt)
        {
            //Also manage the vertical velocity of the character;
            //don't let it separate from the ground.
            if (SupportFinder.HasTraction)
            {
                Vector3 relativeVelocity;
                ComputeRelativeVelocity(out relativeVelocity);
                float verticalVelocity = Vector3.Dot(supportData.Normal, relativeVelocity);
                verticalVelocity += Math.Max(supportData.Depth / dt, 0);
                if (verticalVelocity < 0 && verticalVelocity > -GlueSpeed)
                {
                    ChangeVelocityUnilaterally(-supportData.Normal * verticalVelocity, ref relativeVelocity);
                }

            }
        }


        void IEndOfTimeStepUpdateable.Update(float dt)
        {
            //Teleport the object to the first hit surface.
            //This has to be done after the position update to ensure that no other systems get a chance to make an invalid state visible to the user, which would be corrected
            //jerkily in a subsequent frame.
            //Consider using forces instead.

            //if (IsSupported)
            //    Body.Position += -(goalSupportT - supportData.T) * sweep;
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
            newSpace.Add(HorizontalMotionConstraint);
            //This character controller requires the standard implementation of Space.
            ((Space)newSpace).BoundingBoxUpdater.Finishing += ExpandBoundingBox;

            Body.AngularVelocity = new Vector3();
            Body.LinearVelocity = new Vector3();
        }
        public override void OnRemovalFromSpace(ISpace oldSpace)
        {
            //Remove any supplements from the space too.
            oldSpace.Remove(Body);
            oldSpace.Remove(HorizontalMotionConstraint);
            //This character controller requires the standard implementation of Space.
            ((Space)oldSpace).BoundingBoxUpdater.Finishing -= ExpandBoundingBox;
            SupportFinder.ClearSupportData();
            Body.AngularVelocity = new Vector3();
            Body.LinearVelocity = new Vector3();
        }


    }
}

