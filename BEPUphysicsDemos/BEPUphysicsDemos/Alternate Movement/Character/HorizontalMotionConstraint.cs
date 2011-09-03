using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.Constraints;
using BEPUphysics.DataStructures;
using BEPUphysics.Entities;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.MathExtensions;
using BEPUphysics;
using System.Diagnostics;

namespace BEPUphysicsDemos.AlternateMovement.Character
{
    /// <summary>
    /// Manages the horizontal movement of a character.
    /// </summary>
    public class HorizontalMotionConstraint : EntitySolverUpdateable
    {
        CharacterController character;


        SupportData supportData;
        /// <summary>
        /// Gets or sets the support data used by the constraint.
        /// </summary>
        public SupportData SupportData
        {
            get
            {
                return supportData;
            }
            set
            {
                //If the support changes, perform the necessary bookkeeping to keep the connections up to date.
                var oldSupport = supportData.SupportObject;
                supportData = value;
                if (oldSupport != supportData.SupportObject)
                {
                    OnInvolvedEntitiesChanged();
                }
            }
        }

        Vector2 movementDirection;
        /// <summary>
        /// Gets or sets the goal movement direction.
        /// </summary>
        public Vector2 MovementDirection
        {
            get { return movementDirection; }
            set
            {
                float lengthSquared = value.LengthSquared();
                if (lengthSquared > Toolbox.Epsilon)
                {
                    character.Body.ActivityInformation.Activate();
                    Vector2.Divide(ref value, (float)Math.Sqrt(lengthSquared), out movementDirection);
                }
                else
                {
                    character.Body.ActivityInformation.Activate();
                    movementDirection = new Vector2();
                }
            }
        }

        float speed = 8f;
        /// <summary>
        /// Gets or sets the maximum speed at which the character can move while standing with a support that provides traction.
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
                    throw new Exception("Value must be nonnegative.");
                speed = value;
            }
        }
        float crouchingSpeed = 3f;
        /// <summary>
        /// Gets or sets the maximum speed at which the character can move while crouching with a support that provides traction.
        /// Relative velocities with a greater magnitude will be decelerated.
        /// </summary>
        public float CrouchingSpeed
        {
            get
            {
                return crouchingSpeed;
            }
            set
            {
                if (value < 0)
                    throw new Exception("Value must be nonnegative.");
                crouchingSpeed = value;
            }
        }
        float slidingSpeed = 6;
        /// <summary>
        /// Gets or sets the maximum speed at which the character can move while on a support that does not provide traction.
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
                    throw new Exception("Value must be nonnegative.");
                slidingSpeed = value;
            }
        }
        float airSpeed = 1;
        /// <summary>
        /// Gets or sets the maximum speed at which the character can move with no support.
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
                    throw new Exception("Value must be nonnegative.");
                airSpeed = value;
            }
        }
        float maximumForce = 1000;
        /// <summary>
        /// Gets or sets the maximum force that the character can apply while on a support which provides traction.
        /// </summary>
        public float MaximumForce
        {
            get
            {
                return maximumForce;
            }
            set
            {
                if (value < 0)
                    throw new Exception("Value must be nonnegative.");
                maximumForce = value;
            }
        }
        float maximumSlidingForce = 50;
        /// <summary>
        /// Gets or sets the maximum force that the character can apply while on a support which does not provide traction.
        /// </summary>
        public float MaximumSlidingForce
        {
            get
            {
                return maximumSlidingForce;
            }
            set
            {
                if (value < 0)
                    throw new Exception("Value must be nonnegative.");
                maximumSlidingForce = value;
            }
        }
        float maximumAirForce = 250;
        /// <summary>
        /// Gets or sets the maximum force that the character can apply with no support.
        /// </summary>
        public float MaximumAirForce
        {
            get
            {
                return maximumAirForce;
            }
            set
            {
                if (value < 0)
                    throw new Exception("Value must be nonnegative.");
                maximumAirForce = value;
            }
        }

        float supportForceFactor = 1;
        /// <summary>
        /// Gets or sets the scaling factor of forces applied to the supporting object if it is a dynamic entity.
        /// Low values (below 1) reduce the amount of motion imparted to the support object; it acts 'heavier' as far as horizontal motion is concerned.
        /// High values (above 1) increase the force applied to support objects, making them appear lighter.
        /// Be careful when changing this- it can create impossible situations! Imagine the character standing on a compound object.  If the support force
        /// factor is less than 1, then the character can push the compound around while standing on it!
        /// </summary>
        public float SupportForceFactor
        {
            get
            {
                return supportForceFactor;
            }
            set
            {
                if (value < 0)
                    throw new Exception("Value must be nonnegative.");
                supportForceFactor = value;
            }
        }

        /// <summary>
        /// Gets the movement mode that the character is currently in.
        /// </summary>
        public MovementMode MovementMode { get; private set; }

        float maxSpeed;
        float maxForce;


        Matrix2X2 massMatrix;
        Entity supportEntity;
        Vector3 linearJacobianA1;
        Vector3 linearJacobianA2;
        Vector3 linearJacobianB1;
        Vector3 linearJacobianB2;
        Vector3 angularJacobianB1;
        Vector3 angularJacobianB2;

        Vector2 accumulatedImpulse;
        Vector2 targetVelocity;

        Vector2 positionCorrectionBias;

        Vector3 positionLocalOffset;
        bool wasTryingToMove = false;
        bool hadTraction = false;
        Entity previousSupportEntity;
        float tractionDecelerationTime;
        float timeSinceTransition;

        /// <summary>
        /// Constructs a new horizontal motion constraint.
        /// </summary>
        /// <param name="characterController">Character to be governed by this constraint.</param>
        public HorizontalMotionConstraint(CharacterController characterController)
        {
            this.character = characterController;
            CollectInvolvedEntities();
            //Compute the time it usually takes for the character to slow down while it has traction.
            tractionDecelerationTime = speed / (maximumForce * character.Body.InverseMass);
        }


        protected override void CollectInvolvedEntities(RawList<Entity> outputInvolvedEntities)
        {
            var entityCollidable = supportData.SupportObject as EntityCollidable;
            if (entityCollidable != null)
                outputInvolvedEntities.Add(entityCollidable.Entity);
            outputInvolvedEntities.Add(character.Body);

        }

        /// <summary>
        /// Computes per-frame information necessary for the constraint.
        /// </summary>
        /// <param name="dt">Time step duration.</param>
        public override void Update(float dt)
        {
            //Collect references, pick the mode, and configure the coefficients to be used by the solver.
            bool isTryingToMove = movementDirection.LengthSquared() > 0;
            if (supportData.SupportObject != null)
            {
                //Get an easy reference to the support.
                var support = supportData.SupportObject as EntityCollidable;
                if (support != null)
                {
                    supportEntity = support.Entity;
                }
                else
                {
                    supportEntity = null;

                }
                if (supportData.HasTraction)
                {
                    MovementMode = MovementMode.Traction;
                    if (character.StanceManager.CurrentStance == Stance.Standing)
                        maxSpeed = speed;
                    else
                        maxSpeed = crouchingSpeed;
                    maxForce = maximumForce;
                }
                else
                {
                    MovementMode = MovementMode.Sliding;
                    maxSpeed = slidingSpeed;
                    maxForce = maximumSlidingForce;
                }
            }
            else
            {
                MovementMode = MovementMode.Floating;
                maxSpeed = airSpeed;
                maxForce = maximumAirForce;
                supportEntity = null;
            }
            if (!isTryingToMove)
                maxSpeed = 0;

            maxForce *= dt;



            //Compute the jacobians.  This is basically a PointOnLineJoint with motorized degrees of freedom.
            Vector3 downDirection = character.Body.OrientationMatrix.Down;

            if (MovementMode != MovementMode.Floating)
            {
                //Compute the linear jacobians first.
                if (isTryingToMove)
                {
                    Vector3 velocityDirection;
                    Vector3 offVelocityDirection;
                    //Project the movement direction onto the support plane defined by the support normal.
                    //This projection is NOT along the support normal to the plane; that would cause the character to veer off course when moving on slopes.
                    //Instead, project along the sweep direction to the plane.
                    //For a 6DOF character controller, the lineStart would be different; it must be perpendicular to the local up.
                    Vector3 lineStart = new Vector3(movementDirection.X, 0, movementDirection.Y);
                    Vector3 lineEnd;
                    Vector3.Add(ref lineStart, ref downDirection, out lineEnd);
                    Plane plane = new Plane(character.SupportFinder.HasTraction ? supportData.Normal : supportData.Normal, 0);
                    float t;
                    //This method can return false when the line is parallel to the plane, but previous tests and the slope limit guarantee that it won't happen.
                    Toolbox.GetLinePlaneIntersection(ref lineStart, ref lineEnd, ref plane, out t, out velocityDirection);

                    //The origin->intersection line direction defines the horizontal velocity direction in 3d space.
                    velocityDirection.Normalize();


                    //The normal and velocity direction are perpendicular and normal, so the off velocity direction doesn't need to be normalized.
                    Vector3.Cross(ref velocityDirection, ref supportData.Normal, out offVelocityDirection);

                    linearJacobianA1 = velocityDirection;
                    linearJacobianA2 = offVelocityDirection;
                    linearJacobianB1 = -velocityDirection;
                    linearJacobianB2 = -offVelocityDirection;

                }
                else
                {
                    Vector3 previousLinearJacobianA1 = linearJacobianA1;
                    Vector3 previousLinearJacobianA2 = linearJacobianA2;
                    //If the character isn't trying to move, then the velocity directions are not well defined.
                    //Instead, pick two arbitrary vectors on the support plane.
                    //First guess will be based on the previous jacobian.
                    //Project the old linear jacobian onto the support normal plane.
                    float dot;
                    Vector3.Dot(ref linearJacobianA1, ref supportData.Normal, out dot);
                    Vector3 toRemove;
                    Vector3.Multiply(ref supportData.Normal, dot, out toRemove);
                    Vector3.Subtract(ref linearJacobianA1, ref toRemove, out linearJacobianA1);

                    //Vector3.Cross(ref linearJacobianA2, ref supportData.Normal, out linearJacobianA1);
                    float length = linearJacobianA1.LengthSquared();
                    if (length < Toolbox.Epsilon)
                    {
                        //First guess failed.  Try the right vector.
                        Vector3.Cross(ref Toolbox.RightVector, ref supportData.Normal, out linearJacobianA1);
                        length = linearJacobianA1.LengthSquared();
                        if (length < Toolbox.Epsilon)
                        {
                            //Okay that failed too! try the forward vector.
                            Vector3.Cross(ref Toolbox.ForwardVector, ref supportData.Normal, out linearJacobianA1);
                            length = linearJacobianA1.LengthSquared();
                            //Unless something really weird is happening, we do not need to test any more axes.
                        }

                    }
                    Vector3.Divide(ref linearJacobianA1, (float)Math.Sqrt(length), out linearJacobianA1);
                    //Pick another perpendicular vector.  Don't need to normalize it since the normal and A1 are already normalized and perpendicular.
                    Vector3.Cross(ref linearJacobianA1, ref supportData.Normal, out linearJacobianA2);

                    //B's linear jacobians are just -A's.
                    linearJacobianB1 = -linearJacobianA1;
                    linearJacobianB2 = -linearJacobianA2;

                }

                if (supportEntity != null)
                {
                    //Compute the angular jacobians.
                    Vector3 supportToContact = supportData.Position - supportEntity.Position;
                    //Since we treat the character to have infinite inertia, we're only concerned with the support's angular jacobians.
                    //Note the order of the cross product- it is reversed to negate the result.
                    Vector3.Cross(ref linearJacobianA1, ref supportToContact, out angularJacobianB1);
                    Vector3.Cross(ref linearJacobianA2, ref supportToContact, out angularJacobianB2);

                }
                else
                {
                    //If we're not standing on an entity, there are no angular jacobians.
                    angularJacobianB1 = new Vector3();
                    angularJacobianB2 = new Vector3();
                }
            }
            else
            {
                Vector3 previousLinearJacobianA1 = linearJacobianA1;
                Vector3 previousLinearJacobianA2 = linearJacobianA2;

                //If the character is floating, then the jacobians are simply the movement direction.
                //Note that in a 6DOF character, this will change- but it will still be trivial.
                //In that case, the movement direction will be a 3d vector, and the A2 jacobian will just be
                //linearJacobianA1 x downDirection.
                linearJacobianA1 = new Vector3(movementDirection.X, 0, movementDirection.Y);
                linearJacobianA2 = new Vector3(movementDirection.Y, 0, -movementDirection.X);


            }


            //Compute the target velocity (in constraint space) for this frame.  The hard work has already been done.
            targetVelocity.X = maxSpeed;
            targetVelocity.Y = 0;

            //Compute the effective mass matrix.
            if (supportEntity != null && supportEntity.IsDynamic)
            {
                float m11, m22, m1221 = 0;
                float inverseMass;
                Vector3 intermediate;

                inverseMass = character.Body.InverseMass;
                m11 = inverseMass;
                m22 = inverseMass;


                //Scale the inertia and mass of the support.  This will make the solver view the object as 'heavier' with respect to horizontal motion.
                Matrix3X3 inertiaInverse = supportEntity.InertiaTensorInverse;
                Matrix3X3.Multiply(ref inertiaInverse, supportForceFactor, out inertiaInverse);
                float extra;
                inverseMass = supportForceFactor * supportEntity.InverseMass;
                Matrix3X3.Transform(ref angularJacobianB1, ref inertiaInverse, out intermediate);
                Vector3.Dot(ref intermediate, ref angularJacobianB1, out extra);
                m11 += inverseMass + extra;
                Vector3.Dot(ref intermediate, ref angularJacobianB2, out extra);
                m1221 += extra;
                Matrix3X3.Transform(ref angularJacobianB2, ref inertiaInverse, out intermediate);
                Vector3.Dot(ref intermediate, ref angularJacobianB2, out extra);
                m22 += inverseMass + extra;


                massMatrix.M11 = m11;
                massMatrix.M12 = m1221;
                massMatrix.M21 = m1221;
                massMatrix.M22 = m22;
                Matrix2X2.Invert(ref massMatrix, out massMatrix);


            }
            else
            {
                //If we're not standing on a dynamic entity, then the mass matrix is defined entirely by the character.
                Matrix2X2.CreateScale(character.Body.Mass, out massMatrix);
            }

            //If we're trying to stand still on an object that's moving, use a position correction term to keep the character
            //from drifting due to accelerations. 
            //First thing to do is to check to see if we're moving into a traction/trying to stand still state from a 
            //non-traction || trying to move state.  Either that, or we've switched supports and need to update the offset.
            if (supportEntity != null && ((wasTryingToMove && !isTryingToMove) || (!hadTraction && supportData.HasTraction) || supportEntity != previousSupportEntity))
            {
                //We're transitioning into a new 'use position correction' state.
                //Force a recomputation of the local offset.
                //The time since transition is used as a flag.
                timeSinceTransition = 0;
            }

            //The state is now up to date.  Compute an error and velocity bias, if needed.
            if (!isTryingToMove && supportData.HasTraction && supportEntity != null)
            {
                if (timeSinceTransition >= 0 && timeSinceTransition < tractionDecelerationTime)
                    timeSinceTransition += dt;
                if (timeSinceTransition >= tractionDecelerationTime)
                {
                    Vector3.Multiply(ref downDirection, character.Body.Height * .5f, out positionLocalOffset);
                    positionLocalOffset = (positionLocalOffset + character.Body.Position) - supportEntity.Position;
                    positionLocalOffset = Matrix3X3.TransformTranspose(positionLocalOffset, supportEntity.OrientationMatrix);
                    timeSinceTransition = -1; //Negative 1 means that the offset has been computed.
                }
                if (timeSinceTransition < 0)
                {
                    Vector3 targetPosition;
                    Vector3.Multiply(ref downDirection, character.Body.Height * .5f, out targetPosition);
                    targetPosition += character.Body.Position;
                    Vector3 worldSupportLocation = Matrix3X3.Transform(positionLocalOffset, supportEntity.OrientationMatrix) + supportEntity.Position;
                    Vector3 error;
                    Vector3.Subtract(ref targetPosition, ref worldSupportLocation, out error);
                    //If the error is too large, then recompute the offset.  We don't want the character rubber banding around.
                    if (error.LengthSquared() > .15f * .15f)
                    {
                        Vector3.Multiply(ref downDirection, character.Body.Height * .5f, out positionLocalOffset);
                        positionLocalOffset = (positionLocalOffset + character.Body.Position) - supportEntity.Position;
                        positionLocalOffset = Matrix3X3.TransformTranspose(positionLocalOffset, supportEntity.OrientationMatrix);
                        positionCorrectionBias = new Vector2();
                    }
                    else
                    {
                        //The error in world space is now available.  We can't use this error to directly create a velocity bias, though.
                        //It needs to be transformed into constraint space where the constraint operates.
                        //Use the jacobians!
                        Vector3.Dot(ref error, ref linearJacobianA1, out positionCorrectionBias.X);
                        Vector3.Dot(ref error, ref linearJacobianA2, out positionCorrectionBias.Y);
                        //Scale the error so that a portion of the error is resolved each frame.
                        Vector2.Multiply(ref positionCorrectionBias, .2f / dt, out positionCorrectionBias);
                    }
                }
            }
            else
            {
                timeSinceTransition = 0;
                positionCorrectionBias = new Vector2();
            }

            wasTryingToMove = isTryingToMove;
            hadTraction = supportData.HasTraction;
            previousSupportEntity = supportEntity;

        }


        /// <summary>
        /// Performs any per-frame initialization needed by the constraint that must be done with exclusive access
        /// to the connected objects.
        /// </summary>
        public override void ExclusiveUpdate()
        {
            //Warm start the constraint using the previous impulses and the new jacobians!
#if !WINDOWS
            Vector3 impulse = new Vector3();
            Vector3 torque= new Vector3();
#else
            Vector3 impulse;
            Vector3 torque;
#endif
            float x = accumulatedImpulse.X;
            float y = accumulatedImpulse.Y;
            impulse.X = linearJacobianA1.X * x + linearJacobianA2.X * y;
            impulse.Y = linearJacobianA1.Y * x + linearJacobianA2.Y * y;
            impulse.Z = linearJacobianA1.Z * x + linearJacobianA2.Z * y;

            character.Body.ApplyLinearImpulse(ref impulse);

            if (supportEntity != null && supportEntity.IsDynamic)
            {
                Vector3.Multiply(ref impulse, -supportForceFactor, out impulse);

                x *= supportForceFactor;
                y *= supportForceFactor;
                torque.X = x * angularJacobianB1.X + y * angularJacobianB2.X;
                torque.Y = x * angularJacobianB1.Y + y * angularJacobianB2.Y;
                torque.Z = x * angularJacobianB1.Z + y * angularJacobianB2.Z;


                supportEntity.ApplyLinearImpulse(ref impulse);
                supportEntity.ApplyAngularImpulse(ref torque);
            }
        }

        /// <summary>
        /// Computes a solution to the constraint.
        /// </summary>
        /// <returns>Impulse magnitude computed by the iteration.</returns>
        public override float SolveIteration()
        {
            //The relative velocity's x component is in the movement direction.
            //y is the perpendicular direction.
#if !WINDOWS
            Vector2 relativeVelocity = new Vector2();
#else
            Vector2 relativeVelocity;
#endif

            Vector3 bodyVelocity = character.Body.LinearVelocity;
            Vector3.Dot(ref linearJacobianA1, ref bodyVelocity, out relativeVelocity.X);
            Vector3.Dot(ref linearJacobianA2, ref bodyVelocity, out relativeVelocity.Y);

            float x, y;
            if (supportEntity != null)
            {
                Vector3 supportLinearVelocity = supportEntity.LinearVelocity;
                Vector3 supportAngularVelocity = supportEntity.AngularVelocity;


                Vector3.Dot(ref linearJacobianB1, ref supportLinearVelocity, out x);
                Vector3.Dot(ref linearJacobianB2, ref supportLinearVelocity, out y);
                relativeVelocity.X += x;
                relativeVelocity.Y += y;
                Vector3.Dot(ref angularJacobianB1, ref supportAngularVelocity, out x);
                Vector3.Dot(ref angularJacobianB2, ref supportAngularVelocity, out y);
                relativeVelocity.X += x;
                relativeVelocity.Y += y;

            }

            Vector2.Add(ref relativeVelocity, ref positionCorrectionBias, out relativeVelocity);


            //Create the full velocity change, and convert it to an impulse in constraint space.
            Vector2 lambda;
            Vector2.Subtract(ref targetVelocity, ref relativeVelocity, out lambda);
            Matrix2X2.Transform(ref lambda, ref massMatrix, out lambda);

            //Add and clamp the impulse.

            Vector2 previousAccumulatedImpulse = accumulatedImpulse;
            if (MovementMode == MovementMode.Floating)
            {
                //If it's floating, clamping rules are different.
                //The constraint is not permitted to slow down the character; only speed it up.
                //This offers a hole for an exploit; by jumping and curving just right,
                //the character can accelerate beyond its maximum speed.  A bit like an HL2 speed run.
                accumulatedImpulse.X = MathHelper.Clamp(accumulatedImpulse.X + lambda.X, 0, maxForce);
                accumulatedImpulse.Y = 0;
            }
            else
            {
                Vector2.Add(ref lambda, ref accumulatedImpulse, out accumulatedImpulse);
                float length = accumulatedImpulse.LengthSquared();
                if (length > maxForce * maxForce)
                {
                    Vector2.Multiply(ref accumulatedImpulse, maxForce / (float)Math.Sqrt(length), out accumulatedImpulse);
                }
            }
            Vector2.Subtract(ref accumulatedImpulse, ref previousAccumulatedImpulse, out lambda);


            //Use the jacobians to put the impulse into world space.

#if !WINDOWS
            Vector3 impulse = new Vector3();
            Vector3 torque= new Vector3();
#else
            Vector3 impulse;
            Vector3 torque;
#endif
            x = lambda.X;
            y = lambda.Y;
            impulse.X = linearJacobianA1.X * x + linearJacobianA2.X * y;
            impulse.Y = linearJacobianA1.Y * x + linearJacobianA2.Y * y;
            impulse.Z = linearJacobianA1.Z * x + linearJacobianA2.Z * y;

            character.Body.ApplyLinearImpulse(ref impulse);

            if (supportEntity != null && supportEntity.IsDynamic)
            {
                Vector3.Multiply(ref impulse, -supportForceFactor, out impulse);

                x *= supportForceFactor;
                y *= supportForceFactor;
                torque.X = x * angularJacobianB1.X + y * angularJacobianB2.X;
                torque.Y = x * angularJacobianB1.Y + y * angularJacobianB2.Y;
                torque.Z = x * angularJacobianB1.Z + y * angularJacobianB2.Z;

                supportEntity.ApplyLinearImpulse(ref impulse);
                supportEntity.ApplyAngularImpulse(ref torque);
            }

            return (Math.Abs(lambda.X) + Math.Abs(lambda.Y));


        }


        /// <summary>
        /// Gets the current velocity between the character and its support in constraint space.
        /// The X component corresponds to velocity along the movement direction.
        /// The Y component corresponds to velocity perpendicular to the movement direction and support normal.
        /// </summary>
        public Vector2 RelativeVelocity
        {
            get
            {
#if !WINDOWS
                Vector2 relativeVelocity = new Vector2();
#else
                Vector2 relativeVelocity;
#endif

                Vector3 bodyVelocity = character.Body.LinearVelocity;
                Vector3.Dot(ref linearJacobianA1, ref bodyVelocity, out relativeVelocity.X);
                Vector3.Dot(ref linearJacobianA2, ref bodyVelocity, out relativeVelocity.Y);

                float x, y;
                if (supportEntity != null)
                {
                    Vector3 supportLinearVelocity = supportEntity.LinearVelocity;
                    Vector3 supportAngularVelocity = supportEntity.AngularVelocity;


                    Vector3.Dot(ref linearJacobianB1, ref supportLinearVelocity, out x);
                    Vector3.Dot(ref linearJacobianB2, ref supportLinearVelocity, out y);
                    relativeVelocity.X += x;
                    relativeVelocity.Y += y;
                    Vector3.Dot(ref angularJacobianB1, ref supportAngularVelocity, out x);
                    Vector3.Dot(ref angularJacobianB2, ref supportAngularVelocity, out y);
                    relativeVelocity.X += x;
                    relativeVelocity.Y += y;

                }
                return relativeVelocity;
            }
        }

        /// <summary>
        /// Gets the current velocity between the character and its support.
        /// </summary>
        public Vector3 RelativeWorldVelocity
        {
            get
            {
                Vector3 bodyVelocity = character.Body.LinearVelocity;
                if (supportEntity != null)
                    return bodyVelocity - Toolbox.GetVelocityOfPoint(supportData.Position, supportEntity);
                else
                    return bodyVelocity;
            }
        }



    }
    public enum MovementMode
    {
        Traction,
        Sliding,
        Floating
    }
}
