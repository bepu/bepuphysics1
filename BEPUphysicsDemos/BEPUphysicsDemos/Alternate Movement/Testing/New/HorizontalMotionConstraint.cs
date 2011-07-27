using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.Constraints;
using BEPUphysics.DataStructures;
using BEPUphysics.Entities;
using Microsoft.Xna.Framework;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.MathExtensions;
using BEPUphysics;
using System.Diagnostics;

namespace BEPUphysicsDemos.AlternateMovement.Testing.New
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
                movementDirection = value;
                if (value.LengthSquared() > 0)
                    character.Body.IsActive = true;
            }
        }


        public float Speed = 8f;
        public float SlidingSpeed = 6;
        public float AirSpeed = 4;
        public float Acceleration = 50;
        public float SlidingAcceleration = 5;
        public float AirAcceleration = 15;
        public float Deceleration = 80;
        public float SlidingDeceleration = 1;
        public float MaximumForce = 1000;
        public float MaximumSlidingForce = 50;
        public float MaximumAirForce = 150;

        float supportForceFactor = .3f;
        /// <summary>
        /// Gets or sets the scaling factor of forces applied to the supporting object if it is a dynamic entity.
        /// Low values (below 1) reduce the amount of motion imparted to the support object; it acts 'heavier' as far as horizontal motion is concerned.
        /// High values (above 1) increase the force applied to support objects, making them appear lighter.
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


        float maxSpeed;
        float acceleration;
        float deceleration;
        float maxForce;


        Matrix2X2 massMatrix;
        Entity supportEntity;
        Vector3 linearJacobianA1;
        Vector3 linearJacobianA2;
        Vector3 linearJacobianB1;
        Vector3 linearJacobianB2;
        Vector3 angularJacobianB1;
        Vector3 angularJacobianB2;
        public MovementMode MovementMode;

        Vector2 accumulatedImpulse;
        Vector2 targetVelocity;

        public HorizontalMotionConstraint(CharacterController characterController)
        {
            this.character = characterController;
        }


        protected override void CollectInvolvedEntities(RawList<Entity> outputInvolvedEntities)
        {
            var entityCollidable = supportData.SupportObject as EntityCollidable;
            if (entityCollidable != null)
                outputInvolvedEntities.Add(entityCollidable.Entity);
            outputInvolvedEntities.Add(character.Body);

        }


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
                    maxSpeed = Speed;
                    acceleration = Acceleration;
                    deceleration = Deceleration;
                    maxForce = MaximumForce;
                }
                else
                {
                    MovementMode = MovementMode.Sliding;
                    maxSpeed = SlidingSpeed;
                    acceleration = SlidingAcceleration;
                    deceleration = SlidingDeceleration;
                    maxForce = MaximumSlidingForce;
                }
            }
            else
            {
                MovementMode = MovementMode.Floating;
                maxSpeed = AirSpeed;
                acceleration = AirAcceleration;
                deceleration = 0;
                maxForce = MaximumAirForce;
                supportEntity = null;
            }
            if (!isTryingToMove)
                maxSpeed = 0;

            acceleration *= dt;
            deceleration *= dt;
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
                    Vector3.Cross(ref supportData.Normal, ref velocityDirection, out offVelocityDirection);

                    linearJacobianA1 = velocityDirection;
                    linearJacobianA2 = offVelocityDirection;
                    linearJacobianB1 = -velocityDirection;
                    linearJacobianB2 = -offVelocityDirection;


                }
                else
                {
                    //If the character isn't trying to move, then the velocity directions are not well defined.
                    //Instead, pick two arbitrary vectors on the support plane.
                    //First guess will be based on the previous jacobian.
                    Vector3.Cross(ref linearJacobianA2, ref supportData.Normal, out linearJacobianA1);
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
                //If the character is floating, then the jacobians are simply the movement direction.
                //Note that in a 6DOF character, this will change- but it will still be trivial.
                //In that case, the movement direction will be a 3d vector, and the A2 jacobian will just be
                //linearJacobianA1 x downDirection.
                linearJacobianA1 = new Vector3(movementDirection.X, 0, movementDirection.Y);
                linearJacobianA2 = new Vector3(movementDirection.Y, 0, -movementDirection.X);
            }



            //Compute the target velocity (in constraint space) for this frame.
            //The goals are:
            // relativeVelocity.X should be accelerated to maxSpeed using acceleration.
            // relativeVelocity.X should be decelerated if it is < 0 or > maxSpeed.
            // relativeVelocity.Y should be decelerated.
            //Those rules cover every case, because the coefficients for acceleration/deceleration/maxSpeed
            //have all been configured according to what state the character is in.
            Vector2 relativeVelocity = RelativeVelocity;
            //Compute the contribution from deceleration first.
            //Target the final goal first, then clamp it.
            if (relativeVelocity.X < 0)
                targetVelocity.X = 0;
            else if (relativeVelocity.X > maxSpeed)
                targetVelocity.X = maxSpeed;
            else
                targetVelocity.X = relativeVelocity.X;
            if (relativeVelocity.Y > 0)
                targetVelocity.Y = 0;
            else
                targetVelocity.Y = 0;
            //Clamp it!
            float velocityChangeMagnitude;
            Vector2 change;
            Vector2.Subtract(ref targetVelocity, ref relativeVelocity, out change);
            velocityChangeMagnitude = change.Length();
            if (velocityChangeMagnitude > Toolbox.Epsilon)
            {
                float newLength = Math.Min(velocityChangeMagnitude, deceleration);
                Vector2.Multiply(ref change, newLength / velocityChangeMagnitude, out change);
                Vector2.Add(ref relativeVelocity, ref change, out targetVelocity);
            }
            //Now add in the acceleration component along the X axis.
            float newX = Math.Min(targetVelocity.X + acceleration, maxSpeed);
            if (newX > targetVelocity.X)
                targetVelocity.X = newX;

            //Compute the effective mass matrix.
            if (supportEntity != null)
            {
                float m11, m22, m1221 = 0;
                float inverseMass;
                Vector3 intermediate;

                inverseMass = 1 / character.Body.Mass;
                m11 = inverseMass;
                m22 = inverseMass;

                if (supportEntity.IsDynamic)
                {

                    //Scale the inertia and mass of the support.  This will make the solver view the object as 'heavier' with respect to horizontal motion.
                    Matrix3X3 inertiaInverse = supportEntity.InertiaTensorInverse;
                    Matrix3X3.Multiply(ref inertiaInverse, supportForceFactor, out inertiaInverse);
                    float extra;
                    inverseMass = supportForceFactor / (supportEntity.Mass);
                    Matrix3X3.Transform(ref angularJacobianB1, ref inertiaInverse, out intermediate);
                    Vector3.Dot(ref intermediate, ref angularJacobianB1, out extra);
                    m11 += inverseMass + extra;
                    Vector3.Dot(ref intermediate, ref angularJacobianB2, out extra);
                    m1221 += extra;
                    Matrix3X3.Transform(ref angularJacobianB2, ref inertiaInverse, out intermediate);
                    Vector3.Dot(ref intermediate, ref angularJacobianB2, out extra);
                    m22 += inverseMass + extra;
                }

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


        }

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


        public override float SolveIteration()
        {
            //The relative velocity's x component is in the movement direction.
            //y is the perpendicular direction.
            Vector2 relativeVelocity;

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


            //Create the full velocity change, and convert it to an impulse in constraint space.
            Vector2 lambda;
            Vector2.Subtract(ref targetVelocity, ref relativeVelocity, out lambda);
            Matrix2X2.Transform(ref lambda, ref massMatrix, out lambda);

            //Add and clamp the impulse.
            Vector2 previousAccumulatedImpulse = accumulatedImpulse;
            Vector2.Add(ref lambda, ref accumulatedImpulse, out accumulatedImpulse);
            float length = accumulatedImpulse.LengthSquared();
            if (length > maxForce * maxForce)
            {
                Vector2.Multiply(ref accumulatedImpulse, maxForce / (float)Math.Sqrt(length), out accumulatedImpulse);
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


        Vector2 RelativeVelocity
        {
            get
            {
                Vector2 relativeVelocity;

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


    }
    public enum MovementMode
    {
        Traction,
        Sliding,
        Floating
    }
}
