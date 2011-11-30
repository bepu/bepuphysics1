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
using BEPUphysics.Settings;

namespace BEPUphysicsDemos.AlternateMovement.SphereCharacter
{
    /// <summary>
    /// Keeps a character glued to the ground, if possible.
    /// </summary>
    public class VerticalMotionConstraint : EntitySolverUpdateable
    {
        SphereCharacterController character;


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


        float maximumGlueForce = 5000f;
        /// <summary>
        /// Gets or sets the maximum force that the constraint will apply in attempting to keep the character stuck to the ground.
        /// </summary>
        public float MaximumGlueForce
        {
            get
            {
                return maximumGlueForce;
            }
            set
            {
                if (maximumGlueForce < 0)
                    throw new Exception("Value must be nonnegative.");
                maximumGlueForce = value;
            }
        }
        float maximumForce;

        float supportForceFactor = 1;
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

        /// <summary>
        /// Gets the effective mass felt by the constraint.
        /// </summary>
        public float EffectiveMass
        {
            get
            {
                return effectiveMass;
            }
        }
        float effectiveMass;
        Entity supportEntity;
        Vector3 linearJacobianA;
        Vector3 linearJacobianB;
        Vector3 angularJacobianB;

        float accumulatedImpulse;
        float permittedVelocity;

        /// <summary>
        /// Constructs a new vertical motion constraint.
        /// </summary>
        /// <param name="characterController">Character governed by the constraint.</param>
        public VerticalMotionConstraint(SphereCharacterController characterController)
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

        /// <summary>
        /// Updates the activity state of the constraint.
        /// Called automatically by the solver.
        /// </summary>
        public override void UpdateSolverActivity()
        {
            if (supportData.HasTraction)
                base.UpdateSolverActivity();
            else
                isActiveInSolver = false;
        }

        /// <summary>
        /// Performs any per-frame computation needed by the constraint.
        /// </summary>
        /// <param name="dt">Time step duration.</param>
        public override void Update(float dt)
        {
            //Collect references, pick the mode, and configure the coefficients to be used by the solver.
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

            }
            else
            {
                supportEntity = null;
            }

            maximumForce = maximumGlueForce * dt;

            //If we don't allow the character to get out of the ground, it could apply some significant forces to a dynamic support object.
            //Technically, there exists a better estimate of the necessary speed, but choosing the maximum position correction speed is a nice catch-all.
            //If you change that correction speed, watch out!!! It could significantly change the way the character behaves when trying to glue to surfaces.
            if (supportData.Depth > 0)
                permittedVelocity = CollisionResponseSettings.MaximumPositionCorrectionSpeed;
            else
                permittedVelocity = 0;

            //Compute the jacobians and effective mass matrix.  This constraint works along a single degree of freedom, so the mass matrix boils down to a scalar.

            linearJacobianA = supportData.Normal;
            Vector3.Negate(ref linearJacobianA, out linearJacobianB);
            effectiveMass = character.Body.InverseMass;
            if (supportEntity != null)
            {
                Vector3 offsetB = supportData.Position - supportEntity.Position;
                Vector3.Cross(ref offsetB, ref linearJacobianB, out angularJacobianB);
                if (supportEntity.IsDynamic)
                {
                    //Only dynamic entities can actually contribute anything to the effective mass.
                    //Kinematic entities have infinite mass and inertia, so this would all zero out.
                    Matrix3X3 inertiaInverse = supportEntity.InertiaTensorInverse;
                    Vector3 angularComponentB;
                    Matrix3X3.Transform(ref angularJacobianB, ref inertiaInverse, out angularComponentB);
                    float effectiveMassContribution;
                    Vector3.Dot(ref angularComponentB, ref angularJacobianB, out effectiveMassContribution);

                    effectiveMass += supportForceFactor * (effectiveMassContribution + supportEntity.InverseMass);
                }
            }
            effectiveMass = 1 / effectiveMass;
            //So much nicer and shorter than the horizontal constraint!

        }

        /// <summary>
        /// Performs any per-frame computations needed by the constraint that require exclusive access to the involved entities.
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
            Vector3.Multiply(ref linearJacobianA, accumulatedImpulse, out impulse);

            character.Body.ApplyLinearImpulse(ref impulse);

            if (supportEntity != null && supportEntity.IsDynamic)
            {
                Vector3.Multiply(ref impulse, -supportForceFactor, out impulse);
                Vector3.Multiply(ref angularJacobianB, accumulatedImpulse * supportForceFactor, out torque);

                supportEntity.ApplyLinearImpulse(ref impulse);
                supportEntity.ApplyAngularImpulse(ref torque);
            }
        }

        /// <summary>
        /// Computes a solution to the constraint.
        /// </summary>
        /// <returns>Magnitude of the applied impulse.</returns>
        public override float SolveIteration()
        {
            //The relative velocity's x component is in the movement direction.
            //y is the perpendicular direction.

            float relativeVelocity = RelativeVelocity + permittedVelocity;


            //Create the full velocity change, and convert it to an impulse in constraint space.
            float lambda = -relativeVelocity * effectiveMass;

            //Add and clamp the impulse.
            float previousAccumulatedImpulse = accumulatedImpulse;
            accumulatedImpulse = MathHelper.Clamp(accumulatedImpulse + lambda, 0, maximumForce);
            lambda = accumulatedImpulse - previousAccumulatedImpulse;
            //Use the jacobians to put the impulse into world space.

#if !WINDOWS
            Vector3 impulse = new Vector3();
            Vector3 torque= new Vector3();
#else
            Vector3 impulse;
            Vector3 torque;
#endif
            Vector3.Multiply(ref linearJacobianA, lambda, out impulse);

            character.Body.ApplyLinearImpulse(ref impulse);

            if (supportEntity != null && supportEntity.IsDynamic)
            {
                Vector3.Multiply(ref impulse, -supportForceFactor, out impulse);

                Vector3.Multiply(ref angularJacobianB, lambda * supportForceFactor, out torque);

                supportEntity.ApplyLinearImpulse(ref impulse);
                supportEntity.ApplyAngularImpulse(ref torque);
            }
            return Math.Abs(lambda);


        }

        /// <summary>
        /// Gets the relative velocity between the character and its support along the support normal.
        /// </summary>
        public float RelativeVelocity
        {
            get
            {
                float relativeVelocity;

                Vector3 bodyVelocity = character.Body.LinearVelocity;
                Vector3.Dot(ref linearJacobianA, ref bodyVelocity, out relativeVelocity);

                if (supportEntity != null)
                {
                    Vector3 supportLinearVelocity = supportEntity.LinearVelocity;
                    Vector3 supportAngularVelocity = supportEntity.AngularVelocity;


                    float supportVelocity;
                    Vector3.Dot(ref linearJacobianB, ref supportLinearVelocity, out supportVelocity);
                    relativeVelocity += supportVelocity;
                    Vector3.Dot(ref angularJacobianB, ref supportAngularVelocity, out supportVelocity);
                    relativeVelocity += supportVelocity;

                }
                return relativeVelocity;
            }
        }


    }
}
