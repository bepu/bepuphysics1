using BEPUphysics.Entities;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionTests;
using BEPUphysics.Settings;
using System;

namespace BEPUphysics.Constraints.Collision
{
    /// <summary>
    /// Computes the forces necessary to keep two entities from going through each other at a contact point.
    /// </summary>
    public class ContactPenetrationConstraint : EntitySolverUpdateable
    {
        internal Contact contact;

        ///<summary>
        /// Gets the contact associated with this penetration constraint.
        ///</summary>
        public Contact Contact { get { return contact; } }
        internal float accumulatedImpulse;
        //float linearBX, linearBY, linearBZ;
        internal float angularAX, angularAY, angularAZ;
        internal float angularBX, angularBY, angularBZ;


        private float bias;
        private float linearAX, linearAY, linearAZ;
        private Entity entityA, entityB;
        private bool entityADynamic, entityBDynamic;
        //Inverse effective mass matrix
        internal float velocityToImpulse;
        private ContactManifoldConstraint contactManifoldConstraint;

        internal Vector3 ra, rb;

        ///<summary>
        /// Constructs a new penetration constraint.
        ///</summary>
        public ContactPenetrationConstraint()
        {
            isActive = false;
        }


        ///<summary>
        /// Configures the penetration constraint.
        ///</summary>
        ///<param name="contactManifoldConstraint">Owning manifold constraint.</param>
        ///<param name="contact">Contact associated with the penetration constraint.</param>
        public void Setup(ContactManifoldConstraint contactManifoldConstraint, Contact contact)
        {
            this.contactManifoldConstraint = contactManifoldConstraint;
            this.contact = contact;
            isActive = true;
        }

        ///<summary>
        /// Cleans up the constraint.
        ///</summary>
        public void CleanUp()
        {
            accumulatedImpulse = 0;
            contactManifoldConstraint = null;
            contact = null;
            entityA = null;
            entityB = null;
            isActive = false;
        }

        /// <summary>
        /// Gets the total normal force applied by this penetration constraint to maintain the separation of the involved entities.
        /// </summary>
        public float NormalForce
        {
            get { return accumulatedImpulse; }
        }

        ///<summary>
        /// Gets the relative velocity between the associated entities at the contact point along the contact normal.
        ///</summary>
        public float RelativeVelocity
        {
            get
            {
                float lambda = 0;
                if (entityA != null)
                {
                    lambda = entityA.linearVelocity.X * linearAX + entityA.linearVelocity.Y * linearAY + entityA.linearVelocity.Z * linearAZ +
                             entityA.angularVelocity.X * angularAX + entityA.angularVelocity.Y * angularAY + entityA.angularVelocity.Z * angularAZ;
                }
                if (entityB != null)
                {
                    lambda += -entityB.linearVelocity.X * linearAX - entityB.linearVelocity.Y * linearAY - entityB.linearVelocity.Z * linearAZ +
                              entityB.angularVelocity.X * angularBX + entityB.angularVelocity.Y * angularBY + entityB.angularVelocity.Z * angularBZ;
                }
                return lambda;
            }
        }



        /// <summary>
        /// Computes and applies an impulse to keep the colliders from penetrating.
        /// </summary>
        /// <returns>Impulse applied.</returns>
        public override float SolveIteration()
        {

            //Compute relative velocity
            float lambda = (RelativeVelocity - bias) * velocityToImpulse; //convert to impulse


            //Clamp accumulated impulse
            float previousAccumulatedImpulse = accumulatedImpulse;
            accumulatedImpulse = MathHelper.Max(0, accumulatedImpulse + lambda);
            lambda = accumulatedImpulse - previousAccumulatedImpulse;

            //Apply the impulse
#if !WINDOWS
            Vector3 linear = new Vector3();
            Vector3 angular = new Vector3();
#else
            Vector3 linear, angular;
#endif
            linear.X = lambda * linearAX;
            linear.Y = lambda * linearAY;
            linear.Z = lambda * linearAZ;
            if (entityADynamic)
            {
                angular.X = lambda * angularAX;
                angular.Y = lambda * angularAY;
                angular.Z = lambda * angularAZ;
                entityA.ApplyLinearImpulse(ref linear);
                entityA.ApplyAngularImpulse(ref angular);
            }
            if (entityBDynamic)
            {
                linear.X = -linear.X;
                linear.Y = -linear.Y;
                linear.Z = -linear.Z;
                angular.X = lambda * angularBX;
                angular.Y = lambda * angularBY;
                angular.Z = lambda * angularBZ;
                entityB.ApplyLinearImpulse(ref linear);
                entityB.ApplyAngularImpulse(ref angular);
            }

            return Math.Abs(lambda);
        }

        ///<summary>
        /// Performs the frame's configuration step.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        public override void Update(float dt)
        {
            entityA = contactManifoldConstraint.EntityA;
            entityB = contactManifoldConstraint.EntityB;
            entityADynamic = entityA != null && entityA.isDynamic;
            entityBDynamic = entityB != null && entityB.isDynamic;

            //Set up the jacobians.
            linearAX = -contact.Normal.X;
            linearAY = -contact.Normal.Y;
            linearAZ = -contact.Normal.Z;
            //linearBX = -linearAX;
            //linearBY = -linearAY;
            //linearBZ = -linearAZ;

            //angular A = Ra x N
            if (entityA != null)
            {
                Vector3.Subtract(ref contact.Position, ref entityA.position, out ra);
                angularAX = (ra.Y * linearAZ) - (ra.Z * linearAY);
                angularAY = (ra.Z * linearAX) - (ra.X * linearAZ);
                angularAZ = (ra.X * linearAY) - (ra.Y * linearAX);
            }

            //Angular B = N x Rb
            if (entityB != null)
            {
                Vector3.Subtract(ref contact.Position, ref entityB.position, out rb);
                angularBX = (linearAY * rb.Z) - (linearAZ * rb.Y);
                angularBY = (linearAZ * rb.X) - (linearAX * rb.Z);
                angularBZ = (linearAX * rb.Y) - (linearAY * rb.X);
            }

            //Compute inverse effective mass matrix
            float entryA, entryB;

            //these are the transformed coordinates
            float tX, tY, tZ;
            if (entityADynamic)
            {
                tX = angularAX * entityA.inertiaTensorInverse.M11 + angularAY * entityA.inertiaTensorInverse.M21 + angularAZ * entityA.inertiaTensorInverse.M31;
                tY = angularAX * entityA.inertiaTensorInverse.M12 + angularAY * entityA.inertiaTensorInverse.M22 + angularAZ * entityA.inertiaTensorInverse.M32;
                tZ = angularAX * entityA.inertiaTensorInverse.M13 + angularAY * entityA.inertiaTensorInverse.M23 + angularAZ * entityA.inertiaTensorInverse.M33;
                entryA = tX * angularAX + tY * angularAY + tZ * angularAZ + 1 / entityA.mass;
            }
            else
                entryA = 0;

            if (entityBDynamic)
            {
                tX = angularBX * entityB.inertiaTensorInverse.M11 + angularBY * entityB.inertiaTensorInverse.M21 + angularBZ * entityB.inertiaTensorInverse.M31;
                tY = angularBX * entityB.inertiaTensorInverse.M12 + angularBY * entityB.inertiaTensorInverse.M22 + angularBZ * entityB.inertiaTensorInverse.M32;
                tZ = angularBX * entityB.inertiaTensorInverse.M13 + angularBY * entityB.inertiaTensorInverse.M23 + angularBZ * entityB.inertiaTensorInverse.M33;
                entryB = tX * angularBX + tY * angularBY + tZ * angularBZ + 1 / entityB.mass;
            }
            else
                entryB = 0;

            velocityToImpulse = -1 / (entryA + entryB); //Softness?

            //Bounciness
            bias = MathHelper.Min(
                MathHelper.Max(0, contact.PenetrationDepth - CollisionDetectionSettings.AllowedPenetration) *
                CollisionResponseSettings.PenetrationRecoveryStiffness / dt,
                CollisionResponseSettings.MaximumPositionCorrectionSpeed);
            if (contactManifoldConstraint.materialInteraction.Bounciness > 0)
            {
                //Compute relative velocity
                float relativeVelocity = -RelativeVelocity;
                if (relativeVelocity > CollisionResponseSettings.BouncinessVelocityThreshold)
                    bias = MathHelper.Max(relativeVelocity * contactManifoldConstraint.materialInteraction.Bounciness, bias);
            }

        }

        /// <summary>
        /// Performs any pre-solve iteration work that needs exclusive
        /// access to the members of the solver updateable.
        /// Usually, this is used for applying warmstarting impulses.
        /// </summary>
        public override void ExclusiveUpdate()
        {
            //Warm starting
#if !WINDOWS
            Vector3 linear = new Vector3();
            Vector3 angular = new Vector3();
#else
            Vector3 linear, angular;
#endif
            linear.X = accumulatedImpulse * linearAX;
            linear.Y = accumulatedImpulse * linearAY;
            linear.Z = accumulatedImpulse * linearAZ;
            if (entityADynamic)
            {
                angular.X = accumulatedImpulse * angularAX;
                angular.Y = accumulatedImpulse * angularAY;
                angular.Z = accumulatedImpulse * angularAZ;
                entityA.ApplyLinearImpulse(ref linear);
                entityA.ApplyAngularImpulse(ref angular);
            }
            if (entityBDynamic)
            {
                linear.X = -linear.X;
                linear.Y = -linear.Y;
                linear.Z = -linear.Z;
                angular.X = accumulatedImpulse * angularBX;
                angular.Y = accumulatedImpulse * angularBY;
                angular.Z = accumulatedImpulse * angularBZ;
                entityB.ApplyLinearImpulse(ref linear);
                entityB.ApplyAngularImpulse(ref angular);
            }
        }

        protected internal override void CollectInvolvedEntities(DataStructures.RawList<Entity> outputInvolvedEntities)
        {
            //This should never really have to be called.
            if (entityA != null)
                outputInvolvedEntities.Add(entityA);
            if (entityB != null)
                outputInvolvedEntities.Add(entityB);
        }

    }
}