using System;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.CollisionTests;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using BEPUphysics.CollisionTests.Manifolds;
using BEPUphysics.Constraints.Collision;
using BEPUphysics.PositionUpdating;
using BEPUphysics.Settings;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionShapes.ConvexShapes;

namespace BEPUphysics.NarrowPhaseSystems.Pairs
{
    ///<summary>
    /// Pair handler that manages a pair of two boxes.
    ///</summary>
    public abstract class ConvexPairHandler : Type1PairHandler
    {

        ConvexContactManifoldConstraint contactConstraint = new ConvexContactManifoldConstraint();


        protected override ContactManifoldConstraint ContactConstraint
        {
            get { return contactConstraint; }
        }



        ///<summary>
        /// Updates the time of impact for the pair.
        ///</summary>
        ///<param name="requester">Collidable requesting the update.</param>
        ///<param name="dt">Timestep duration.</param>
        public override void UpdateTimeOfImpact(Collidable requester, float dt)
        {
            var collidableA = CollidableA as ConvexCollidable;
            var collidableB = CollidableB as ConvexCollidable;
            var overlap = BroadPhaseOverlap;
            if (
                    (overlap.entryA.IsActive || overlap.entryB.IsActive) && //At least one has to be active.
                    (
                        (
                            collidableA.entity.PositionUpdateMode == PositionUpdateMode.Continuous &&   //If both are continuous, only do the process for A.
                            collidableB.entity.PositionUpdateMode == PositionUpdateMode.Continuous &&
                            overlap.entryA == requester
                        ) ||
                        (
                            collidableA.entity.PositionUpdateMode == PositionUpdateMode.Continuous ^   //If only one is continuous, then we must do it.
                            collidableB.entity.PositionUpdateMode == PositionUpdateMode.Continuous
                        )
                    )
                )
            {
                //Only perform the test if the minimum radii are small enough relative to the size of the velocity.
                //Discrete objects have already had their linear motion integrated, so don't use their velocity.
                Vector3 velocity;
                if (collidableA.entity.PositionUpdateMode == PositionUpdateMode.Discrete)
                    velocity = collidableB.entity.linearVelocity;
                else if (collidableB.entity.PositionUpdateMode == PositionUpdateMode.Discrete)
                    Vector3.Negate(ref collidableA.entity.linearVelocity, out velocity);
                else
                    Vector3.Subtract(ref collidableB.entity.linearVelocity, ref collidableA.entity.linearVelocity, out velocity);
                Vector3.Multiply(ref velocity, dt, out velocity);
                float velocitySquared = velocity.LengthSquared();

                var minimumRadiusA = collidableA.Shape.minimumRadius * MotionSettings.CoreShapeScaling;
                timeOfImpact = 1;
                if (minimumRadiusA * minimumRadiusA < velocitySquared)
                {
                    //Spherecast A against B.
                    RayHit rayHit;
                    if (GJKToolbox.CCDSphereCast(new Ray(collidableA.worldTransform.Position, -velocity), minimumRadiusA, collidableB.Shape, ref collidableB.worldTransform, timeOfImpact, out rayHit))
                        timeOfImpact = rayHit.T;
                }

                var minimumRadiusB = collidableB.Shape.minimumRadius * MotionSettings.CoreShapeScaling;
                if (minimumRadiusB * minimumRadiusB < velocitySquared)
                {
                    //Spherecast B against A.
                    RayHit rayHit;
                    if (GJKToolbox.CCDSphereCast(new Ray(collidableB.worldTransform.Position, velocity), minimumRadiusB, collidableA.Shape, ref collidableA.worldTransform, timeOfImpact, out rayHit))
                        timeOfImpact = rayHit.T;
                }

                //If it's intersecting, throw our hands into the air and give up.
                //This is generally a perfectly acceptable thing to do, since it's either sitting
                //inside another object (no ccd makes sense) or we're still in an intersecting case
                //from a previous frame where CCD took place and a contact should have been created
                //to deal with interpenetrating velocity.  Sometimes that contact isn't sufficient,
                //but it's good enough.
                if (timeOfImpact == 0)
                    timeOfImpact = 1;
            }

        }

        internal override void GetContactInformation(int index, out ContactInformation info)
        {
            info.Contact = ContactManifold.contacts.Elements[index];
            //Find the contact's normal force.
            float totalNormalImpulse = 0;
            info.NormalForce = 0;
            for (int i = 0; i < contactConstraint.penetrationConstraints.count; i++)
            {
                totalNormalImpulse += contactConstraint.penetrationConstraints.Elements[i].accumulatedImpulse;
                if(contactConstraint.penetrationConstraints.Elements[i].contact == info.Contact)
                {
                    info.NormalForce = contactConstraint.penetrationConstraints.Elements[i].accumulatedImpulse;
                }
            }
            //Compute friction force.  Since we are using central friction, this is 'faked.'
            float radius;
            Vector3.Distance(ref contactConstraint.slidingFriction.manifoldCenter, ref info.Contact.Position, out radius);
            info.FrictionForce = (info.NormalForce / totalNormalImpulse) * (contactConstraint.slidingFriction.accumulatedImpulse.Length() + contactConstraint.twistFriction.accumulatedImpulse * radius);

            //Compute relative velocity
            Vector3 velocity;
            Vector3.Subtract(ref info.Contact.Position, ref EntityA.position, out velocity);
            Vector3.Cross(ref EntityA.angularVelocity, ref velocity, out velocity);
            Vector3.Add(ref velocity, ref EntityA.linearVelocity, out info.RelativeVelocity);

            Vector3.Subtract(ref info.Contact.Position, ref EntityB.position, out velocity);
            Vector3.Cross(ref EntityB.angularVelocity, ref velocity, out velocity);
            Vector3.Add(ref velocity, ref EntityB.linearVelocity, out velocity);

            Vector3.Subtract(ref info.RelativeVelocity, ref velocity, out info.RelativeVelocity);
        }

    }

}
