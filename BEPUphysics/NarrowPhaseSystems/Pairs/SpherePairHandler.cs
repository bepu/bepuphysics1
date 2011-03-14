using System;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.CollisionTests;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using BEPUphysics.Constraints.Collision;
using BEPUphysics.PositionUpdating;
using BEPUphysics.Settings;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUphysics.CollisionShapes.ConvexShapes;

namespace BEPUphysics.NarrowPhaseSystems.Pairs
{
    ///<summary>
    /// Handles a sphere-sphere collision pair.
    ///</summary>
    public class SpherePairHandler : CollidablePairHandler
    {
        ConvexCollidable<SphereShape> sphereA;
        ConvexCollidable<SphereShape> sphereB;

        //Using a non-convex one since they have slightly lower overhead than their Convex friends when dealing with a single contact point.
        NonConvexContactManifoldConstraint contactConstraint = new NonConvexContactManifoldConstraint();

        //The 'colliding' boolean will track whether or not the contact is 'in the manifold' or not.
        bool colliding;
        //Only need one!
        Contact contact = new Contact();

        void OnContactAdded(Contact contact)
        {
            contactConstraint.AddContact(contact);


            if (!suppressEvents)
            {
                sphereA.events.OnContactCreated(sphereB, this, contact);
                sphereB.events.OnContactCreated(sphereA, this, contact);
            }
            if (Parent != null)
                Parent.OnContactAdded(contact);

        }

        void OnContactRemoved(Contact contact)
        {
            contactConstraint.RemoveContact(contact);



            if (!suppressEvents)
            {
                sphereA.events.OnContactRemoved(sphereB, this, contact);
                sphereB.events.OnContactRemoved(sphereA, this, contact);
            }
            if (Parent != null)
                Parent.OnContactRemoved(contact);


        }

        ///<summary>
        /// Initializes the pair handler.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        ///<param name="entryB">Second entry in the pair.</param>
        public override void Initialize(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {


            sphereA = entryA as ConvexCollidable<SphereShape>;
            sphereB = entryB as ConvexCollidable<SphereShape>;

            if (sphereA == null || sphereB == null)
            {
                throw new Exception("Inappropriate types used to initialize pair.");
            }

            if (!suppressEvents)
            {
                sphereA.events.OnPairCreated(sphereB, this);
                sphereB.events.OnPairCreated(sphereA, this);
            }
            contactConstraint.Initialize(sphereA.entity, sphereB.entity, this);
            UpdateMaterialProperties();

        }

        ///<summary>
        /// Forces an update of the pair's material properties.
        ///</summary>
        public override void UpdateMaterialProperties()
        {
            contactConstraint.UpdateMaterialProperties(
                sphereA.entity == null ? null : sphereA.entity.material,
                sphereB.entity == null ? null : sphereB.entity.material);
        }

        ///<summary>
        /// Called when the pair handler is added to the narrow phase.
        ///</summary>
        public override void OnAddedToNarrowPhase()
        {
            sphereA.pairs.Add(this);
            sphereB.pairs.Add(this);
        }

        ///<summary>
        /// Cleans up the pair handler.
        ///</summary>
        public override void CleanUp()
        {
            //TODO: These cleanup systems are convoluted, fragile, and duplicated all over the place.  Clean up the clean ups.
            previouslyColliding = false;

            if (colliding)
                OnContactRemoved(contact);
            //If there are still any constraints, remove solver updateables.
            if (contactConstraint.solver != null)
            {
                contactConstraint.pair = null; //Setting the pair to null tells the constraint that it's going to be orphaned.  It will be cleaned up on removal.
                if (Parent != null)
                    Parent.RemoveSolverUpdateable(contactConstraint);
                else if (NarrowPhase != null)
                    NarrowPhase.EnqueueRemovedSolverUpdateable(contactConstraint);
            }
            else
                contactConstraint.CleanUpReferences();//The constraint isn't in the solver, so we can safely clean it up directly.
            contactConstraint.CleanUp();


            if (colliding && !suppressEvents)
            {
                sphereA.events.OnCollisionEnded(sphereB, this);
                sphereB.events.OnCollisionEnded(sphereA, this);
                colliding = false;
            }

            sphereA.pairs.Remove(this);
            sphereB.pairs.Remove(this);

            if (!suppressEvents)
            {
                sphereA.events.OnPairRemoved(sphereB);
                sphereB.events.OnPairRemoved(sphereA);
            }

            sphereA = null;
            sphereB = null;




            base.CleanUp();
        }

        bool previouslyColliding;

        ///<summary>
        /// Updates the pair handler.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        public override void UpdateCollision(float dt)
        {

            if (!suppressEvents)
            {
                sphereA.events.OnPairUpdated(sphereB, this);
                sphereB.events.OnPairUpdated(sphereA, this);
            }

            ContactData contactData;
            if (colliding = SphereTester.AreSpheresColliding(sphereA.Shape, sphereB.Shape, ref sphereA.worldTransform.Position, ref sphereB.worldTransform.Position, out contactData))
            {
                contact.Position = contactData.Position;
                contact.Normal = contactData.Normal;
                contact.PenetrationDepth = contactData.PenetrationDepth;
                if (!previouslyColliding)
                    OnContactAdded(contact);
            }
            else
            {
                if (previouslyColliding)
                    OnContactRemoved(contact);
            }





            if (colliding)
            {
                if (!suppressEvents)
                {
                    sphereA.events.OnPairTouching(sphereB, this);
                    sphereB.events.OnPairTouching(sphereA, this);
                }

                if (!previouslyColliding)
                {
                    //New collision.

                    //Add a solver item.
                    if (Parent != null)
                        Parent.AddSolverUpdateable(contactConstraint);
                    else if (NarrowPhase != null)
                        NarrowPhase.EnqueueGeneratedSolverUpdateable(contactConstraint);

                    //And notify the pair members.
                    if (!suppressEvents)
                    {
                        sphereA.events.OnInitialCollisionDetected(sphereB, this);
                        sphereB.events.OnInitialCollisionDetected(sphereA, this);
                    }
                }
            }
            else if (previouslyColliding)
            {
                //Just exited collision.

                //Remove the solver item.
                if (Parent != null)
                    Parent.RemoveSolverUpdateable(contactConstraint);
                else if (NarrowPhase != null)
                    NarrowPhase.EnqueueRemovedSolverUpdateable(contactConstraint);

                if (!suppressEvents)
                {
                    sphereA.events.OnCollisionEnded(sphereB, this);
                    sphereB.events.OnCollisionEnded(sphereA, this);
                }
            }
            previouslyColliding = colliding;
        }


        ///<summary>
        /// Updates the time of impact for the pair.
        ///</summary>
        ///<param name="requester">Collidable requesting the update.</param>
        ///<param name="dt">Timestep duration.</param>
        public override void UpdateTimeOfImpact(Collidable requester, float dt)
        {
            //TODO: This conditional early outing stuff could be pulled up into a common system, along with most of the pair handler.
            var overlap = BroadPhaseOverlap;
            if (
                    (overlap.entryA.IsActive || overlap.entryB.IsActive) && //At least one has to be active.
                    (
                        (
                            sphereA.entity.PositionUpdateMode == PositionUpdateMode.Continuous &&   //If both are continuous, only do the process for A.
                            sphereB.entity.PositionUpdateMode == PositionUpdateMode.Continuous &&
                            overlap.entryA == requester
                        ) ||
                        (
                            sphereA.entity.PositionUpdateMode == PositionUpdateMode.Continuous ^   //If only one is continuous, then we must do it.
                            sphereB.entity.PositionUpdateMode == PositionUpdateMode.Continuous
                        )
                    )
                )
            {
                //Only perform the test if the minimum radii are small enough relative to the size of the velocity.
                Vector3 velocity;
                if (sphereA.entity.PositionUpdateMode == PositionUpdateMode.Discrete)
                    velocity = sphereB.entity.linearVelocity;
                else if (sphereB.entity.PositionUpdateMode == PositionUpdateMode.Discrete)
                    Vector3.Negate(ref sphereA.entity.linearVelocity, out velocity);
                else
                    Vector3.Subtract(ref sphereB.entity.linearVelocity, ref sphereA.entity.linearVelocity, out velocity);
                Vector3.Multiply(ref velocity, dt, out velocity);
                float velocitySquared = velocity.LengthSquared();

                var minimumRadiusA = sphereA.minimumRadius * MotionSettings.CoreShapeScaling;
                timeOfImpact = 1;
                if (minimumRadiusA * minimumRadiusA < velocitySquared)
                {
                    //Spherecast A against B.
                    RayHit rayHit;
                    if (GJKToolbox.SphereCast(new Ray(sphereA.worldTransform.Position, -velocity), minimumRadiusA, sphereB.Shape, ref sphereB.worldTransform, 1, out rayHit))
                    {
                        timeOfImpact = rayHit.T;
                    }
                }

                var minimumRadiusB = sphereB.minimumRadius * MotionSettings.CoreShapeScaling;
                if (minimumRadiusB * minimumRadiusB < velocitySquared)
                {
                    //Spherecast B against A.
                    RayHit rayHit;
                    if (GJKToolbox.SphereCast(new Ray(sphereB.worldTransform.Position, velocity), minimumRadiusB, sphereA.Shape, ref sphereA.worldTransform, 1, out rayHit) &&
                        rayHit.T < timeOfImpact)
                    {
                        timeOfImpact = rayHit.T;
                    }
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


        internal override int ContactCount
        {
            get { return colliding ? 1 : 0; }
        }

        internal override void GetContactInformation(int index, out ContactInformation info)
        {
            info.Contact = contact;
            info.NormalForce = contactConstraint.ContactPenetrationConstraints[0].accumulatedImpulse;
            info.FrictionForce = contactConstraint.ContactFrictionConstraints[0].accumulatedImpulse;

            //Compute relative velocity
            Vector3 velocity;
            Vector3.Subtract(ref info.Contact.Position, ref sphereA.entity.position, out velocity);
            Vector3.Cross(ref sphereA.entity.angularVelocity, ref velocity, out velocity);
            Vector3.Add(ref velocity, ref sphereA.entity.linearVelocity, out info.RelativeVelocity);

            Vector3.Subtract(ref info.Contact.Position, ref sphereB.entity.position, out velocity);
            Vector3.Cross(ref sphereB.entity.angularVelocity, ref velocity, out velocity);
            Vector3.Add(ref velocity, ref sphereB.entity.linearVelocity, out velocity);

            Vector3.Subtract(ref info.RelativeVelocity, ref velocity, out info.RelativeVelocity);
        }

    }

}
