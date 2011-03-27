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
    public class BoxPairHandler : CollidablePairHandler
    {
        ConvexCollidable<BoxShape> boxA;
        ConvexCollidable<BoxShape> boxB;

        BoxContactManifold contactManifold = new BoxContactManifold();
        ConvexContactManifoldConstraint contactConstraint = new ConvexContactManifoldConstraint();

        Action<Contact> contactAddedDelegate;
        Action<Contact> contactRemovedDelegate;

        ///<summary>
        /// Constructs a new box pair handler.
        ///</summary>
        public BoxPairHandler()
        {
            contactAddedDelegate = OnContactAdded;
            contactRemovedDelegate = OnContactRemoved;
        }

        void OnContactAdded(Contact contact)
        {
            contactConstraint.AddContact(contact);


            if (!suppressEvents)
            {
                boxA.events.OnContactCreated(boxB, this, contact);
                boxB.events.OnContactCreated(boxA, this, contact);
            }
            if (Parent != null)
                Parent.OnContactAdded(contact);

        }

        void OnContactRemoved(Contact contact)
        {
            contactConstraint.RemoveContact(contact);



            if (!suppressEvents)
            {
                boxA.events.OnContactRemoved(boxB, this, contact);
                boxB.events.OnContactRemoved(boxA, this, contact);
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


            boxA = entryA as ConvexCollidable<BoxShape>;
            boxB = entryB as ConvexCollidable<BoxShape>;

            if (boxA == null || boxB == null)
            {
                throw new Exception("Inappropriate types used to initialize pair.");
            }

            if (!suppressEvents)
            {
                boxA.events.OnPairCreated(boxB, this);
                boxB.events.OnPairCreated(boxA, this);
            }


            contactManifold.Initialize(boxA, boxB);
            contactManifold.ContactAdded += contactAddedDelegate;
            contactManifold.ContactRemoved += contactRemovedDelegate;

            contactConstraint.Initialize(boxA.entity, boxB.entity, this);
            UpdateMaterialProperties();
        }

        ///<summary>
        /// Forces an update of the pair's material properties.
        ///</summary>
        public override void UpdateMaterialProperties()
        {
            contactConstraint.UpdateMaterialProperties(
                boxA.entity == null ? null : boxA.entity.material,
                boxB.entity == null ? null : boxB.entity.material);
        }

        ///<summary>
        /// Called when the pair handler is added to the narrow phase.
        ///</summary>
        public override void OnAddedToNarrowPhase()
        {
            //Note: This is not in the Initialize befcause Initialize is is multithreaded.
            //This accesses probably shared information.
            boxA.pairs.Add(this);
            boxB.pairs.Add(this);
        }

        ///<summary>
        /// Cleans up the pair handler.
        ///</summary>
        public override void CleanUp()
        {
            //TODO: These cleanup systems are convoluted, fragile, and duplicated all over the place.  Clean up the clean ups.
            previousContactCount = 0;

            for (int i = contactManifold.contacts.count - 1; i >= 0; i--)
            {
                OnContactRemoved(contactManifold.contacts[i]);
            }

            //If the constraint is still in the solver, then request to have it removed.
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

            if (contactManifold.contacts.count > 0 && !suppressEvents)
            {
                boxA.events.OnCollisionEnded(boxB, this);
                boxB.events.OnCollisionEnded(boxA, this);
            }

            //Note: CleanUps occur sequentially, like the OnAddedToNarrowPhase.
            //That's why these two removes occur here instead of another "OnRemovedFromNarrowPhase."
            boxA.pairs.Remove(this);
            boxB.pairs.Remove(this);

            if (!suppressEvents)
            {
                boxA.events.OnPairRemoved(boxB);
                boxB.events.OnPairRemoved(boxA);
            }

            boxA = null;
            boxB = null;


            contactManifold.CleanUp();



            base.CleanUp();
        }

        int previousContactCount;

        ///<summary>
        /// Updates the pair handler.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        public override void UpdateCollision(float dt)
        {

            if (!suppressEvents)
            {
                boxA.events.OnPairUpdated(boxB, this);
                boxB.events.OnPairUpdated(boxA, this);
            }

            contactManifold.Update(dt);




            if (contactManifold.contacts.count > 0)
            {
                if (!suppressEvents)
                {
                    boxA.events.OnPairTouching(boxB, this);
                    boxB.events.OnPairTouching(boxA, this);
                }

                if (previousContactCount == 0)
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
                        boxA.events.OnInitialCollisionDetected(boxB, this);
                        boxB.events.OnInitialCollisionDetected(boxA, this);
                    }
                }
            }
            else if (previousContactCount > 0)
            {
                //Just exited collision.

                //Remove the solver item.
                if (Parent != null)
                    Parent.RemoveSolverUpdateable(contactConstraint);
                else if (NarrowPhase != null)
                    NarrowPhase.EnqueueRemovedSolverUpdateable(contactConstraint);

                if (!suppressEvents)
                {
                    boxA.events.OnCollisionEnded(boxB, this);
                    boxB.events.OnCollisionEnded(boxA, this);
                }
            }
            previousContactCount = contactManifold.contacts.count;
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
                            boxA.entity.PositionUpdateMode == PositionUpdateMode.Continuous &&   //If both are continuous, only do the process for A.
                            boxB.entity.PositionUpdateMode == PositionUpdateMode.Continuous &&
                            overlap.entryA == requester
                        ) ||
                        (
                            boxA.entity.PositionUpdateMode == PositionUpdateMode.Continuous ^   //If only one is continuous, then we must do it.
                            boxB.entity.PositionUpdateMode == PositionUpdateMode.Continuous
                        )
                    )
                )
            {
                //Only perform the test if the minimum radii are small enough relative to the size of the velocity.
                //Discrete objects have already had their linear motion integrated, so don't use their velocity.
                Vector3 velocity;
                if (boxA.entity.PositionUpdateMode == PositionUpdateMode.Discrete)
                    velocity = boxB.entity.linearVelocity;
                else if (boxB.entity.PositionUpdateMode == PositionUpdateMode.Discrete)
                    Vector3.Negate(ref boxA.entity.linearVelocity, out velocity);
                else
                    Vector3.Subtract(ref boxB.entity.linearVelocity, ref boxA.entity.linearVelocity, out velocity);
                Vector3.Multiply(ref velocity, dt, out velocity);
                float velocitySquared = velocity.LengthSquared();

                var minimumRadiusA = boxA.Shape.minimumRadius * MotionSettings.CoreShapeScaling;
                timeOfImpact = 1;
                if (minimumRadiusA * minimumRadiusA < velocitySquared)
                {
                    //Spherecast A against B.
                    RayHit rayHit;
                    if (GJKToolbox.CCDSphereCast(new Ray(boxA.worldTransform.Position, -velocity), minimumRadiusA, boxB.Shape, ref boxB.worldTransform, timeOfImpact, out rayHit))
                        timeOfImpact = rayHit.T;
                }

                var minimumRadiusB = boxB.Shape.minimumRadius * MotionSettings.CoreShapeScaling;
                if (minimumRadiusB * minimumRadiusB < velocitySquared)
                {
                    //Spherecast B against A.
                    RayHit rayHit;
                    if (GJKToolbox.CCDSphereCast(new Ray(boxB.worldTransform.Position, velocity), minimumRadiusB, boxA.Shape, ref boxA.worldTransform, timeOfImpact, out rayHit))
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
            info.Contact = contactManifold.contacts.Elements[index];
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
            Vector3.Subtract(ref info.Contact.Position, ref boxA.entity.position, out velocity);
            Vector3.Cross(ref boxA.entity.angularVelocity, ref velocity, out velocity);
            Vector3.Add(ref velocity, ref boxA.entity.linearVelocity, out info.RelativeVelocity);

            Vector3.Subtract(ref info.Contact.Position, ref boxB.entity.position, out velocity);
            Vector3.Cross(ref boxB.entity.angularVelocity, ref velocity, out velocity);
            Vector3.Add(ref velocity, ref boxB.entity.linearVelocity, out velocity);

            Vector3.Subtract(ref info.RelativeVelocity, ref velocity, out info.RelativeVelocity);
        }

        internal override int ContactCount
        {
            get { return contactManifold.contacts.count; }
        }
    }

}
