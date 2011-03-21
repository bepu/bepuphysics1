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
    /// Handles a triangle-convex collision pair.
    ///</summary>
    public class TriangleConvexPairHandler : CollidablePairHandler
    {
        ConvexCollidable<TriangleShape> triangle;
        ConvexCollidable convex;

        TriangleConvexContactManifold contactManifold = new TriangleConvexContactManifold();
        ConvexContactManifoldConstraint contactConstraint = new ConvexContactManifoldConstraint();

        Action<Contact> contactAddedDelegate;
        Action<Contact> contactRemovedDelegate;

        ///<summary>
        /// Constructs a new pair handler.
        ///</summary>
        public TriangleConvexPairHandler()
        {
            contactAddedDelegate = OnContactAdded;
            contactRemovedDelegate = OnContactRemoved;
        }

        void OnContactAdded(Contact contact)
        {
            contactConstraint.AddContact(contact);


            if (!suppressEvents)
            {
                triangle.events.OnContactCreated(convex, this, contact);
                convex.events.OnContactCreated(triangle, this, contact);
            }
            if (Parent != null)
                Parent.OnContactAdded(contact);

        }

        void OnContactRemoved(Contact contact)
        {
            contactConstraint.RemoveContact(contact);

            if (!suppressEvents)
            {
                triangle.events.OnContactRemoved(convex, this, contact);
                convex.events.OnContactRemoved(triangle, this, contact);
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

            triangle = entryA as ConvexCollidable<TriangleShape>;
            convex = entryB as ConvexCollidable;

            if (triangle == null || convex == null)
            {
                triangle = entryB as ConvexCollidable<TriangleShape>;
                convex = entryA as ConvexCollidable;

                if (triangle == null || convex == null)
                    throw new Exception("Inappropriate types used to initialize pair.");
            }

            if (!suppressEvents)
            {
                triangle.events.OnPairCreated(convex, this);
                convex.events.OnPairCreated(triangle, this);
            }

            contactManifold.Initialize(convex, triangle);
            contactManifold.ContactAdded += contactAddedDelegate;
            contactManifold.ContactRemoved += contactRemovedDelegate;

            contactConstraint.Initialize(convex.entity, triangle.entity, this);
            UpdateMaterialProperties();

        }

        ///<summary>
        /// Forces an update of the pair's material properties.
        ///</summary>
        public override void UpdateMaterialProperties()
        {
            contactConstraint.UpdateMaterialProperties(
                 convex.entity == null ? null : convex.entity.material,
                 triangle.entity == null ? null : triangle.entity.material);
        }

        ///<summary>
        /// Called when the pair handler is added to the narrow phase.
        ///</summary>
        public override void OnAddedToNarrowPhase()
        {
            triangle.pairs.Add(this);
            convex.pairs.Add(this);
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
                triangle.events.OnCollisionEnded(convex, this);
                convex.events.OnCollisionEnded(triangle, this);
            }

            triangle.pairs.Remove(this);
            convex.pairs.Remove(this);

            if (!suppressEvents)
            {
                triangle.events.OnPairRemoved(convex);
                convex.events.OnPairRemoved(triangle);
            }

            triangle = null;
            convex = null;


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
                triangle.events.OnPairUpdated(convex, this);
                convex.events.OnPairUpdated(triangle, this);
            }


            contactManifold.Update(dt);



            if (contactManifold.contacts.count > 0)
            {
                if (!suppressEvents)
                {
                    triangle.events.OnPairTouching(convex, this);
                    convex.events.OnPairTouching(triangle, this);
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
                        triangle.events.OnInitialCollisionDetected(convex, this);
                        convex.events.OnInitialCollisionDetected(triangle, this);
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
                    triangle.events.OnCollisionEnded(convex, this);
                    convex.events.OnCollisionEnded(triangle, this);
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
                            convex.entity.PositionUpdateMode == PositionUpdateMode.Continuous &&   //If both are continuous, only do the process for A.
                            triangle.entity.PositionUpdateMode == PositionUpdateMode.Continuous &&
                            overlap.entryA == requester
                        ) ||
                        (
                            convex.entity.PositionUpdateMode == PositionUpdateMode.Continuous ^   //If only one is continuous, then we must do it.
                            triangle.entity.PositionUpdateMode == PositionUpdateMode.Continuous
                        )
                    )
                )
            {





                //Only perform the test if the minimum radii are small enough relative to the size of the velocity.
                Vector3 velocity;
                Vector3.Subtract(ref triangle.entity.linearVelocity, ref convex.entity.linearVelocity, out velocity);
                Vector3.Multiply(ref velocity, dt, out velocity);
                float velocitySquared = velocity.LengthSquared();

                var minimumRadiusA = convex.Shape.minimumRadius * MotionSettings.CoreShapeScaling;
                timeOfImpact = 1;
                if (minimumRadiusA * minimumRadiusA < velocitySquared)
                {
                    //Spherecast A against B.
                    RayHit rayHit;
                    if (GJKToolbox.SphereCast(new Ray(convex.worldTransform.Position, -velocity), minimumRadiusA, triangle.Shape, ref triangle.worldTransform, 1, out rayHit))
                    {
                        if (triangle.Shape.sidedness != TriangleSidedness.DoubleSided)
                        {                
                            //Only perform sweep if the object is in danger of hitting the object.
                            //Triangles can be one sided, so check the impact normal against the triangle normal.
                            Vector3 AB, AC;
                            Vector3.Subtract(ref triangle.Shape.vB, ref triangle.Shape.vA, out AB);
                            Vector3.Subtract(ref triangle.Shape.vC, ref triangle.Shape.vA, out AC);
                            Vector3 normal;
                            Vector3.Cross(ref AB, ref AC, out normal);

                            float dot;
                            Vector3.Dot(ref rayHit.Normal, ref normal, out dot);
                            if (triangle.Shape.sidedness == TriangleSidedness.Counterclockwise && dot < 0 ||
                                triangle.Shape.sidedness == TriangleSidedness.Clockwise && dot > 0)
                            {
                                timeOfImpact = rayHit.T;
                            }
                        }
                        else
                        {
                            timeOfImpact = rayHit.T;
                        }
                    }
                }

                //TECHNICALLY, the triangle should be casted too.  But, given the way triangles are usually used and their tiny minimum radius, ignoring it is usually just fine.
                //var minimumRadiusB = triangle.minimumRadius * MotionSettings.CoreShapeScaling;
                //if (minimumRadiusB * minimumRadiusB < velocitySquared)
                //{
                //    //Spherecast B against A.
                //    RayHit rayHit;
                //    if (GJKToolbox.SphereCast(new Ray(triangle.entity.position, velocity), minimumRadiusB, convex.Shape, ref convex.worldTransform, 1, out rayHit) &&
                //        rayHit.T < timeOfImpact)
                //    {
                //        if (triangle.Shape.sidedness != TriangleSidedness.DoubleSided)
                //        {
                //            float dot;
                //            Vector3.Dot(ref rayHit.Normal, ref normal, out dot);
                //            if (dot > 0)
                //            {
                //                timeOfImpact = rayHit.T;
                //            }
                //        }
                //        else
                //        {
                //            timeOfImpact = rayHit.T;
                //        }
                //    }
                //}

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
            get { return contactManifold.contacts.count; }
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
                if (contactConstraint.penetrationConstraints.Elements[i].contact == info.Contact)
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
            Vector3.Subtract(ref info.Contact.Position, ref triangle.entity.position, out velocity);
            Vector3.Cross(ref triangle.entity.angularVelocity, ref velocity, out velocity);
            Vector3.Add(ref velocity, ref triangle.entity.linearVelocity, out info.RelativeVelocity);

            Vector3.Subtract(ref info.Contact.Position, ref convex.entity.position, out velocity);
            Vector3.Cross(ref convex.entity.angularVelocity, ref velocity, out velocity);
            Vector3.Add(ref velocity, ref convex.entity.linearVelocity, out velocity);

            Vector3.Subtract(ref info.RelativeVelocity, ref velocity, out info.RelativeVelocity);
        }
    }

}
