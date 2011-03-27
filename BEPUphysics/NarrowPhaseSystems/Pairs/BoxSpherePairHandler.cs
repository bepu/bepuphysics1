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
    /// Handles a box and sphere in a collision.
    ///</summary>
    public class BoxSpherePairHandler : CollidablePairHandler
    {
        ConvexCollidable<BoxShape> box;
        ConvexCollidable<SphereShape> sphere;

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
                box.events.OnContactCreated(sphere, this, contact);
                sphere.events.OnContactCreated(box, this, contact);
            }
            if (Parent != null)
                Parent.OnContactAdded(contact);

        }

        void OnContactRemoved(Contact contact)
        {
            contactConstraint.RemoveContact(contact);



            if (!suppressEvents)
            {
                box.events.OnContactRemoved(sphere, this, contact);
                sphere.events.OnContactRemoved(box, this, contact);
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


            box = entryA as ConvexCollidable<BoxShape>;
            sphere = entryB as ConvexCollidable<SphereShape>;

            if (box == null || sphere == null)
            {
                box = entryB as ConvexCollidable<BoxShape>;
                sphere = entryA as ConvexCollidable<SphereShape>;
                if (box == null || sphere == null)
                {
                    throw new Exception("Inappropriate types used to initialize pair.");
                }
            }

            if (!suppressEvents)
            {
                box.events.OnPairCreated(sphere, this);
                sphere.events.OnPairCreated(box, this);
            }

            contactConstraint.Initialize(box.entity, sphere.entity, this);
            UpdateMaterialProperties();

        }

        ///<summary>
        /// Forces an update of the pair's material properties.
        ///</summary>
        public override void UpdateMaterialProperties()
        {
            contactConstraint.UpdateMaterialProperties(
                box.entity == null ? null : box.entity.material,
                sphere.entity == null ? null : sphere.entity.material);
        }


        ///<summary>
        /// Called when the pair handler is added to the narrow phase.
        ///</summary>
        public override void OnAddedToNarrowPhase()
        {
            box.pairs.Add(this);
            sphere.pairs.Add(this);

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
                box.events.OnCollisionEnded(sphere, this);
                sphere.events.OnCollisionEnded(box, this);
                colliding = false;
            }

            box.pairs.Remove(this);
            sphere.pairs.Remove(this);

            if (!suppressEvents)
            {
                box.events.OnPairRemoved(sphere);
                sphere.events.OnPairRemoved(box);
            }

            box = null;
            sphere = null;




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
                box.events.OnPairUpdated(sphere, this);
                sphere.events.OnPairUpdated(box, this);
            }

            ContactData contactData;
            if (colliding = BoxSphereTester.AreShapesColliding(box.Shape, sphere.Shape, ref box.worldTransform, ref sphere.worldTransform.Position, out contactData))
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
                    box.events.OnPairTouching(sphere, this);
                    sphere.events.OnPairTouching(box, this);
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
                        box.events.OnInitialCollisionDetected(sphere, this);
                        sphere.events.OnInitialCollisionDetected(box, this);
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
                    box.events.OnCollisionEnded(sphere, this);
                    sphere.events.OnCollisionEnded(box, this);
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
                            box.entity.PositionUpdateMode == PositionUpdateMode.Continuous &&   //If both are continuous, only do the process for A.
                            sphere.entity.PositionUpdateMode == PositionUpdateMode.Continuous &&
                            overlap.entryA == requester
                        ) ||
                        (
                            box.entity.PositionUpdateMode == PositionUpdateMode.Continuous ^   //If only one is continuous, then we must do it.
                            sphere.entity.PositionUpdateMode == PositionUpdateMode.Continuous
                        )
                    )
                )
            {
                //timeOfImpact = 1;
                //float radius = sphere.Shape.collisionMargin - .001f;
                //while (radius > Toolbox.Epsilon)
                //{
                //    float planeY = box.worldTransform.Position.Y + box.Shape.halfHeight + radius;
                //    //if (sphere.worldTransform.Position.Y - planeY < Toolbox.Epsilon)
                //    //{
                //    //    timeOfImpact = 1;
                //    //    return;
                //    //}

                //    if (sphere.entity.linearVelocity.Y >= 0)
                //    {
                //        timeOfImpact = 1;
                //        return;
                //    }
                //    float t = (sphere.worldTransform.Position.Y - planeY) / (-sphere.entity.linearVelocity.Y * dt);
                //    if (t <= 1 && t > 0)
                //    {
                //        timeOfImpact = t;
                //        return;
                //    }
                //    if (t > 1)
                //    {
                //        timeOfImpact = 1;
                //        return;
                //    }
                //    radius /= 2;
                //}
                //if (timeOfImpact <= 0)
                //    timeOfImpact = 1;
                //return;

                //Only perform the test if the minimum radii are small enough relative to the size of the velocity.
                Vector3 velocity;
                if (box.entity.PositionUpdateMode == PositionUpdateMode.Discrete)
                    velocity = sphere.entity.linearVelocity;
                else if (sphere.entity.PositionUpdateMode == PositionUpdateMode.Discrete)
                    Vector3.Negate(ref box.entity.linearVelocity, out velocity);
                else
                    Vector3.Subtract(ref sphere.entity.linearVelocity, ref box.entity.linearVelocity, out velocity);
                Vector3.Multiply(ref velocity, dt, out velocity);
                float velocitySquared = velocity.LengthSquared();

                var minimumRadiusA = box.Shape.minimumRadius * MotionSettings.CoreShapeScaling;
                timeOfImpact = 1;
                if (minimumRadiusA * minimumRadiusA < velocitySquared)
                {
                    //Spherecast A against B.
                    RayHit rayHit;
                    if (GJKToolbox.CCDSphereCast(new Ray(box.worldTransform.Position, -velocity), minimumRadiusA, sphere.Shape, ref sphere.worldTransform, timeOfImpact, out rayHit))
                        timeOfImpact = rayHit.T;
                }

                var minimumRadiusB = sphere.Shape.minimumRadius * MotionSettings.CoreShapeScaling;
                if (minimumRadiusB * minimumRadiusB < velocitySquared)
                {
                    RayHit rayHit;
                    if (GJKToolbox.CCDSphereCast(new Ray(sphere.worldTransform.Position, velocity), minimumRadiusB, box.Shape, ref box.worldTransform, timeOfImpact, out rayHit))
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

        bool TryToi(float minimumRadiusSphere, Vector3 velocity, out float toi)
        {
            //Spherecast B against A.
            RayHit rayHit;
            if (GJKToolbox.SphereCast(new Ray(sphere.worldTransform.Position, velocity), minimumRadiusSphere, box.Shape, ref box.worldTransform, 1, out rayHit) &&
                rayHit.T < timeOfImpact)
            {
                toi = rayHit.T;
                return true;
            }
            minimumRadiusSphere /= 2;
            toi = 1;
            if (minimumRadiusSphere < Toolbox.Epsilon)
                return false;
            return TryToi(minimumRadiusSphere / 2, velocity, out toi);

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
            Vector3.Subtract(ref info.Contact.Position, ref sphere.entity.position, out velocity);
            Vector3.Cross(ref sphere.entity.angularVelocity, ref velocity, out velocity);
            Vector3.Add(ref velocity, ref sphere.entity.linearVelocity, out info.RelativeVelocity);

            Vector3.Subtract(ref info.Contact.Position, ref box.entity.position, out velocity);
            Vector3.Cross(ref box.entity.angularVelocity, ref velocity, out velocity);
            Vector3.Add(ref velocity, ref box.entity.linearVelocity, out velocity);

            Vector3.Subtract(ref info.RelativeVelocity, ref velocity, out info.RelativeVelocity);
        }
    }

}
