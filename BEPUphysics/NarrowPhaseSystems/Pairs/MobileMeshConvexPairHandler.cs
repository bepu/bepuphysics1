﻿using System;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.CollisionTests;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using BEPUphysics.CollisionTests.Manifolds;
using BEPUphysics.Constraints.Collision;
using BEPUphysics.DataStructures;
using BEPUphysics.PositionUpdating;
using BEPUphysics.Settings;
using Microsoft.Xna.Framework;
using BEPUphysics.MathExtensions;
using BEPUphysics.ResourceManagement;
using BEPUphysics.CollisionShapes.ConvexShapes;

namespace BEPUphysics.NarrowPhaseSystems.Pairs
{
    ///<summary>
    /// Handles a mobile mesh-convex collision pair.
    ///</summary>
    public class MobileMeshConvexPairHandler : CollidablePairHandler
    {
        MobileMeshCollidable mobileMesh;
        ConvexCollidable convex;

        MobileMeshConvexContactManifold contactManifold = new MobileMeshConvexContactManifold();
        NonConvexContactManifoldConstraint contactConstraint = new NonConvexContactManifoldConstraint();

        Action<Contact> contactAddedDelegate;
        Action<Contact> contactRemovedDelegate;

        ///<summary>
        /// Constructs a pair handler.
        ///</summary>
        public MobileMeshConvexPairHandler()
        {
            contactAddedDelegate = OnContactAdded;
            contactRemovedDelegate = OnContactRemoved;
        }

        void OnContactAdded(Contact contact)
        {
            contactConstraint.AddContact(contact);


            if (!suppressEvents)
            {
                mobileMesh.events.OnContactCreated(convex, this, contact);
                convex.events.OnContactCreated(mobileMesh, this, contact);
            }
            if (Parent != null)
                Parent.OnContactAdded(contact);

        }

        void OnContactRemoved(Contact contact)
        {
            contactConstraint.RemoveContact(contact);

            if (!suppressEvents)
            {
                mobileMesh.events.OnContactRemoved(convex, this, contact);
                convex.events.OnContactRemoved(mobileMesh, this, contact);
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

            mobileMesh = entryA as MobileMeshCollidable;
            convex = entryB as ConvexCollidable;

            if (mobileMesh == null || convex == null)
            {
                mobileMesh = entryB as MobileMeshCollidable;
                convex = entryA as ConvexCollidable;

                if (mobileMesh == null || convex == null)
                    throw new Exception("Inappropriate types used to initialize pair.");
            }

            if (!suppressEvents)
            {
                mobileMesh.events.OnPairCreated(convex, this);
                convex.events.OnPairCreated(mobileMesh, this);
            }

            contactManifold.Initialize(mobileMesh, convex);
            contactManifold.ContactAdded += contactAddedDelegate;
            contactManifold.ContactRemoved += contactRemovedDelegate;


            contactConstraint.Initialize(convex.entity, mobileMesh.entity, this);
            UpdateMaterialProperties();

        }

        ///<summary>
        /// Forces an update of the pair's material properties.
        ///</summary>
        public override void UpdateMaterialProperties()
        {
            contactConstraint.UpdateMaterialProperties(
              convex.entity == null ? null : convex.entity.material,
              mobileMesh.entity == null ? null : mobileMesh.entity.material);
        }

        ///<summary>
        /// Called when the pair handler is added to the narrow phase.
        ///</summary>
        public override void OnAddedToNarrowPhase()
        {
            mobileMesh.pairs.Add(this);
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
                mobileMesh.events.OnCollisionEnded(convex, this);
                convex.events.OnCollisionEnded(mobileMesh, this);
            }

            mobileMesh.pairs.Remove(this);
            convex.pairs.Remove(this);

            if (!suppressEvents)
            {
                mobileMesh.events.OnPairRemoved(convex);
                convex.events.OnPairRemoved(mobileMesh);
            }

            mobileMesh = null;
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
                mobileMesh.events.OnPairUpdated(convex, this);
                convex.events.OnPairUpdated(mobileMesh, this);
            }


            contactManifold.Update(dt);



            if (contactManifold.contacts.count > 0)
            {
                if (!suppressEvents)
                {
                    mobileMesh.events.OnPairTouching(convex, this);
                    convex.events.OnPairTouching(mobileMesh, this);
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
                        mobileMesh.events.OnInitialCollisionDetected(convex, this);
                        convex.events.OnInitialCollisionDetected(mobileMesh, this);
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
                    mobileMesh.events.OnCollisionEnded(convex, this);
                    convex.events.OnCollisionEnded(mobileMesh, this);
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
                            mobileMesh.entity.PositionUpdateMode == PositionUpdateMode.Continuous &&
                            overlap.entryA == requester
                        ) ||
                        (
                            convex.entity.PositionUpdateMode == PositionUpdateMode.Continuous ^   //If only one is continuous, then we must do it.
                            mobileMesh.entity.PositionUpdateMode == PositionUpdateMode.Continuous
                        )
                    )
                )
            {
                //TODO: This system could be made more robust by using a similar region-based rejection of edges.
                //CCD events are awfully rare under normal circumstances, so this isn't usually an issue.

                //Only perform the test if the minimum radii are small enough relative to the size of the velocity.
                Vector3 velocity;
                Vector3.Subtract(ref convex.entity.linearVelocity, ref mobileMesh.entity.linearVelocity, out velocity);
                Vector3.Multiply(ref velocity, dt, out velocity);
                float velocitySquared = velocity.LengthSquared();

                var minimumRadius = convex.Shape.minimumRadius * MotionSettings.CoreShapeScaling;
                timeOfImpact = 1;
                if (minimumRadius * minimumRadius < velocitySquared)
                {
                    TriangleSidedness sidedness;
                    switch (mobileMesh.Shape.solidity)
                    {
                        case CollisionShapes.MobileMeshSolidity.Clockwise:
                            sidedness = TriangleSidedness.Clockwise;
                            break;
                        case CollisionShapes.MobileMeshSolidity.Counterclockwise:
                            sidedness = TriangleSidedness.Counterclockwise;
                            break;
                        case CollisionShapes.MobileMeshSolidity.DoubleSided:
                            sidedness = TriangleSidedness.DoubleSided;
                            break;
                        case CollisionShapes.MobileMeshSolidity.Solid:
                        default:
                            sidedness = mobileMesh.Shape.solidSidedness;
                            break;
                    }
                    Matrix3X3 orientation;
                    Matrix3X3.CreateFromQuaternion(ref mobileMesh.worldTransform.Orientation, out orientation);
                    var triangle = Resources.GetTriangle();
                    triangle.collisionMargin = 0;
                    //Spherecast against all triangles to find the earliest time.
                    for (int i = 0; i < contactManifold.overlappedTriangles.count; i++)
                    {
                        MeshBoundingBoxTreeData data = mobileMesh.Shape.TriangleMesh.Data;
                        int triangleIndex = contactManifold.overlappedTriangles.Elements[i];
                        data.GetTriangle(triangleIndex, out triangle.vA, out triangle.vB, out triangle.vC);
                        Matrix3X3.Transform(ref triangle.vA, ref orientation, out triangle.vA);
                        Matrix3X3.Transform(ref triangle.vB, ref orientation, out triangle.vB);
                        Matrix3X3.Transform(ref triangle.vC, ref orientation, out triangle.vC);
                        Vector3.Add(ref triangle.vA, ref mobileMesh.worldTransform.Position, out triangle.vA);
                        Vector3.Add(ref triangle.vB, ref mobileMesh.worldTransform.Position, out triangle.vB);
                        Vector3.Add(ref triangle.vC, ref mobileMesh.worldTransform.Position, out triangle.vC);
                        //Put the triangle into 'localish' space of the convex.
                        Vector3.Subtract(ref triangle.vA, ref convex.worldTransform.Position, out triangle.vA);
                        Vector3.Subtract(ref triangle.vB, ref convex.worldTransform.Position, out triangle.vB);
                        Vector3.Subtract(ref triangle.vC, ref convex.worldTransform.Position, out triangle.vC);

                        RayHit rayHit;
                        if (GJKToolbox.CCDSphereCast(new Ray(Toolbox.ZeroVector, velocity), minimumRadius, triangle, ref Toolbox.RigidIdentity, timeOfImpact, out rayHit) &&
                            rayHit.T > Toolbox.BigEpsilon)
                        {

                            if (sidedness != TriangleSidedness.DoubleSided)
                            {
                                Vector3 AB, AC;
                                Vector3.Subtract(ref triangle.vB, ref triangle.vA, out AB);
                                Vector3.Subtract(ref triangle.vC, ref triangle.vA, out AC);
                                Vector3 normal;
                                Vector3.Cross(ref AB, ref AC, out normal);
                                float dot;
                                Vector3.Dot(ref normal, ref rayHit.Normal, out dot);
                                //Only perform sweep if the object is in danger of hitting the object.
                                //Triangles can be one sided, so check the impact normal against the triangle normal.
                                if (sidedness == TriangleSidedness.Counterclockwise && dot < 0 ||
                                    sidedness == TriangleSidedness.Clockwise && dot > 0)
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
                    Resources.GiveBack(triangle);
                }



            }

        }


        internal override void GetContactInformation(int index, out ContactInformation info)
        {
            info.Contact = contactManifold.contacts.Elements[index];
            //Find the contact's normal and friction forces.
            info.FrictionForce = 0;
            info.NormalForce = 0;
            for (int i = 0; i < contactConstraint.frictionConstraints.count; i++)
            {
                if (contactConstraint.frictionConstraints.Elements[i].PenetrationConstraint.contact == info.Contact)
                {
                    info.FrictionForce = contactConstraint.frictionConstraints.Elements[i].accumulatedImpulse;
                    info.NormalForce = contactConstraint.frictionConstraints.Elements[i].PenetrationConstraint.accumulatedImpulse;
                    break;
                }
            }

            //Compute relative velocity
            Vector3 velocity;
            Vector3.Subtract(ref info.Contact.Position, ref convex.entity.position, out velocity);
            Vector3.Cross(ref convex.entity.angularVelocity, ref velocity, out velocity);
            Vector3.Add(ref velocity, ref convex.entity.linearVelocity, out info.RelativeVelocity);
        }

        internal override int ContactCount
        {
            get { return contactManifold.contacts.count; }
        }

    }

}