using System;
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
using BEPUphysics.Entities;

namespace BEPUphysics.NarrowPhaseSystems.Pairs
{
    ///<summary>
    /// Handles a mobile mesh-mesh collision pair.
    ///</summary>
    public abstract class MobileMeshMeshPairHandler : CollidablePairHandler
    {
        MobileMeshCollidable mobileMesh;
        protected abstract Collidable OtherMesh { get; }
        protected abstract Entity OtherEntity { get; }

        MobileMeshConvexContactManifold contactManifold = new MobileMeshConvexContactManifold();
        NonConvexContactManifoldConstraint contactConstraint = new NonConvexContactManifoldConstraint();

        Action<Contact> contactAddedDelegate;
        Action<Contact> contactRemovedDelegate;

        ///<summary>
        /// Constructs a pair handler.
        ///</summary>
        public MobileMeshMeshPairHandler()
        {
            contactAddedDelegate = OnContactAdded;
            contactRemovedDelegate = OnContactRemoved;
        }

        void OnContactAdded(Contact contact)
        {
            contactConstraint.AddContact(contact);


            if (!suppressEvents)
            {
                mobileMesh.events.OnContactCreated(OtherMesh, this, contact);
                OtherMesh.EventTriggerer.OnContactCreated(mobileMesh, this, contact);
            }
            if (Parent != null)
                Parent.OnContactAdded(contact);

        }

        void OnContactRemoved(Contact contact)
        {
            contactConstraint.RemoveContact(contact);

            if (!suppressEvents)
            {
                mobileMesh.events.OnContactRemoved(OtherMesh, this, contact);
                OtherMesh.EventTriggerer.OnContactRemoved(mobileMesh, this, contact);
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
            //Child initialization is responsible for setting up the other entry.
            mobileMesh = entryA as MobileMeshCollidable;

            if (mobileMesh == null)
            {
                mobileMesh = entryB as MobileMeshCollidable;

                if (mobileMesh == null)
                    throw new Exception("Inappropriate types used to initialize pair.");
            }

            if (!suppressEvents)
            {
                mobileMesh.events.OnPairCreated(OtherMesh, this);
                OtherMesh.EventTriggerer.OnPairCreated(mobileMesh, this);
            }

            contactManifold.Initialize(mobileMesh, OtherMesh);
            contactManifold.ContactAdded += contactAddedDelegate;
            contactManifold.ContactRemoved += contactRemovedDelegate;


            contactConstraint.Initialize(OtherEntity, mobileMesh.entity, this);
            UpdateMaterialProperties();

        }

        ///<summary>
        /// Forces an update of the pair's material properties.
        ///</summary>
        public override void UpdateMaterialProperties()
        {
            contactConstraint.UpdateMaterialProperties(
              OtherEntity == null ? null : OtherEntity.material,
              mobileMesh.entity == null ? null : mobileMesh.entity.material);
        }

        ///<summary>
        /// Called when the pair handler is added to the narrow phase.
        ///</summary>
        public override void OnAddedToNarrowPhase()
        {
            mobileMesh.pairs.Add(this);
            OtherMesh.pairs.Add(this);
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
                mobileMesh.events.OnCollisionEnded(OtherMesh, this);
                OtherMesh.EventTriggerer.OnCollisionEnded(mobileMesh, this);
            }

            CollidableA.pairs.Remove(this);
            CollidableB.pairs.Remove(this);

            if (!suppressEvents)
            {
                CollidableA.EventTriggerer.OnPairRemoved(OtherMesh);
                CollidableB.EventTriggerer.OnPairRemoved(mobileMesh);
            }

            mobileMesh = null;
            //Child cleanup is responsible for cleaning up the other reference.

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
                mobileMesh.events.OnPairUpdated(OtherMesh, this);
                OtherMesh.EventTriggerer.OnPairUpdated(mobileMesh, this);
            }


            contactManifold.Update(dt);



            if (contactManifold.contacts.count > 0)
            {
                if (!suppressEvents)
                {
                    mobileMesh.events.OnPairTouching(OtherMesh, this);
                    OtherMesh.EventTriggerer.OnPairTouching(mobileMesh, this);
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
                        mobileMesh.events.OnInitialCollisionDetected(OtherMesh, this);
                        OtherMesh.EventTriggerer.OnInitialCollisionDetected(mobileMesh, this);
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
                    mobileMesh.events.OnCollisionEnded(OtherMesh, this);
                    OtherMesh.EventTriggerer.OnCollisionEnded(mobileMesh, this);
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
            timeOfImpact = 1;
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
            Vector3.Subtract(ref info.Contact.Position, ref OtherEntity.position, out velocity);
            Vector3.Cross(ref OtherEntity.angularVelocity, ref velocity, out velocity);
            Vector3.Add(ref velocity, ref OtherEntity.linearVelocity, out info.RelativeVelocity);
        }

        internal override int ContactCount
        {
            get { return contactManifold.contacts.count; }
        }

    }

}
