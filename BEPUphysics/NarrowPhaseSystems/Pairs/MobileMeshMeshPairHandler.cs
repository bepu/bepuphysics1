using System;
using System.Collections.Generic;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.Constraints;
using BEPUphysics.Constraints.Collision;
using BEPUphysics.DataStructures;
using BEPUphysics.ResourceManagement;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.CollisionTests;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Entities;
using Microsoft.Xna.Framework;

namespace BEPUphysics.NarrowPhaseSystems.Pairs
{

    ///<summary>
    /// Handles a compound and convex collision pair.
    ///</summary>
    public abstract class MobileMeshMeshPairHandler : CollidablePairHandler, IPairHandlerParent
    {
        protected class OpposingTriangle
        {
            internal TriangleShape shape;
            internal TriangleCollidable collidable;
            internal MobileMeshConvexPairHandler pairHandler;

            public OpposingTriangle()
            {
                shape = new TriangleShape();
                collidable = new TriangleCollidable(shape);
                pairHandler = new MobileMeshConvexPairHandler();
            }

            public void Initialize(MobileMeshMeshPairHandler owner, MobileMeshCollidable mesh, Entity entity)
            {
                collidable.entity = entity;
                pairHandler.Parent = owner;
                pairHandler.broadPhaseOverlap = new BroadPhaseOverlap(mesh, collidable, owner.CollisionRule);
                pairHandler.Initialize(mesh, collidable);

            }

            internal void CleanUp()
            {
                pairHandler.CleanUp();
                collidable.entity = null;
            }
        }


        ContactManifoldConstraintGroup manifoldConstraintGroup;

        protected MobileMeshCollidable mesh;

        Dictionary<int, OpposingTriangle> subPairs = new Dictionary<int, OpposingTriangle>();
        HashSet<int> containedPairs = new HashSet<int>();
        RawList<int> pairsToRemove = new RawList<int>();


        protected override Collidable CollidableA
        {
            get { return mesh; }
        }
        protected override Entities.Entity EntityA
        {
            get { return mesh.entity; }
        }

        protected MobileMeshMeshPairHandler()
        {
            manifoldConstraintGroup = new ContactManifoldConstraintGroup();
        }

        int contactCount;

        ///<summary>
        /// Forces an update of the pair's material properties.
        ///</summary>
        public override void UpdateMaterialProperties()
        {
            foreach (var pair in subPairs.Values)
            {
                pair.pairHandler.UpdateMaterialProperties();
            }
        }

        ///<summary>
        /// Initializes the pair handler.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        ///<param name="entryB">Second entry in the pair.</param>
        public override void Initialize(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {
            //Other member of the pair is initialized by the child.
            mesh = entryA as MobileMeshCollidable;
            if (mesh == null)
            {
                mesh = entryB as MobileMeshCollidable;
                if (mesh == null)
                {
                    throw new Exception("Inappropriate types used to initialize pair.");
                }
            }

            manifoldConstraintGroup.Initialize(EntityB, EntityA);

            base.Initialize(entryA, entryB);
        }


        ///<summary>
        /// Cleans up the pair handler.
        ///</summary>
        public override void CleanUp()
        { 
            //The pair handler cleanup will get rid of contacts.
            foreach (var pairHandler in subPairs.Values)
            {
                pairHandler.CleanUp();
            }
            subPairs.Clear();
            base.CleanUp();

            mesh = null;
            //Child type needs to null out other reference.
        }

        static LockingResourcePool<OpposingTriangle> opposingTriangles = new LockingResourcePool<OpposingTriangle>();

        protected void TryToAdd(int triangleIndex, Collidable b)
        {
            if (!subPairs.ContainsKey(triangleIndex))
            {
                var triangle = opposingTriangles.Take();
                triangle.Initialize(this, mesh, EntityB);
                subPairs.Add(triangleIndex, triangle);
            }
            containedPairs.Add(triangleIndex);

        }

        protected abstract void GetTriangleVertices(int i, out Vector3 vA, out Vector3 vB, out Vector3 vC);

        protected void UpdateTriangles()
        {
            foreach (int index in subPairs.Keys)
            {
                var tri = subPairs[index];
                var shape = tri.shape;
                GetTriangleVertices(index, out shape.vA, out shape.vB, out shape.vC);
                Vector3 center;
                Vector3.Add(ref shape.vA, ref shape.vB, out center);
                Vector3.Add(ref center, ref shape.vC, out center);
                Vector3.Divide(ref center, 3, out center);
                Vector3.Subtract(ref shape.vA, ref center, out shape.vA);
                Vector3.Subtract(ref shape.vB, ref center, out shape.vB);
                Vector3.Subtract(ref shape.vC, ref center, out shape.vC);
                tri.collidable.worldTransform.Position = center;

            }
        }

        protected abstract void UpdateContainedPairs();

        ///<summary>
        /// Updates the pair handler's contacts.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        protected virtual void UpdateContacts(float dt)
        {
            UpdateContainedPairs();
            //Eliminate old pairs.
            foreach (int pair in subPairs.Keys)
            {
                if (!containedPairs.Contains(pair))
                    pairsToRemove.Add(pair);
            }
            for (int i = 0; i < pairsToRemove.count; i++)
            {
                var toReturn = subPairs[pairsToRemove.Elements[i]];
                subPairs.Remove(pairsToRemove.Elements[i]);
                toReturn.CleanUp(); 
                opposingTriangles.GiveBack(toReturn);
            }
            containedPairs.Clear();
            pairsToRemove.Clear();


            UpdateTriangles();
            foreach (var pair in subPairs.Values)
            {
                pair.pairHandler.UpdateCollision(dt);
            }


        }


        ///<summary>
        /// Updates the pair handler.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        public override void UpdateCollision(float dt)
        {

            if (!suppressEvents)
            {
                CollidableA.EventTriggerer.OnPairUpdated(CollidableB, this);
                CollidableB.EventTriggerer.OnPairUpdated(CollidableA, this);
            }

            UpdateContacts(dt);


            if (contactCount > 0)
            {
                if (!suppressEvents)
                {
                    CollidableA.EventTriggerer.OnPairTouching(CollidableB, this);
                    CollidableB.EventTriggerer.OnPairTouching(CollidableA, this);
                }

                if (previousContactCount == 0)
                {
                    //collision started!
                    CollidableA.EventTriggerer.OnInitialCollisionDetected(CollidableB, this);
                    CollidableB.EventTriggerer.OnInitialCollisionDetected(CollidableA, this);

                    //No solver updateable addition in this method since it's handled by the "AddSolverUpdateable" method.
                }
            }
            else if (previousContactCount > 0 && !suppressEvents)
            {
                //collision ended!
                CollidableA.EventTriggerer.OnCollisionEnded(CollidableB, this);
                CollidableB.EventTriggerer.OnCollisionEnded(CollidableA, this);

                //No solver updateable removal in this method since it's handled by the "RemoveSolverUpdateable" method.
            }
            previousContactCount = contactCount;

        }

        ///<summary>
        /// Updates the time of impact for the pair.
        ///</summary>
        ///<param name="requester">Collidable requesting the update.</param>
        ///<param name="dt">Timestep duration.</param>
        public override void UpdateTimeOfImpact(Collidable requester, float dt)
        {
            timeOfImpact = 1;
            foreach (var pair in subPairs.Values)
            {
                //The system uses the identity of the requester to determine if it needs to do handle the TOI calculation.
                //Use the child pair's own entries as a proxy.
                if (mesh == requester)
                    pair.pairHandler.UpdateTimeOfImpact(mesh, dt);
                else
                    pair.pairHandler.UpdateTimeOfImpact(pair.collidable, dt);
                if (pair.pairHandler.timeOfImpact < timeOfImpact)
                    timeOfImpact = pair.pairHandler.timeOfImpact;
            }
        }


        internal override void GetContactInformation(int index, out ContactInformation info)
        {
            foreach (var pair in subPairs.Values)
            {
                int count = pair.pairHandler.Contacts.Count;
                if (index - count < 0)
                {
                    pair.pairHandler.GetContactInformation(index, out info);
                    return;
                }
                index -= count;
            }
            throw new IndexOutOfRangeException("Contact index is not present in the pair.");

        }


        /// <summary>
        /// Gets the number of contacts in the pair handler.
        /// </summary>
        public override int ContactCount
        {
            get { return contactCount; }
        }


        void IPairHandlerParent.AddSolverUpdateable(EntitySolverUpdateable addedItem)
        {

            manifoldConstraintGroup.Add(addedItem);
            //If this is the first child solver item to be added, we need to add ourselves to our parent too.
            if (manifoldConstraintGroup.SolverUpdateables.Count == 1)
            {
                if (Parent == null)
                {
                    NarrowPhase.EnqueueGeneratedSolverUpdateable(manifoldConstraintGroup);
                }
                else
                {
                    Parent.AddSolverUpdateable(manifoldConstraintGroup);
                }
            }

        }

        void IPairHandlerParent.RemoveSolverUpdateable(EntitySolverUpdateable removedItem)
        {

            manifoldConstraintGroup.Remove(removedItem);

            //If this is the last child solver item, we need to remove ourselves from our parent too.
            if (manifoldConstraintGroup.SolverUpdateables.Count == 0)
            {
                if (Parent == null)
                {
                    NarrowPhase.EnqueueRemovedSolverUpdateable(manifoldConstraintGroup);
                }
                else
                {
                    Parent.RemoveSolverUpdateable(manifoldConstraintGroup);
                }
            }


        }


        void IPairHandlerParent.OnContactAdded(Contact contact)
        {
            contactCount++;
            OnContactAdded(contact);
        }

        void IPairHandlerParent.OnContactRemoved(Contact contact)
        {
            contactCount--;
            OnContactRemoved(contact);
        }
    }
}
