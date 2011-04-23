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

namespace BEPUphysics.NarrowPhaseSystems.Pairs
{
    ///<summary>
    /// Handles a compound and convex collision pair.
    ///</summary>
    public abstract class GroupPairHandler : CollidablePairHandler, IPairHandlerParent
    {
        ContactManifoldConstraintGroup manifoldConstraintGroup;
        
        Dictionary<CollidablePair, CollidablePairHandler> subPairs = new Dictionary<CollidablePair, CollidablePairHandler>();
        HashSet<CollidablePair> containedPairs = new HashSet<CollidablePair>();
        RawList<CollidablePair> pairsToRemove = new RawList<CollidablePair>();


        ///<summary>
        /// Gets a list of the pairs associated with children.
        ///</summary>
        public ReadOnlyDictionary<CollidablePair, CollidablePairHandler> ChildPairs
        {
            get;
            private set;
        }



        ///<summary>
        /// Constructs a new compound-convex pair handler.
        ///</summary>
        public GroupPairHandler()
        {
            manifoldConstraintGroup = new ContactManifoldConstraintGroup();
            ChildPairs = new ReadOnlyDictionary<CollidablePair, CollidablePairHandler>(subPairs);
        }




        ///<summary>
        /// Forces an update of the pair's material properties.
        ///</summary>
        public override void UpdateMaterialProperties()
        {
            foreach (CollidablePairHandler pairHandler in subPairs.Values)
            {
                pairHandler.UpdateMaterialProperties();
            }
        }

        ///<summary>
        /// Initializes the pair handler.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        ///<param name="entryB">Second entry in the pair.</param>
        public override void Initialize(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {

            //Child initialization is responsible for setting up the entries.
            //Child initialization is responsible for setting up the manifold, if any.
            manifoldConstraintGroup.Initialize(EntityA, EntityB);

            base.Initialize(entryA, entryB);
        }

        ///<summary>
        /// Cleans up the pair handler.
        ///</summary>
        public override void CleanUp()
        {

            //The pair handler cleanup will get rid of contacts.
            foreach (CollidablePairHandler pairHandler in subPairs.Values)
            {
                pairHandler.CleanUp();
            }
            subPairs.Clear();
            //don't need to remove constraints directly from our group, since cleaning up our children should get rid of them.


            base.CleanUp();

            //Child type needs to null out the references.
        }



        protected void TryToAdd(Collidable a, Collidable b)
        {
            CollisionRule rule;
            if ((rule = CollisionRules.collisionRuleCalculator(a.collisionRules, b.collisionRules)) < CollisionRule.NoNarrowPhasePair)
            {
                var pair = new CollidablePair(a, b);
                if (!subPairs.ContainsKey(pair))
                {
                    CollidablePairHandler newPair = NarrowPhaseHelper.GetPairHandler(ref pair, rule);
                    if (newPair != null)
                    {
                        newPair.Parent = this;
                        subPairs.Add(pair, newPair);
                    }
                }
                containedPairs.Add(pair);
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
            foreach (CollidablePair pair in subPairs.Keys)
            {
                if (!containedPairs.Contains(pair))
                    pairsToRemove.Add(pair);
            }
            for (int i = 0; i < pairsToRemove.count; i++)
            {
                CollidablePairHandler toReturn = subPairs[pairsToRemove.Elements[i]];
                subPairs.Remove(pairsToRemove.Elements[i]);
                toReturn.CleanUp();
                (toReturn as INarrowPhasePair).Factory.GiveBack(toReturn);

            }
            containedPairs.Clear();
            pairsToRemove.Clear();

            foreach (CollidablePairHandler pair in subPairs.Values)
            {
                if (pair.BroadPhaseOverlap.collisionRule < CollisionRule.NoNarrowPhaseUpdate) //Don't test if the collision rules say don't.
                    pair.UpdateCollision(dt);
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
            foreach (CollidablePairHandler pair in subPairs.Values)
            {
                //The system uses the identity of the requester to determine if it needs to do handle the TOI calculation.
                //Use the child pair's own entries as a proxy.
                if (BroadPhaseOverlap.entryA == requester)
                    pair.UpdateTimeOfImpact(pair.BroadPhaseOverlap.entryA as Collidable, dt);
                else
                    pair.UpdateTimeOfImpact(pair.BroadPhaseOverlap.entryB as Collidable, dt);
                if (pair.timeOfImpact < timeOfImpact)
                    timeOfImpact = pair.timeOfImpact;
            }
        }


        internal override void GetContactInformation(int index, out ContactInformation info)
        {
            foreach (CollidablePairHandler pair in subPairs.Values)
            {
                int count = pair.Contacts.Count;
                if (index - count < 0)
                {
                    pair.GetContactInformation(index, out info);
                    return;
                }
                index -= count;
            }
            throw new IndexOutOfRangeException("Contact index is not present in the pair.");

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




        int contactCount;
        public override int ContactCount
        {
            get { return contactCount; }
        }
    }
}
