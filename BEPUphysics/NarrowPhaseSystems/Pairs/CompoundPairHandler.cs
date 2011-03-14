using System;
using System.Collections.Generic;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.Constraints;
using BEPUphysics.Constraints.Collision;
using BEPUphysics.DataStructures;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.CollisionTests;

namespace BEPUphysics.NarrowPhaseSystems.Pairs
{
    ///<summary>
    /// Handles a compound-compound collision pair.
    ///</summary>
    public class CompoundPairHandler : CollidablePairHandler, IPairHandlerParent
    {
        ContactManifoldConstraintGroup manifoldConstraintGroup;

        CompoundCollidable compoundInfoA;

        CompoundCollidable compoundInfoB;



        Dictionary<CollidablePair, CollidablePairHandler> subPairs = new Dictionary<CollidablePair, CollidablePairHandler>();
        HashSet<CollidablePair> containedPairs = new HashSet<CollidablePair>();
        RawList<CollidablePair> pairsToRemove = new RawList<CollidablePair>();
        RawList<TreeOverlapPair<CompoundChild, CompoundChild>> overlappedElements = new RawList<TreeOverlapPair<CompoundChild, CompoundChild>>();

        ///<summary>
        /// Gets the pair handlers associated with children of the compound.
        ///</summary>
        public ReadOnlyDictionary<CollidablePair, CollidablePairHandler> ChildPairs
        {
            get;
            private set;
        }


        ///<summary>
        /// Constructs a new compound pair handler.
        ///</summary>
        public CompoundPairHandler()
        {
            manifoldConstraintGroup = new ContactManifoldConstraintGroup();
            ChildPairs = new ReadOnlyDictionary<CollidablePair, CollidablePairHandler>(subPairs);
        }


        ///<summary>
        /// Initializes the pair handler.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        ///<param name="entryB">Second entry in the pair.</param>
        public override void Initialize(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {
            compoundInfoA = entryA as CompoundCollidable;
            compoundInfoB = entryB as CompoundCollidable;

            if (compoundInfoA == null || compoundInfoB == null)
            {
                throw new Exception("Inappropriate types used to initialize pair.");
            }

            manifoldConstraintGroup.Initialize(compoundInfoA.entity, compoundInfoB.entity);

            if (!suppressEvents)
            {
                compoundInfoA.events.OnPairCreated(compoundInfoB, this);
                compoundInfoB.events.OnPairCreated(compoundInfoA, this);
            }
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
        /// Called when the pair handler is added to the narrow phase.
        ///</summary>
        public override void OnAddedToNarrowPhase()
        {
            compoundInfoA.pairs.Add(this);
            compoundInfoB.pairs.Add(this);
        }

        ///<summary>
        /// Cleans up the pair handler.
        ///</summary>
        public override void CleanUp()
        {

            previousContactCount = 0;

            //The pair handler cleanup will get rid of contacts.
            //If there are still any, we need to notify listeners that the collision is ending.
            //They will be notified after the contacts are removed.
            bool shouldNotify = contactCount > 0 && !suppressEvents;

            foreach (CollidablePairHandler pairHandler in subPairs.Values)
            {
                pairHandler.CleanUp();
            }
            subPairs.Clear();
            //don't need to remove constraints directly from our group, since cleaning up our children should get rid of them.
            //Don't need to remove our own group constraint either, since it gets auto-removed when the children constraints are removed.
            //manifoldConstraintGroup.CleanUp();

            if (shouldNotify)
            {
                compoundInfoA.events.OnCollisionEnded(compoundInfoB, this);
                compoundInfoB.events.OnCollisionEnded(compoundInfoA, this);
            }

            compoundInfoA.pairs.Remove(this);
            compoundInfoB.pairs.Remove(this);
            

            if (!suppressEvents)
            {
                compoundInfoA.events.OnPairRemoved(compoundInfoB);
                compoundInfoB.events.OnPairRemoved(compoundInfoA);
            }

            compoundInfoA = null;
            compoundInfoB = null;

            base.CleanUp();

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


        private void UpdatePairs()
        {


            compoundInfoA.hierarchy.Tree.GetOverlaps(compoundInfoB.hierarchy.Tree, overlappedElements);
            for (int i = 0; i < overlappedElements.count; i++)
            {
                CollidablePair pair = new CollidablePair(overlappedElements.Elements[i].OverlapA.CollisionInformation, overlappedElements.Elements[i].OverlapB.CollisionInformation);
                CollisionRule rule;
                if ((rule = CollisionRules.collisionRuleCalculator(pair.collidableA.collisionRules, pair.collidableB.collisionRules)) < CollisionRule.NoNarrowPhasePair)
                {
                    if (!subPairs.ContainsKey(pair))
                    {
                        CollidablePairHandler newPair = NarrowPhaseHelper.GetPairHandler(ref pair, rule);
                        newPair.Parent = this;
                        subPairs.Add(pair, newPair);
                    }
                    containedPairs.Add(pair);
                }
            }



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


            overlappedElements.Clear();
            containedPairs.Clear();
            pairsToRemove.Clear();
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
                compoundInfoA.events.OnPairUpdated(compoundInfoB, this);
                compoundInfoB.events.OnPairUpdated(compoundInfoA, this);
            }

            UpdatePairs();
            foreach (CollidablePairHandler pair in subPairs.Values)
            {
                if (pair.BroadPhaseOverlap.collisionRule < CollisionRule.NoNarrowPhaseUpdate) //Don't test if the collision rules say don't.
                    pair.UpdateCollision(dt);
            }


            if (contactCount > 0)
            {
                if (!suppressEvents)
                {
                    compoundInfoA.events.OnPairTouching(compoundInfoB, this);
                    compoundInfoB.events.OnPairTouching(compoundInfoA, this);
                }

                if (previousContactCount == 0)
                {
                    //collision started!
                    compoundInfoA.events.OnInitialCollisionDetected(compoundInfoB, this);
                    compoundInfoB.events.OnInitialCollisionDetected(compoundInfoA, this);

                    //No solver updateable addition in this method since it's handled by the "AddSolverUpdateable" method.
                }
            }
            else if (previousContactCount > 0 && !suppressEvents)
            {
                //collision ended!
                compoundInfoA.events.OnCollisionEnded(compoundInfoB, this);
                compoundInfoB.events.OnCollisionEnded(compoundInfoA, this);

                //No solver updateable removal in this method since it's handled by the "RemoveSolverUpdateable" method.
            }
            previousContactCount = contactCount;

        }






        void IPairHandlerParent.OnContactAdded(Contact contact)
        {
            contactCount++;
            if (!suppressEvents)
            {
                compoundInfoA.events.OnContactCreated(compoundInfoB, this, contact);
                compoundInfoB.events.OnContactCreated(compoundInfoA, this, contact);
            }
        }

        void IPairHandlerParent.OnContactRemoved(Contact contact)
        {
            contactCount--;
            if (!suppressEvents)
            {
                compoundInfoA.events.OnContactRemoved(compoundInfoB, this, contact);
                compoundInfoB.events.OnContactRemoved(compoundInfoA, this, contact);
            }
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

        int contactCount;
        internal override int ContactCount
        {
            get { return contactCount; }
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
            throw new IndexOutOfRangeException("Contact index is not present in the compound-compound pair.");

        }
    }
}
