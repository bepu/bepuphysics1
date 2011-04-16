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
    public abstract class CompoundGroupPairHandler : GroupPairHandler
    {
        ContactManifoldConstraintGroup manifoldConstraintGroup;

        protected CompoundCollidable compoundInfo;

        Dictionary<CollidablePair, CollidablePairHandler> subPairs = new Dictionary<CollidablePair, CollidablePairHandler>();
        HashSet<CollidablePair> containedPairs = new HashSet<CollidablePair>();
        RawList<CollidablePair> pairsToRemove = new RawList<CollidablePair>();

        protected override Collidable CollidableA
        {
            get { return compoundInfo; }
        }
        protected override Entities.Entity EntityA
        {
            get { return compoundInfo.entity; }
        }

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
        public CompoundGroupPairHandler()
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
            //Other member of the pair is initialized by the child.
            compoundInfo = entryA as CompoundCollidable;
            if (compoundInfo == null)
            {
                compoundInfo = entryB as CompoundCollidable;
                if (compoundInfo == null)
                {
                    throw new Exception("Inappropriate types used to initialize pair.");
                }
            }

            base.Initialize(entryA, entryB);
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

            compoundInfo = null;
            //Child type needs to null out other reference.
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
        protected override void UpdateContacts(float dt)
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
    }
}
