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
    public class CompoundPairHandler : CompoundGroupPairHandler
    {
        
        CompoundCollidable compoundInfoB;

        protected override Collidable CollidableB
        {
            get { return compoundInfoB; }
        }

        protected override Entities.Entity EntityB
        {
            get { return compoundInfoB.entity; }
        }


        ///<summary>
        /// Initializes the pair handler.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        ///<param name="entryB">Second entry in the pair.</param>
        public override void Initialize(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {
            compoundInfoB = entryB as CompoundCollidable;
            if (compoundInfoB == null)
            {
                throw new Exception("Inappropriate types used to initialize pair.");
            }

            base.Initialize(entryA, entryB);
        }


        ///<summary>
        /// Cleans up the pair handler.
        ///</summary>
        public override void CleanUp()
        {

            base.CleanUp();
            compoundInfoB = null;


        }

        RawList<TreeOverlapPair<CompoundChild, CompoundChild>> overlappedElements = new RawList<TreeOverlapPair<CompoundChild, CompoundChild>>();
        protected override void UpdateContainedPairs()
        {
            mesh.hierarchy.Tree.GetOverlaps(compoundInfoB.hierarchy.Tree, overlappedElements);
            for (int i = 0; i < overlappedElements.count; i++)
            {
                TryToAdd(overlappedElements.Elements[i].OverlapA.CollisionInformation, overlappedElements.Elements[i].OverlapB.CollisionInformation);
            }
            overlappedElements.Clear();
        }


    }
}
