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
    /// Handles a compound-terrain collision pair.
    ///</summary>
    public class CompoundTerrainPairHandler : CompoundGroupPairHandler
    {

        Terrain terrain;

        protected override Collidable CollidableB
        {
            get { return terrain; }
        }
        protected override Entities.Entity EntityB
        {
            get { return null; }
        }

        ///<summary>
        /// Initializes the pair handler.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        ///<param name="entryB">Second entry in the pair.</param>
        public override void Initialize(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {
            terrain = entryA as Terrain;
            if (terrain == null)
            {

                terrain = entryB as Terrain;
                if (terrain == null)
                {
                    throw new Exception("Inappropriate types used to initialize pair.");
                }
            }

            base.Initialize(entryA, entryB);
        }


        ///<summary>
        /// Cleans up the pair handler.
        ///</summary>
        public override void CleanUp()
        {

            base.CleanUp();
            terrain = null;


        }





        protected override void UpdateContainedPairs()
        {
            //TODO: Static triangle meshes have a worldspace hierarchy that could be more efficiently traversed with a tree vs tree test.
            //This is just a lot simpler to manage in the short term.

            var overlappedElements = Resources.GetCompoundChildList();
            compoundInfo.hierarchy.Tree.GetOverlaps(terrain.boundingBox, overlappedElements);
            for (int i = 0; i < overlappedElements.count; i++)
            {
                TryToAdd(overlappedElements.Elements[i].CollisionInformation, terrain);

            }

            Resources.GiveBack(overlappedElements);
        }



    }
}
