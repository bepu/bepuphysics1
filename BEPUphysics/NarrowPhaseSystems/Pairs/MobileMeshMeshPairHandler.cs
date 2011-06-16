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
    /// Handles a mobile mesh and mesh collision pair.
    ///</summary>
    public abstract class MobileMeshMeshPairHandler : MeshGroupPairHandler
    {
        protected MobileMeshCollidable mobileMesh;

        protected override Collidable CollidableA
        {
            get { return mobileMesh; }
        }
        protected override Entities.Entity EntityA
        {
            get { return mobileMesh.entity; }
        }
        protected override Materials.Material MaterialA
        {
            get { return mobileMesh.entity.material; }
        }


        ///<summary>
        /// Initializes the pair handler.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        ///<param name="entryB">Second entry in the pair.</param>
        public override void Initialize(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {
            //Other member of the pair is initialized by the child.
            mobileMesh = entryA as MobileMeshCollidable;
            if (mobileMesh == null)
            {
                mobileMesh = entryB as MobileMeshCollidable;
                if (mobileMesh == null)
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

            mobileMesh = null;
            //Child type needs to null out other reference.
        }



    }
}
