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
using Microsoft.Xna.Framework;

namespace BEPUphysics.NarrowPhaseSystems.Pairs
{
    ///<summary>
    /// Handles a mobile mesh-static mesh collision pair.
    ///</summary>
    public class MobileMeshStaticMeshPairHandler : MobileMeshMeshPairHandler
    {

        StaticMesh staticMesh;

        protected override Collidable CollidableB
        {
            get { return staticMesh; }
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
            staticMesh = entryA as StaticMesh;
            if (staticMesh == null)
            {
                staticMesh = entryB as StaticMesh;
                if (staticMesh == null)
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
            staticMesh = null;


        }




        protected override void UpdateContainedPairs()
        {
            var overlappedElements = Resources.GetIntList();
            staticMesh.Mesh.Tree.GetOverlaps(mesh.boundingBox, overlappedElements);
            for (int i = 0; i < overlappedElements.Count; i++)
            {
                TryToAdd(overlappedElements[i], staticMesh);
            }

            Resources.GiveBack(overlappedElements);

        }

        protected override void GetTriangleVertices(int i, out Vector3 vA, out Vector3 vB, out Vector3 vC)
        {
            staticMesh.Shape.TriangleMeshData.GetTriangle(i, out vA, out vB, out vC);
        }


    }
}
