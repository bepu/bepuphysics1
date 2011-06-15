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
    /// Handles a compound-static mesh collision pair.
    ///</summary>
    public class MobileMeshStaticMeshPairHandler : MobileMeshMeshPairHandler
    {


        StaticMesh mesh;

        protected override Collidable CollidableB
        {
            get { return mesh; }
        }
        protected override Entities.Entity EntityB
        {
            get { return null; }
        }

        protected override TriangleCollidable GetOpposingCollidable(int index)
        {
            //Construct a TriangleCollidable from the static mesh.
            var toReturn = Resources.GetTriangleCollidable();
            toReturn.Shape.sidedness = mesh.sidedness;
            toReturn.Shape.collisionMargin = mobileMesh.Shape.MeshCollisionMargin;
            return toReturn;
        }

        protected override void ConfigureCollidable(TriangleEntry entry)
        {
            var shape = entry.Collidable.Shape;
            mesh.Mesh.Data.GetTriangle(entry.Index, out shape.vA, out shape.vB, out shape.vC);

        }

        ///<summary>
        /// Initializes the pair handler.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        ///<param name="entryB">Second entry in the pair.</param>
        public override void Initialize(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {
            mesh = entryA as StaticMesh;
            if (mesh == null)
            {
                mesh = entryB as StaticMesh;
                if (mesh == null)
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
            mesh = null;


        }




        protected override void UpdateContainedPairs()
        {
            //TODO: Static triangle meshes have a worldspace hierarchy that could be more efficiently traversed with a tree vs tree test.
            //This is just a lot simpler to manage in the short term.

            var overlappedElements = Resources.GetIntList();
            mesh.Mesh.Tree.GetOverlaps(mobileMesh.boundingBox, overlappedElements);
            for (int i = 0; i < overlappedElements.count; i++)
            {
                TryToAdd(overlappedElements.Elements[i]);
            }

            Resources.GiveBack(overlappedElements);

        }


    }
}
