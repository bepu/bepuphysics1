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
    public abstract class MobileMeshMeshPairHandler : GroupPairHandler
    {
        protected MobileMeshCollidable mesh;

        Dictionary<int, CollidablePairHandler> subPairs = new Dictionary<int, CollidablePairHandler>();
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

            base.Initialize(entryA, entryB);
        }

        ///<summary>
        /// Cleans up the pair handler.
        ///</summary>
        public override void CleanUp()
        {
            base.CleanUp();

            mesh = null;
            //Child type needs to null out other reference.
        }


        protected void TryToAdd(int triangleIndex, Collidable b)
        {
            if (!subPairs.ContainsKey(triangleIndex))
            {
                var a = new TriangleCollidable(new CollisionShapes.ConvexShapes.TriangleShape());
                a.entity = EntityB;
                CollidablePair pair = new CollidablePair(a, b);
                CollidablePairHandler newPair = NarrowPhaseHelper.GetPairHandler(ref pair, CollisionRule);
                if (newPair != null)
                {
                    newPair.Parent = this;
                    subPairs.Add(triangleIndex, newPair);
                }
            }
            containedPairs.Add(triangleIndex);

        }

        protected void UpdateTriangles()
        {
            foreach (var pair in subPairs.Values)
            {
            }
        }
    }
}
