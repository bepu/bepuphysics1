using System;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.CollisionTests;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using BEPUphysics.Constraints.Collision;
using BEPUphysics.PositionUpdating;
using BEPUphysics.Settings;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionTests.Manifolds;

namespace BEPUphysics.NarrowPhaseSystems.Pairs
{
    ///<summary>
    /// Handles a box and sphere in a collision.
    ///</summary>
    public class BoxSpherePairHandler : ConvexPairHandler
    {
        ConvexCollidable<BoxShape> box;
        ConvexCollidable<SphereShape> sphere;

        //Using a non-convex one since they have slightly lower overhead than their Convex friends when dealing with a single contact point.
        BoxSphereContactManifold contactManifold = new BoxSphereContactManifold();
        NonConvexContactManifoldConstraint contactConstraint = new NonConvexContactManifoldConstraint();

        protected override Collidable CollidableA
        {
            get { return box; }
        }

        protected override Collidable CollidableB
        {
            get { return sphere; }
        }

        protected override ContactManifoldConstraint ContactConstraint
        {
            get
            {
                return contactConstraint;
            }
        }

        protected override ContactManifold ContactManifold
        {
            get { return contactManifold; }
        }

        protected override Entities.Entity EntityA
        {
            get { return box.entity; }
        }

        protected override Entities.Entity EntityB
        {
            get { return sphere.entity; }
        }
        
        ///<summary>
        /// Initializes the pair handler.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        ///<param name="entryB">Second entry in the pair.</param>
        public override void Initialize(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {


            box = entryA as ConvexCollidable<BoxShape>;
            sphere = entryB as ConvexCollidable<SphereShape>;

            if (box == null || sphere == null)
            {
                box = entryB as ConvexCollidable<BoxShape>;
                sphere = entryA as ConvexCollidable<SphereShape>;
                if (box == null || sphere == null)
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

            box = null;
            sphere = null;

        }

    }

}
