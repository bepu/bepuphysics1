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
    /// Handles a sphere-sphere collision pair.
    ///</summary>
    public class SpherePairHandler : ConvexPairHandler
    {
        ConvexCollidable<SphereShape> sphereA;
        ConvexCollidable<SphereShape> sphereB;

        //Using a non-convex one since they have slightly lower overhead than their Convex friends when dealing with a single contact point.
        SphereContactManifold contactManifold = new SphereContactManifold();
        NonConvexContactManifoldConstraint contactConstraint = new NonConvexContactManifoldConstraint();

        protected override Collidable CollidableA
        {
            get { return sphereA; }
        }
        protected override Collidable CollidableB
        {
            get { return sphereB; }
        }
        protected override Entities.Entity EntityA
        {
            get { return sphereA.entity; }
        }
        protected override Entities.Entity EntityB
        {
            get { return sphereB.entity; }
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
        ///<summary>
        /// Initializes the pair handler.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        ///<param name="entryB">Second entry in the pair.</param>
        public override void Initialize(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {


            sphereA = entryA as ConvexCollidable<SphereShape>;
            sphereB = entryB as ConvexCollidable<SphereShape>;

            if (sphereA == null || sphereB == null)
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

            sphereA = null;
            sphereB = null;




        }



    }

}
