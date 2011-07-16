using System;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.CollisionTests;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using BEPUphysics.CollisionTests.Manifolds;
using BEPUphysics.Constraints.Collision;
using BEPUphysics.PositionUpdating;
using BEPUphysics.Settings;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionShapes.ConvexShapes;

namespace BEPUphysics.NarrowPhaseSystems.Pairs
{
    ///<summary>
    /// Handles a triangle-convex collision pair.
    ///</summary>
    public class TriangleConvexPairHandler : ConvexConstraintPairHandler
    {
        ConvexCollidable<TriangleShape> triangle;
        ConvexCollidable convex;

        TriangleConvexContactManifold contactManifold = new TriangleConvexContactManifold();

        protected override Collidable CollidableA
        {
            get { return convex; }
        }
        protected override Collidable CollidableB
        {
            get { return triangle; }
        }

        protected override Entities.Entity EntityA
        {
            get { return convex.entity; }
        }
        protected override Entities.Entity EntityB
        {
            get { return triangle.entity; }
        }
        /// <summary>
        /// Gets the contact manifold used by the pair handler.
        /// </summary>
        public override ContactManifold ContactManifold
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

            triangle = entryA as ConvexCollidable<TriangleShape>;
            convex = entryB as ConvexCollidable;

            if (triangle == null || convex == null)
            {
                triangle = entryB as ConvexCollidable<TriangleShape>;
                convex = entryA as ConvexCollidable;

                if (triangle == null || convex == null)
                    throw new Exception("Inappropriate types used to initialize pair.");
            }

            //Contact normal goes from A to B.
            broadPhaseOverlap.entryA = convex;
            broadPhaseOverlap.entryB = triangle;

            base.Initialize(entryA, entryB);

        }

        ///<summary>
        /// Cleans up the pair handler.
        ///</summary>
        public override void CleanUp()
        {
            base.CleanUp();

            triangle = null;
            convex = null;

        }




        ///<summary>
        /// Updates the time of impact for the pair.
        ///</summary>
        ///<param name="requester">Collidable requesting the update.</param>
        ///<param name="dt">Timestep duration.</param>
        public override void UpdateTimeOfImpact(Collidable requester, float dt)
        {
            //TODO: This conditional early outing stuff could be pulled up into a common system, along with most of the pair handler.
            var overlap = BroadPhaseOverlap;
            if (
                    (overlap.entryA.IsActive || overlap.entryB.IsActive) && //At least one has to be active.
                    (
                        (
                            convex.entity.PositionUpdateMode == PositionUpdateMode.Continuous &&   //If both are continuous, only do the process for A.
                            triangle.entity.PositionUpdateMode == PositionUpdateMode.Continuous &&
                            overlap.entryA == requester
                        ) ||
                        (
                            convex.entity.PositionUpdateMode == PositionUpdateMode.Continuous ^   //If only one is continuous, then we must do it.
                            triangle.entity.PositionUpdateMode == PositionUpdateMode.Continuous
                        )
                    )
                )
            {





                //Only perform the test if the minimum radii are small enough relative to the size of the velocity.
                Vector3 velocity;
                Vector3.Subtract(ref triangle.entity.linearVelocity, ref convex.entity.linearVelocity, out velocity);
                Vector3.Multiply(ref velocity, dt, out velocity);
                float velocitySquared = velocity.LengthSquared();

                var minimumRadiusA = convex.Shape.minimumRadius * MotionSettings.CoreShapeScaling;
                timeOfImpact = 1;
                if (minimumRadiusA * minimumRadiusA < velocitySquared)
                {
                    //Spherecast A against B.
                    RayHit rayHit;
                    if (GJKToolbox.CCDSphereCast(new Ray(convex.worldTransform.Position, -velocity), minimumRadiusA, triangle.Shape, ref triangle.worldTransform, timeOfImpact, out rayHit))
                    {
                        if (triangle.Shape.sidedness != TriangleSidedness.DoubleSided)
                        {                
                            //Only perform sweep if the object is in danger of hitting the object.
                            //Triangles can be one sided, so check the impact normal against the triangle normal.
                            Vector3 AB, AC;
                            Vector3.Subtract(ref triangle.Shape.vB, ref triangle.Shape.vA, out AB);
                            Vector3.Subtract(ref triangle.Shape.vC, ref triangle.Shape.vA, out AC);
                            Vector3 normal;
                            Vector3.Cross(ref AB, ref AC, out normal);

                            float dot;
                            Vector3.Dot(ref rayHit.Normal, ref normal, out dot);
                            if (triangle.Shape.sidedness == TriangleSidedness.Counterclockwise && dot < 0 ||
                                triangle.Shape.sidedness == TriangleSidedness.Clockwise && dot > 0)
                            {
                                timeOfImpact = rayHit.T;
                            }
                        }
                        else
                        {
                            timeOfImpact = rayHit.T;
                        }
                    }
                }

                //TECHNICALLY, the triangle should be casted too.  But, given the way triangles are usually used and their tiny minimum radius, ignoring it is usually just fine.
                //var minimumRadiusB = triangle.minimumRadius * MotionSettings.CoreShapeScaling;
                //if (minimumRadiusB * minimumRadiusB < velocitySquared)
                //{
                //    //Spherecast B against A.
                //    RayHit rayHit;
                //    if (GJKToolbox.SphereCast(new Ray(triangle.entity.position, velocity), minimumRadiusB, convex.Shape, ref convex.worldTransform, 1, out rayHit) &&
                //        rayHit.T < timeOfImpact)
                //    {
                //        if (triangle.Shape.sidedness != TriangleSidedness.DoubleSided)
                //        {
                //            float dot;
                //            Vector3.Dot(ref rayHit.Normal, ref normal, out dot);
                //            if (dot > 0)
                //            {
                //                timeOfImpact = rayHit.T;
                //            }
                //        }
                //        else
                //        {
                //            timeOfImpact = rayHit.T;
                //        }
                //    }
                //}

                //If it's intersecting, throw our hands into the air and give up.
                //This is generally a perfectly acceptable thing to do, since it's either sitting
                //inside another object (no ccd makes sense) or we're still in an intersecting case
                //from a previous frame where CCD took place and a contact should have been created
                //to deal with interpenetrating velocity.  Sometimes that contact isn't sufficient,
                //but it's good enough.
                if (timeOfImpact == 0)
                    timeOfImpact = 1;
            }

        }


    }

}
