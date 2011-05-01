using System;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using Microsoft.Xna.Framework;

namespace BEPUphysics.CollisionTests.CollisionAlgorithms
{
    ///<summary>
    /// Tests convex shapes against other convex shapes for contact generation.
    ///</summary>
    public class GeneralConvexPairTester
    {
        //TODO: warmstarted calculations like those within this tester will carry over bad information if the shape of an object is changed.
        //Need to notify the system to take appropriate action when a shape changes...

        ///<summary>
        /// Whether or not to use simplex caching in general case convex-convex collisions.
        /// This will improve performance in simulations relying on the general case system, 
        /// but may decrease quality of behavior for curved shapes.
        ///</summary>
        public static bool UseSimplexCaching;
        private CollisionState state = CollisionState.Separated;

        Vector3 localSeparatingAxis;
        CachedSimplex cachedSimplex;

        protected internal ConvexCollidable collidableA;
        protected internal ConvexCollidable collidableB;

        ///<summary>
        /// Gets the first collidable in the pair.
        ///</summary>
        public ConvexCollidable CollidableA
        {
            get
            {
                return collidableA;
            }
        }
        ///<summary>
        /// Gets the second collidable in the pair.
        ///</summary>
        public ConvexCollidable CollidableB
        {
            get
            {
                return collidableB;
            }
        }


        ///<summary>
        /// Generates a contact between the objects, if possible.
        ///</summary>
        ///<param name="contact">Contact created between the pair, if possible.</param>
        ///<returns>Whether or not the objects were colliding.</returns>
        public bool GenerateContactCandidate(out ContactData contact)
        {
            //Generate contacts.  This will just find one closest point using general supportmapping based systems like MPR and GJK.

            //The collision system moves through a state machine depending on the latest collision generation result.
            //At first, assume that the pair is completely separating.  This is almost always the correct guess for new pairs.
            //An extremely fast, warm-startable boolean GJK test can be performed.  If it returns with nonintersection, we can quit and do nothing.
            //If the initial boolean GJK test finds intersection, move onto a shallow contact test.
            //The shallow contact test is a different kind of GJK test that finds the closest points between the shape pair.  It's not as speedy as the boolean version.
            //The algorithm is run between the marginless versions of the shapes, so that the closest points will form a contact somewhere in the space separating the cores.
            //If the closest point system finds no intersection and returns the closest points, the state is changed to ShallowContact.
            //If the closest point system finds intersection of the core shapes, then the state is changed to DeepContact, and MPR is run to determine contact information.
            //The system tries to escape from deep contact to shallow contact, and from shallow contact to separated whenever possible.

            //Here's the state flow:
            //On Separated: BooleanGJK
            //  -Intersecting -> Go to ShallowContact.
            //  -Nonintersecting -> Do nothing.
            //On ShallowContact: ClosestPointsGJK
            //  -Intersecting -> Go to DeepContact.
            //  -Nonintersecting: Go to Separated (without test) if squared distance > margin squared, otherwise use closest points to make contact.
            //On DeepContact: MPR
            //  -Intersecting -> Go to ShallowContact if penetration depth < margin
            //  -Nonintersecting -> This case is rare, but not impossible.  Go to Separated (without test).

            switch (state)
            {
                case CollisionState.Separated:
                    if (GJKToolbox.AreShapesIntersecting(collidableA.Shape, collidableB.Shape, ref collidableA.worldTransform, ref collidableB.worldTransform, ref localSeparatingAxis))
                    {
                        state = CollisionState.ShallowContact;
                        return DoShallowContact(out contact);
                    }
                    contact = new ContactData();
                    return false;
                case CollisionState.ShallowContact:
                    return DoShallowContact(out contact);
                case CollisionState.DeepContact:
                    return DoDeepContact(out contact);
            }
            contact = new ContactData();
            return false;
        }

        private bool DoShallowContact(out ContactData contact)
        {
            Vector3 closestA, closestB;

            //RigidTransform transform = RigidTransform.Identity;
            //Vector3 closestAnew, closestBnew;
            //CachedSimplex cachedTest = cachedSimplex;
            //bool intersecting = GJKToolbox.GetClosestPoints(informationA.Shape, informationB.Shape, ref informationA.worldTransform, ref informationB.worldTransform, ref cachedTest, out closestAnew, out closestBnew);

            ////bool otherIntersecting = OldGJKVerifier.GetClosestPointsBetweenObjects(informationA.Shape, informationB.Shape, ref informationA.worldTransform, ref informationB.worldTransform, 0, 0, out closestA, out closestB);
            //bool otherIntersecting = GJKToolbox.GetClosestPoints(informationA.Shape, informationB.Shape, ref informationA.worldTransform, ref informationB.worldTransform, out closestA, out closestB);

            //Vector3 closestAold, closestBold;
            //bool oldIntersecting = OldGJKVerifier.GetClosestPointsBetweenObjects(informationA.Shape, informationB.Shape, ref informationA.worldTransform, ref informationB.worldTransform, 0, 0, out closestAold, out closestBold);

            //if (otherIntersecting != intersecting || (!otherIntersecting && !intersecting &&
            //    Vector3.DistanceSquared(closestAnew, closestBnew) - Vector3.DistanceSquared(closestA, closestB) > .0001f &&
            //    (Vector3.DistanceSquared(closestA, closestAnew) > .0001f ||
            //    Vector3.DistanceSquared(closestB, closestBnew) > .0001f)))// ||
            //    //Math.Abs(Vector3.Dot(closestB - closestA, closestBnew - closestAnew) - Vector3.Dot(closestB - closestA, closestB - closestA)) > Toolbox.Epsilon)))
            //    Debug.WriteLine("Break.");

            //Vector3 sub;
            //Vector3.Subtract(ref closestA, ref closestB, out sub);
            //if (sub.LengthSquared() < Toolbox.Epsilon)

            bool intersecting;
            if (UseSimplexCaching)
                intersecting = GJKToolbox.GetClosestPoints(collidableA.Shape, collidableB.Shape, ref collidableA.worldTransform, ref collidableB.worldTransform, ref cachedSimplex, out closestA, out closestB);
            else
            {
                //The initialization of the pair creates a pretty decent simplex to start from.
                //Just don't try to update it.
                CachedSimplex preInitializedSimplex = cachedSimplex;
                intersecting = GJKToolbox.GetClosestPoints(collidableA.Shape, collidableB.Shape, ref collidableA.worldTransform, ref collidableB.worldTransform, ref preInitializedSimplex, out closestA, out closestB);
            }
            if (intersecting)
            //if (OldGJKVerifier.GetClosestPointsBetweenObjects(informationA.Shape, informationB.Shape, ref informationA.worldTransform, ref informationB.worldTransform, 0, 0, out closestA, out closestB))
            {
                state = CollisionState.DeepContact;
                return DoDeepContact(out contact);
            }
            Vector3 displacement;
            Vector3.Subtract(ref closestB, ref closestA, out displacement);
            float distanceSquared = displacement.LengthSquared();
            float margin = collidableA.Shape.collisionMargin + collidableB.Shape.collisionMargin;


            if (distanceSquared < margin * margin)
            {
                //Generate a contact.
                contact = new ContactData();
                //Displacement is from A to B.  point = A + t * AB, where t = marginA / margin.
                if (margin > Toolbox.Epsilon) //Avoid a NaN!
                    Vector3.Multiply(ref displacement, collidableA.Shape.collisionMargin / margin, out contact.Position); //t * AB
                else
                    contact.Position = new Vector3();
                    
                Vector3.Add(ref closestA, ref contact.Position, out contact.Position); //A + t * AB.


                contact.Normal = displacement;
                float distance = (float)Math.Sqrt(distanceSquared);
                Vector3.Divide(ref contact.Normal, distance, out contact.Normal);
                contact.PenetrationDepth = margin - distance;
                return true;

            }
            //Too shallow to make a contact- move back to separation.
            state = CollisionState.Separated;
            contact = new ContactData();
            return false;
        }

        private bool DoDeepContact(out ContactData contact)
        {
            if (MPRToolbox.AreObjectsColliding(collidableA.Shape, collidableB.Shape, ref collidableA.worldTransform, ref collidableB.worldTransform, out contact))
            {
                if (contact.PenetrationDepth < collidableA.Shape.collisionMargin + collidableB.Shape.collisionMargin)
                    state = CollisionState.ShallowContact; //If it's emerged from the deep contact, we can go back to using the preferred GJK method.
                return true;
            }
            //This is rare, but could happen.
            state = CollisionState.Separated;
            return false;

        }

        ///<summary>
        /// Initializes the pair tester.
        ///</summary>
        ///<param name="shapeA">First shape in the pair.</param>
        ///<param name="shapeB">Second shape in the pair.</param>
        public void Initialize(Collidable shapeA, Collidable shapeB)
        {
            collidableA = shapeA as ConvexCollidable;
            collidableB = shapeB as ConvexCollidable;
            cachedSimplex = new CachedSimplex {State = SimplexState.Point};// new CachedSimplex(informationA.Shape, informationB.Shape, ref informationA.worldTransform, ref informationB.worldTransform);
        }

        ///<summary>
        /// Cleans up the pair tester.
        ///</summary>
        public void CleanUp()
        {
            state = CollisionState.Separated;
            cachedSimplex = new CachedSimplex();
            localSeparatingAxis = new Vector3();
            collidableA = null;
            collidableB = null;
        }


        enum CollisionState
        {
            Separated,
            ShallowContact,
            DeepContact
        }


    }

}
