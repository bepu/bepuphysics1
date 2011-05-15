using System;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.MathExtensions;
using BEPUphysics.Settings;
using BEPUphysics.DataStructures;

namespace BEPUphysics.CollisionTests.CollisionAlgorithms
{
    ///<summary>
    /// Persistent tester that compares triangles against convex objects.
    ///</summary>
    public class TriangleConvexPairTester
    {
        internal TriangleShape triangle;
        internal ConvexShape convex;

        internal CollisionState state = CollisionState.Plane;
        private const int EscapeAttemptPeriod = 10;
        int escapeAttempts;
        ///<summary>
        /// Whether or not the pair tester was updated during the last attempt.
        ///</summary>
        public bool Updated;

        Vector3 localSeparatingAxis;

        //Relies on the triangle being located in the local space of the convex object.  The convex transform is used to transform the
        //contact points back from the convex's local space into world space.
        ///<summary>
        /// Generates a contact between the triangle and convex.
        ///</summary>
        ///<param name="contactList">Contact between the shapes, if any.</param>
        ///<returns>Whether or not the shapes are colliding.</returns>
        public bool GenerateContactCandidate(out TinyStructList<ContactData> contactList)
        {
            switch (state)
            {
                case CollisionState.Plane:
                    return DoPlaneTest(out contactList);
                case CollisionState.ExternalSeparated:
                    return DoExternalSeparated(out contactList);
                case CollisionState.ExternalNear:
                    return DoExternalNear(out contactList);
                case CollisionState.Deep:
                    return DoDeepContact(out contactList);
                default:
                    contactList = new TinyStructList<ContactData>();
                    return false;
            }



        }


        private bool DoPlaneTest(out TinyStructList<ContactData> contactList)
        {


            //Find closest point between object and plane.
            Vector3 reverseNormal;
            Vector3 ab, ac;
            Vector3.Subtract(ref triangle.vB, ref triangle.vA, out ab);
            Vector3.Subtract(ref triangle.vC, ref triangle.vA, out ac);
            Vector3.Cross(ref ac, ref ab, out reverseNormal);
            //Convex position dot normal is ALWAYS zero.  The thing to look at is the plane's 'd'.
            //If the distance along the normal is positive, then the convex is 'behind' that normal.
            float dotA;
            Vector3.Dot(ref triangle.vA, ref reverseNormal, out dotA);

            contactList = new TinyStructList<ContactData>();
            switch (triangle.sidedness)
            {
                case TriangleSidedness.DoubleSided:
                    if (dotA < 0)
                    {
                        //The reverse normal is pointing towards the convex.
                        //It needs to point away from the convex so that the direction
                        //will get the proper extreme point.
                        Vector3.Negate(ref reverseNormal, out reverseNormal);
                        dotA = -dotA;
                    }
                    break;
                case TriangleSidedness.Clockwise:
                    if (dotA < 0)
                    {
                        //The reverse normal is pointing towards the convex.
                        return false;
                    }
                    break;
                case TriangleSidedness.Counterclockwise:
                    if (dotA > 0)
                    {
                        //The reverse normal is pointing away from the convex.
                        return false;
                    }

                    //The reverse normal is pointing towards the convex.
                    //It needs to point away from the convex so that the direction
                    //will get the proper extreme point.
                    Vector3.Negate(ref reverseNormal, out reverseNormal);
                    dotA = -dotA;
                    break;
            }
            Vector3 extremePoint;
            convex.GetLocalExtremePointWithoutMargin(ref reverseNormal, out extremePoint);


            //See if the extreme point is within the face or not.
            //It might seem like the easy "depth" test should come first, since a barycentric
            //calculation takes a bit more time.  However, transferring from plane to depth is 'rare' 
            //(like all transitions), and putting this test here is logically closer to its requirements'
            //computation.

            if (GetVoronoiRegion(ref extremePoint) != VoronoiRegion.ABC)
            {
                state = CollisionState.ExternalSeparated;
                return DoExternalSeparated(out contactList);
            }



            float dotE;
            Vector3.Dot(ref extremePoint, ref reverseNormal, out dotE);
            float t = (dotA - dotE) / reverseNormal.LengthSquared();



            Vector3 offset;
            Vector3.Multiply(ref reverseNormal, t, out offset);

            //Compare the distance from the plane to the convex object.
            float distanceSquared = offset.LengthSquared();

            float marginSum = triangle.collisionMargin + convex.collisionMargin;
            //TODO: Could just normalize early and avoid computing point plane before it's necessary.  
            //Exposes a sqrt but...
            if (t <= 0 || distanceSquared < marginSum * marginSum)
            {
                //The convex object is in the margin of the plane.
                //All that's left is to create the contact.


                var contact = new ContactData();
                //Displacement is from A to B.  point = A + t * AB, where t = marginA / margin.
                if (marginSum > Toolbox.Epsilon) //This can be zero! It would cause a NaN is unprotected.
                    Vector3.Multiply(ref offset, convex.collisionMargin / marginSum, out contact.Position); //t * AB
                else contact.Position = new Vector3();
                Vector3.Add(ref extremePoint, ref contact.Position, out contact.Position); //A + t * AB.

                float normalLength = reverseNormal.Length();
                Vector3.Divide(ref reverseNormal, normalLength, out contact.Normal);
                float distance = normalLength * t;



                contact.PenetrationDepth = marginSum - distance;

                if (contact.PenetrationDepth > marginSum)
                {
                    //Check to see if the inner sphere is touching the plane.
                    //This does not override other tests; there can be more than one contact from a single triangle.

                    ContactData alternateContact;
                    if (TryInnerSphereContact(out alternateContact))// && alternateContact.PenetrationDepth > contact.PenetrationDepth)
                    {
                        contactList.Add(ref alternateContact);
                    }

                    //The convex object is stuck deep in the plane!
                    //The most problematic case for this is when
                    //an object is right on top of a cliff.
                    //The lower, vertical triangle may occasionally detect
                    //a contact with the object, but would compute an extremely
                    //deep depth if the normal plane test was used.

                    //Verify that the depth is correct by trying another approach.
                    CollisionState previousState = state;
                    state = CollisionState.ExternalNear;
                    TinyStructList<ContactData> alternateContacts;
                    if (DoExternalNear(out alternateContacts))
                    {
                        alternateContacts.Get(0, out alternateContact);
                        if (alternateContact.PenetrationDepth + .01f < contact.PenetrationDepth) //Bias against the subtest's result, since the plane version will probably have a better position.
                        {
                            //It WAS a bad contact.
                            contactList.Add(ref alternateContact);
                            //DoDeepContact (which can be called from within DoExternalNear) can generate two contacts, but the second contact would just be an inner sphere (which we already generated).
                            //DoExternalNear can only generate one contact.  So we only need the first contact!
                            //TODO: This is a fairly fragile connection between the two stages.  Consider robustifying. (Also, the TryInnerSphereContact is done twice! This process is very rare for marginful pairs, though)
                        }
                        else
                        {
                            //Well, it really is just that deep.
                            contactList.Add(ref contact);
                            state = previousState;
                        }
                    }
                    else
                    {
                        //If the external near test finds that there was no collision at all, 
                        //just return to plane testing.  If the point turns up outside the face region
                        //next time, the system will adapt.
                        state = previousState;
                        return false;
                    }
                }
                else
                {
                    contactList.Add(ref contact);
                }
                return true;

            }
            return false;


        }

        public enum VoronoiRegion
        {
            A,
            B,
            C,
            AB,
            AC,
            BC,
            ABC
        }



        private bool DoExternalSeparated(out TinyStructList<ContactData> contactList)
        {

            if (GJKToolbox.AreShapesIntersecting(convex, triangle, ref Toolbox.RigidIdentity, ref Toolbox.RigidIdentity, ref localSeparatingAxis))
            {
                state = CollisionState.ExternalNear;
                return DoExternalNear(out contactList);
            }
            TryToEscape();
            contactList = new TinyStructList<ContactData>();
            return false;
        }

        private bool DoExternalNear(out TinyStructList<ContactData> contactList)
        {

            Vector3 closestA, closestB;


            //Don't bother trying to do any clever caching.  The continually transforming simplex makes it very rarely useful.
            //TODO: Initialize the simplex of the GJK method using the 'true' center of the triangle.
            //If left unmodified, the simplex that is used in GJK will just be a point at 0,0,0, which of course is at the origin.
            //This causes an instant-out, always.  Not good!
            //By giving the contributing simplex the average centroid, it has a better guess.
            Vector3 triangleCentroid;
            Vector3.Add(ref triangle.vA, ref triangle.vB, out triangleCentroid);
            Vector3.Add(ref triangleCentroid, ref triangle.vC, out triangleCentroid);
            Vector3.Multiply(ref triangleCentroid, .33333333f, out triangleCentroid);

            var initialSimplex = new CachedSimplex();
            initialSimplex.State = SimplexState.Point;
            initialSimplex.LocalSimplexB.A = triangleCentroid;
            if (GJKToolbox.GetClosestPoints(convex, triangle, ref Toolbox.RigidIdentity, ref Toolbox.RigidIdentity, ref initialSimplex, out closestA, out closestB))
            {
                state = CollisionState.Deep;
                return DoDeepContact(out contactList);
            }
            Vector3 displacement;
            Vector3.Subtract(ref closestB, ref closestA, out displacement);
            float distanceSquared = displacement.LengthSquared();
            float margin = convex.collisionMargin + triangle.collisionMargin;

            contactList = new TinyStructList<ContactData>();
            if (distanceSquared < margin * margin)
            {
                //Try to generate a contact.
                var contact = new ContactData();

                //Determine if the normal points in the appropriate direction given the sidedness of the triangle.
                if (triangle.sidedness != TriangleSidedness.DoubleSided)
                {
                    Vector3 triangleNormal, ab, ac;
                    Vector3.Subtract(ref triangle.vB, ref triangle.vA, out ab);
                    Vector3.Subtract(ref triangle.vC, ref triangle.vA, out ac);
                    Vector3.Cross(ref ab, ref ac, out triangleNormal);
                    float dot;
                    Vector3.Dot(ref triangleNormal, ref displacement, out dot);
                    if (triangle.sidedness == TriangleSidedness.Clockwise && dot > 0)
                        return false;
                    if (triangle.sidedness == TriangleSidedness.Counterclockwise && dot < 0)
                        return false;
                }


                //Displacement is from A to B.  point = A + t * AB, where t = marginA / margin.
                if (margin > Toolbox.Epsilon) //This can be zero! It would cause a NaN if unprotected.
                    Vector3.Multiply(ref displacement, convex.collisionMargin / margin, out contact.Position); //t * AB
                else contact.Position = new Vector3();
                Vector3.Add(ref closestA, ref contact.Position, out contact.Position); //A + t * AB.



                contact.Normal = displacement;
                float distance = (float)Math.Sqrt(distanceSquared);
                Vector3.Divide(ref contact.Normal, distance, out contact.Normal);
                contact.PenetrationDepth = margin - distance;



                contactList.Add(ref contact);
                TryToEscape(ref contact.Position);
                return true;

            }
            //Too far to make a contact- move back to separation.
            state = CollisionState.ExternalSeparated;
            return false;
        }

        private bool DoDeepContact(out TinyStructList<ContactData> contactList)
        {

            //Vector3 triangleNormal, ab, ac;
            //Vector3.Subtract(ref triangle.vB, ref triangle.vA, out ab);
            //Vector3.Subtract(ref triangle.vC, ref triangle.vA, out ac);
            //Vector3.Cross(ref ab, ref ac, out triangleNormal);
            //float dot;
            //if (previousState == CollisionState.ExternalSeparated || previousState == CollisionState.Plane) //If it was shallow before, then its closest points will be used to find the normal.
            //{

            //    //Instead of using relative velocity as a heuristic, use the triangle's normal.  Calibrate it to point the correct direction.
            //    //The backup direction of A-B tends to pick directions that point outward, so the normal will almost always be a good perpendicular option.
            //    //If ExternalNear runs immediately before, though, it may be wiser to just use its separating axis instead.

            //    //The calibrated direction can't just be based on the center position of the object, though.
            //    //It needs to be compared against 
            //    Vector3.Dot(ref triangleNormal, ref triangle.vA, out dot);
            //    if (dot < 0)
            //        Vector3.Negate(ref triangleNormal, out localDirection);
            //    else
            //        localDirection = triangleNormal;
            //}
            //if (localDirection.LengthSquared() < Toolbox.Epsilon)
            //    localDirection = Vector3.Up;

            //Vector3 center;
            //Vector3.Add(ref triangle.vA, ref triangle.vB, out center);
            //Vector3.Add(ref center, ref triangle.vC, out center);
            //Vector3.Divide(ref center, 3, out center);


            //if (MPRTesting.GetContact(convex, triangle, ref Toolbox.RigidIdentity, ref Toolbox.RigidIdentity, ref center, ref localDirection, out contact))
            //{
            //    //Determine if the normal points in the appropriate direction given the sidedness of the triangle.
            //    if (triangle.sidedness != TriangleSidedness.DoubleSided)
            //    {
            //        Vector3.Dot(ref triangleNormal, ref contact.Normal, out dot);
            //        if (triangle.sidedness == TriangleSidedness.Clockwise && dot > 0)
            //            return false;
            //        if (triangle.sidedness == TriangleSidedness.Counterclockwise && dot < 0)
            //            return false;
            //    }

            //    if (contact.PenetrationDepth < convex.collisionMargin + triangle.collisionMargin)
            //        state = CollisionState.ExternalNear;
            //    return true;
            //}
            ////This is rare, but could happen.
            //state = CollisionState.ExternalSeparated;
            //return false;

            //Find the origin to triangle center offset.
            Vector3 center;
            Vector3.Add(ref triangle.vA, ref triangle.vB, out center);
            Vector3.Add(ref center, ref triangle.vC, out center);
            Vector3.Divide(ref center, 3, out center);

            ContactData contact;

            contactList = new TinyStructList<ContactData>();

            if (MPRTesting.GetLocalOverlapPosition(convex, triangle, ref center, ref Toolbox.RigidIdentity, out contact.Position))
            {

                float dot;


                Vector3 triangleNormal, ab, ac;
                Vector3.Subtract(ref triangle.vB, ref triangle.vA, out ab);
                Vector3.Subtract(ref triangle.vC, ref triangle.vA, out ac);
                Vector3.Cross(ref ab, ref ac, out triangleNormal);
                triangleNormal.Normalize();

                //Project the direction onto the triangle plane.
                Vector3.Dot(ref triangleNormal, ref center, out dot);
                Vector3 trianglePlaneDirection;
                Vector3.Multiply(ref triangleNormal, dot, out trianglePlaneDirection);
                Vector3.Subtract(ref center, ref trianglePlaneDirection, out trianglePlaneDirection);
                dot = trianglePlaneDirection.LengthSquared();
                if (dot > Toolbox.Epsilon)
                {
                    Vector3.Divide(ref trianglePlaneDirection, (float)Math.Sqrt(dot), out trianglePlaneDirection);
                    MPRTesting.LocalSurfaceCast(convex, triangle, ref Toolbox.RigidIdentity, ref trianglePlaneDirection, out contact.PenetrationDepth, out contact.Normal);
                    //Check to see if the normal is facing in the proper direction, considering that this may not be a two-sided triangle.
                    Vector3.Dot(ref triangleNormal, ref contact.Normal, out dot);
                    if ((triangle.sidedness == TriangleSidedness.Clockwise && dot > 0) || (triangle.sidedness == TriangleSidedness.Counterclockwise && dot < 0))
                    {
                        //Normal was facing the wrong way.
                        contact.PenetrationDepth = float.MaxValue;
                        contact.Normal = new Vector3();
                    }
                }
                else
                {
                    contact.PenetrationDepth = float.MaxValue;
                    contact.Normal = new Vector3();
                }


                //Try the depth along the positive triangle normal.

                Vector3 candidateNormal;
                float candidateDepth;
                //If it's clockwise, this direction is unnecessary (the resulting normal would be invalidated by the onesidedness of the triangle).
                if (triangle.sidedness != TriangleSidedness.Clockwise)
                {
                    MPRTesting.LocalSurfaceCast(convex, triangle, ref Toolbox.RigidIdentity, ref triangleNormal, out candidateDepth, out candidateNormal);
                    if (candidateDepth < contact.PenetrationDepth)
                    {
                        contact.Normal = candidateNormal;
                        contact.PenetrationDepth = candidateDepth;
                    }
                }

                //Try the depth along the negative triangle normal.

                //If it's counterclockwise, this direction is unnecessary (the resulting normal would be invalidated by the onesidedness of the triangle).
                if (triangle.sidedness != TriangleSidedness.Counterclockwise)
                {
                    Vector3.Negate(ref triangleNormal, out triangleNormal);
                    MPRTesting.LocalSurfaceCast(convex, triangle, ref Toolbox.RigidIdentity, ref triangleNormal, out candidateDepth, out candidateNormal);
                    if (candidateDepth < contact.PenetrationDepth)
                    {
                        contact.Normal = candidateNormal;
                        contact.PenetrationDepth = candidateDepth;
                    }
                }







                //Correct the penetration depth.
                MPRTesting.LocalSurfaceCast(convex, triangle, ref Toolbox.RigidIdentity, ref contact.Normal, out contact.PenetrationDepth, out center); //Center is just a trash variable now.


                ////The local casting can optionally continue.  Eventually, it will converge to the local minimum.
                //while (true)
                //{
                //    MPRTesting.LocalSurfaceCast(collidableA.Shape, collidableB.Shape, ref localTransformB, ref contact.Normal, out depthCandidate, out normalCandidate);
                //    if (contact.PenetrationDepth - depthCandidate <= Toolbox.BigEpsilon)
                //        break;

                //    contact.PenetrationDepth = depthCandidate;
                //    contact.Normal = normalCandidate;
                //}

                contact.Id = -1;

                if (contact.PenetrationDepth < convex.collisionMargin + triangle.collisionMargin)
                {
                    state = CollisionState.ExternalNear; //If it's emerged from the deep contact, we can go back to using the preferred GJK method.
                }
                contactList.Add(ref contact);
            }



            if (TryInnerSphereContact(out contact))
            {
                contactList.Add(ref contact);
            }
            if (contactList.count > 0)
                return true;

            state = CollisionState.ExternalSeparated;
            return false;











            //if (MPRToolbox.AreObjectsColliding(convex, triangle, ref Toolbox.RigidIdentity, ref Toolbox.RigidIdentity, out contact))
            //{
            //    //Determine if the normal points in the appropriate direction given the sidedness of the triangle.
            //    if (triangle.sidedness != TriangleSidedness.DoubleSided)
            //    {
            //        Vector3 triangleNormal, ab, ac;
            //        Vector3.Subtract(ref triangle.vB, ref triangle.vA, out ab);
            //        Vector3.Subtract(ref triangle.vC, ref triangle.vA, out ac);
            //        Vector3.Cross(ref ab, ref ac, out triangleNormal);
            //        float dot;
            //        Vector3.Dot(ref triangleNormal, ref contact.Normal, out dot);
            //        if (triangle.sidedness == TriangleSidedness.Clockwise && dot > 0)
            //            return false;
            //        if (triangle.sidedness == TriangleSidedness.Counterclockwise && dot < 0)
            //            return false;
            //    }

            //    if (contact.PenetrationDepth < convex.collisionMargin + triangle.collisionMargin)
            //        state = CollisionState.ExternalNear; //If it's emerged from the deep contact, we can go back to using the preferred GJK method.
            //    return true;
            //}
            ////This is rare, but could happen.
            //state = CollisionState.ExternalSeparated;
            //return false;




        }


        void TryToEscape()
        {
            if (++escapeAttempts == EscapeAttemptPeriod)
            {
                escapeAttempts = 0;
                state = CollisionState.Plane;
            }
        }

        void TryToEscape(ref Vector3 position)
        {
            if (++escapeAttempts == EscapeAttemptPeriod && GetVoronoiRegion(ref position) == VoronoiRegion.ABC)
            {
                escapeAttempts = 0;
                state = CollisionState.Plane;
            }
        }


        private bool TryInnerSphereContact(out ContactData contact)
        {
            Vector3 closestPoint;
            if (Toolbox.GetClosestPointOnTriangleToPoint(ref triangle.vA, ref triangle.vB, ref triangle.vC, ref Toolbox.ZeroVector, out closestPoint))
            {
                state = CollisionState.Plane;
            }
            float length = closestPoint.LengthSquared();
            float minimumRadius = convex.minimumRadius * (MotionSettings.CoreShapeScaling + .01f);
            if (length < minimumRadius * minimumRadius)
            {
                Vector3 triangleNormal, ab, ac;
                Vector3.Subtract(ref triangle.vB, ref triangle.vA, out ab);
                Vector3.Subtract(ref triangle.vC, ref triangle.vA, out ac);
                Vector3.Cross(ref ab, ref ac, out triangleNormal);
                float dot;
                Vector3.Dot(ref closestPoint, ref triangleNormal, out dot);
                if ((triangle.sidedness == TriangleSidedness.Clockwise && dot > 0) || (triangle.sidedness == TriangleSidedness.Counterclockwise && dot < 0))
                {
                    //Normal was facing the wrong way.
                    contact = new ContactData();
                    return false;
                }

                length = (float)Math.Sqrt(length);
                contact.Position = closestPoint;

                if (length > Toolbox.Epsilon) //Watch out for NaN's!
                {
                    Vector3.Divide(ref closestPoint, length, out contact.Normal);
                }
                else
                {
                    //The direction is undefined.  Use the triangle's normal.
                    //One sided triangles can only face in the appropriate direction.
                    if (triangle.sidedness == TriangleSidedness.Clockwise)
                        contact.Normal = triangleNormal;
                    else
                        Vector3.Negate(ref triangleNormal, out contact.Normal);
                }

                contact.PenetrationDepth = convex.minimumRadius - length;
                contact.Id = -1;
                return true;
            }
            contact = new ContactData();
            return false;
        }

        ///<summary>
        /// Determines what voronoi region a given point is in.
        ///</summary>
        ///<param name="p">Point to test.</param>
        ///<returns>Voronoi region containing the point.</returns>
        public VoronoiRegion GetVoronoiRegion(ref Vector3 p)
        {
            //The point we are comparing against the triangle is 0,0,0, so instead of storing an "A->P" vector,
            //just use -A.
            //Same for B->, C->P...

            Vector3 ab, ac, ap;
            Vector3.Subtract(ref triangle.vB, ref triangle.vA, out ab);
            Vector3.Subtract(ref triangle.vC, ref triangle.vA, out ac);
            Vector3.Subtract(ref p, ref triangle.vA, out ap);

            //Check to see if it's outside A.
            float APdotAB, APdotAC;
            Vector3.Dot(ref ap, ref ab, out APdotAB);
            Vector3.Dot(ref ap, ref ac, out APdotAC);
            if (APdotAC <= 0f && APdotAB <= 0)
            {
                //It is A!
                return VoronoiRegion.A;
            }

            //Check to see if it's outside B.
            float BPdotAB, BPdotAC;
            Vector3 bp;
            Vector3.Subtract(ref p, ref triangle.vB, out bp);
            Vector3.Dot(ref ab, ref bp, out BPdotAB);
            Vector3.Dot(ref ac, ref bp, out BPdotAC);
            if (BPdotAB >= 0f && BPdotAC <= BPdotAB)
            {
                //It is B!
                return VoronoiRegion.B;
            }

            //Check to see if it's outside AB.
            float vc = APdotAB * BPdotAC - BPdotAB * APdotAC;
            if (vc <= 0 && APdotAB > 0 && BPdotAB < 0) //Note > and < instead of => <=; avoids possibly division by zero
            {
                return VoronoiRegion.AB;
            }

            //Check to see if it's outside C.
            float CPdotAB, CPdotAC;
            Vector3 cp;
            Vector3.Subtract(ref p, ref triangle.vC, out cp);
            Vector3.Dot(ref ab, ref cp, out CPdotAB);
            Vector3.Dot(ref ac, ref cp, out CPdotAC);
            if (CPdotAC >= 0f && CPdotAB <= CPdotAC)
            {
                //It is C!
                return VoronoiRegion.C;
            }

            //Check if it's outside AC.    
            float vb = CPdotAB * APdotAC - APdotAB * CPdotAC;
            if (vb <= 0f && APdotAC > 0f && CPdotAC < 0f) //Note > instead of >= and < instead of <=; prevents bad denominator
            {
                return VoronoiRegion.AC;
            }

            //Check if it's outside BC.
            float va = BPdotAB * CPdotAC - CPdotAB * BPdotAC;
            float d3d4;
            float d6d5;
            if (va <= 0f && (d3d4 = BPdotAC - BPdotAB) > 0f && (d6d5 = CPdotAB - CPdotAC) > 0f)//Note > instead of >= and < instead of <=; prevents bad denominator
            {
                return VoronoiRegion.BC;
            }


            //On the face of the triangle.
            return VoronoiRegion.ABC;


        }

        ///<summary>
        /// Initializes the pair tester.
        ///</summary>
        ///<param name="convex">Convex shape to use.</param>
        ///<param name="triangle">Triangle shape to use.</param>
        public void Initialize(ConvexShape convex, TriangleShape triangle)
        {
            this.convex = convex;
            this.triangle = triangle;
        }

        /// <summary>
        /// Cleans up the pair tester.
        /// </summary>
        public void CleanUp()
        {
            triangle = null;
            convex = null;
            state = CollisionState.Plane;
            escapeAttempts = 0;
            localSeparatingAxis = new Vector3();
            Updated = false;
        }

        internal enum CollisionState
        {
            Plane,
            ExternalSeparated,
            ExternalNear,
            Deep
        }


    }

}
