using System;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.MathExtensions;
using BEPUphysics.Settings;

namespace BEPUphysics.CollisionTests.CollisionAlgorithms
{
    ///<summary>
    /// Persistent tester that compares triangles against convex objects.
    ///</summary>
    public class TriangleMeshConvexPairTester
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
        ///<param name="contact">Contact between the shapes, if any.</param>
        ///<returns>Whether or not the shapes are colliding.</returns>
        public bool GenerateContactCandidate(out ContactData contact)
        {
            switch (state)
            {
                case CollisionState.Plane:
                    return DoPlaneTest(out contact);
                case CollisionState.ExternalSeparated:
                    return DoExternalSeparated(out contact);
                case CollisionState.ExternalNear:
                    return DoExternalNear(out contact);
                case CollisionState.Deep:
                    return DoDeepContact(out contact);
                default:
                    contact = new ContactData();
                    return false;
            }



        }


        private bool DoPlaneTest(out ContactData contact)
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
                        contact = new ContactData();
                        return false;
                    }
                    break;
                case TriangleSidedness.Counterclockwise:
                    if (dotA > 0)
                    {
                        //The reverse normal is pointing away from the convex.
                        contact = new ContactData();
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
                return DoExternalSeparated(out contact);
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


                contact = new ContactData();
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
                    //This overrides other depth tests.

                    ContactData alternateContact;
                    if (TryInnerSphereContact(out alternateContact))
                    {
                        contact = alternateContact;
                        return true;
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
                    if (DoExternalNear(out alternateContact))
                    {
                        if (alternateContact.PenetrationDepth + .01f < contact.PenetrationDepth) //Bias against the MPR test's result, since the plane version will probably have a better position.
                        {
                            //It WAS a bad contact.
                            contact = alternateContact;
                        }
                        else
                        {
                            //Well, it really is just that deep.
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
                return true;

            }
            contact = new ContactData();
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



        private bool DoExternalSeparated(out ContactData contact)
        {

            if (GJKToolbox.AreShapesIntersecting(convex, triangle, ref Toolbox.RigidIdentity, ref Toolbox.RigidIdentity, ref localSeparatingAxis))
            {
                state = CollisionState.ExternalNear;
                return DoExternalNear(out contact);
            }
            TryToEscape();
            contact = new ContactData();
            return false;
        }

        private bool DoExternalNear(out ContactData contact)
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
                return DoDeepContact(out contact);
            }
            Vector3 displacement;
            Vector3.Subtract(ref closestB, ref closestA, out displacement);
            float distanceSquared = displacement.LengthSquared();
            float margin = convex.collisionMargin + triangle.collisionMargin;


            if (distanceSquared < margin * margin)
            {
                //Try to generate a contact.
                contact = new ContactData();

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
                if (margin > Toolbox.Epsilon) //This can be zero! It would cause a NaN is unprotected.
                    Vector3.Multiply(ref displacement, convex.collisionMargin / margin, out contact.Position); //t * AB
                else contact.Position = new Vector3();
                Vector3.Add(ref closestA, ref contact.Position, out contact.Position); //A + t * AB.



                contact.Normal = displacement;
                float distance = (float)Math.Sqrt(distanceSquared);
                Vector3.Divide(ref contact.Normal, distance, out contact.Normal);
                contact.PenetrationDepth = margin - distance;




                TryToEscape(ref contact.Position);
                return true;

            }
            //Too far to make a contact- move back to separation.
            state = CollisionState.ExternalSeparated;
            contact = new ContactData();
            return false;
        }

        private bool DoDeepContact(out ContactData contact)
        {
            if (TryInnerSphereContact(out contact))
                return true;

            if (MPRToolbox.AreObjectsColliding(convex, triangle, ref Toolbox.RigidIdentity, ref Toolbox.RigidIdentity, out contact))
            {
                //Determine if the normal points in the appropriate direction given the sidedness of the triangle.
                if (triangle.sidedness != TriangleSidedness.DoubleSided)
                {
                    Vector3 triangleNormal, ab, ac;
                    Vector3.Subtract(ref triangle.vB, ref triangle.vA, out ab);
                    Vector3.Subtract(ref triangle.vC, ref triangle.vA, out ac);
                    Vector3.Cross(ref ab, ref ac, out triangleNormal);
                    float dot;
                    Vector3.Dot(ref triangleNormal, ref contact.Normal, out dot);
                    if (triangle.sidedness == TriangleSidedness.Clockwise && dot > 0)
                        return false;
                    if (triangle.sidedness == TriangleSidedness.Counterclockwise && dot < 0)
                        return false;
                }

                if (contact.PenetrationDepth < convex.collisionMargin + triangle.collisionMargin)
                    state = CollisionState.ExternalNear; //If it's emerged from the deep contact, we can go back to using the preferred GJK method.
                return true;
            }
            //This is rare, but could happen.
            state = CollisionState.ExternalSeparated;
            return false;
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
                length = (float)Math.Sqrt(length);
                contact.Position = closestPoint;
                if (length > Toolbox.Epsilon) //Watch out for NaN's!
                    Vector3.Divide(ref closestPoint, length, out contact.Normal);
                else
                {
                    //This isn't fast, but this is an extremely rare event.
                    contact.Normal = triangle.GetNormal(Toolbox.RigidIdentity);
                }

                //The penetration depth could also be approximated rather than computed.  This is basically an approximation anyway.
                contact.PenetrationDepth = MPRToolbox.FindPenetrationDepth(triangle, convex, ref contact.Position, ref Toolbox.RigidIdentity, ref contact.Normal);
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
