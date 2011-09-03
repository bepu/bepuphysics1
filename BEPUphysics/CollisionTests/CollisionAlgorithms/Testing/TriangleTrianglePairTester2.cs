using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.CollisionShapes.ConvexShapes;
 
using BEPUphysics.DataStructures;
using BEPUphysics.MathExtensions;

namespace BEPUphysics.CollisionTests.CollisionAlgorithms.Testing
{
    /// <summary>
    /// Generates candidates between two triangles and manages the persistent state of the pair.
    /// </summary>
    public class TriangleTrianglePairTester2 : TrianglePairTester
    {
        //TriangleB sits at the origin.

        internal TriangleShape triangleB;

        bool wasInDeepContact = false;

        //Relies on the triangle being located in the local space of the convex object.  The convex transform is used to transform the
        //contact points back from the convex's local space into world space.

        //TODO: Could compute feature ids and use them to match contacts for better sliding/simulation quality.
        //Since the ids are used to store voronoi region data, including the feature ids would need to be done using a clever encoding.
        //Modulo divide for region.  ABC region can have multiple contacts.  Truncated division specifies which contact.
        //Other regions (edge-edge) only have one contact.

        void GetContact(ref Vector3 closestA, ref Vector3 closestB, float distance, float marginA, float marginB, out ContactData contact)
        {
            Vector3 direction;
            Vector3.Subtract(ref closestB, ref closestA, out direction);
            float t = distance * marginA / (marginA + marginB);

            Vector3.Divide(ref direction, distance, out contact.Normal);
            Vector3.Multiply(ref contact.Normal, t, out contact.Position);
            Vector3.Add(ref contact.Position, ref closestA, out contact.Position);
            contact.PenetrationDepth = marginA + marginB - distance;
            contact.Id = -1;
        }

        void GetContact(ref Vector3 closestA, ref Vector3 closestB, ref Vector3 normal, float distance, float marginA, float marginB, out ContactData contact)
        {
            Vector3 direction;
            Vector3.Subtract(ref closestB, ref closestA, out direction);
            float t = marginA / (marginA + marginB);

            Vector3.Multiply(ref direction, t, out contact.Position);
            Vector3.Add(ref contact.Position, ref closestA, out contact.Position);
            contact.Normal = normal;
            contact.PenetrationDepth = marginA + marginB - distance;
            contact.Id = -1;
        }

        ///<summary>
        /// Generates a contact between the triangle and convex.
        ///</summary>
        ///<param name="contactList">Contact between the shapes, if any.</param>
        ///<returns>Whether or not the shapes are colliding.</returns>
        public override bool GenerateContactCandidate(out TinyStructList<ContactData> contactList)
        {
            contactList = new TinyStructList<ContactData>();
            //Compute the normals of the triangles.
            Vector3 AB, AC;
            Vector3.Subtract(ref triangle.vB, ref triangle.vA, out AB);
            Vector3.Subtract(ref triangle.vC, ref triangle.vA, out AC);
            Vector3 normal, normalB;
            Vector3.Cross(ref AB, ref AC, out normal);
            Vector3.Subtract(ref triangleB.vB, ref triangleB.vA, out AB);
            Vector3.Subtract(ref triangleB.vC, ref triangleB.vA, out AC);
            Vector3.Cross(ref AB, ref AC, out normalB);
            normal.Normalize();
            normalB.Normalize();
            //Compute the location of the triangles along their normals.
            float positionA;
            Vector3.Dot(ref triangle.vA, ref normal, out positionA);
            Vector3 centerA;
            Vector3.Add(ref triangle.vA, ref triangle.vB, out centerA);
            Vector3.Add(ref triangle.vC, ref centerA, out centerA);
            float triangleAPositionOnB;
            Vector3.Dot(ref centerA, ref normalB, out triangleAPositionOnB);
            triangleAPositionOnB *= 1 / 3f;
            //Position "B" is known to be zero, because the triangle is centered on the origin.
            //Calibrate triangle A's normal.
            switch (triangle.sidedness)
            {
                case TriangleSidedness.DoubleSided:
                    if (positionA > 0)
                    {
                        //The normal is pointing away from the other triangle.
                        Vector3.Negate(ref normal, out normal);
                        positionA = -positionA;
                    }
                    break;
                case TriangleSidedness.Clockwise:
                    Vector3.Negate(ref normal, out normal);
                    positionA = -positionA;
                    break;
            }
            //Calibrate triangle B's normal.
            switch (triangleB.sidedness)
            {
                case TriangleSidedness.DoubleSided:
                    if (triangleAPositionOnB < 0)
                    {
                        //The normal is pointing away from the other triangle.
                        Vector3.Negate(ref normalB, out normalB);
                        triangleAPositionOnB = -triangleAPositionOnB;
                    }
                    break;
                case TriangleSidedness.Clockwise:
                    Vector3.Negate(ref normalB, out normalB);
                    triangleAPositionOnB = -triangleAPositionOnB;
                    break;
            }

            //Go through each vertex on triangle B and compute its distances to triangle A's plane.
            float dAonA, dBonA, dConA;
            Vector3.Dot(ref triangleB.vA, ref normal, out dAonA);
            Vector3.Dot(ref triangleB.vB, ref normal, out dBonA);
            Vector3.Dot(ref triangleB.vC, ref normal, out dConA);
            dAonA = dAonA - positionA;
            dBonA = dBonA - positionA;
            dConA = dConA - positionA;
            //Go through each vertex on triangle "A" and compute its distances to triangle B's plane.
            float dAonB, dBonB, dConB;
            Vector3.Dot(ref triangle.vA, ref normalB, out dAonB);
            Vector3.Dot(ref triangle.vB, ref normalB, out dBonB);
            Vector3.Dot(ref triangle.vC, ref normalB, out dConB);

            //If all the vertices of a triangle are on the same side of the other triangle, make sure that it's not going to violate one of the triangle's sidednesses.
            //Note that the only time the closest point is one of these vertices is when all vertices are on the same side of the triangle.  The other edges must still be
            //tested though.

            float collisionMargin = triangle.collisionMargin + triangleB.collisionMargin;

            bool definitelyNotIntersecting = false;
            if (dAonA >= 0 && dBonA >= 0 && dConA >= 0)
            {
                if (dAonA > collisionMargin && dBonA >= collisionMargin && dConA >= collisionMargin)
                    return false; //There can be no collision at all!
                //All vertices on B are on a valid side of triangle A.
                definitelyNotIntersecting = true;

            }
            else if (dAonA < 0 && dBonA < 0 && dConA < 0)
            {
                //Cannot collide.
                //If the objects were doubled sided, the normals would be calibrated to face each other (at least one vertex would be on the other side).
                //That means the triangle is one-sided.
                //If the objects are one-sided, then being on the negative side means the objects cannot create a normal which faces the correct direction.
                wasInDeepContact = false;
                contactList = new TinyStructList<ContactData>();
                return false;

            }
            if (dAonB >= 0 && dBonB >= 0 && dConB >= 0)
            {
                if (dAonB > collisionMargin && dBonB >= collisionMargin && dConB >= collisionMargin)
                    return false; //There can be no collision at all!
                //All vertices on A are on a valid side of triangle B.
                definitelyNotIntersecting = true;

            }
            else if (dAonB < 0 && dBonB < 0 && dConB < 0)
            {
                //Cannot collide.
                //If the objects were doubled sided, the normals would be calibrated to face each other (at least one vertex would be on the other side).
                //That means the triangle is one-sided.
                //If the objects are one-sided, then being on the negative side means the objects cannot create a normal which faces the correct direction.
                wasInDeepContact = false;
                contactList = new TinyStructList<ContactData>();
                return false;

            }

            //TODO: Check margin order relative to edge owners.
            #region Face-vertex tests
            //Instead of generating a single closest point, this generates a contact point for each vertex that is close enough to the triangle.
            //Only the closest edge will be used (if it is closer than any of the vertex contacts), but it will be in addition to these vertex contacts.

            //Keep in mind that contact normals should face towards triangle A.

            //Test all of B's vertices against triangle A.
            Vector3 negatedNormalA;
            Vector3.Negate(ref normal, out negatedNormalA);
            Vector3 closestPointOnTriangle;
            if (dAonA >= 0 && dAonA < collisionMargin && IsPointInTriangle(ref triangleB.vA, ref triangle.vA, ref triangle.vB, ref triangle.vC, ref normal, -dAonA, out closestPointOnTriangle))
            {
                ContactData contact;
                GetContact(ref closestPointOnTriangle, ref triangleB.vA, ref negatedNormalA, dAonA, triangle.collisionMargin, triangleB.collisionMargin, out contact);
                contact.Id = (int)VoronoiRegion.ABC;
                contactList.Add(ref contact);

            }
            if (dBonA >= 0 && dBonA < collisionMargin && IsPointInTriangle(ref triangleB.vB, ref triangle.vA, ref triangle.vB, ref triangle.vC, ref normal, -dBonA, out closestPointOnTriangle))
            {
                ContactData contact;
                GetContact(ref closestPointOnTriangle, ref triangleB.vB, ref negatedNormalA, dBonA, triangle.collisionMargin, triangleB.collisionMargin, out contact);
                contact.Id = (int)VoronoiRegion.ABC;
                contactList.Add(ref contact);

            }
            if (dConA >= 0 && dConA < collisionMargin && IsPointInTriangle(ref triangleB.vC, ref triangle.vA, ref triangle.vB, ref triangle.vC, ref normal, -dConA, out closestPointOnTriangle))
            {
                ContactData contact;
                GetContact(ref closestPointOnTriangle, ref triangleB.vC, ref negatedNormalA, dConA, triangle.collisionMargin, triangleB.collisionMargin, out contact);
                contact.Id = (int)VoronoiRegion.ABC;
                contactList.Add(ref contact);
            }

            //Test all of A's vertices against triangle B.
            if (dAonB >= 0 && dAonB < collisionMargin && IsPointInTriangle(ref triangle.vA, ref triangleB.vA, ref triangleB.vB, ref triangleB.vC, ref normalB, -dAonB, out closestPointOnTriangle))
            {
                ContactData contact;
                GetContact(ref closestPointOnTriangle, ref triangle.vA, ref normalB, dAonB, triangle.collisionMargin, triangleB.collisionMargin, out contact);
                contact.Id = (int)VoronoiRegion.A;
                contactList.Add(ref contact);
            }
            if (dBonB >= 0 && dBonB < collisionMargin && IsPointInTriangle(ref triangle.vB, ref triangleB.vA, ref triangleB.vB, ref triangleB.vC, ref normalB, -dBonB, out closestPointOnTriangle))
            {
                ContactData contact;
                GetContact(ref closestPointOnTriangle, ref triangle.vB, ref normalB, dBonB, triangle.collisionMargin, triangleB.collisionMargin, out contact);
                contact.Id = (int)VoronoiRegion.B;
                contactList.Add(ref contact);
            }
            if (dConB >= 0 && dConB < collisionMargin && IsPointInTriangle(ref triangle.vC, ref triangleB.vA, ref triangleB.vB, ref triangleB.vC, ref normalB, -dConB, out closestPointOnTriangle))
            {
                ContactData contact;
                GetContact(ref closestPointOnTriangle, ref triangle.vC, ref normalB, dConB, triangle.collisionMargin, triangleB.collisionMargin, out contact);
                contact.Id = (int)VoronoiRegion.C;
                contactList.Add(ref contact);
            }
            #endregion

            //Now do edge-edge tests.
            if (definitelyNotIntersecting)
            {
                //We can't do closest points tests between edges without knowing that the triangles are NOT intersecting.
                //But since we do, let's go!
                //This process will find the closest edge-edge pair.
                Vector3 closestPointA = new Vector3(), closestPointB = new Vector3();
                float distanceSquared = float.MaxValue;
                float s, t;
                Vector3 closestPointACandidate, closestPointBCandidate;
                float distanceSquaredCandidate;
                #region Edge Tests
                VoronoiRegion region;
                //AB vs AB
                Toolbox.GetClosestPointsBetweenSegments(ref triangle.vB, ref triangle.vA, ref triangleB.vB, ref triangleB.vA, out s, out t, out closestPointACandidate, out closestPointBCandidate);
                Vector3.DistanceSquared(ref closestPointBCandidate, ref closestPointACandidate, out distanceSquaredCandidate);

                closestPointA = closestPointACandidate;
                closestPointB = closestPointBCandidate;
                region = VoronoiRegion.AB;
                distanceSquared = distanceSquaredCandidate;

                //AB vs BC
                Toolbox.GetClosestPointsBetweenSegments(ref triangle.vB, ref triangle.vA, ref triangleB.vC, ref triangleB.vB, out s, out t, out closestPointACandidate, out closestPointBCandidate);
                Vector3.DistanceSquared(ref closestPointBCandidate, ref closestPointACandidate, out distanceSquaredCandidate);
                if (distanceSquaredCandidate < distanceSquared)
                {
                    closestPointA = closestPointACandidate;
                    closestPointB = closestPointBCandidate;
                    region = VoronoiRegion.AB;
                    distanceSquared = distanceSquaredCandidate;
                }
                //AB vs CA
                Toolbox.GetClosestPointsBetweenSegments(ref triangle.vB, ref triangle.vA, ref triangleB.vA, ref triangleB.vC, out s, out t, out closestPointACandidate, out closestPointBCandidate);
                Vector3.DistanceSquared(ref closestPointBCandidate, ref closestPointACandidate, out distanceSquaredCandidate);
                if (distanceSquaredCandidate < distanceSquared)
                {
                    closestPointA = closestPointACandidate;
                    closestPointB = closestPointBCandidate;
                    region = VoronoiRegion.AB;
                    distanceSquared = distanceSquaredCandidate;
                }
                //BC vs AB
                Toolbox.GetClosestPointsBetweenSegments(ref triangle.vC, ref triangle.vB, ref triangleB.vB, ref triangleB.vA, out s, out t, out closestPointACandidate, out closestPointBCandidate);
                Vector3.DistanceSquared(ref closestPointBCandidate, ref closestPointACandidate, out distanceSquaredCandidate);
                if (distanceSquaredCandidate < distanceSquared)
                {
                    closestPointA = closestPointACandidate;
                    closestPointB = closestPointBCandidate;
                    region = VoronoiRegion.BC;
                    distanceSquared = distanceSquaredCandidate;
                }
                //BC vs BC
                Toolbox.GetClosestPointsBetweenSegments(ref triangle.vC, ref triangle.vB, ref triangleB.vC, ref triangleB.vB, out s, out t, out closestPointACandidate, out closestPointBCandidate);
                Vector3.DistanceSquared(ref closestPointBCandidate, ref closestPointACandidate, out distanceSquaredCandidate);
                if (distanceSquaredCandidate < distanceSquared)
                {
                    closestPointA = closestPointACandidate;
                    closestPointB = closestPointBCandidate;
                    region = VoronoiRegion.BC;
                    distanceSquared = distanceSquaredCandidate;
                }
                //BC vs CA
                Toolbox.GetClosestPointsBetweenSegments(ref triangle.vC, ref triangle.vB, ref triangleB.vA, ref triangleB.vC, out s, out t, out closestPointACandidate, out closestPointBCandidate);
                Vector3.DistanceSquared(ref closestPointBCandidate, ref closestPointACandidate, out distanceSquaredCandidate);
                if (distanceSquaredCandidate < distanceSquared)
                {
                    closestPointA = closestPointACandidate;
                    closestPointB = closestPointBCandidate;
                    region = VoronoiRegion.BC;
                    distanceSquared = distanceSquaredCandidate;
                }
                //CA vs AB
                Toolbox.GetClosestPointsBetweenSegments(ref triangle.vA, ref triangle.vC, ref triangleB.vB, ref triangleB.vA, out s, out t, out closestPointACandidate, out closestPointBCandidate);
                Vector3.DistanceSquared(ref closestPointBCandidate, ref closestPointACandidate, out distanceSquaredCandidate);
                if (distanceSquaredCandidate < distanceSquared)
                {
                    closestPointA = closestPointACandidate;
                    closestPointB = closestPointBCandidate;
                    region = VoronoiRegion.AC;
                    distanceSquared = distanceSquaredCandidate;
                }
                //CA vs BC
                Toolbox.GetClosestPointsBetweenSegments(ref triangle.vA, ref triangle.vC, ref triangleB.vC, ref triangleB.vB, out s, out t, out closestPointACandidate, out closestPointBCandidate);
                Vector3.DistanceSquared(ref closestPointBCandidate, ref closestPointACandidate, out distanceSquaredCandidate);
                if (distanceSquaredCandidate < distanceSquared)
                {
                    closestPointA = closestPointACandidate;
                    closestPointB = closestPointBCandidate;
                    region = VoronoiRegion.AC;
                    distanceSquared = distanceSquaredCandidate;
                }
                //CA vs CA
                Toolbox.GetClosestPointsBetweenSegments(ref triangle.vA, ref triangle.vC, ref triangleB.vA, ref triangleB.vC, out s, out t, out closestPointACandidate, out closestPointBCandidate);
                Vector3.DistanceSquared(ref closestPointBCandidate, ref closestPointACandidate, out distanceSquaredCandidate);
                if (distanceSquaredCandidate < distanceSquared)
                {
                    closestPointA = closestPointACandidate;
                    closestPointB = closestPointBCandidate;
                    region = VoronoiRegion.AC;
                    distanceSquared = distanceSquaredCandidate;
                }
                #endregion
                wasInDeepContact = false;

                //Only add the edge-edge contact if it is closer than the face contacts we found earlier.
                //The edge-edge tests are not 'area aware,' they would gladly create an incorrect contact if they aren't stopped.
                float distance = (float)Math.Sqrt(distanceSquared);
                bool shouldAdd = true;
                ContactData contact;
                float depth = collisionMargin - distance;
                if (depth >= 0) //Don't add if it's too far away.
                {
                    for (int i = 0; i < contactList.count; i++)
                    {
                        contactList.Get(i, out contact);
                        if (contact.PenetrationDepth > depth)
                        {
                            //A face contact is closer.  Don't add the edge-edge contact!
                            shouldAdd = false;
                            break;
                        }
                    }
                    if (shouldAdd)
                    {
                        GetContact(ref closestPointB, ref closestPointA, distance, triangle.collisionMargin, triangleB.collisionMargin, out contact);
                        contact.Id = (int)region;
                        contactList.Add(ref contact);
                    }
                }
                return contactList.count > 0;
            }
            else
            {
                #region Explanation
                //If it gets here, we know two things:
                //1) The triangles may be intersecting.
                //2) In the nonintersecting case, the closest points are from edge-edge tests.

                //Further, don't worry about the global correctness of the deep contact case.  It never lasts long.  Just do something acceptable.


                //If they are possibly intersecting, an additional test is required.
                //Intersect the edges of A with B's plane.  This is the easiest route because B's plane is aligned with the origin.
                //In non-degenerate cases, two edges will intersect the plane.

                //Those two intersection points form a line segment on B's plane.  Test that line (infinite) against B's edges.
                //This provides two intervals: 0 to 1 for A's plane-segment, and B1 to B2 for the intersections with triangle B's edges.
                //If a's interval intersects with b's interval, then the triangles overlap.

                //If none of B's edges are intersected by the line, then there is no collision.  If only one is intersected, numerical issues occurred; no collision.
                //If THREE edges are intersected, then pick two that have dissimilar times of impact.  This can happen when the line enters/exits at vertices.

                //The edges composing a collision can be identified based on the interval order and what edges the endpoints were computed using.
                //The possible configurations are as follows:
                //1) A1 A2 B1 B2 (not intersecting, generating edges of A2 and B1 define closest points)
                //2) A1 B1 A2 B2 (intersecting, generating edges of B1 and A2 crossed is possible minimum axis, along with face normals)
                //3) A1 B1 B2 A2 (intersecting, if B1-A1 < A2-B2, use A1 and B1 to create an axis, otherwise use B2 and A2.  Also face normals.)
                //4) B1 B2 A1 A2 (not intersecting, generating edges of B2 and A1 define closest points)
                //5) B1 A1 B2 A2 (intersecting, generating edges of A1 and B2 crossed is possible minimum axis, along with face normals)
                //6) B1 A1 A2 B2 (intersecting, if A1-B1 < B2-A2, use A1 and B1 to create an axis, otherwise use B2 and A2.  Also face normals.)

                //Note that these form three conceptual batches.  
                //1 and 4 just take the two interior edges as input.
                //2 and 5 also take the interior edges as input, but perform a different operation.
                //3 and 6 are trivially equivalent.
                #endregion

                //First, compute the intersections of triangle A's edges with B's plane.

                Vector3 A1;
                Vector3 A2;
                Vector3 A11;
                Vector3 A12;
                Vector3 A21, A22;
                VoronoiRegion regionA1;
                VoronoiRegion regionA2;
                #region Find A edge intersections with B's plane
                A11 = triangle.vA;
                A12 = triangle.vB;
                regionA1 = VoronoiRegion.AB;
                if (IntersectSegmentAgainstPlane(ref A11, ref A12, ref normalB, out A1))
                {
                    //The first try at A11/A12 got a good result.
                    A21 = triangle.vB;
                    A22 = triangle.vC;
                    regionA2 = VoronoiRegion.BC;
                    if (IntersectSegmentAgainstPlane(ref A21, ref A22, ref normalB, out A2))
                    {
                        //The first try for A21/A22 got a good result! Check for degeneracy.
                        float distanceSquared;
                        Vector3.DistanceSquared(ref A1, ref A2, out distanceSquared);
                        if (distanceSquared < Toolbox.BigEpsilon)
                        {
                            //The two edge-plane intersections are way too close together.  The plane went right through a vertex or there's a degeneracy.
                            A21 = triangle.vC;
                            A22 = triangle.vA;
                            regionA2 = VoronoiRegion.AC;
                            if (!IntersectSegmentAgainstPlane(ref A21, ref A22, ref normalB, out A2))
                            {
                                //That's strange! Something bad is happening.  Embrace the degeneracy.
                                //This could imply that a vertex is perfectly aligned with the plane.
                                //In this case, the face-vertex tests will most likely have already identified this as a contact.
                                //Short circuit out of the rest.
                                return contactList.count > 0;
                            }
                        }
                    }
                    else
                    {
                        //A21/A22 are still undefined.  Try again!                           
                        A21 = triangle.vC;
                        A22 = triangle.vA;
                        regionA2 = VoronoiRegion.AC;
                        if (!IntersectSegmentAgainstPlane(ref A21, ref A22, ref normalB, out A2))
                        {
                            //That's strange! Something bad is happening.
                            //It's likely that a single vertex is *very* close to the plane, such that numerical issues
                            //caused a failure to find the other edge associated with the vertex.
                            //In this case, the face-vertex tests will most likely have already identified this as a contact.
                            //Short circuit out of the rest.
                            return contactList.count > 0;
                        }
                    }
                }
                else
                {
                    //Still need to find a good A11/A12.
                    A11 = triangle.vB;
                    A12 = triangle.vC;
                    regionA1 = VoronoiRegion.BC;
                    if (IntersectSegmentAgainstPlane(ref A11, ref A12, ref normalB, out A1))
                    {
                        //There we go!  Now try for a A21/A22.
                        A21 = triangle.vC;
                        A22 = triangle.vA;
                        regionA2 = VoronoiRegion.AC;
                        if (!IntersectSegmentAgainstPlane(ref A21, ref A22, ref normalB, out A2))
                        {
                            //That's strange! Something bad is happening.
                            //It's likely that a single vertex is *very* close to the plane, such that numerical issues
                            //caused a failure to find the other edge associated with the vertex.
                            //In this case, the face-vertex tests will most likely have already identified this as a contact.
                            //Short circuit out of the rest.
                            return contactList.count > 0;
                        }
                    }
                    else
                    {
                        //STILL do not have a A11/A12! It's impossible to find a full interval at this point.
                        //It's either not intersecting the plane at all, or it's possible that the next test would find an intersection.
                        //Either way, short circuit the rest.
                        return contactList.count > 0;
                    }
                }
                #endregion

                //Intersect that line (A1 + (A2 - A1) * t) against the three edges of B.

                Vector3 B1, B2;
                float B1t, B2t;
                Vector3 B11, B12, B21, B22;
                #region Find intersections of B's edges with A1A2 line
                float t1, t2;
                Vector3 closestA, closestB;
                //Test AB against the line.
                B11 = triangleB.vA;
                B12 = triangleB.vB;
                Toolbox.GetClosestPointsBetweenLines(ref A1, ref A2, ref B11, ref B12, out B1t, out t1, out closestA, out closestB);
                //The first segment is infinite.  The triangle edge we're testing against is not.  Make sure t is between 0 and 1.
                if (t1 >= 0 && t1 <= 1)
                {
                    //The lines intersect!
                    B1 = closestB;
                    //Now find a B2.
                    B21 = triangleB.vB;
                    B22 = triangleB.vC;
                    Toolbox.GetClosestPointsBetweenLines(ref A1, ref A2, ref B21, ref B22, out B2t, out t2, out closestA, out closestB);
                    if (t2 >= 0 && t2 <= 1)
                    {
                        //The lines intersect!
                        B2 = closestB;
                        if (Math.Abs(B1t - B2t) < Toolbox.BigEpsilon)
                        {
                            //But they are too close.  The line passed through a vertex.  Try the other edge.
                            B21 = triangleB.vC;
                            B22 = triangleB.vA;
                            Toolbox.GetClosestPointsBetweenLines(ref A1, ref A2, ref B21, ref B22, out B2t, out t2, out closestA, out closestB);
                            if (t2 < 0 || t2 > 1)
                            {
                                //That's strange!  The line must have just grazed a vertex; it hit two edges at the same spot.
                                //Could also be a degenerate triangle.  Either way, we've done as much as we need to.
                                return contactList.count > 0;
                            }
                            B2 = closestB;
                        }
                    }
                    else
                    {
                        //Still need a B2.  Try the last option.
                        B21 = triangleB.vC;
                        B22 = triangleB.vA;
                        Toolbox.GetClosestPointsBetweenLines(ref A1, ref A2, ref B21, ref B22, out B2t, out t2, out closestA, out closestB);
                        if (t2 < 0 || t2 > 1)
                        {
                            //That's strange!  The line must have just grazed a vertex; it only hit a single edge.
                            return contactList.count > 0;
                        }
                        B2 = closestB;
                    }
                }
                else
                {
                    //Still need to find B1..
                    B11 = triangleB.vB;
                    B12 = triangleB.vC;
                    Toolbox.GetClosestPointsBetweenLines(ref A1, ref A2, ref B11, ref B12, out B1t, out t1, out closestA, out closestB);
                    if (t1 >= 0 && t1 <= 1)
                    {
                        //Found it!
                        B1 = closestB;
                        //Now find a B2!  Only one option left.
                        B21 = triangleB.vC;
                        B22 = triangleB.vA;
                        Toolbox.GetClosestPointsBetweenLines(ref A1, ref A2, ref B21, ref B22, out B2t, out t2, out closestA, out closestB);
                        if (t2 < 0 || t2 > 1)
                        {
                            //That's strange!  The line must have just grazed a vertex; it hit two edges at the same spot.
                            //Could also be a degenerate triangle.  Either way, we've done as much as we need to.
                            return contactList.count > 0;
                        }
                        B2 = closestB;
                    }
                    else
                    {
                        //Only one option left for B1, which leaves no options for B2... There's nothing we can do at this point;
                        //Either no edges were hit, or only one edge is hit.  Either way, we're done.
                        return contactList.count > 0;
                    }
                }
                #endregion

                //If we get here, that means we have two nice clean intervals, A1 A2 and B1 B2.
                //The numerical values of the endpoints are:
                //A1: 0
                //A2: 1
                //B1: B1t
                //B2: B2t

                //If the intervals do not overlap, the triangles are separated.
                //This occurs when A2 < B1 or B2 < A1.

                //If the triangles do not overlap, pick the two closest edges (A2 and B1 or B2 and A1) and find the closest points between those segments.

                //Unlike testing every edge-edge pair, we do not have to check the depth on this contact against the other contacts before adding it.
                //That's because the tests thus far guarantee that the contact will be valid, even if it's not the deepest.
                #region A1 A2 B1 B2
                if (1 < B1t)
                {
                    //The edges are A2 and B1.
                    float s, t;
                    Toolbox.GetClosestPointsBetweenSegments(ref A21, ref A22, ref B11, ref B12, out s, out t, out closestA, out closestB);
                    float distanceSquared;
                    Vector3.DistanceSquared(ref closestA, ref closestB, out distanceSquared);
                    if (distanceSquared < collisionMargin * collisionMargin)
                    {
                        ContactData contact;
                        GetContact(ref closestB, ref closestA, (float)Math.Sqrt(distanceSquared), triangle.collisionMargin, triangleB.collisionMargin, out contact);
                        contact.Id = (int)regionA2;
                        contactList.Add(ref contact);
                    }
                    return contactList.count > 0;
                }
                #endregion

                #region B1 B2 A1 A2
                if (B2t < 0)
                {
                    //The edges are B2 and A1.
                    float s, t;
                    Toolbox.GetClosestPointsBetweenSegments(ref B21, ref B22, ref A11, ref A12, out s, out t, out closestB, out closestA);
                    float distanceSquared;
                    Vector3.DistanceSquared(ref closestA, ref closestB, out distanceSquared);
                    if (distanceSquared < collisionMargin * collisionMargin)
                    {
                        ContactData contact;
                        GetContact(ref closestB, ref closestA, (float)Math.Sqrt(distanceSquared), triangle.collisionMargin, triangleB.collisionMargin, out contact);
                        contact.Id = (int)regionA1;
                        contactList.Add(ref contact);
                    }
                    return contactList.count > 0;
                }
                #endregion

                //The remaining cases all involve intersecting triangles.
                //It's either a partial overlap, or a full overlap.  In a full overlap, the interval of one triangle fully contains the interval of the other triangle.
                //Let's do the partial overlaps first.
                //The cases for this are:
                //A1 B1 A2 B2
                //B1 A1 B2 A2

                //The full overlap cases are:
                //A1 B1 B2 A2
                //B1 A1 A2 B2

                //So if B1 > A1 && A2 < B2, or A1 > B1 && B2 < A2, then it's a partial overlap.

                ContactData edgeContact = new ContactData();
                #region A1 B1 A2 B2
                if (B1t > 0 && 1 < B2t)
                {
                    //Partial overlap.  Involved edges are B1 and A2.
                    //Compute the edge direction (B1 edge x A2 edge).
                    Vector3 edgeB1, edgeA2;
                    Vector3.Subtract(ref B12, ref B11, out edgeB1);
                    Vector3.Subtract(ref A22, ref A21, out edgeA2);
                    Vector3 cross;
                    Vector3.Cross(ref edgeB1, ref edgeA2, out cross);
                    float lengthSquared = cross.LengthSquared();
                    edgeContact.PenetrationDepth = float.MaxValue;
                    Vector3.Add(ref A2, ref B1, out edgeContact.Position);
                    Vector3.Multiply(ref edgeContact.Position, .5f, out edgeContact.Position);
                    if (lengthSquared > Toolbox.Epsilon)
                    {
                        Vector3.Divide(ref cross, (float)Math.Sqrt(lengthSquared), out edgeContact.Normal);
                        //Don't bother computing penetration depth using the direction.
                        //Instead, use the distance between B1 and A2.  Project it onto the axis.
                        Vector3 overlapDirection;
                        Vector3.Subtract(ref B1, ref A2, out overlapDirection);
                        float depth;
                        Vector3.Dot(ref overlapDirection, ref edgeContact.Normal, out depth);
                        float calibrateDot;
                        Vector3.Dot(ref centerA, ref edgeContact.Normal, out calibrateDot);
                        if (calibrateDot < 0)
                            Vector3.Negate(ref edgeContact.Normal, out edgeContact.Normal);
                        edgeContact.PenetrationDepth = collisionMargin + Math.Abs(depth);

                        edgeContact.Id = (int)regionA2;

                    }
                    //If the edges were parallel, ignore the edges and just use the face normals.

                    goto TestFaceNormals;
                }
                #endregion

                #region B1 A1 B2 A2
                if (0 > B1t && B2t < 1)
                {
                    //Partial overlap.  Involved edges are A1 and B2.
                    //Compute the edge direction (A1 edge x B2 edge).
                    Vector3 edgeA1, edgeB2;
                    Vector3.Subtract(ref A12, ref A11, out edgeA1);
                    Vector3.Subtract(ref B22, ref B21, out edgeB2);
                    Vector3 cross;
                    Vector3.Cross(ref edgeA1, ref edgeB2, out cross);
                    float lengthSquared = cross.LengthSquared();
                    edgeContact.PenetrationDepth = float.MaxValue;
                    Vector3.Add(ref A1, ref B2, out edgeContact.Position);
                    Vector3.Multiply(ref edgeContact.Position, .5f, out edgeContact.Position);
                    if (lengthSquared > Toolbox.Epsilon)
                    {
                        Vector3.Divide(ref cross, (float)Math.Sqrt(lengthSquared), out edgeContact.Normal);
                        //Don't bother computing penetration depth using the direction.
                        //Instead, use the distance between B1 and A2.  Project it onto the axis.
                        Vector3 overlapDirection;
                        Vector3.Subtract(ref A1, ref B2, out overlapDirection);
                        float depth;
                        Vector3.Dot(ref overlapDirection, ref edgeContact.Normal, out depth);
                        float calibrateDot;
                        Vector3.Dot(ref centerA, ref edgeContact.Normal, out calibrateDot);
                        if (calibrateDot < 0)
                            Vector3.Negate(ref edgeContact.Normal, out edgeContact.Normal);
                        edgeContact.PenetrationDepth = collisionMargin + Math.Abs(depth);

                        edgeContact.Id = (int)regionA1;

                    }
                    //If the edges were parallel, ignore the edges and just use the face normals.

                    goto TestFaceNormals;
                }
                #endregion

                //The remaining cases are:
                //A1 B1 B2 A2
                //B1 A1 A2 B2
                //Both of these correspond to full overlap.
                //For each, pick the way out that is shortest in terms of the overlap.
                #region A1 B1 B2 A2
                if (0 < B1t)
                {
                    //The edge contact position is in the middle of the interior interval (B1 B2).
                    Vector3.Add(ref B1, ref B2, out edgeContact.Position);
                    Vector3.Multiply(ref edgeContact.Position, .5f, out edgeContact.Position);

                    //For A1 B1 B2 A2, that means if B1-A1 < A2-B2, then use A1 and B1 as the involved edges.  Otherwise, use A2 and B2.
                    if (B1t < 1 - B2t)
                    {
                        //Full overlap.  Involved edges are A1 and B1.
                        //Compute the edge direction (A1 edge x B1 edge).
                        Vector3 edgeA1, edgeB1;
                        Vector3.Subtract(ref A12, ref B11, out edgeA1);
                        Vector3.Subtract(ref B12, ref B11, out edgeB1);
                        Vector3 cross;
                        Vector3.Cross(ref edgeA1, ref edgeB1, out cross);
                        float lengthSquared = cross.LengthSquared();
                        edgeContact.PenetrationDepth = float.MaxValue;

                        if (lengthSquared > Toolbox.Epsilon)
                        {
                            Vector3.Divide(ref cross, (float)Math.Sqrt(lengthSquared), out edgeContact.Normal);
                            //Don't bother computing penetration depth using the direction.
                            //Instead, use the distance between A1 and B1.  Project it onto the axis.
                            Vector3 overlapDirection;
                            Vector3.Subtract(ref A1, ref B1, out overlapDirection);
                            float depth;
                            Vector3.Dot(ref overlapDirection, ref edgeContact.Normal, out depth);
                            float calibrateDot;
                            Vector3.Dot(ref centerA, ref edgeContact.Normal, out calibrateDot);
                            if (calibrateDot < 0)
                                Vector3.Negate(ref edgeContact.Normal, out edgeContact.Normal);
                            edgeContact.PenetrationDepth = collisionMargin + Math.Abs(depth);

                            edgeContact.Id = (int)regionA1;

                        }
                    }
                    else
                    {
                        //Full overlap.  Involved edges are A2 and B2.
                        //Compute the edge direction (A2 edge x B2 edge).
                        Vector3 edgeA2, edgeB2;
                        Vector3.Subtract(ref A22, ref A21, out edgeA2);
                        Vector3.Subtract(ref B22, ref B21, out edgeB2);
                        Vector3 cross;
                        Vector3.Cross(ref edgeA2, ref edgeB2, out cross);
                        float lengthSquared = cross.LengthSquared();
                        edgeContact.PenetrationDepth = float.MaxValue;
                        if (lengthSquared > Toolbox.Epsilon)
                        {
                            Vector3.Divide(ref cross, (float)Math.Sqrt(lengthSquared), out edgeContact.Normal);
                            //Don't bother computing penetration depth using the direction.
                            //Instead, use the distance between A1 and B1.  Project it onto the axis.
                            Vector3 overlapDirection;
                            Vector3.Subtract(ref A2, ref B2, out overlapDirection);
                            float depth;
                            Vector3.Dot(ref overlapDirection, ref edgeContact.Normal, out depth);
                            float calibrateDot;
                            Vector3.Dot(ref centerA, ref edgeContact.Normal, out calibrateDot);
                            if (calibrateDot < 0)
                                Vector3.Negate(ref edgeContact.Normal, out edgeContact.Normal);
                            edgeContact.PenetrationDepth = collisionMargin + Math.Abs(depth);

                            edgeContact.Id = (int)regionA2;

                        }
                    }
                    //If the edges were parallel, ignore the edges and just use the face normals.

                    goto TestFaceNormals;
                }
                #endregion

                #region B1 A1 A2 B2
                if (0 > B1t)
                {
                    //The edge contact position is in the middle of the interior interval (A1 A2).
                    Vector3.Add(ref A1, ref A2, out edgeContact.Position);
                    Vector3.Multiply(ref edgeContact.Position, .5f, out edgeContact.Position);

                    //For B1 A1 A2 B2, that means if A1-B1 < B2-A2, then use B1 and A1 as the involved edges.  Otherwise, use B2 and A2.
                    if (-B1t < B2t - 1)
                    {
                        //Full overlap.  Involved edges are B1 and A1.
                        //Compute the edge direction (B1 edge x A1 edge).
                        Vector3 edgeB1, edgeA1;
                        Vector3.Subtract(ref B12, ref B11, out edgeB1);
                        Vector3.Subtract(ref A12, ref A11, out edgeA1);
                        Vector3 cross;
                        Vector3.Cross(ref edgeB1, ref edgeA1, out cross);
                        float lengthSquared = cross.LengthSquared();
                        edgeContact.PenetrationDepth = float.MaxValue;
                        if (lengthSquared > Toolbox.Epsilon)
                        {
                            Vector3.Divide(ref cross, (float)Math.Sqrt(lengthSquared), out edgeContact.Normal);
                            //Don't bother computing penetration depth using the direction.
                            //Instead, use the distance between B1 and A1.  Project it onto the axis.
                            Vector3 overlapDirection;
                            Vector3.Subtract(ref edgeContact.Position, ref B1, out overlapDirection);
                            float depth;
                            Vector3.Dot(ref overlapDirection, ref edgeContact.Normal, out depth);
                            float calibrateDot;
                            Vector3.Dot(ref centerA, ref edgeContact.Normal, out calibrateDot);
                            if (calibrateDot < 0)
                                Vector3.Negate(ref edgeContact.Normal, out edgeContact.Normal);
                            edgeContact.PenetrationDepth = collisionMargin + Math.Abs(depth);


                            edgeContact.Id = (int)regionA1;

                        }
                    }
                    else
                    {
                        //Full overlap.  Involved edges are B2 and A2.
                        //Compute the edge direction (B2 edge x A2 edge).
                        Vector3 edgeB2, edgeA2;
                        Vector3.Subtract(ref B22, ref B21, out edgeB2);
                        Vector3.Subtract(ref A22, ref A21, out edgeA2);
                        Vector3 cross;
                        Vector3.Cross(ref edgeB2, ref edgeA2, out cross);
                        float lengthSquared = cross.LengthSquared();
                        edgeContact.PenetrationDepth = float.MaxValue;
                        if (lengthSquared > Toolbox.Epsilon)
                        {
                            Vector3.Divide(ref cross, (float)Math.Sqrt(lengthSquared), out edgeContact.Normal);
                            //Don't bother computing penetration depth using the direction.
                            //Instead, use the distance between the edge contact position and B2.  Project it onto the axis.
                            Vector3 overlapDirection;
                            Vector3.Subtract(ref B2, ref edgeContact.Position, out overlapDirection);
                            float depth;
                            Vector3.Dot(ref overlapDirection, ref edgeContact.Normal, out depth);
                            float calibrateDot;
                            Vector3.Dot(ref centerA, ref edgeContact.Normal, out calibrateDot);
                            if (calibrateDot < 0)
                                Vector3.Negate(ref edgeContact.Normal, out edgeContact.Normal);
                            edgeContact.PenetrationDepth = collisionMargin + Math.Abs(depth);

                            edgeContact.Id = (int)regionA2;

                        }
                    }
                    //If the edges were parallel, ignore the edges and just use the face normals.

                    goto TestFaceNormals;
                }
                #endregion

            TestFaceNormals:
                //Look at vertices that are on the NEGATIVE side of each triangle.  Pick the deepest one.  That is the depth.
                float deepestOnA;
                if (dAonA < dBonA && dAonA < dConA)
                    deepestOnA = dAonA;
                else if (dBonA < dConA)
                    deepestOnA = dBonA;
                else
                    deepestOnA = dConA;
                float deepestOnB;
                if (dAonB < dBonB && dAonB < dConB)
                    deepestOnB = dAonB;
                else if (dBonB < dConB)
                    deepestOnB = dBonB;
                else
                    deepestOnB = dConB;
                ContactData faceContact;
                if (deepestOnA > deepestOnB)
                {
                    //A's is the deepest. 
                    faceContact.PenetrationDepth = collisionMargin - deepestOnA;
                    faceContact.Normal = normal;
                }
                else
                {
                    //B's is the deepest. 
                    faceContact.PenetrationDepth = collisionMargin - deepestOnB;
                    faceContact.Normal = normalB;
                }
                faceContact.Position = edgeContact.Position;
                faceContact.Id = (int)VoronoiRegion.ABC;
                if (faceContact.PenetrationDepth < edgeContact.PenetrationDepth)
                {
                    float calibrationDot;
                    Vector3.Dot(ref faceContact.Normal, ref centerA, out calibrationDot);
                    if (calibrationDot < 0)
                    {
                        Vector3.Negate(ref faceContact.Normal, out faceContact.Normal);
                    }
                    contactList.Add(ref faceContact);

                }
                else
                    contactList.Add(ref edgeContact);

            }

            if (triangle.sidedness != TriangleSidedness.DoubleSided || triangleB.sidedness != TriangleSidedness.DoubleSided)
                for (int i = contactList.count - 1; i >= 0; i--)
                {
                    ContactData contact;
                    contactList.Get(i, out contact);
                    if (triangle.sidedness != TriangleSidedness.DoubleSided)
                    {
                        float dot;
                        Vector3.Dot(ref normal, ref contact.Normal, out dot);
                        if (dot < 0)
                        {
                            contactList.RemoveAt(i);
                            continue;
                        }
                    }
                    if (triangleB.sidedness != TriangleSidedness.DoubleSided)
                    {
                        float dot;
                        Vector3.Dot(ref normalB, ref contact.Normal, out dot);
                        if (dot < 0)
                        {
                            contactList.RemoveAt(i);
                            continue;
                        }
                    }

                }
            return contactList.count > 0;


        }

        bool IntersectSegmentAgainstPlane(ref Vector3 a, ref Vector3 b, ref Vector3 normal, out Vector3 intersection)
        {
            //This method is only used to intersect triangle A edges against triangle B's plane.
            //Triangle B's plane is aligned with the origin, so we don't have to worry about the d component of the plane.
            Vector3 direction;
            Vector3.Subtract(ref b, ref a, out direction);
            //The normal is normalized.
            float dot;
            Vector3.Dot(ref normal, ref direction, out dot);
            float distanceFromPlane;
            Vector3.Dot(ref a, ref normal, out distanceFromPlane);
            float t;
            if (Math.Abs(dot) > 0)
                t = -distanceFromPlane / dot;
            else
            {
                //The line is parallel to the plane's surface.
                intersection = new Vector3();
                t = float.MaxValue;
                return false;
            }
            if (t < 0 || t > 1)
            {
                //The line intersects, but the segment does not.
                intersection = new Vector3();
                return false;
            }

            //It intersects and it's within the range, so compute the intersection!
            Vector3.Multiply(ref direction, t, out intersection);
            Vector3.Add(ref intersection, ref a, out intersection);
            return true;



        }

        bool IsPointInTriangle(ref Vector3 point, ref Vector3 a, ref Vector3 b, ref Vector3 c, ref Vector3 normal, float offset, out Vector3 closestPointOnTriangle)
        {
            Vector3.Multiply(ref normal, offset, out closestPointOnTriangle);
            Vector3.Add(ref point, ref closestPointOnTriangle, out closestPointOnTriangle);
            //Technically, the information computed by our caller could optimize this test.
            return Toolbox.IsPointInsideTriangle(ref a, ref b, ref c, ref closestPointOnTriangle);
        }


        public override VoronoiRegion GetRegion(ref ContactData contact)
        {
            return (VoronoiRegion)contact.Id;
        }


        public override bool ShouldCorrectContactNormal
        {
            get
            {
                return wasInDeepContact;
            }
        }

        ///<summary>
        /// Initializes the pair tester.
        ///</summary>
        ///<param name="convex">Convex shape to use.</param>
        ///<param name="triangle">Triangle shape to use.</param>
        public override void Initialize(ConvexShape convex, TriangleShape triangle)
        {
            this.triangleB = (TriangleShape)convex;
            this.triangle = triangle;
        }

        /// <summary>
        /// Cleans up the pair tester.
        /// </summary>
        public override void CleanUp()
        {
            triangle = null;
            triangleB = null;
            Updated = false;
        }
    }
}
