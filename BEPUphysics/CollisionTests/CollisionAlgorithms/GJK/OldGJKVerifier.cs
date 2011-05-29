using System.Collections.Generic;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;
using BEPUphysics.ResourceManagement;

namespace BEPUphysics.CollisionTests.CollisionAlgorithms.GJK
{
    public static class OldGJKVerifier
    {



        /// <summary>
        /// Determines the closest points between two entities.
        /// </summary>
        /// <param name="objA">First entity for testing.</param>
        /// <param name="objB">Second entity for testing.</param>
        /// <param name="positionA">Location to consider as the geometric center of the first object.</param>
        /// <param name="positionB">Location to consider as the geometric center of the second object.</param>
        /// <param name="orientationA">Orientation to use in lieu of the first object's rotation.</param>
        /// <param name="orientationB">Orientation to use in lieu of the second object's rotation.</param>
        /// <param name="marginA">Extra space around the first entity.</param>
        /// <param name="marginB">Extra space around the second entity.</param>
        /// <param name="closestA">Closest point on objA.</param>
        /// <param name="closestB">Closest point on objB.</param>
        /// <returns>Separation vector between the two objects.</returns>
        public static bool GetClosestPointsBetweenObjects(ConvexShape objA, ConvexShape objB, ref RigidTransform transformA, ref RigidTransform transformB,
                                                             float marginA, float marginB, out Vector3 closestA, out Vector3 closestB)
        {

            closestA = Toolbox.ZeroVector;
            closestB = Toolbox.ZeroVector;
            List<Vector3> tempQ = Resources.GetVectorList();
            List<Vector3> tempA = Resources.GetVectorList();
            List<Vector3> tempB = Resources.GetVectorList();
            List<Vector3> q = Resources.GetVectorList();
            List<Vector3> a = Resources.GetVectorList();
            List<Vector3> b = Resources.GetVectorList();
            List<int> subsimplex = Resources.GetIntList();
            List<float> baryCoords = Resources.GetFloatList();

            Vector3 p = Toolbox.ZeroVector, v;
            Vector3 pv;
            float dot;


            float lastDot = float.MaxValue;
            //Initialize the set.

            a.Add(transformA.Position);
            b.Add(transformB.Position);
            Vector3.Subtract(ref transformA.Position, ref transformB.Position, out v);
            q.Add(v);

            int count = 0;
            float max = -float.MaxValue;
            while (true)
            {
                //Find the closest point on q to the origin.
                FindPointOfMinimumNorm(q, subsimplex, baryCoords, out p);
                //If p is the origin, then the objects are intersecting.
                if (p.LengthSquared() < Toolbox.Epsilon)
                {
                    Resources.GiveBack(tempQ);
                    Resources.GiveBack(tempA);
                    Resources.GiveBack(tempB);
                    Resources.GiveBack(q);
                    Resources.GiveBack(a);
                    Resources.GiveBack(b);
                    Resources.GiveBack(subsimplex);
                    Resources.GiveBack(baryCoords);

                    return true;
                }

                //Reduce the simplex to the smallest subset.
                tempQ.Clear();
                tempA.Clear();
                tempB.Clear();
                foreach (int k in subsimplex)
                {
                    //Recompile the simplices.
                    tempQ.Add(q[k]);
                    tempA.Add(a[k]);
                    tempB.Add(b[k]);
                }
                q.Clear();
                a.Clear();
                b.Clear();
                for (int k = 0; k < tempQ.Count; k++)
                {
                    q.Add(tempQ[k]);
                    a.Add(tempA[k]);
                    b.Add(tempB[k]);
                }

                //find v, a supporting point in direction -p.
                Vector3 negP;
                Vector3.Negate(ref p, out negP);
                Vector3 aCandidate;
                objA.GetExtremePointWithoutMargin(negP, ref transformA, out aCandidate);
                Vector3 bCandidate;
                objB.GetExtremePointWithoutMargin(p, ref transformB, out bCandidate);
                Vector3.Subtract(ref aCandidate, ref bCandidate, out v);
                //If v is no more extreme than p, return ||p|| as the distance.
                Vector3.Add(ref v, ref negP, out pv);
                Vector3.Dot(ref pv, ref negP, out dot);
                if (dot <= Toolbox.Epsilon * max || (count > 20 && dot <= lastDot) || count > 30)
                {
                    //Calculate this iteration's closest points using the subsimplex list.
                    GetBarycenter(a, baryCoords, out closestA);
                    GetBarycenter(b, baryCoords, out closestB);

                    Resources.GiveBack(tempQ);
                    Resources.GiveBack(tempA);
                    Resources.GiveBack(tempB);
                    Resources.GiveBack(q);
                    Resources.GiveBack(a);
                    Resources.GiveBack(b);
                    Resources.GiveBack(subsimplex);
                    Resources.GiveBack(baryCoords);

                    return false;
                }
                lastDot = dot;
                //Add v to the set and go to the next iteration.
                q.Add(v);
                a.Add(aCandidate);
                b.Add(bCandidate);

                count++;
                max = -float.MaxValue;
                foreach (Vector3 y in q)
                {
                    float lengthSquared = y.LengthSquared();
                    if (lengthSquared > max)
                        max = lengthSquared;
                }
            }
        }


        /// <summary>
        /// Applies barycentric coordinates to a set of points.
        /// </summary>
        /// <param name="q">Set of points to apply coordinates to.</param>
        /// <param name="baryCoords">Barycentric coordinates to apply to the set.</param>
        /// <param name="barycenter">Barycenter of the set defined by the coordinates.</param>
        internal static void GetBarycenter(List<Vector3> q, List<float> baryCoords, out Vector3 barycenter)
        {
            barycenter = Toolbox.ZeroVector;
            for (int k = 0; k < q.Count; k++)
            {
                Vector3 temp = q[k];
                Vector3.Multiply(ref temp, baryCoords[k], out temp);
                Vector3.Add(ref barycenter, ref temp, out barycenter);
            }
        }

        #region GJK Helper Methods

        /// <summary>
        /// Determines the point closest to the origin from the convex hull of the set of points q.
        /// Set of points must be between 1 and 4 points.
        /// </summary>
        /// <param name="q">Set of 1-4 points.</param>
        /// <param name="subsimplex">The source of the voronoi region which contains the point, referencing the input list indices.</param>
        /// <param name="baryCoords">Barycentric coordinates of the point on the subsimplex closest to the origin.</param>
        /// <param name="closestPoint">Closest point on the hull of the set to the origin.</param>
        public static void FindPointOfMinimumNorm(List<Vector3> q, List<int> subsimplex, List<float> baryCoords, out Vector3 closestPoint)
        {
            subsimplex.Clear();
            baryCoords.Clear();
            if (q.Count == 4)
            {
                if (q[0] == q[1] || q[0] == q[2] || q[0] == q[3]) //Degenerate Tetrahedron
                    GetClosestPointOnTriangleToPoint(q, 1, 2, 3, ref Toolbox.ZeroVector, subsimplex, baryCoords, out closestPoint);
                else if (q[1] == q[2] || q[1] == q[3])
                    GetClosestPointOnTriangleToPoint(q, 0, 2, 3, ref Toolbox.ZeroVector, subsimplex, baryCoords, out closestPoint);
                else if (q[2] == q[3])
                    GetClosestPointOnTriangleToPoint(q, 0, 1, 2, ref Toolbox.ZeroVector, subsimplex, baryCoords, out closestPoint);
                else GetClosestPointOnTetrahedronToPoint(q, ref Toolbox.ZeroVector, subsimplex, baryCoords, out closestPoint);
            }
            else if (q.Count == 3)
            {
                if (q[0] == q[1] || q[0] == q[2])
                    GetClosestPointOnSegmentToPoint(q, 1, 2, ref Toolbox.ZeroVector, subsimplex, baryCoords, out closestPoint);
                else if (q[1] == q[2])
                    GetClosestPointOnSegmentToPoint(q, 0, 1, ref Toolbox.ZeroVector, subsimplex, baryCoords, out closestPoint);
                else GetClosestPointOnTriangleToPoint(q, 0, 1, 2, ref Toolbox.ZeroVector, subsimplex, baryCoords, out closestPoint);
            }
            else if (q.Count == 2)
            {
                if (q[0] == q[1])
                {
                    subsimplex.Add(0);
                    baryCoords.Add(1);
                    closestPoint = q[0];
                }
                else GetClosestPointOnSegmentToPoint(q, 0, 1, ref Toolbox.ZeroVector, subsimplex, baryCoords, out closestPoint);
            }
            else if (q.Count == 1)
            {
                subsimplex.Add(0);
                baryCoords.Add(1);
                closestPoint = q[0];
            }
            else closestPoint = Toolbox.NoVector;
        }

        #endregion

        /// <summary>
        /// Determines the closest point on the provided segment ab to point p.
        /// </summary>
        /// <param name="q">List of points in the containing simplex.</param>
        /// <param name="i">Index of first endpoint of segment.</param>
        /// <param name="j">Index of second endpoint of segment.</param>
        /// <param name="p">Point for comparison.</param>
        /// <param name="subsimplex">The source of the voronoi region which contains the point, enumerated as a = 0, b = 1.</param>
        /// <param name="baryCoords">Barycentric coordinates of the point.</param>
        /// <param name="closestPoint">Closest point on the edge to p.</param>
        public static void GetClosestPointOnSegmentToPoint(List<Vector3> q, int i, int j, ref Vector3 p, List<int> subsimplex, List<float> baryCoords, out Vector3 closestPoint)
        {
            Vector3 a = q[i];
            Vector3 b = q[j];
            subsimplex.Clear();
            baryCoords.Clear();
            Vector3 ab;
            Vector3.Subtract(ref b, ref a, out ab);
            Vector3 ap;
            Vector3.Subtract(ref p, ref a, out ap);
            float t;
            Vector3.Dot(ref ap, ref ab, out t);
            if (t <= 0)
            {
                subsimplex.Add(i);
                baryCoords.Add(1);
                closestPoint = a;
            }
            else
            {
                float denom = ab.X * ab.X + ab.Y * ab.Y + ab.Z * ab.Z;
                if (t >= denom)
                {
                    subsimplex.Add(j);
                    baryCoords.Add(1);
                    closestPoint = b;
                }
                else
                {
                    t = t / denom;
                    subsimplex.Add(i);
                    subsimplex.Add(j);
                    baryCoords.Add(1 - t);
                    baryCoords.Add(t);
                    Vector3 tab;
                    Vector3.Multiply(ref ab, t, out tab);
                    Vector3.Add(ref a, ref tab, out closestPoint);
                }
            }
        }

        /// <summary>
        /// Determines the closest point on a triangle given by points a, b, and c to point p and provides the subsimplex whose voronoi region contains the point.
        /// </summary>
        /// <param name="q">Simplex containing triangle for testing.</param>
        /// <param name="i">Index of first vertex of triangle.</param>
        /// <param name="j">Index of second vertex of triangle.</param>
        /// <param name="k">Index of third vertex of triangle.</param>
        /// <param name="p">Point for comparison.</param>
        /// <param name="subsimplex">The source of the voronoi region which contains the point, enumerated as a = 0, b = 1, c = 2.</param>
        /// <param name="baryCoords">Barycentric coordinates of the point on the triangle.</param>
        /// <param name="closestPoint">Closest point on tetrahedron to point.</param>
        public static void GetClosestPointOnTriangleToPoint(List<Vector3> q, int i, int j, int k, ref Vector3 p, List<int> subsimplex, List<float> baryCoords, out Vector3 closestPoint)
        {
            subsimplex.Clear();
            baryCoords.Clear();
            float v, w;
            Vector3 a = q[i];
            Vector3 b = q[j];
            Vector3 c = q[k];
            Vector3 ab;
            Vector3.Subtract(ref b, ref a, out ab);
            Vector3 ac;
            Vector3.Subtract(ref c, ref a, out ac);
            //Vertex region A?
            Vector3 ap;
            Vector3.Subtract(ref p, ref a, out ap);
            float d1;
            Vector3.Dot(ref ab, ref ap, out d1);
            float d2;
            Vector3.Dot(ref ac, ref ap, out d2);
            if (d1 <= 0 && d2 < 0)
            {
                subsimplex.Add(i);
                baryCoords.Add(1);
                closestPoint = a;
                return; //barycentric coordinates (1,0,0)
            }
            //Vertex region B?
            Vector3 bp;
            Vector3.Subtract(ref p, ref b, out bp);
            float d3;
            Vector3.Dot(ref ab, ref bp, out d3);
            float d4;
            Vector3.Dot(ref ac, ref bp, out d4);
            if (d3 >= 0 && d4 <= d3)
            {
                subsimplex.Add(j);
                baryCoords.Add(1);
                closestPoint = b;
                return; //barycentric coordinates (0,1,0)
            }
            //Edge region AB?
            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0 && d1 >= 0 && d3 <= 0)
            {
                subsimplex.Add(i);
                subsimplex.Add(j);
                v = d1 / (d1 - d3);
                baryCoords.Add(1 - v);
                baryCoords.Add(v);
                Vector3.Multiply(ref ab, v, out closestPoint);
                Vector3.Add(ref closestPoint, ref a, out closestPoint);
                return; //barycentric coordinates (1-v, v, 0)
            }
            //Vertex region C?
            Vector3 cp;
            Vector3.Subtract(ref p, ref c, out cp);
            float d5;
            Vector3.Dot(ref ab, ref cp, out d5);
            float d6;
            Vector3.Dot(ref ac, ref cp, out d6);
            if (d6 >= 0 && d5 <= d6)
            {
                subsimplex.Add(k);
                baryCoords.Add(1);
                closestPoint = c;
                return; //barycentric coordinates (0,0,1)
            }
            //Edge region AC?
            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0 && d2 >= 0 && d6 <= 0)
            {
                subsimplex.Add(i);
                subsimplex.Add(k);
                w = d2 / (d2 - d6);
                baryCoords.Add(1 - w);
                baryCoords.Add(w);
                Vector3.Multiply(ref ac, w, out closestPoint);
                Vector3.Add(ref closestPoint, ref a, out closestPoint);
                return; //barycentric coordinates (1-w, 0, w)
            }
            //Edge region BC?
            float va = d3 * d6 - d5 * d4;
            if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
            {
                subsimplex.Add(j);
                subsimplex.Add(k);
                w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                baryCoords.Add(1 - w);
                baryCoords.Add(w);
                Vector3.Subtract(ref c, ref b, out closestPoint);
                Vector3.Multiply(ref closestPoint, w, out closestPoint);
                Vector3.Add(ref closestPoint, ref b, out closestPoint);
                return; //barycentric coordinates (0, 1 - w ,w)
            }
            //Inside triangle?
            subsimplex.Add(i);
            subsimplex.Add(j);
            subsimplex.Add(k);
            float denom = 1 / (va + vb + vc);
            v = vb * denom;
            w = vc * denom;
            baryCoords.Add(1 - v - w);
            baryCoords.Add(v);
            baryCoords.Add(w);
            Vector3 abv;
            Vector3.Multiply(ref ab, v, out abv);
            Vector3 acw;
            Vector3.Multiply(ref ac, w, out acw);
            Vector3.Add(ref a, ref abv, out closestPoint);
            Vector3.Add(ref closestPoint, ref acw, out closestPoint);
            //return a + ab * v + ac * w; //barycentric coordinates (1 - v - w, v, w)
        }


        /// <summary>
        /// Determines the closest point on a tetrahedron to a provided point p.
        /// </summary>
        /// <param name="tetrahedron">List of 4 points composing the tetrahedron.</param>
        /// <param name="p">Point for comparison.</param>
        /// <param name="subsimplex">The source of the voronoi region which contains the point, enumerated as a = 0, b = 1, c = 2, d = 3.</param>
        /// <param name="baryCoords">Barycentric coordinates of p on the tetrahedron.</param>
        /// <param name="closestPoint">Closest point on the tetrahedron to the point.</param>
        public static void GetClosestPointOnTetrahedronToPoint(List<Vector3> tetrahedron, ref Vector3 p, List<int> subsimplex, List<float> baryCoords, out Vector3 closestPoint)
        {
            List<int> subsimplexCandidate = Resources.GetIntList();
            List<float> baryCoordsCandidate = Resources.GetFloatList();
            Vector3 a = tetrahedron[0];
            Vector3 b = tetrahedron[1];
            Vector3 c = tetrahedron[2];
            Vector3 d = tetrahedron[3];
            closestPoint = p;
            Vector3 pq;
            float bestSqDist = float.MaxValue;
            subsimplex.Clear();
            subsimplex.Add(0); //Provides a baseline; if the object is not outside of any planes, then it's inside and the subsimplex is the tetrahedron itself.
            subsimplex.Add(1);
            subsimplex.Add(2);
            subsimplex.Add(3);
            baryCoords.Clear();
            Vector3 q;
            bool baryCoordsFound = false;

            // If point outside face abc then compute closest point on abc
            if (ArePointsOnOppositeSidesOfPlane(ref p, ref d, ref a, ref b, ref c))
            {
                GetClosestPointOnTriangleToPoint(tetrahedron, 0, 1, 2, ref p, subsimplexCandidate, baryCoordsCandidate, out q);
                Vector3.Subtract(ref q, ref p, out pq);
                float sqDist = pq.LengthSquared();
                // Update best closest point if (squared) distance is less than current best
                if (sqDist < bestSqDist)
                {
                    bestSqDist = sqDist;
                    closestPoint = q;
                    subsimplex.Clear();
                    baryCoords.Clear();
                    for (int k = 0; k < subsimplexCandidate.Count; k++)
                    {
                        subsimplex.Add(subsimplexCandidate[k]);
                        baryCoords.Add(baryCoordsCandidate[k]);
                    }
                    //subsimplex.AddRange(subsimplexCandidate);
                    //baryCoords.AddRange(baryCoordsCandidate);
                    baryCoordsFound = true;
                }
            }
            // Repeat test for face acd
            if (ArePointsOnOppositeSidesOfPlane(ref p, ref b, ref a, ref c, ref d))
            {
                GetClosestPointOnTriangleToPoint(tetrahedron, 0, 2, 3, ref p, subsimplexCandidate, baryCoordsCandidate, out q);
                Vector3.Subtract(ref q, ref p, out pq);
                float sqDist = pq.LengthSquared();
                if (sqDist < bestSqDist)
                {
                    bestSqDist = sqDist;
                    closestPoint = q;
                    subsimplex.Clear();
                    baryCoords.Clear();
                    for (int k = 0; k < subsimplexCandidate.Count; k++)
                    {
                        subsimplex.Add(subsimplexCandidate[k]);
                        baryCoords.Add(baryCoordsCandidate[k]);
                    }
                    //subsimplex.AddRange(subsimplexCandidate);
                    //baryCoords.AddRange(baryCoordsCandidate);
                    baryCoordsFound = true;
                }
            }
            // Repeat test for face adb
            if (ArePointsOnOppositeSidesOfPlane(ref p, ref c, ref a, ref d, ref b))
            {
                GetClosestPointOnTriangleToPoint(tetrahedron, 0, 3, 1, ref p, subsimplexCandidate, baryCoordsCandidate, out q);
                Vector3.Subtract(ref q, ref p, out pq);
                float sqDist = pq.LengthSquared();
                if (sqDist < bestSqDist)
                {
                    bestSqDist = sqDist;
                    closestPoint = q;
                    subsimplex.Clear();
                    baryCoords.Clear();
                    for (int k = 0; k < subsimplexCandidate.Count; k++)
                    {
                        subsimplex.Add(subsimplexCandidate[k]);
                        baryCoords.Add(baryCoordsCandidate[k]);
                    }
                    //subsimplex.AddRange(subsimplexCandidate);
                    //baryCoords.AddRange(baryCoordsCandidate);
                    baryCoordsFound = true;
                }
            }
            // Repeat test for face bdc
            if (ArePointsOnOppositeSidesOfPlane(ref p, ref a, ref b, ref d, ref c))
            {
                GetClosestPointOnTriangleToPoint(tetrahedron, 1, 3, 2, ref p, subsimplexCandidate, baryCoordsCandidate, out q);
                Vector3.Subtract(ref q, ref p, out pq);
                float sqDist = pq.LengthSquared();
                if (sqDist < bestSqDist)
                {
                    closestPoint = q;
                    subsimplex.Clear();
                    baryCoords.Clear();
                    for (int k = 0; k < subsimplexCandidate.Count; k++)
                    {
                        subsimplex.Add(subsimplexCandidate[k]);
                        baryCoords.Add(baryCoordsCandidate[k]);
                    }
                    //subsimplex.AddRange(subsimplexCandidate);
                    //baryCoords.AddRange(baryCoordsCandidate);
                    baryCoordsFound = true;
                }
            }
            if (!baryCoordsFound)
            {
                //subsimplex is the entire tetrahedron, can only occur when objects intersect!  Determinants of each of the tetrahedrons based on triangles composing the sides and the point itself.
                //This is basically computing the volume of parallelepipeds (triple scalar product).
                //Could be quicker just to do it directly.
                float abcd = (new Matrix(tetrahedron[0].X, tetrahedron[0].Y, tetrahedron[0].Z, 1,
                                         tetrahedron[1].X, tetrahedron[1].Y, tetrahedron[1].Z, 1,
                                         tetrahedron[2].X, tetrahedron[2].Y, tetrahedron[2].Z, 1,
                                         tetrahedron[3].X, tetrahedron[3].Y, tetrahedron[3].Z, 1)).Determinant();
                float pbcd = (new Matrix(p.X, p.Y, p.Z, 1,
                                         tetrahedron[1].X, tetrahedron[1].Y, tetrahedron[1].Z, 1,
                                         tetrahedron[2].X, tetrahedron[2].Y, tetrahedron[2].Z, 1,
                                         tetrahedron[3].X, tetrahedron[3].Y, tetrahedron[3].Z, 1)).Determinant();
                float apcd = (new Matrix(tetrahedron[0].X, tetrahedron[0].Y, tetrahedron[0].Z, 1,
                                         p.X, p.Y, p.Z, 1,
                                         tetrahedron[2].X, tetrahedron[2].Y, tetrahedron[2].Z, 1,
                                         tetrahedron[3].X, tetrahedron[3].Y, tetrahedron[3].Z, 1)).Determinant();
                float abpd = (new Matrix(tetrahedron[0].X, tetrahedron[0].Y, tetrahedron[0].Z, 1,
                                         tetrahedron[1].X, tetrahedron[1].Y, tetrahedron[1].Z, 1,
                                         p.X, p.Y, p.Z, 1,
                                         tetrahedron[3].X, tetrahedron[3].Y, tetrahedron[3].Z, 1)).Determinant();
                abcd = 1 / abcd;
                baryCoords.Add(pbcd * abcd); //u
                baryCoords.Add(apcd * abcd); //v
                baryCoords.Add(abpd * abcd); //w
                baryCoords.Add(1 - baryCoords[0] - baryCoords[1] - baryCoords[2]); //x = 1-u-v-w
            }
            Resources.GiveBack(subsimplexCandidate);
            Resources.GiveBack(baryCoordsCandidate);
        }


        /// <summary>
        /// Determines if vectors o and p are on opposite sides of the plane defined by a, b, and c.
        /// </summary>
        /// <param name="o">First point for comparison.</param>
        /// <param name="p">Second point for comparison.</param>
        /// <param name="a">First vertex of the plane.</param>
        /// <param name="b">Second vertex of plane.</param>
        /// <param name="c">Third vertex of plane.</param>
        /// <returns>Whether or not vectors o and p reside on opposite sides of the plane.</returns>
        public static bool ArePointsOnOppositeSidesOfPlane(ref Vector3 o, ref Vector3 p, ref Vector3 a, ref Vector3 b, ref Vector3 c)
        {
            Vector3 ab, ac, ap, ao;
            Vector3.Subtract(ref b, ref a, out ab);
            Vector3.Subtract(ref c, ref a, out ac);
            Vector3.Subtract(ref p, ref a, out ap);
            Vector3.Subtract(ref o, ref a, out ao);
            Vector3 q;
            Vector3.Cross(ref ab, ref ac, out q);
            float signp;
            Vector3.Dot(ref ap, ref q, out signp);
            float signo;
            Vector3.Dot(ref ao, ref q, out signo);
            if (signp * signo <= 0)
                return true;
            return false;
        }




        /// <summary>
        /// Determines whether or not a finite ray intersects an entity.
        /// Uses a GJK-based algorithm for all entities.
        /// It is recommended that the normal rayCast method be used unless a problem is encountered.
        /// </summary>
        /// <param name="origin">Origin of the ray.</param>
        /// <param name="direction">Direction of the ray.</param>
        /// <param name="length">Maximum length of the ray in units of the magnitude of the direction.</param>
        /// <param name="target">Entity to check for intersection.</param>
        /// <param name="withMargin">Whether or not to include the collision margin of the body.</param>
        /// <param name="hitLocation">First point along the ray hitting the target entity.</param>
        /// <param name="hitNormal">Normal of the tangent plane of the object defined at the hit location.</param>
        /// <param name="t">Distance along the ray, in units of the direction's length, to the hit location.</param>
        /// <returns>Whether or not the ray intersects the entity.</returns>
        public static bool RayCastGJK(Vector3 origin, Vector3 direction, float length, ConvexShape target, RigidTransform transform, out Vector3 hitLocation, out Vector3 hitNormal, out float t)
        {
            origin -= transform.Position;
            Quaternion conjugate = Quaternion.Conjugate(transform.Orientation);
            origin = Vector3.Transform(origin, conjugate);
            direction = Vector3.Transform(direction, conjugate);

            Vector3 w, p;
            t = 0; // hit location parameter
            hitLocation = origin;
            hitNormal = Toolbox.ZeroVector;
            Vector3 v = hitLocation;
            List<Vector3> simplex = Resources.GetVectorList();
            List<int> subsimplex = Resources.GetIntList();
            List<float> baryCoords = Resources.GetFloatList(); //Keep em handy, maybe some day I'll have a use for them.
            List<Vector3> temp = Resources.GetVectorList();
            float vw, vdir;
            float max = -float.MaxValue;
            int count = 0;
            while (v.LengthSquared() > max * Toolbox.BigEpsilon)
            {
                count++;
                if (count > 50)
                {
                    hitLocation = Toolbox.NoVector;
                    hitNormal = Toolbox.NoVector;
                    Resources.GiveBack(simplex);
                    Resources.GiveBack(subsimplex);
                    Resources.GiveBack(baryCoords);
                    Resources.GiveBack(temp);
                    return false;
                }
                target.GetLocalExtremePoint(v, out p);
                w = hitLocation - p;
                vw = Vector3.Dot(v, w);
                if (vw > 0)
                {
                    vdir = Vector3.Dot(v, direction);
                    if (vdir >= 0)
                    {
                        hitLocation = Toolbox.NoVector;
                        hitNormal = Toolbox.NoVector;
                        Resources.GiveBack(simplex);
                        Resources.GiveBack(subsimplex);
                        Resources.GiveBack(baryCoords);
                        Resources.GiveBack(temp);
                        return false;
                    }
                    t = t - vw / vdir;
                    if (t > length)
                    {
                        hitLocation = Toolbox.NoVector;
                        hitNormal = Toolbox.NoVector;
                        Resources.GiveBack(simplex);
                        Resources.GiveBack(subsimplex);
                        Resources.GiveBack(baryCoords);
                        Resources.GiveBack(temp);
                        return false;
                    }
                    hitLocation = origin + t * direction;
                    hitNormal = v;
                }
                temp.Clear();
                simplex.Add(p);
                for (int k = 0; k < simplex.Count; k++)
                    temp.Add(hitLocation - simplex[k]);


                FindPointOfMinimumNorm(temp, subsimplex, baryCoords, out v);


                temp.Clear();
                for (int k = 0; k < subsimplex.Count; k++)
                    temp.Add(simplex[subsimplex[k]]);
                simplex.Clear();
                foreach (Vector3 ssv in temp)
                {
                    simplex.Add(ssv);
                }
                //simplex.AddRange(temp);

                max = -float.MaxValue;
                foreach (Vector3 y in simplex)
                {
                    float lengthSquared = (y - origin).LengthSquared();
                    if (lengthSquared > max)
                        max = lengthSquared;
                }
            }
            Resources.GiveBack(simplex);
            Resources.GiveBack(subsimplex);
            Resources.GiveBack(baryCoords);
            Resources.GiveBack(temp);

            hitNormal = Vector3.Transform(hitNormal, transform.Orientation);
            hitLocation = Vector3.Transform(hitLocation, transform.Orientation);
            hitLocation += transform.Position;
            return true;
        }



        public static bool ConvexCast(ConvexShape objA, ConvexShape objB, ref Vector3 sweepA, ref Vector3 sweepB, ref RigidTransform transformA, ref RigidTransform transformB, out RayHit hit)
        {
            Vector3 w, pA, pB, p;
            hit.T = 0;
            float t = 0; // hit location parameter
            hit.Location = Toolbox.ZeroVector;
            hit.Normal = Toolbox.ZeroVector;
            Vector3 direction;
            Vector3.Subtract(ref sweepB, ref sweepA, out direction);
            Vector3 v;
            Vector3.Subtract(ref transformA.Position, ref transformB.Position, out v);
            List<Vector3> simplex = Resources.GetVectorList();
            List<Vector3> simplexA = Resources.GetVectorList();
            List<Vector3> simplexB = Resources.GetVectorList();
            List<int> subsimplex = Resources.GetIntList();
            List<float> baryCoords = Resources.GetFloatList();
            List<Vector3> temp = Resources.GetVectorList();
            List<Vector3> tempA = Resources.GetVectorList();
            List<Vector3> tempB = Resources.GetVectorList();
            float vw, vdir;
            int count = 0;
            float max = -float.MaxValue;
            float lengthSquared = 0;
            while (v.LengthSquared() > Toolbox.Epsilon * max || count < 1)
            {
                objA.GetExtremePoint(v, ref transformA, out pA);
                Vector3 negV;
                Vector3.Negate(ref v, out negV);
                objB.GetExtremePoint(negV, ref transformB, out pB);
                Vector3.Subtract(ref pA, ref pB, out p);
                //p = findMinkowskiDifferenceExtremePoint(objA, objB, v, marginA, marginB);
                Vector3.Subtract(ref hit.Location, ref p, out w);
                Vector3.Dot(ref v, ref w, out vw);
                if (vw > 0)
                {
                    Vector3.Dot(ref v, ref direction, out vdir);
                    if (vdir >= 0)
                    {
                        hit.Location = Toolbox.NoVector;
                        hit.Normal = Toolbox.NoVector;
                        hit.T = float.NaN;
                        Resources.GiveBack(simplex);
                        Resources.GiveBack(simplexA);
                        Resources.GiveBack(simplexB);
                        Resources.GiveBack(subsimplex);
                        Resources.GiveBack(baryCoords);
                        Resources.GiveBack(temp);
                        Resources.GiveBack(tempA);
                        Resources.GiveBack(tempB);
                        return false;
                    }
                    t = t - vw / vdir;
                    if (t > 1)
                    {
                        hit.Location = Toolbox.NoVector;
                        hit.Normal = Toolbox.NoVector;
                        hit.T = float.NaN;
                        Resources.GiveBack(simplex);
                        Resources.GiveBack(simplexA);
                        Resources.GiveBack(simplexB);
                        Resources.GiveBack(subsimplex);
                        Resources.GiveBack(baryCoords);
                        Resources.GiveBack(temp);
                        Resources.GiveBack(tempA);
                        Resources.GiveBack(tempB);
                        return false;
                    }
                    Vector3.Multiply(ref direction, t, out hit.Location);
                    hit.T = t;
                    hit.Normal = v;
                }
                temp.Clear();
                tempA.Clear();
                tempB.Clear();
                simplex.Add(p);
                simplexA.Add(pA);
                simplexB.Add(pB);
                for (int k = 0; k < simplex.Count; k++)
                    temp.Add(hit.Location - simplex[k]);
                FindPointOfMinimumNorm(temp, subsimplex, baryCoords, out v);
                temp.Clear();
                for (int k = 0; k < subsimplex.Count; k++)
                {
                    temp.Add(simplex[subsimplex[k]]);
                    tempA.Add(simplexA[subsimplex[k]]);
                    tempB.Add(simplexB[subsimplex[k]]);
                }
                simplex.Clear();
                simplexA.Clear();
                simplexB.Clear();
                for (int k = 0; k < temp.Count; k++)
                {
                    simplex.Add(temp[k]);
                    simplexA.Add(tempA[k]);
                    simplexB.Add(tempB[k]);
                }
                count++;
                if (count > 50)
                {
                    hit.Location = Toolbox.NoVector;
                    hit.Normal = Toolbox.NoVector;
                    hit.T = float.NaN;
                    Resources.GiveBack(simplex);
                    Resources.GiveBack(simplexA);
                    Resources.GiveBack(simplexB);
                    Resources.GiveBack(subsimplex);
                    Resources.GiveBack(baryCoords);
                    Resources.GiveBack(temp);
                    Resources.GiveBack(tempA);
                    Resources.GiveBack(tempB);
                    return false;
                }

                max = -float.MaxValue;
                foreach (Vector3 y in simplex)
                {
                    lengthSquared = y.LengthSquared();
                    if (lengthSquared > max)
                        max = lengthSquared;
                }
                //Debug.WriteLine(max);
                //max = Math.Max(200, max);
            }
            //TODO: See if this sort of average is needed.  Its a little overhead!
            hit.Normal.Normalize();
            Vector3 aSimplex;
            GetBarycenter(simplexA, baryCoords, out aSimplex);
            Vector3 bSimplex;
            GetBarycenter(simplexB, baryCoords, out bSimplex);
            Vector3 sweepAToi, sweepBToi;
            Vector3.Multiply(ref sweepA, hit.T, out sweepAToi);
            Vector3.Multiply(ref sweepB, hit.T, out sweepBToi);
            Vector3 added;
            Vector3.Add(ref aSimplex, ref bSimplex, out added);
            Vector3.Add(ref added, ref sweepAToi, out added);
            Vector3.Add(ref added, ref sweepBToi, out added);
            Vector3.Multiply(ref added, .5f, out hit.Location);
            //hitLocation = (aSimplex + sweepA * toi + bSimplex + sweepB * toi) / 2;
            //Debug.WriteLine(count);
            Resources.GiveBack(simplex);
            Resources.GiveBack(simplexA);
            Resources.GiveBack(simplexB);
            Resources.GiveBack(subsimplex);
            Resources.GiveBack(baryCoords);
            Resources.GiveBack(temp);
            Resources.GiveBack(tempA);
            Resources.GiveBack(tempB);
            return true;
        }
    }
}
