using System;
using System.Collections.Generic;
using BEPUphysics.Entities;
using BEPUphysics.ResourceManagement;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.MathExtensions;
using BEPUphysics.CollisionTests;
using BEPUphysics.DataStructures;

namespace BEPUphysics
{
    //TODO: It would be nice to split and improve this monolith into individually superior, organized components.


    /// <summary>
    /// Helper class with many algorithms for intersection testing and 3D math.
    /// </summary>
    public static class Toolbox
    {
        /// <summary>
        /// Large tolerance value.
        /// </summary>
        public const float BigEpsilon = 1E-5f;

        /// <summary>
        /// Tolerance value.
        /// </summary>
        public const float Epsilon = 1E-7f;

        /// <summary>
        /// Represents an invalid Vector3.
        /// </summary>
        public static readonly Vector3 NoVector = new Vector3(-float.MaxValue, -float.MaxValue, -float.MaxValue);

        /// <summary>
        /// Reference for a vector with dimensions (0,0,1).
        /// </summary>
        public static Vector3 BackVector = Vector3.Backward;

        /// <summary>
        /// Reference for a vector with dimensions (0,-1,0).
        /// </summary>
        public static Vector3 DownVector = Vector3.Down;

        /// <summary>
        /// Reference for a vector with dimensions (0,0,-1).
        /// </summary>
        public static Vector3 ForwardVector = Vector3.Forward;

        /// <summary>
        /// Refers to the identity quaternion.
        /// </summary>
        public static Quaternion IdentityOrientation = Quaternion.Identity;

        /// <summary>
        /// Reference for a vector with dimensions (-1,0,0).
        /// </summary>
        public static Vector3 LeftVector = Vector3.Left;

        /// <summary>
        /// Reference for a vector with dimensions (1,0,0).
        /// </summary>
        public static Vector3 RightVector = Vector3.Right;

        /// <summary>
        /// Reference for a vector with dimensions (0,1,0).
        /// </summary>
        public static Vector3 UpVector = Vector3.Up;

        /// <summary>
        /// Matrix containing zeroes for every element.
        /// </summary>
        public static Matrix ZeroMatrix = new Matrix(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

        /// <summary>
        /// Reference for a vector with dimensions (0,0,0).
        /// </summary>
        public static Vector3 ZeroVector = Vector3.Zero;

        /// <summary>
        /// Refers to the rigid identity transformation.
        /// </summary>
        public static RigidTransform RigidIdentity = RigidTransform.Identity;

        #region Segment/Ray-Triangle Tests

        /// <summary>
        /// Determines the intersection between a ray and a triangle.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length to travel in units of the direction's length.</param>
        /// <param name="sidedness">Sidedness of the triangle to test.</param>
        /// <param name="a">First vertex of the triangle.</param>
        /// <param name="b">Second vertex of the triangle.</param>
        /// <param name="c">Third vertex of the triangle.</param>
        /// <param name="hit">Hit data of the ray, if any</param>
        /// <returns>Whether or not the ray and triangle intersect.</returns>
        public static bool FindRayTriangleIntersection(ref Ray ray, float maximumLength, TriangleSidedness sidedness, ref Vector3 a, ref Vector3 b, ref Vector3 c, out RayHit hit)
        {
            hit = new RayHit();
            Vector3 ab, ac;
            Vector3.Subtract(ref b, ref a, out ab);
            Vector3.Subtract(ref c, ref a, out ac);

            Vector3.Cross(ref ab, ref ac, out hit.Normal);
            if (hit.Normal.LengthSquared() < Epsilon)
                return false; //Degenerate triangle!

            float d;
            Vector3.Dot(ref ray.Direction, ref hit.Normal, out d);
            d = -d;
            switch (sidedness)
            {
                case TriangleSidedness.DoubleSided:
                    if (d <= 0) //Pointing the wrong way.  Flip the normal.
                    {
                        Vector3.Negate(ref hit.Normal, out hit.Normal);
                        d = -d;
                    }
                    break;
                case TriangleSidedness.Clockwise:
                    if (d <= 0) //Pointing the wrong way.  Can't hit.
                        return false;

                    break;
                case TriangleSidedness.Counterclockwise:
                    if (d >= 0) //Pointing the wrong way.  Can't hit.
                        return false;

                    Vector3.Negate(ref hit.Normal, out hit.Normal);
                    d = -d;
                    break;
            }

            Vector3 ap;
            Vector3.Subtract(ref ray.Position, ref a, out ap);

            Vector3.Dot(ref ap, ref hit.Normal, out hit.T);
            hit.T /= d;
            if (hit.T < 0 || hit.T > maximumLength)
                return false;//Hit is behind origin, or too far away.

            Vector3.Multiply(ref ray.Direction, hit.T, out hit.Location);
            Vector3.Add(ref ray.Position, ref hit.Location, out hit.Location);

            // Compute barycentric coordinates
            Vector3.Subtract(ref hit.Location, ref a, out ap);
            float ABdotAB, ABdotAC, ABdotAP;
            float ACdotAC, ACdotAP;
            Vector3.Dot(ref ab, ref ab, out ABdotAB);
            Vector3.Dot(ref ab, ref ac, out ABdotAC);
            Vector3.Dot(ref ab, ref ap, out ABdotAP);
            Vector3.Dot(ref ac, ref ac, out ACdotAC);
            Vector3.Dot(ref ac, ref ap, out ACdotAP);

            float denom = 1 / (ABdotAB * ACdotAC - ABdotAC * ABdotAC);
            float u = (ACdotAC * ABdotAP - ABdotAC * ACdotAP) * denom;
            float v = (ABdotAB * ACdotAP - ABdotAC * ABdotAP) * denom;

            return (u >= -Toolbox.BigEpsilon) && (v >= -Toolbox.BigEpsilon) && (u + v <= 1 + Toolbox.BigEpsilon);

        }

        /// <summary>
        /// Finds the intersection between the given segment and the given plane defined by three points.
        /// </summary>
        /// <param name="a">First endpoint of segment.</param>
        /// <param name="b">Second endpoint of segment.</param>
        /// <param name="d">First vertex of a triangle which lies on the plane.</param>
        /// <param name="e">Second vertex of a triangle which lies on the plane.</param>
        /// <param name="f">Third vertex of a triangle which lies on the plane.</param>
        /// <param name="q">Intersection point.</param>
        /// <returns>Whether or not the segment intersects the plane.</returns>
        public static bool GetSegmentPlaneIntersection(Vector3 a, Vector3 b, Vector3 d, Vector3 e, Vector3 f, out Vector3 q)
        {
            Plane p;
            p.Normal = Vector3.Cross(e - d, f - d);
            p.D = Vector3.Dot(p.Normal, d);
            float t;
            return GetSegmentPlaneIntersection(a, b, p, out t, out q);
        }

        /// <summary>
        /// Finds the intersection between the given segment and the given plane.
        /// </summary>
        /// <param name="a">First endpoint of segment.</param>
        /// <param name="b">Second enpoint of segment.</param>
        /// <param name="p">Plane for comparison.</param>
        /// <param name="q">Intersection point.</param>
        /// <returns>Whether or not the segment intersects the plane.</returns>
        public static bool GetSegmentPlaneIntersection(Vector3 a, Vector3 b, Plane p, out Vector3 q)
        {
            float t;
            return GetLinePlaneIntersection(ref a, ref b, ref p, out t, out q) && t >= 0 && t <= 1;
        }

        /// <summary>
        /// Finds the intersection between the given segment and the given plane.
        /// </summary>
        /// <param name="a">First endpoint of segment.</param>
        /// <param name="b">Second endpoint of segment.</param>
        /// <param name="p">Plane for comparison.</param>
        /// <param name="t">Interval along segment to intersection.</param>
        /// <param name="q">Intersection point.</param>
        /// <returns>Whether or not the segment intersects the plane.</returns>
        public static bool GetSegmentPlaneIntersection(Vector3 a, Vector3 b, Plane p, out float t, out Vector3 q)
        {
            return GetLinePlaneIntersection(ref a, ref b, ref p, out t, out q) && t >= 0 && t <= 1;
        }

        /// <summary>
        /// Finds the intersection between the given line and the given plane.
        /// </summary>
        /// <param name="a">First endpoint of segment defining the line.</param>
        /// <param name="b">Second endpoint of segment defining the line.</param>
        /// <param name="p">Plane for comparison.</param>
        /// <param name="t">Interval along line to intersection (A + t * AB).</param>
        /// <param name="q">Intersection point.</param>
        /// <returns>Whether or not the line intersects the plane.  If false, the line is parallel to the plane's surface.</returns>
        public static bool GetLinePlaneIntersection(ref Vector3 a, ref Vector3 b, ref Plane p, out float t, out Vector3 q)
        {
            Vector3 ab;
            Vector3.Subtract(ref b, ref a, out ab);
            float denominator;
            Vector3.Dot(ref p.Normal, ref ab, out denominator);
            if (denominator < Epsilon && denominator > -Epsilon)
            {
                //Surface of plane and line are parallel (or very close to it).
                q = new Vector3();
                t = float.MaxValue;
                return false;
            }
            float numerator;
            Vector3.Dot(ref p.Normal, ref a, out numerator);
            t = (p.D - numerator) / denominator;
            //Compute the intersection position.
            Vector3.Multiply(ref ab, t, out q);
            Vector3.Add(ref a, ref q, out q);
            return true;
        }

        /// <summary>
        /// Finds the intersection between the given ray and the given plane.
        /// </summary>
        /// <param name="a">First endpoint of segment defining the line.</param>
        /// <param name="b">Second endpoint of segment defining the line.</param>
        /// <param name="p">Plane for comparison.</param>
        /// <param name="t">Interval along line to intersection (A + t * AB).</param>
        /// <param name="q">Intersection point.</param>
        /// <returns>Whether or not the line intersects the plane.  If false, the line is parallel to the plane's surface.</returns>
        public static bool GetRayPlaneIntersection(ref Ray ray, ref Plane p, out float t, out Vector3 q)
        {
            float denominator;
            Vector3.Dot(ref p.Normal, ref ray.Direction, out denominator);
            if (denominator < Epsilon && denominator > -Epsilon)
            {
                //Surface of plane and line are parallel (or very close to it).
                q = new Vector3();
                t = float.MaxValue;
                return false;
            }
            float numerator;
            Vector3.Dot(ref p.Normal, ref ray.Position, out numerator);
            t = (p.D - numerator) / denominator;
            //Compute the intersection position.
            Vector3.Multiply(ref ray.Direction, t, out q);
            Vector3.Add(ref ray.Position, ref q, out q);
            return t >= 0;
        }

        #endregion

        #region Point-Triangle Tests

        /// <summary>
        /// Determines the closest point on a triangle given by points a, b, and c to point p.
        /// </summary>
        /// <param name="a">First vertex of triangle.</param>
        /// <param name="b">Second vertex of triangle.</param>
        /// <param name="c">Third vertex of triangle.</param>
        /// <param name="p">Point for comparison.</param>
        /// <param name="closestPoint">Closest point on tetrahedron to point.</param>
        /// <returns>Voronoi region containing the closest point.</returns>
        public static VoronoiRegion GetClosestPointOnTriangleToPoint(ref Vector3 a, ref Vector3 b, ref Vector3 c, ref Vector3 p, out Vector3 closestPoint)
        {
            float v, w;
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
                closestPoint = a;
                return VoronoiRegion.A;
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
                closestPoint = b;
                return VoronoiRegion.B;
            }
            //Edge region AB?
            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0 && d1 >= 0 && d3 <= 0)
            {
                v = d1 / (d1 - d3);
                Vector3.Multiply(ref ab, v, out closestPoint);
                Vector3.Add(ref closestPoint, ref a, out closestPoint);
                return VoronoiRegion.AB;
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
                closestPoint = c;
                return VoronoiRegion.C;
            }
            //Edge region AC?
            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0 && d2 >= 0 && d6 <= 0)
            {
                w = d2 / (d2 - d6);
                Vector3.Multiply(ref ac, w, out closestPoint);
                Vector3.Add(ref closestPoint, ref a, out closestPoint);
                return VoronoiRegion.AC;
            }
            //Edge region BC?
            float va = d3 * d6 - d5 * d4;
            if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
            {
                w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                Vector3.Subtract(ref c, ref b, out closestPoint);
                Vector3.Multiply(ref closestPoint, w, out closestPoint);
                Vector3.Add(ref closestPoint, ref b, out closestPoint);
                return VoronoiRegion.BC;
            }
            //Inside triangle?
            float denom = 1 / (va + vb + vc);
            v = vb * denom;
            w = vc * denom;
            Vector3 abv;
            Vector3.Multiply(ref ab, v, out abv);
            Vector3 acw;
            Vector3.Multiply(ref ac, w, out acw);
            Vector3.Add(ref a, ref abv, out closestPoint);
            Vector3.Add(ref closestPoint, ref acw, out closestPoint);
            return VoronoiRegion.ABC;
        }

        /// <summary>
        /// Determines the closest point on a triangle given by points a, b, and c to point p and provides the subsimplex whose voronoi region contains the point.
        /// </summary>
        /// <param name="a">First vertex of triangle.</param>
        /// <param name="b">Second vertex of triangle.</param>
        /// <param name="c">Third vertex of triangle.</param>
        /// <param name="p">Point for comparison.</param>
        /// <param name="subsimplex">The source of the voronoi region which contains the point.</param>
        /// <param name="closestPoint">Closest point on tetrahedron to point.</param>
        public static void GetClosestPointOnTriangleToPoint(ref Vector3 a, ref Vector3 b, ref Vector3 c, ref Vector3 p, RawList<Vector3> subsimplex, out Vector3 closestPoint)
        {
            subsimplex.Clear();
            float v, w;
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
                subsimplex.Add(a);
                closestPoint = a;
                return;
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
                subsimplex.Add(b);
                closestPoint = b;
                return;
            }
            //Edge region AB?
            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0 && d1 >= 0 && d3 <= 0)
            {
                subsimplex.Add(a);
                subsimplex.Add(b);
                v = d1 / (d1 - d3);
                Vector3.Multiply(ref ab, v, out closestPoint);
                Vector3.Add(ref closestPoint, ref a, out closestPoint);
                return;
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
                subsimplex.Add(c);
                closestPoint = c;
                return;
            }
            //Edge region AC?
            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0 && d2 >= 0 && d6 <= 0)
            {
                subsimplex.Add(a);
                subsimplex.Add(c);
                w = d2 / (d2 - d6);
                Vector3.Multiply(ref ac, w, out closestPoint);
                Vector3.Add(ref closestPoint, ref a, out closestPoint);
                return;
            }
            //Edge region BC?
            float va = d3 * d6 - d5 * d4;
            if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
            {
                subsimplex.Add(b);
                subsimplex.Add(c);
                w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                Vector3.Subtract(ref c, ref b, out closestPoint);
                Vector3.Multiply(ref closestPoint, w, out closestPoint);
                Vector3.Add(ref closestPoint, ref b, out closestPoint);
                return;
            }
            //Inside triangle?
            subsimplex.Add(a);
            subsimplex.Add(b);
            subsimplex.Add(c);
            float denom = 1 / (va + vb + vc);
            v = vb * denom;
            w = vc * denom;
            Vector3 abv;
            Vector3.Multiply(ref ab, v, out abv);
            Vector3 acw;
            Vector3.Multiply(ref ac, w, out acw);
            Vector3.Add(ref a, ref abv, out closestPoint);
            Vector3.Add(ref closestPoint, ref acw, out closestPoint);
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
        public static void GetClosestPointOnTriangleToPoint(RawList<Vector3> q, int i, int j, int k, ref Vector3 p, RawList<int> subsimplex, RawList<float> baryCoords, out Vector3 closestPoint)
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
        /// Determines if supplied point is within the triangle as defined by the provided vertices.
        /// </summary>
        /// <param name="vA">A vertex of the triangle.</param>
        /// <param name="vB">A vertex of the triangle.</param>
        /// <param name="vC">A vertex of the triangle.</param>
        /// <param name="p">The point for comparison against the triangle.</param>
        /// <returns>Whether or not the point is within the triangle.</returns>
        public static bool IsPointInsideTriangle(ref Vector3 vA, ref Vector3 vB, ref Vector3 vC, ref Vector3 p)
        {
            float u, v, w;
            GetBarycentricCoordinates(ref p, ref vA, ref vB, ref vC, out u, out v, out w);
            //Are the barycoords valid?
            return (u > -Epsilon) && (v > -Epsilon) && (w > -Epsilon);
        }

        /// <summary>
        /// Determines if supplied point is within the triangle as defined by the provided vertices.
        /// </summary>
        /// <param name="vA">A vertex of the triangle.</param>
        /// <param name="vB">A vertex of the triangle.</param>
        /// <param name="vC">A vertex of the triangle.</param>
        /// <param name="p">The point for comparison against the triangle.</param>
        /// <param name="margin">Extra area on the edges of the triangle to include.  Can be negative.</param>
        /// <returns>Whether or not the point is within the triangle.</returns>
        public static bool IsPointInsideTriangle(ref Vector3 vA, ref Vector3 vB, ref Vector3 vC, ref Vector3 p, float margin)
        {
            float u, v, w;
            GetBarycentricCoordinates(ref p, ref vA, ref vB, ref vC, out u, out v, out w);
            //Are the barycoords valid?
            return (u > -margin) && (v > -margin) && (w > -margin);
        }

        #endregion

        #region Point-Line Tests

        /// <summary>
        /// Determines the closest point on the provided segment ab to point p.
        /// </summary>
        /// <param name="a">First endpoint of segment.</param>
        /// <param name="b">Second endpoint of segment.</param>
        /// <param name="p">Point for comparison.</param>
        /// <param name="closestPoint">Closest point on the edge to p.</param>
        public static void GetClosestPointOnSegmentToPoint(ref Vector3 a, ref Vector3 b, ref Vector3 p, out Vector3 closestPoint)
        {
            Vector3 ab;
            Vector3.Subtract(ref b, ref a, out ab);
            Vector3 ap;
            Vector3.Subtract(ref p, ref a, out ap);
            float t;
            Vector3.Dot(ref ap, ref ab, out t);
            if (t <= 0)
            {
                closestPoint = a;
            }
            else
            {
                float denom = ab.X * ab.X + ab.Y * ab.Y + ab.Z * ab.Z;
                if (t >= denom)
                {
                    closestPoint = b;
                }
                else
                {
                    t = t / denom;
                    Vector3 tab;
                    Vector3.Multiply(ref ab, t, out tab);
                    Vector3.Add(ref a, ref tab, out closestPoint);
                }
            }
        }

        /// <summary>
        /// Determines the closest point on the provided segment ab to point p.
        /// </summary>
        /// <param name="a">First endpoint of segment.</param>
        /// <param name="b">Second endpoint of segment.</param>
        /// <param name="p">Point for comparison.</param>
        /// <param name="subsimplex">The source of the voronoi region which contains the point.</param>
        /// <param name="closestPoint">Closest point on the edge to p.</param>
        public static void GetClosestPointOnSegmentToPoint(ref Vector3 a, ref Vector3 b, ref Vector3 p, List<Vector3> subsimplex, out Vector3 closestPoint)
        {
            subsimplex.Clear();
            Vector3 ab;
            Vector3.Subtract(ref b, ref a, out ab);
            Vector3 ap;
            Vector3.Subtract(ref p, ref a, out ap);
            float t;
            Vector3.Dot(ref ap, ref ab, out t);
            if (t <= 0)
            {
                //t = 0;//Don't need this for returning purposes.
                subsimplex.Add(a);
                closestPoint = a;
            }
            else
            {
                float denom = ab.X * ab.X + ab.Y * ab.Y + ab.Z * ab.Z;
                if (t >= denom)
                {
                    //t = 1;//Don't need this for returning purposes.
                    subsimplex.Add(b);
                    closestPoint = b;
                }
                else
                {
                    t = t / denom;
                    subsimplex.Add(a);
                    subsimplex.Add(b);
                    Vector3 tab;
                    Vector3.Multiply(ref ab, t, out tab);
                    Vector3.Add(ref a, ref tab, out closestPoint);
                }
            }
        }

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
        /// Determines the shortest distance from the point to the line.
        /// </summary>
        /// <param name="p">Point to check against the line.</param>
        /// <param name="a">First point on the line for comparison.</param>
        /// <param name="b">Second point on the line for comparison.</param>
        /// <returns>Shortest distance from the point to the line.</returns>
        public static float GetDistanceFromPointToLine(Vector3 p, Vector3 a, Vector3 b)
        {
            Vector3 vl = b - a;
            float numerator = Vector3.Cross(vl, p - a).Length();
            return numerator / vl.Length();
        }

        /// <summary>
        /// Determines the shortest squared distance from the point to the line.
        /// </summary>
        /// <param name="p">Point to check against the line.</param>
        /// <param name="a">First point on the line.</param>
        /// <param name="b">Second point on the line.</param>
        /// <returns>Shortest squared distance from the point to the line.</returns>
        public static float GetSquaredDistanceFromPointToLine(Vector3 p, Vector3 a, Vector3 b)
        {
            Vector3 ap = p - a;
            Vector3 ab = b - a;
            float e = Vector3.Dot(ap, ab);
            return Vector3.Dot(ap, ap) - e * e / Vector3.Dot(ab, ab);
        }

        #endregion

        #region Line-Line Tests

        /// <summary>
        /// Determines if the given directions are parallel.
        /// </summary>
        /// <param name="dirA">First line direction.</param>
        /// <param name="dirB">Second line direction.</param>
        /// <returns>Whether or not the given directions are parallel.</returns>
        public static bool AreSegmentsParallel(Vector3 dirA, Vector3 dirB)
        {
            return (GetSquaredDistanceLinePoint(ZeroVector, dirA, dirB) < Epsilon);
        }

        /// <summary>
        /// Computes closest points c1 and c2 betwen segments p1q1 and p2q2.
        /// </summary>
        /// <param name="p1">First point of first segment.</param>
        /// <param name="q1">Second point of first segment.</param>
        /// <param name="p2">First point of second segment.</param>
        /// <param name="q2">Second point of second segment.</param>
        /// <param name="c1">Closest point on first segment.</param>
        /// <param name="c2">Closest point on second segment.</param>
        public static void GetClosestPointsBetweenSegments(Vector3 p1, Vector3 q1, Vector3 p2, Vector3 q2, out Vector3 c1, out Vector3 c2)
        {
            float s, t;
            GetClosestPointsBetweenSegments(ref p1, ref q1, ref p2, ref q2, out s, out t, out c1, out c2);
        }

        /// <summary>
        /// Computes closest points c1 and c2 betwen segments p1q1 and p2q2.
        /// </summary>
        /// <param name="p1">First point of first segment.</param>
        /// <param name="q1">Second point of first segment.</param>
        /// <param name="p2">First point of second segment.</param>
        /// <param name="q2">Second point of second segment.</param>
        /// <param name="s">Distance along the line to the point for first segment.</param>
        /// <param name="t">Distance along the line to the point for second segment.</param>
        /// <param name="c1">Closest point on first segment.</param>
        /// <param name="c2">Closest point on second segment.</param>
        public static void GetClosestPointsBetweenSegments(ref Vector3 p1, ref Vector3 q1, ref Vector3 p2, ref Vector3 q2,
                                                           out float s, out float t, out Vector3 c1, out Vector3 c2)
        {
            //Segment direction vectors
            Vector3 d1;
            Vector3.Subtract(ref q1, ref p1, out d1);
            Vector3 d2;
            Vector3.Subtract(ref q2, ref p2, out d2);
            Vector3 r;
            Vector3.Subtract(ref p1, ref p2, out r);
            //distance
            float a = d1.LengthSquared();
            float e = d2.LengthSquared();
            float f;
            Vector3.Dot(ref d2, ref r, out f);

            if (a <= Epsilon && e <= Epsilon)
            {
                //These segments are more like points.
                s = t = 0.0f;
                c1 = p1;
                c2 = p2;
                return;
            }
            if (a <= Epsilon)
            {
                // First segment is basically a point.
                s = 0.0f;
                t = MathHelper.Clamp(f / e, 0.0f, 1.0f);
            }
            else
            {
                float c = Vector3.Dot(d1, r);
                if (e <= Epsilon)
                {
                    // Second segment is basically a point.
                    t = 0.0f;
                    s = MathHelper.Clamp(-c / a, 0.0f, 1.0f);
                }
                else
                {
                    float b = Vector3.Dot(d1, d2);
                    float denom = a * e - b * b;

                    // If segments not parallel, compute closest point on L1 to L2, and
                    // clamp to segment S1. Else pick some s (here .5f)
                    if (denom != 0.0f)
                        s = MathHelper.Clamp((b * f - c * e) / denom, 0.0f, 1.0f);
                    else //Parallel, just use .5f
                        s = .5f;


                    t = (b * s + f) / e;

                    if (t < 0)
                    {
                        //Closest point is before the segment.
                        t = 0;
                        s = MathHelper.Clamp(-c / a, 0, 1);
                    }
                    else if (t > 1)
                    {
                        //Closest point is after the segment.
                        t = 1;
                        s = MathHelper.Clamp((b - c) / a, 0, 1);
                    }
                }
            }

            Vector3.Multiply(ref d1, s, out c1);
            Vector3.Add(ref c1, ref p1, out c1);
            Vector3.Multiply(ref d2, t, out c2);
            Vector3.Add(ref c2, ref p2, out c2);
        }


        /// <summary>
        /// Computes closest points c1 and c2 betwen lines p1q1 and p2q2.
        /// </summary>
        /// <param name="p1">First point of first segment.</param>
        /// <param name="q1">Second point of first segment.</param>
        /// <param name="p2">First point of second segment.</param>
        /// <param name="q2">Second point of second segment.</param>
        /// <param name="s">Distance along the line to the point for first segment.</param>
        /// <param name="t">Distance along the line to the point for second segment.</param>
        /// <param name="c1">Closest point on first segment.</param>
        /// <param name="c2">Closest point on second segment.</param>
        public static void GetClosestPointsBetweenLines(ref Vector3 p1, ref Vector3 q1, ref Vector3 p2, ref Vector3 q2,
                                                           out float s, out float t, out Vector3 c1, out Vector3 c2)
        {
            //Segment direction vectors
            Vector3 d1;
            Vector3.Subtract(ref q1, ref p1, out d1);
            Vector3 d2;
            Vector3.Subtract(ref q2, ref p2, out d2);
            Vector3 r;
            Vector3.Subtract(ref p1, ref p2, out r);
            //distance
            float a = d1.LengthSquared();
            float e = d2.LengthSquared();
            float f;
            Vector3.Dot(ref d2, ref r, out f);

            if (a <= Epsilon && e <= Epsilon)
            {
                //These segments are more like points.
                s = t = 0.0f;
                c1 = p1;
                c2 = p2;
                return;
            }
            if (a <= Epsilon)
            {
                // First segment is basically a point.
                s = 0.0f;
                t = MathHelper.Clamp(f / e, 0.0f, 1.0f);
            }
            else
            {
                float c = Vector3.Dot(d1, r);
                if (e <= Epsilon)
                {
                    // Second segment is basically a point.
                    t = 0.0f;
                    s = MathHelper.Clamp(-c / a, 0.0f, 1.0f);
                }
                else
                {
                    float b = Vector3.Dot(d1, d2);
                    float denom = a * e - b * b;

                    // If segments not parallel, compute closest point on L1 to L2, and
                    // clamp to segment S1. Else pick some s (here .5f)
                    if (denom != 0f)
                        s = (b * f - c * e) / denom;
                    else //Parallel, just use .5f
                        s = .5f;


                    t = (b * s + f) / e;
                }
            }

            Vector3.Multiply(ref d1, s, out c1);
            Vector3.Add(ref c1, ref p1, out c1);
            Vector3.Multiply(ref d2, t, out c2);
            Vector3.Add(ref c2, ref p2, out c2);
        }

        /// <summary>
        /// Determines the minimum distance between two lines.
        /// </summary>
        /// <param name="p1">First point of the first line for comparison.</param>
        /// <param name="p2">Second point of the first line for comparison.</param>
        /// <param name="p3">First point of the second line for comparison.</param>
        /// <param name="p4">Second point of the second line for comparison.</param>
        /// <returns>Minimum distance between two lines.</returns>
        public static float GetDistanceBetweenLines(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4)
        {
            Vector3 pa, pb;
            if (GetLineLineIntersection(p1, p2, p3, p4, out pa, out pb))
            {
                return Vector3.Distance(pa, pb);
            }
            return 0;
        }

        /// <summary>
        /// Determines the intersection of two parallel segments represented by two points.
        /// </summary>
        /// <param name="p1">First endpoint of first segment.</param>
        /// <param name="q1">Second endpoint of first segment.</param>
        /// <param name="p2">First endpoint of second segment.</param>
        /// <param name="q2">Second endpoint of second segment.</param>
        /// <param name="a">First endpoint of intersection area.</param>
        /// <param name="b">Last endpoint of intersection area.</param>
        /// <returns>Whether or not the segments are parallel.</returns>
        public static bool GetIntersectionParallelSegments(Vector3 p1, Vector3 q1, Vector3 p2, Vector3 q2, out Vector3 a, out Vector3 b)
        {
            //Just do a series of clamps and then use interval arithmatic to identify the a and b.
            a = NoVector;
            b = NoVector;
            Vector3 edge1Dir = q1 - p1;
            Vector3 edge2Dir = q2 - p2;
            if (!AreSegmentsParallel(edge1Dir, edge2Dir))
                return false;
            float a1 = Vector3.Dot(p1, edge1Dir);
            float b1 = Vector3.Dot(q1, edge1Dir);
            float a2 = Vector3.Dot(p2, edge1Dir);
            float b2 = Vector3.Dot(q2, edge1Dir);
            if (a1 < a2 && a1 < b2 && b1 < a2 && b1 < b2)
                return false;
            float clampa1 = Clamp(a1, a2, b2);
            float clampb1 = Clamp(b1, a2, b2);
            float clampa2 = Clamp(a2, a1, b1);
            float clampb2 = Clamp(b2, a1, b1);
            float min = Math.Min(clampa1, Math.Min(clampb1, Math.Min(clampa2, clampb2)));
            float max = Math.Max(clampa1, Math.Max(clampb1, Math.Max(clampa2, clampb2)));
            float axisLengthInverse = 1 / edge1Dir.LengthSquared();
            a = (min * axisLengthInverse) * edge1Dir;
            b = (max * axisLengthInverse) * edge1Dir;
            return true;
        }

        /// <summary>
        /// Finds the shortest line segment between two lines.
        /// </summary>
        /// <param name="p1">First point of the first line for comparison.</param>
        /// <param name="p2">Second point of the first line for comparison.</param>
        /// <param name="p3">First point of the second line for comparison.</param>
        /// <param name="p4">Second point of the second line for comparison.</param>
        /// <param name="pa">First point of the shortest line.</param>
        /// <param name="pb">Second point of the shortest line.</param>
        /// <returns>Whether or not an intersection could be identified.</returns>
        public static bool GetLineLineIntersection(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4, out Vector3 pa, out Vector3 pb)
        {
            //Thanks to Paul Bourke for the base of this function.
            //Note on usage: This returns a line-line intersect in the form of a vector from pa->pb, allowing for lines which are skew (IE not coplanar)
            Vector3 p13, p43, p21;
            pa = pb = NoVector;
            float d1343, d4321, d1321, d4343, d2121, mua, mub;
            float numer, denom;
            p13 = p1 - p3;
            p43 = p4 - p3;

            if (p43.LengthSquared() < Epsilon)
                return false;
            p21 = p2 - p1;
            if (p21.LengthSquared() < Epsilon)
                return false;

            d1343 = Vector3.Dot(p13, p43);
            d4321 = Vector3.Dot(p43, p21);
            d1321 = Vector3.Dot(p13, p21);
            d4343 = Vector3.Dot(p43, p43);
            d2121 = Vector3.Dot(p21, p21);

            denom = d2121 * d4343 - d4321 * d4321;
            if (Math.Abs(denom) < Epsilon)
                return false;
            numer = d1343 * d4321 - d1321 * d4343;

            mua = numer / denom;
            mub = (d1343 + d4321 * (mua)) / d4343;

            pa.X = p1.X + mua * p21.X;
            pa.Y = p1.Y + mua * p21.Y;
            pa.Z = p1.Z + mua * p21.Z;
            pb.X = p3.X + mub * p43.X;
            pb.Y = p3.Y + mub * p43.Y;
            pb.Z = p3.Z + mub * p43.Z;

            return true;
        }

        /// <summary>
        /// Finds the intersection point between two coplanar segments.
        /// Note: Does not explicitly test coplanarity; functionality for non-coplanar parameters is undefined.
        /// </summary>
        /// <param name="p1">First point of the first line for comparison.</param>
        /// <param name="p2">Second point of the first line for comparison.</param>
        /// <param name="p3">First point of the second line for comparison.</param>
        /// <param name="p4">Second point of the second line for comparison.</param>
        /// <param name="intersection">Point of intersection.</param>
        /// <returns>Intersection between the two coplanar lines.</returns>
        public static bool GetLineLineIntersection(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4, out Vector3 intersection)
        {
            //This intersection method only returns a single intersection point between two assumed coplanar lines.

            intersection = NoVector;
            Vector3 a = p2 - p1;
            Vector3 b = p4 - p3;
            Vector3 c = p3 - p1;
            Vector3 d = p1 - p3;
            Vector3 denom = Vector3.Cross(a, b);
            float denomLengthSquared = denom.LengthSquared();
            if (Math.Abs(Vector3.Dot(d, Vector3.Cross(a, b))) < Epsilon && denomLengthSquared > 0)
            {
                //The lines are coplanar, or close enough for the end result to be valid.
                intersection = p1 + (a) * (Vector3.Dot(Vector3.Cross(c, b), Vector3.Cross(a, b)) / denomLengthSquared);
                return true;
            }
            /*
            if (isPointCollinear(p1, p3, p4) && isPointCollinear(p2, p3, p4))
            {//Lines are collinear
                //Determine if the points intersect.
                //TODO: Collinear test actually makes the final result less accurate due to counting off vertices and such.  Consider removing.
                List<Vector3> lines1 = new List<Vector3>();
                List<Vector3> lines2 = new List<Vector3>();
                lines1.Add(p1); lines1.Add(p2);
                lines2.Add(p3); lines2.Add(p4);
                if (areIntersecting(lines1, lines2, p1))
                {
                    intersection = (p2 + p3) / 2;
                    return true;
                }
            }
            */

            return false;
        }

        /// <summary>
        /// Finds the squared distance from a point to a line.
        /// </summary>
        /// <param name="a">First point on the line.</param>
        /// <param name="b">Second point on the line.</param>
        /// <param name="p">Point for comparison.</param>
        /// <returns>Squared distance from the point to the line.</returns>
        public static float GetSquaredDistanceLinePoint(Vector3 a, Vector3 b, Vector3 p)
        {
            Vector3 u = b - a;
            return Vector3.Cross(u, a - p).LengthSquared() / u.LengthSquared();
        }

        /// <summary>
        /// Determines if the given point is collinear with the line.
        /// </summary>
        /// <param name="point">Point for comparison.</param>
        /// <param name="a">First endpoint of the line.</param>
        /// <param name="b">Second endpoint of the line.</param>
        /// <returns>Whether or not the point is collinear with the line.</returns>
        public static bool IsPointCollinear(Vector3 point, Vector3 a, Vector3 b)
        {
            return Vector3.Cross((a - point), (point - b)).LengthSquared() < Epsilon;
        }

        #endregion

        #region Triangle-Triangle Tests

        /*
        /// <summary>
        /// Finds the intersection between two triangles.
        /// </summary>
        /// <param name="va1">First vertex of the first triangle.</param>
        /// <param name="vb1">Second vertex of the first triangle.</param>
        /// <param name="vc1">Third vertex of the first triangle.</param>
        /// <param name="va2">First vertex of the second triangle.</param>
        /// <param name="vb2">Second vertex of the second triangle.</param>
        /// <param name="vc2">Third vertex of the second triangle.</param>
        /// <param name="intersection">Point representing the intersection of the two triangles.</param>
        /// <returns>Whether or not the triangles intersect.</returns>
        public static bool getTriangleTriangleIntersection(Vector3 va1, Vector3 vb1, Vector3 vc1, Vector3 va2, Vector3 vb2, Vector3 vc2, out Vector3 intersection)
        {
            intersection = noVector;
            Vector3 t1normal = Vector3.Cross(vb1 - va1, vc1 - va1);
            Vector3 t2normal = Vector3.Cross(vb2 - va2, vc2 - va2);
            float t1d = -Vector3.Dot(t1normal, va1); //plane distances
            float t2d = -Vector3.Dot(t2normal, va2);
            float d10 = Vector3.Dot(t2normal, va1) + t2d;
            float d20 = Vector3.Dot(t1normal, va2) + t1d;
            float d11 = Vector3.Dot(t2normal, vb1) + t2d;
            float d21 = Vector3.Dot(t1normal, vb2) + t1d;
            float d12 = Vector3.Dot(t2normal, vc1) + t2d;
            float d22 = Vector3.Dot(t1normal, vc2) + t1d;
            if ((d1[0] * d1[1] > epsilon && d1[0] * d1[2] > epsilon) || (d2[0] * d2[1] > epsilon && d2[0] * d2[2] > epsilon))
            {//Triangles do not intersect if all distances from triangle to oppposing plane are positive or all are negative.
                return false;
            }
            if (!(Math.Abs(d10) < epsilon && Math.Abs(d11) < epsilon && Math.Abs(d12) < epsilon &&
                  Math.Abs(d20) < epsilon && Math.Abs(d21) < epsilon && Math.Abs(d22) < epsilon))
            {//If the triangles are not coplanar:
                float c = Vector3.Dot(t1normal, t2normal);
                Vector3 direction = Vector3.Cross(t1normal, t2normal); //of plane intersection line
                Vector3 origin = ((t2d * c - t1d) / (1 - c * c)) * t1normal + ((t1d * c - t2d) / (1 - c * c)) * t2normal + direction * 1000; //Calculation of origin could be slightly sped by using a direct linear equation method. //TODO: dir*1000 fudge needs to be remembered.
                List<Vector3> lineIntersections = new List<Vector3>();
                List<Vector3> t1intersections = new List<Vector3>();
                List<Vector3> t2intersections = new List<Vector3>();
                Vector3 isect;
                //Triangle 1 intersections with intersection line.
                if (LineLineIntersect(origin, origin + direction, va1.position, vb1.position, out isect))
                {
                    lineIntersections.Add(isect);
                    t1intersections.Add(isect);
                }
                if (LineLineIntersect(origin, origin + direction, vb1.position, vc1.position, out isect))
                {
                    lineIntersections.Add(isect);
                    t1intersections.Add(isect);
                }
                if (LineLineIntersect(origin, origin + direction, vc1.position, va1.position, out isect))
                {
                    lineIntersections.Add(isect);
                    t1intersections.Add(isect);
                }
                //Triangle 2 intersections with intersection line.
                if (LineLineIntersect(origin, origin + direction, va2.position, vb2.position, out isect))
                {
                    lineIntersections.Add(isect);
                    t2intersections.Add(isect);
                }
                if (LineLineIntersect(origin, origin + direction, vb2.position, vc2.position, out isect))
                {
                    lineIntersections.Add(isect);
                    t2intersections.Add(isect);
                }
                if (LineLineIntersect(origin, origin + direction, vc2.position, va2.position, out isect))
                {
                    lineIntersections.Add(isect);
                    t2intersections.Add(isect);
                }
                //Determine if intersections on line are interconnected or seperate.  If seperate, return false as there is no collision.
                if (!areIntervalsIntersecting(t1intersections, t2intersections, origin))
                    return false;
                sortLines(lineIntersections, origin);
                intersection = (lineIntersections[(lineIntersections.Count - 1) / 2] + lineIntersections[(lineIntersections.Count - 1) / 2 + 1]) / 2;

                return true;

            }
            else
            {//If the triangle are coplanar:

                if (coplanarTriangleIntersection(T1, T2, out intersection))
                    return true;
                else
                {
                    intersection = noVector;
                    return false;
                }
            }

        }

        /// <summary>
        /// Returns the intersection of two coplanar triangles.
        /// Note: Does not verify the coplanarity; functionality for non-coplanar triangles is undefined.
        /// </summary>
        /// <param name="T1">First triangle for comparison.</param>
        /// <param name="T2">Second triangle for comparison.</param>
        /// <param name="intersection">Represents the point of intersection of triangles.</param>
        /// <returns>Whether or not the triangles intersect.</returns>
        public static bool coplanarTriangleIntersection(PolyhedronTriangle T1, PolyhedronTriangle T2, out Vector3 intersection)
        {//Finds an intersection between two coplanar triangles in 3d space.  Generated from the average of line intersections.
            intersection = zeroVector;
            //If a triangle fully contains the other triangle:
            bool[] T1pointsInsideT2 = new bool[3];
            bool[] T2pointsInsideT1 = new bool[3];
            for (int k = 0; k < 3; k++)
            {
                T1pointsInsideT2[k] = isPointInsideTriangle(ref T1.vertices[k].position, T2);
                T2pointsInsideT1[k] = isPointInsideTriangle(ref T2.vertices[k].position, T1);
            }
            if (!T1pointsInsideT2[0] && !T1pointsInsideT2[1] && !T1pointsInsideT2[2] && !T2pointsInsideT1[0] && !T2pointsInsideT1[1] && !T2pointsInsideT1[2])
            {//Neither triangle has any vertices of the other triangle within it.
                return false;
            }
            if (T1pointsInsideT2[0] && T1pointsInsideT2[1] && T1pointsInsideT2[2])
            {
                intersection = (T1.vertices[0].position + T1.vertices[1].position + T1.vertices[2].position) / 3;
                return true;
            }
            if (T2pointsInsideT1[0] && T2pointsInsideT1[1] && T2pointsInsideT1[2])
            {
                intersection = (T2.vertices[0].position + T2.vertices[1].position + T2.vertices[2].position) / 3;
                return true;
            }

            //From here on out, we are guaranteed to have an intersection.  If not, something is wrong.
            List<Vector3> intersections = new List<Vector3>();
            Vector3 isect;
            Vector3[,] t1edges = T1.edgePositions;
            Vector3[,] t2edges = T2.edgePositions;
            for (int k = 0; k < 3; k++)
            {//Test each segment of T1 against each segment of T2.
                for (int j = 0; j < 3; j++)
                {
                    if (LineLineIntersect(t1edges[k, 0], t1edges[k, 1], t2edges[j, 0], t2edges[j, 1], out isect))
                    {
                        intersections.Add(isect);
                        if (Vector3.DistanceSquared(t1edges[k, 0], t1edges[k, 1]) < Vector3.DistanceSquared(t1edges[k, 0], isect)) // Remove any invalid catches.  Include in first if?
                            intersections.Remove(isect);
                        else
                            if (Vector3.DistanceSquared(t2edges[j, 0], t2edges[j, 1]) < Vector3.DistanceSquared(t2edges[j, 0], isect))
                                intersections.Remove(isect);
                            else
                                if (!isPointInsideTriangle(ref isect, T1)) //To be valid, must lie within both of the triangles.
                                    intersections.Remove(isect);
                                else
                                    if (!isPointInsideTriangle(ref isect, T2))
                                        intersections.Remove(isect);
                    }

                }
            }


            if (intersections.Count == 0)
            {
                intersection = noVector;
                return false;
            }
            for (int k = 0; k < intersections.Count; k++)
            {
                intersection += intersections[k];
            }
            intersection /= intersections.Count;
            return true;
        }

        /// <summary>
        /// Determines if the intervals of intersection are intersecting.
        /// </summary>
        /// <param name="lines1">Intersections of triangle 1 with the intersection line.</param>
        /// <param name="lines2">Intersections of triangle 2 with the intersection line.</param>
        /// <param name="origin">A point on the intersection line.</param>
        /// <returns>Whether or not the intervals of intersection intersect.</returns>
        private static bool areIntervalsIntersecting(List<Vector3> lines1, List<Vector3> lines2, Vector3 origin)
        {
            for (int k = 0; k < lines1.Count; k++)
            {//Remove origin for distance calculations.
                lines1[k] = lines1[k] - origin;
            }
            for (int k = 0; k < lines2.Count; k++)
            {//Remove origin for distance calculations.
                lines2[k] = lines2[k] - origin;
            }
            List<float> lines1Distances = new List<float>();
            for (int k = 0; k < lines1.Count; k++)
            {//Add the distances to a group specific list.
                lines1Distances.Add(lines1[k].LengthSquared());
            }
            List<float> lines2Distances = new List<float>();
            for (int k = 0; k < lines2.Count; k++)
            {//Add the distances to a group specific list.
                lines2Distances.Add(lines2[k].LengthSquared());
            }
            // If lines1 are all greater than lines2 OR if lines1 are all less than lines2, then it does not intersect.
            if (Math.Abs(lines1Distances[0] - lines2Distances[0]) < epsilon)
                return true;
            if (lines1Distances[0] < lines2Distances[0]) //For it to be not intersecting, all of 1 must be less than 2
                for (int i = 0; i < lines1.Count; i++)
                {
                    for (int j = 0; j < lines2.Count; j++)
                    {
                        if (!(lines1Distances[i] <= lines2Distances[j] - epsilon))
                            return true;
                    }
                }
            else if (lines1Distances[0] > lines2Distances[0])
                for (int i = 0; i < lines1.Count; i++)
                {
                    for (int j = 0; j < lines2.Count; j++)
                    {
                        if (!(lines1Distances[i] >= lines2Distances[j] + epsilon))
                            return true;
                    }
                }
            return false;
        }*/

        #endregion

        #region Sorting Vectors

        /// <summary>
        /// Acts as a comparator condition based on length for two vectors.
        /// </summary>
        /// <param name="v1">First vector for comparison.</param>
        /// <param name="v2">Second vector for comparison.</param>
        /// <returns>Comparator int.</returns>
        internal static int CompareVectorLengths(Vector3 v1, Vector3 v2)
        {
            float v1Length = v1.LengthSquared();
            float v2Length = v2.LengthSquared();
            if (v1Length < v2Length)
            {
                return -1;
            }
            return 1;
        }

        #endregion

        #region Point-Plane Tests

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
        /// Determines the distance between a point and a plane..
        /// </summary>
        /// <param name="point">Point to project onto plane.</param>
        /// <param name="normal">Normal of the plane.</param>
        /// <param name="pointOnPlane">Point located on the plane.</param>
        /// <returns>Distance from the point to the plane.</returns>
        public static float GetDistancePointToPlane(Vector3 point, Vector3 normal, Vector3 pointOnPlane)
        {
            return (Vector3.Dot(normal, point) - Vector3.Dot(pointOnPlane, normal)) / normal.LengthSquared();
        }

        /// <summary>
        /// Determines the location of the point when projected onto the plane defined by the normal and a point on the plane.
        /// </summary>
        /// <param name="point">Point to project onto plane.</param>
        /// <param name="normal">Normal of the plane.</param>
        /// <param name="pointOnPlane">Point located on the plane.</param>
        /// <returns>Projected location of point onto plane.</returns>
        public static Vector3 GetPointProjectedOnPlane(Vector3 point, Vector3 normal, Vector3 pointOnPlane)
        {
            float dot;
            Vector3.Dot(ref normal, ref point, out dot);
            float dot2;
            Vector3.Dot(ref pointOnPlane, ref normal, out dot2);
            float t = (dot - dot2) / normal.LengthSquared();
            Vector3 toReturn;
            Vector3 multiply;
            Vector3.Multiply(ref normal, t, out multiply);
            Vector3.Subtract(ref point, ref multiply, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Determines if a point is within a set of planes defined by the edges of a triangle.
        /// </summary>
        /// <param name="point">Point for comparison.</param>
        /// <param name="planes">Edge planes.</param>
        /// <param name="centroid">A point known to be inside of the planes.</param>
        /// <returns>Whether or not the point is within the edge planes.</returns>
        public static bool IsPointWithinFaceExtrusion(Vector3 point, List<Plane> planes, Vector3 centroid)
        {
            foreach (Plane plane in planes)
            {
                float centroidPlaneDot = plane.DotCoordinate(centroid);
                float pointPlaneDot = plane.DotCoordinate(point);
                if (!((centroidPlaneDot <= Epsilon && pointPlaneDot <= Epsilon) || (centroidPlaneDot >= -Epsilon && pointPlaneDot >= -Epsilon)))
                {
                    //Point's NOT the same side of the centroid, so it's 'outside.'
                    return false;
                }
            }
            return true;
        }

        /// <summary>
        /// Determines if a point lies within the bounds of a set of planes representing edge planes.
        /// If it is outside, a set of possible separating planes will be provided.
        /// </summary>
        /// <param name="point">Location to test for inclusion.</param>
        /// <param name="planes">Representation of edge planes on a face.</param>
        /// <param name="centroid">A point determined to be within all planes.</param>
        /// <param name="separatingPlanes">Possible planes that the point may come from wtihin the object (as with a line segment).</param>
        /// <returns>Whether or not the point is within the extrusion.</returns>
        public static bool IsPointWithinFaceExtrusion(Vector3 point, List<Plane> planes, Vector3 centroid, out List<Plane> separatingPlanes)
        {
            separatingPlanes = new List<Plane>();
            bool outside = false;
            foreach (Plane plane in planes)
            {
                float centroidPlaneDot = plane.DotCoordinate(centroid);
                float pointPlaneDot = plane.DotCoordinate(point);
                if (!((centroidPlaneDot <= 0 && pointPlaneDot <= 0) || (centroidPlaneDot >= 0 && pointPlaneDot >= 0)))
                {
                    //Point's NOT the same side of the centroid, so it's 'outside.'
                    outside = true;
                    separatingPlanes.Add(plane);
                }
            }
            if (outside == false)
                return true;
            return false;
        }

        #endregion

        #region Tetrahedron Tests

        //static List<int> subsimplexCandidate = new List<int>();
        //static List<float> baryCoordsCandidate = new List<float>();

        /// <summary>
        /// Determines the closest point on a tetrahedron to a provided point p.
        /// </summary>
        /// <param name="a">First vertex of the tetrahedron.</param>
        /// <param name="b">Second vertex of the tetrahedron.</param>
        /// <param name="c">Third vertex of the tetrahedron.</param>
        /// <param name="d">Fourth vertex of the tetrahedron.</param>
        /// <param name="p">Point for comparison.</param>
        /// <param name="closestPoint">Closest point on the tetrahedron to the point.</param>
        public static void GetClosestPointOnTetrahedronToPoint(ref Vector3 a, ref Vector3 b, ref Vector3 c, ref Vector3 d, ref Vector3 p, out Vector3 closestPoint)
        {
            // Start out assuming point inside all halfspaces, so closest to itself
            closestPoint = p;
            Vector3 pq;
            Vector3 q;
            float bestSqDist = float.MaxValue;
            // If point outside face abc then compute closest point on abc
            if (ArePointsOnOppositeSidesOfPlane(ref p, ref d, ref a, ref b, ref c))
            {
                GetClosestPointOnTriangleToPoint(ref a, ref b, ref c, ref p, out q);
                Vector3.Subtract(ref q, ref p, out pq);
                float sqDist = pq.X * pq.X + pq.Y * pq.Y + pq.Z * pq.Z;
                // Update best closest point if (squared) distance is less than current best
                if (sqDist < bestSqDist)
                {
                    bestSqDist = sqDist;
                    closestPoint = q;
                }
            }
            // Repeat test for face acd
            if (ArePointsOnOppositeSidesOfPlane(ref p, ref b, ref a, ref c, ref d))
            {
                GetClosestPointOnTriangleToPoint(ref a, ref c, ref d, ref p, out q);
                Vector3.Subtract(ref q, ref p, out pq);
                float sqDist = pq.X * pq.X + pq.Y * pq.Y + pq.Z * pq.Z;
                if (sqDist < bestSqDist)
                {
                    bestSqDist = sqDist;
                    closestPoint = q;
                }
            }
            // Repeat test for face adb
            if (ArePointsOnOppositeSidesOfPlane(ref p, ref c, ref a, ref d, ref b))
            {
                GetClosestPointOnTriangleToPoint(ref a, ref d, ref b, ref p, out q);
                Vector3.Subtract(ref q, ref p, out pq);
                float sqDist = pq.X * pq.X + pq.Y * pq.Y + pq.Z * pq.Z;
                if (sqDist < bestSqDist)
                {
                    bestSqDist = sqDist;
                    closestPoint = q;
                }
            }
            // Repeat test for face bdc
            if (ArePointsOnOppositeSidesOfPlane(ref p, ref a, ref b, ref d, ref c))
            {
                GetClosestPointOnTriangleToPoint(ref b, ref d, ref c, ref p, out q);
                Vector3.Subtract(ref q, ref p, out pq);
                float sqDist = pq.X * pq.X + pq.Y * pq.Y + pq.Z * pq.Z;
                if (sqDist < bestSqDist)
                {
                    closestPoint = q;
                }
            }
        }

        /// <summary>
        /// Determines the closest point on a tetrahedron to a provided point p.
        /// </summary>
        /// <param name="a">First vertex of the tetrahedron.</param>
        /// <param name="b">Second vertex of the tetrahedron.</param>
        /// <param name="c">Third vertex of the tetrahedron.</param>
        /// <param name="d">Fourth vertex of the tetrahedron.</param>
        /// <param name="p">Point for comparison.</param>
        /// <param name="subsimplex">The source of the voronoi region which contains the point.</param>
        /// <param name="closestPoint">Closest point on the tetrahedron to the point.</param>
        public static void GetClosestPointOnTetrahedronToPoint(ref Vector3 a, ref Vector3 b, ref Vector3 c, ref Vector3 d, ref Vector3 p, RawList<Vector3> subsimplex, out Vector3 closestPoint)
        {
            // Start out assuming point inside all halfspaces, so closest to itself
            subsimplex.Clear();
            subsimplex.Add(a); //Provides a baseline; if the object is not outside of any planes, then it's inside and the subsimplex is the tetrahedron itself.
            subsimplex.Add(b);
            subsimplex.Add(c);
            subsimplex.Add(d);
            closestPoint = p;
            Vector3 pq;
            Vector3 q;
            float bestSqDist = float.MaxValue;
            // If point outside face abc then compute closest point on abc
            if (ArePointsOnOppositeSidesOfPlane(ref p, ref d, ref a, ref b, ref c))
            {
                GetClosestPointOnTriangleToPoint(ref a, ref b, ref c, ref p, subsimplex, out q);
                Vector3.Subtract(ref q, ref p, out pq);
                float sqDist = pq.X * pq.X + pq.Y * pq.Y + pq.Z * pq.Z;
                // Update best closest point if (squared) distance is less than current best
                if (sqDist < bestSqDist)
                {
                    bestSqDist = sqDist;
                    closestPoint = q;
                }
            }
            // Repeat test for face acd
            if (ArePointsOnOppositeSidesOfPlane(ref p, ref b, ref a, ref c, ref d))
            {
                GetClosestPointOnTriangleToPoint(ref a, ref c, ref d, ref p, subsimplex, out q);
                Vector3.Subtract(ref q, ref p, out pq);
                float sqDist = pq.X * pq.X + pq.Y * pq.Y + pq.Z * pq.Z;
                if (sqDist < bestSqDist)
                {
                    bestSqDist = sqDist;
                    closestPoint = q;
                }
            }
            // Repeat test for face adb
            if (ArePointsOnOppositeSidesOfPlane(ref p, ref c, ref a, ref d, ref b))
            {
                GetClosestPointOnTriangleToPoint(ref a, ref d, ref b, ref p, subsimplex, out q);
                Vector3.Subtract(ref q, ref p, out pq);
                float sqDist = pq.X * pq.X + pq.Y * pq.Y + pq.Z * pq.Z;
                if (sqDist < bestSqDist)
                {
                    bestSqDist = sqDist;
                    closestPoint = q;
                }
            }
            // Repeat test for face bdc
            if (ArePointsOnOppositeSidesOfPlane(ref p, ref a, ref b, ref d, ref c))
            {
                GetClosestPointOnTriangleToPoint(ref b, ref d, ref c, ref p, subsimplex, out q);
                Vector3.Subtract(ref q, ref p, out pq);
                float sqDist = pq.X * pq.X + pq.Y * pq.Y + pq.Z * pq.Z;
                if (sqDist < bestSqDist)
                {
                    closestPoint = q;
                }
            }
        }

        /// <summary>
        /// Determines the closest point on a tetrahedron to a provided point p.
        /// </summary>
        /// <param name="tetrahedron">List of 4 points composing the tetrahedron.</param>
        /// <param name="p">Point for comparison.</param>
        /// <param name="subsimplex">The source of the voronoi region which contains the point, enumerated as a = 0, b = 1, c = 2, d = 3.</param>
        /// <param name="baryCoords">Barycentric coordinates of p on the tetrahedron.</param>
        /// <param name="closestPoint">Closest point on the tetrahedron to the point.</param>
        public static void GetClosestPointOnTetrahedronToPoint(RawList<Vector3> tetrahedron, ref Vector3 p, RawList<int> subsimplex, RawList<float> baryCoords, out Vector3 closestPoint)
        {
            var subsimplexCandidate = Resources.GetIntList();
            var baryCoordsCandidate = Resources.GetFloatList();
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

        #endregion




        #region Convex Hull

        /// <summary>
        /// Finds the vector that is furthest along the given direction within the list.
        /// </summary>
        /// <param name="direction">Direction to find the furthest point along.</param>
        /// <param name="pointIndices">Indices from the point set to test.</param>
        /// <param name="points">List of all points to be indexed from.</param>
        /// <returns>The point furthest along the direction in the list of points.</returns>
        /// <param name="maxIndex">Index of the maximum point in the list.</param>
        public static Vector3 GetExtremePointOfSet(Vector3 direction, IList<int> pointIndices, IList<Vector3> points, out int maxIndex)
        {
            float max = float.MinValue;
            Vector3 maximum = ZeroVector;
            maxIndex = 0;
            for (int k = 0; k < pointIndices.Count; k++)
            {
                float dot = Vector3.Dot(points[pointIndices[k]], direction);
                if (dot > max)
                {
                    max = dot;
                    maximum = points[pointIndices[k]];
                    maxIndex = pointIndices[k];
                }
            }
            return maximum;
        }

        /// <summary>
        /// Finds the two points which are most extreme along the given direction within the list.
        /// </summary>
        /// <param name="direction">Direction to find the furthest points along.</param>
        /// <param name="points">List of points to test against.</param>
        /// <param name="minimum">Index of the point furthest back along the direction in the point list.</param>
        /// <param name="maximum">Index of the point furthest forward along the direction in the point list.</param>
        /// <returns>The point furthest along the direction in the list of points.</returns>
        public static void GetExtremePointsOfSet(Vector3 direction, IList<Vector3> points, out int minimum, out int maximum)
        {
            float max = float.MinValue;
            float min = float.MaxValue;
            minimum = 0;
            maximum = 0;

            for (int k = 0; k < points.Count; k++)
            {
                float dot = Vector3.Dot(points[k], direction);
                if (dot > max)
                {
                    max = dot;
                    maximum = k;
                }
                if (dot < min)
                {
                    min = dot;
                    minimum = k;
                }
            }
        }
        internal static void GetExtremePointsOfSet(Vector3 direction, IList<Vector3> points, out int minimum, out int maximum, out float min, out float max)
        {
            max = float.MinValue;
            min = float.MaxValue;
            minimum = 0;
            maximum = 0;

            for (int k = 0; k < points.Count; k++)
            {
                float dot = Vector3.Dot(points[k], direction);
                if (dot > max)
                {
                    max = dot;
                    maximum = k;
                }
                if (dot < min)
                {
                    min = dot;
                    minimum = k;
                }
            }
        }
        private static Vector3 FindNormal(IList<Vector3> points, IList<int> indices, int startIndex)
        {
            return Vector3.Cross(points[indices[startIndex + 2]] - points[indices[startIndex]], points[indices[startIndex + 1]] - points[indices[startIndex]]);
        }

        private static void MaintainEdge(int a, int b, IList<int> edges)
        {
            bool contained = false;
            int index = 0;
            for (int k = 0; k < edges.Count; k += 2)
            {
                if ((edges[k] == a && edges[k + 1] == b) || (edges[k] == b && edges[k + 1] == a))
                {
                    contained = true;
                    index = k;
                }
            }
            //If it isn't present, add it to the edge list.
            if (!contained)
            {
                edges.Add(a);
                edges.Add(b);
            }
            else
            {
                //If it is present, that means both edge-connected triangles were deleted now, so get rid of it.
                edges.RemoveAt(index);
                edges.RemoveAt(index);
            }
        }

        private static void RemovePointsInPolyhedronIfInside(IList<int> outsidePoints, IList<Vector3> points, IList<int> indices)
        {
            var outsidePointsTemp = Resources.GetIntList();
            for (int k = 0; k < outsidePoints.Count; k++)
            {
                outsidePointsTemp.Add(outsidePoints[k]);
            }
            outsidePoints.Clear();
            for (int k = 0; k < indices.Count && outsidePointsTemp.Count > 0; k += 3)
            {
                Vector3 normal = FindNormal(points, indices, k);
                for (int n = 0; n < outsidePointsTemp.Count; n++)
                {
                    Vector3 point = points[outsidePointsTemp[n]];
                    if (Vector3.Dot(point, normal) - Vector3.Dot(points[indices[k]], normal) > 0)
                    {
                        outsidePoints.Add(outsidePointsTemp[n]);
                        outsidePointsTemp.RemoveAt(n);
                        n--;
                    }
                }
            }
            Resources.GiveBack(outsidePointsTemp);
        }

        private static void VerifyWindings(IList<int> indices, IList<Vector3> points)
        {
            //Find centroid
            Vector3 centroid = ZeroVector;
            for (int k = 0; k < indices.Count; k++)
            {
                centroid += points[indices[k]];
            }
            centroid /= indices.Count;
            //Go through every triangle
            for (int k = 0; k < indices.Count; k += 3)
            {
                //Check if it faces away or towards the centroid
                if (Vector3.Dot(points[indices[k]] - centroid, FindNormal(points, indices, k)) < 0)
                {
                    //If it's towards, flip winding
                    int temp = indices[k + 1];
                    indices[k + 1] = indices[k + 2];
                    indices[k + 2] = temp;
                }
            }
        }

        /// <summary>
        /// Identifies the indices of points in a set which are on the outer convex hull of the set.
        /// </summary>
        /// <param name="points">List of points in the set.</param>
        /// <param name="indices">List of indices composing the triangulated surface of the convex hull.
        /// Each group of 3 indices represents a triangle on the surface of the hull.</param>
        public static void GetConvexHull(IList<Vector3> points, IList<int> indices)
        {
            //Points is what will be used as a vertex buffer.
            var outsidePoints = Resources.GetIntList();
            var edges = Resources.GetIntList();

            var toRemove = Resources.GetIntList();
            //Populate the outside points
            for (int k = 0; k < points.Count; k++)
            {
                outsidePoints.Add(k);
            }
            //Find an initial tetrahedron
            var initialTetrahedron = Resources.GetIntList();
            /*float volume = 0;
            Random random = new Random();
            Vector3 dir;
            int count = 0;


            while (initialTetrahedron.Count != 4 && count < 100)
            {
                dir = new Vector3((float)random.NextDouble() - .5f, (float)random.NextDouble() - .5f, (float)random.NextDouble() - .5f);
                getExtremePointOfSet(dir, outsidePoints, points, out maxIndex);
                if(!initialTetrahedron.Contains(maxIndex))
                    initialTetrahedron.Add(maxIndex);
                if (initialTetrahedron.Count == 4)
                {
                    //(a-d) * ((b-d)x(c-d)
                    volume = Vector3.Dot(Vector3.Cross(points[initialTetrahedron[1]] - points[initialTetrahedron[3]], points[initialTetrahedron[2]] - points[initialTetrahedron[3]]), points[initialTetrahedron[0]] - points[initialTetrahedron[3]]);
                    if (Math.Abs(volume) < epsilon)
                        initialTetrahedron.RemoveAt(3);
                }
                count++;

            }*/
            int min, max;
            GetExtremePointsOfSet(Vector3.Up, points, out min, out max);
            if (min == max)
                throw new ArgumentException("Point set is degenerate.");
            initialTetrahedron.Add(min);
            initialTetrahedron.Add(max);
            Vector3 direction = NoVector;
            for (int i = 0; i < points.Count; i++)
            {
                if (i != min && i != max)
                {
                    direction = Vector3.Cross(points[min] - points[i], points[max] - points[i]);
                    if (direction.LengthSquared() > BigEpsilon)
                    {
                        break;
                    }
                }
            }
            float minDistance, maxDistance;
            float lineMin = Vector3.Dot(direction, points[min]);
            GetExtremePointsOfSet(direction, points, out min, out max, out minDistance, out maxDistance);

            if (Math.Abs(minDistance - lineMin) < BigEpsilon)
            {
                if (Math.Abs(maxDistance - lineMin) < BigEpsilon)
                {
                    throw new ArgumentException("Point set is degenerate.");
                }
                initialTetrahedron.Add(max);
            }
            else
            {
                initialTetrahedron.Add(min);
            }

            direction = Vector3.Cross(points[initialTetrahedron[1]] - points[initialTetrahedron[0]], points[initialTetrahedron[2]] - points[initialTetrahedron[0]]);

            lineMin = Vector3.Dot(direction, points[initialTetrahedron[0]]);
            GetExtremePointsOfSet(direction, points, out min, out max, out minDistance, out maxDistance);

            if (Math.Abs(minDistance - lineMin) < BigEpsilon)
            {
                if (Math.Abs(maxDistance - lineMin) < BigEpsilon)
                {
                    throw new ArgumentException("Point set is degenerate.");
                }
                initialTetrahedron.Add(max);
            }
            else
            {
                initialTetrahedron.Add(min);
            }


            //Add initial tetrahedron triangles to indices list, remove from outside points, and remove all interior points from outside points.
            if (initialTetrahedron.Count == 4)
            {
                indices.Add(initialTetrahedron[0]);
                indices.Add(initialTetrahedron[1]);
                indices.Add(initialTetrahedron[2]);

                indices.Add(initialTetrahedron[1]);
                indices.Add(initialTetrahedron[2]);
                indices.Add(initialTetrahedron[3]);

                indices.Add(initialTetrahedron[2]);
                indices.Add(initialTetrahedron[3]);
                indices.Add(initialTetrahedron[0]);

                indices.Add(initialTetrahedron[3]);
                indices.Add(initialTetrahedron[0]);
                indices.Add(initialTetrahedron[1]);

                for (int k = 0; k < 4; k++)
                {
                    outsidePoints.Remove(initialTetrahedron[k]);
                }
                VerifyWindings(indices, points);
                RemovePointsInPolyhedronIfInside(outsidePoints, points, indices);
            }
            else
                throw new ArgumentException("Could not form an initial tetrahedron from the input points; ensure that the input point set has volume.");
            Resources.GiveBack(initialTetrahedron);

            while (outsidePoints.Count > 0)
            {
                //While the convex hull is incomplete
                for (int k = 0; k < indices.Count; k += 3)
                {
                    //For each triangle
                    //Find the normal of the triangle
                    Vector3 normal = FindNormal(points, indices, k);
                    //Get the furthest point
                    int maxIndex;
                    Vector3 maximum = GetExtremePointOfSet(normal, outsidePoints, points, out maxIndex);
                    //If the point is visible by the triangle, continue
                    if (Vector3.Dot(maximum, normal) - Vector3.Dot(points[indices[k]], normal) > 0)
                    {
                        //It's been picked! Remove the maximum point from the outside.
                        outsidePoints.Remove(maxIndex);
                        //Remove any triangles that can see the point, including itself!
                        edges.Clear();
                        toRemove.Clear();
                        for (int n = 0; n < indices.Count; n += 3)
                        {
                            //Go through each triangle, if it can be seen, delete it and use maintainEdge on its edges.
                            normal = FindNormal(points, indices, n);
                            if (Vector3.Dot(maximum, normal) - Vector3.Dot(points[indices[n]], normal) > 0)
                            {
                                //This triangle can see it!
                                MaintainEdge(indices[n], indices[n + 1], edges);
                                MaintainEdge(indices[n], indices[n + 2], edges);
                                MaintainEdge(indices[n + 1], indices[n + 2], edges);
                                indices.RemoveAt(n);
                                indices.RemoveAt(n);
                                indices.RemoveAt(n);
                                n -= 3;
                            }
                        }
                        //Create new triangles
                        for (int n = 0; n < edges.Count; n += 2)
                        {
                            //For each edge, create a triangle with the extreme point.
                            indices.Add(edges[n]);
                            indices.Add(edges[n + 1]);
                            indices.Add(maxIndex);
                        }
                        VerifyWindings(indices, points);
                        //Remove all points from the outsidePoints if they are inside the polyhedron
                        RemovePointsInPolyhedronIfInside(outsidePoints, points, indices);

                        //The list has been significantly messed with, so restart the loop.
                        break;
                    }
                }
            }
            //"Hullify" the points.

            Resources.GiveBack(outsidePoints);
            Resources.GiveBack(edges);
            Resources.GiveBack(toRemove);
        }

        /// <summary>
        /// Identifies the points on the surface of hull.
        /// </summary>
        /// <param name="points">List of points in the set.</param>
        /// <param name="outputSurfacePoints">Unique points on the surface of the convex hull.</param>
        public static void GetConvexHull(IList<Vector3> points, IList<Vector3> outputSurfacePoints)
        {
            var indices = Resources.GetIntList();
            GetConvexHull(points, indices, outputSurfacePoints);
            Resources.GiveBack(indices);
        }

        /// <summary>
        /// Identifies the points on the surface of hull.
        /// </summary>
        /// <param name="points">List of points in the set.</param>
        /// <param name="indices">List of indices composing the triangulated surface of the convex hull.
        /// Each group of 3 indices represents a triangle on the surface of the hull.</param>
        /// <param name="outputSurfacePoints">Unique points on the surface of the convex hull.</param>
        public static void GetConvexHull(IList<Vector3> points, IList<int> outputIndices, IList<Vector3> outputSurfacePoints)
        {
            //TODO: This isn't incredibly fast, but neither is the original GetConvexHull method.
            //This whole system could use some optimization.
            GetConvexHull(points, outputIndices);

            var alreadyContainedIndices = Resources.GetIntList();

            for (int i = outputIndices.Count - 1; i >= 0; i--)
            {
                int index = outputIndices[i];
                if (!alreadyContainedIndices.Contains(index))
                {
                    outputSurfacePoints.Add(points[index]);
                    alreadyContainedIndices.Add(index);
                }
            }

            Resources.GiveBack(alreadyContainedIndices);


        }
        #endregion

        #region Miscellaneous
        ////Note: this method is a nettle-style sweep.  It can miss some collisions, even 'properly' implemented.
        //public static bool SweepSphereAgainstTriangle(Ray ray, float radius, ref Vector3 vA, ref Vector3 vB, ref Vector3 vC, TriangleSidedness sidedness, int maximumLength, out RayHit rayHit)
        //{
        //    //Put it into 'sort of local space.'
        //    Vector3 a, b, c;
        //    Vector3.Subtract(ref vA, ref ray.Position, out a);
        //    Vector3.Subtract(ref vB, ref ray.Position, out b);
        //    Vector3.Subtract(ref vC, ref ray.Position, out c);

        //    //Only perform sweep if the object is in danger of hitting the object.
        //    //Triangles can be one sided, so check the velocity against the triangle normal.
        //    Vector3 normal;
        //    Vector3 AB, AC;
        //    Vector3.Subtract(ref b, ref a, out AB);
        //    Vector3.Subtract(ref c, ref a, out AC);
        //    Vector3.Cross(ref AB, ref AC, out normal);
        //    float dot;
        //    Vector3.Dot(ref ray.Direction, ref normal, out dot);
        //    //Calibrate the normal so that it opposes the velocity and check the sidedness.
        //    switch (sidedness)
        //    {
        //        case TriangleSidedness.Clockwise:
        //            if (dot < 0)
        //            {
        //                rayHit = new RayHit();
        //                return false;
        //            }
        //            break;
        //        case TriangleSidedness.Counterclockwise:
        //            if (dot > 0)
        //            {
        //                rayHit = new RayHit();
        //                return false;
        //            }
        //            Vector3.Negate(ref normal, out normal);
        //            break;
        //        case TriangleSidedness.DoubleSided:
        //            if (dot > 0)
        //                Vector3.Negate(ref normal, out normal);
        //            break;
        //    }


        //    normal.Normalize();

        //    //If the collision is going to happen inside the triangle, then the first collision
        //    //will happen along the segment starting at the offsetOrigin and continuing along the ray direction.
        //    Vector3 offsetOrigin;
        //    Vector3.Multiply(ref normal, -radius, out offsetOrigin); //we're in the sphere's local space, so the origin is its position.

        //    //Find the location that the ray hits the triangle plane.
        //    Vector3 displacement;
        //    Vector3.Subtract(ref offsetOrigin, ref a, out displacement);
        //    float distanceFromPlane;
        //    Vector3.Dot(ref displacement, ref normal, out distanceFromPlane);
        //    Vector3.Dot(ref ray.Direction, ref normal, out dot);

        //    float t = -distanceFromPlane / dot;
        //    Vector3 planeIntersection;
        //    Vector3.Multiply(ref ray.Direction, t, out planeIntersection);
        //    Vector3.Add(ref offsetOrigin, ref planeIntersection, out planeIntersection);

        //    Vector3 closestPoint;
        //    if (Toolbox.GetClosestPointOnTriangleToPoint(ref a, ref b, ref c, ref planeIntersection, out closestPoint))
        //    {
        //        //The point was on the face of the triangle, so we're done.
        //        rayHit.T = t;
        //        Vector3.Add(ref planeIntersection, ref ray.Position, out rayHit.Location);
        //        rayHit.Normal = normal;
        //        return rayHit.T >= 0 && rayHit.T <= maximumLength;
        //    }

        //    //The intersection point was outside the triangle.  Cast back from the computed closest point
        //    //to the sphere.
        //    Ray backwardsRay;
        //    backwardsRay.Position = closestPoint;
        //    Vector3.Negate(ref ray.Direction, out backwardsRay.Direction);
        //    bool toReturn = RayCastSphere(ref backwardsRay, ref ZeroVector, radius, maximumLength, out rayHit);
        //    if (toReturn)
        //    {
        //        Vector3.Add(ref ray.Position, ref rayHit.Location, out rayHit.Location);
        //        rayHit.Normal = normal;
        //    }
        //    return toReturn;
        //}

        ///<summary>
        /// Tests a ray against a sphere.
        ///</summary>
        ///<param name="ray">Ray to test.</param>
        ///<param name="spherePosition">Position of the sphere.</param>
        ///<param name="radius">Radius of the sphere.</param>
        ///<param name="maximumLength">Maximum length of the ray in units of the ray direction's length.</param>
        ///<param name="hit">Hit data of the ray, if any.</param>
        ///<returns>Whether or not the ray hits the sphere.</returns>
        public static bool RayCastSphere(ref Ray ray, ref Vector3 spherePosition, float radius, float maximumLength, out RayHit hit)
        {
            Vector3 normalizedDirection;
            float length = ray.Direction.Length();
            Vector3.Divide(ref ray.Direction, length, out normalizedDirection);
            maximumLength *= length;
            hit = new RayHit();
            Vector3 m;
            Vector3.Subtract(ref ray.Position, ref spherePosition, out m);
            float b = Vector3.Dot(m, normalizedDirection);
            float c = m.LengthSquared() - radius * radius;

            if (c > 0 && b > 0)
                return false;
            float discriminant = b * b - c;
            if (discriminant < 0)
                return false;

            hit.T = -b - (float)Math.Sqrt(discriminant);
            if (hit.T < 0)
                hit.T = 0;
            if (hit.T > maximumLength)
                return false;
            hit.T /= length;
            Vector3.Multiply(ref normalizedDirection, hit.T, out hit.Location);
            Vector3.Add(ref hit.Location, ref ray.Position, out hit.Location);
            Vector3.Subtract(ref hit.Location, ref spherePosition, out hit.Normal);
            hit.Normal.Normalize();
            return true;
        }


        /// <summary>
        /// Computes a bounding box and expands it.
        /// </summary>
        /// <param name="shape">Shape to compute the bounding box of.</param>
        /// <param name="transform">Transform to use to position the shape.</param>
        /// <param name="sweep">Extra to add to the bounding box.</param>
        /// <param name="boundingBox">Expanded bounding box.</param>
        public static void GetExpandedBoundingBox(ref ConvexShape shape, ref RigidTransform transform, ref Vector3 sweep, out BoundingBox boundingBox)
        {
            shape.GetBoundingBox(ref transform, out boundingBox);
            ExpandBoundingBox(ref boundingBox, ref sweep);

        }
        /// <summary>
        /// Expands a bounding box by the given sweep.
        /// </summary>
        /// <param name="boundingBox">Bounding box to expand.</param>
        /// <param name="sweep">Sweep to expand the bounding box with.</param>
        public static void ExpandBoundingBox(ref BoundingBox boundingBox, ref Vector3 sweep)
        {
            if (sweep.X > 0)
                boundingBox.Max.X += sweep.X;
            else
                boundingBox.Min.X += sweep.X;

            if (sweep.Y > 0)
                boundingBox.Max.Y += sweep.Y;
            else
                boundingBox.Min.Y += sweep.Y;

            if (sweep.Z > 0)
                boundingBox.Max.Z += sweep.Z;
            else
                boundingBox.Min.Z += sweep.Z;
        }

        /// <summary>
        /// Computes the bounding box of three points.
        /// </summary>
        /// <param name="a">First vertex of the triangle.</param>
        /// <param name="b">Second vertex of the triangle.</param>
        /// <param name="c">Third vertex of the triangle.</param>
        /// <param name="aabb">Bounding box of the triangle.</param>
        public static void GetTriangleBoundingBox(ref Vector3 a, ref Vector3 b, ref Vector3 c, out BoundingBox aabb)
        {
#if !WINDOWS
            aabb = new BoundingBox();
#endif
            //X axis
            if (a.X > b.X && a.X > c.X)
            {
                //A is max
                aabb.Max.X = a.X;
                if (b.X > c.X)
                    aabb.Min.X = c.X; //C is min
                else
                    aabb.Min.X = b.X; //B is min
            }
            else if (b.X > c.X)
            {
                //B is max
                aabb.Max.X = b.X;
                if (a.X > c.X)
                    aabb.Min.X = c.X; //C is min
                else
                    aabb.Min.X = a.X; //A is min
            }
            else
            {
                //C is max
                aabb.Max.X = c.X;
                if (a.X > b.X)
                    aabb.Min.X = b.X; //B is min
                else
                    aabb.Min.X = a.X; //A is min
            }
            //Y axis
            if (a.Y > b.Y && a.Y > c.Y)
            {
                //A is max
                aabb.Max.Y = a.Y;
                if (b.Y > c.Y)
                    aabb.Min.Y = c.Y; //C is min
                else
                    aabb.Min.Y = b.Y; //B is min
            }
            else if (b.Y > c.Y)
            {
                //B is max
                aabb.Max.Y = b.Y;
                if (a.Y > c.Y)
                    aabb.Min.Y = c.Y; //C is min
                else
                    aabb.Min.Y = a.Y; //A is min
            }
            else
            {
                //C is max
                aabb.Max.Y = c.Y;
                if (a.Y > b.Y)
                    aabb.Min.Y = b.Y; //B is min
                else
                    aabb.Min.Y = a.Y; //A is min
            }
            //Z axis
            if (a.Z > b.Z && a.Z > c.Z)
            {
                //A is max
                aabb.Max.Z = a.Z;
                if (b.Z > c.Z)
                    aabb.Min.Z = c.Z; //C is min
                else
                    aabb.Min.Z = b.Z; //B is min
            }
            else if (b.Z > c.Z)
            {
                //B is max
                aabb.Max.Z = b.Z;
                if (a.Z > c.Z)
                    aabb.Min.Z = c.Z; //C is min
                else
                    aabb.Min.Z = a.Z; //A is min
            }
            else
            {
                //C is max
                aabb.Max.Z = c.Z;
                if (a.Z > b.Z)
                    aabb.Min.Z = b.Z; //B is min
                else
                    aabb.Min.Z = a.Z; //A is min
            }
        }

        /// <summary>
        /// Clamps a value between a minimum and maximum.
        /// </summary>
        /// <param name="n">Value to clamp.</param>
        /// <param name="min">Minimum value allowed.</param>
        /// <param name="max">Maximum value allowed.</param>
        /// <returns>Clamped value.</returns>
        public static float Clamp(float n, float min, float max)
        {
            if (n < min) return min;
            if (n > max) return max;
            return n;
        }

        /// <summary>
        /// Compares the vectors for equality.
        /// </summary>
        /// <param name="a">First vector for testing.</param>
        /// <param name="b">Second vector for testing.</param>
        /// <returns>Whether or not the two vectors are equal.</returns>
        public static bool AreVectorsEqual(Vector3 a, Vector3 b)
        {
            if (a.X == b.X && a.Y == b.Y && a.Z == b.Z)
                return true;
            return false;
        }

        /// <summary>
        /// Determines if two vectors have similar components.
        /// </summary>
        /// <param name="a">First vector for comparison.</param>
        /// <param name="b">Second vector for comparison.</param>
        /// <returns>Whether or not the vectors are similar.</returns>
        public static bool AreVectorsSimilar(Vector3 a, Vector3 b)
        {
            const float tolerance = .001f;
            if (Math.Abs(a.X - b.X) < tolerance && Math.Abs(a.Y - b.Y) < tolerance && Math.Abs(a.Z - b.Z) < tolerance)
                return true;
            return false;
        }

        /// <summary>
        /// Determines if a list contains a Vector3 representing an axis (both negative and positive), allowing for a small amount of error.
        /// </summary>
        /// <param name="list">List to check.</param>
        /// <param name="toCheck">Axis to check.</param>
        /// <returns>Whether or not the list contains the provided axis.</returns>
        public static bool ContainsSimilarAxis(List<Vector3> list, Vector3 toCheck)
        {
            foreach (Vector3 inList in list)
            {
                if (AreVectorsSimilar(toCheck, inList))
                    return true;
                if (AreVectorsSimilar(-toCheck, inList))
                    return true;
            }
            return false;
        }

        /// <summary>
        /// Computes the angle change represented by a normalized quaternion.
        /// </summary>
        /// <param name="q">Quaternion to be converted.</param>
        /// <returns>Angle around the axis represented by the quaternion.</returns>
        public static float GetAngleFromQuaternion(ref Quaternion q)
        {
            float qw = Math.Abs(q.W);
            if (qw > 1)
                return 0;
            return 2 * (float)Math.Acos(qw);
        }

        /// <summary>
        /// Computes the axis angle representation of a normalized quaternion.
        /// </summary>
        /// <param name="q">Quaternion to be converted.</param>
        /// <param name="axis">Axis represented by the quaternion.</param>
        /// <param name="angle">Angle around the axis represented by the quaternion.</param>
        public static void GetAxisAngleFromQuaternion(ref Quaternion q, out Vector3 axis, out float angle)
        {
#if !WINDOWS
            axis = new Vector3();
#endif
            float qx = q.X;
            float qy = q.Y;
            float qz = q.Z;
            float qw = q.W;
            if (qw < 0)
            {
                qx = -qx;
                qy = -qy;
                qz = -qz;
                qw = -qw;
            }
            if (qw > 1 - 1e-12)
            {
                axis = UpVector;
                angle = 0;
            }
            else
            {
                angle = 2 * (float)Math.Acos(qw);
                float denominator = 1 / (float)Math.Sqrt(1 - qw * qw);
                axis.X = qx * denominator;
                axis.Y = qy * denominator;
                axis.Z = qz * denominator;
            }
        }



        /// <summary>
        /// Returns the cross product matrix for the given vector.
        /// </summary>
        /// <param name="v">Vector to be used in finding the cross product matrix.</param>
        /// <returns>Cross product matrix form of the vector.</returns>
        public static Matrix GetCrossProductMatrix(Vector3 v)
        {
            //return new Matrix(0, v.Z, -v.Y, 0, -v.Z, 0, v.X, 0, v.Y, -v.X, 0, 0, 0, 0, 0, 0);
            return new Matrix(0, -v.Z, v.Y, 0, v.Z, 0, -v.X, 0, -v.Y, v.X, 0, 0, 0, 0, 0, 0);
        }




        /// <summary>
        /// Converts a vector into a matrix.
        /// </summary>
        /// <param name="v">Vector to be matrix-fied.</param>
        /// <returns>Matrix form of the vector.</returns>
        public static Matrix GetMatrixFromVector(Vector3 v)
        {
            return new Matrix(v.X, v.Y, v.Z, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        }

        /// <summary>
        /// Computes the outer product of the given vectors.
        /// </summary>
        /// <param name="a">First vector.</param>
        /// <param name="b">Second vector.</param>
        /// <returns>Outer product result.</returns>
        public static Matrix GetOuterProduct(Vector3 a, Vector3 b)
        {
            return new Matrix(a.X * b.X, a.X * b.Y, a.X * b.Z, 0,
                              a.Y * b.X, a.Y * b.Y, a.Y * b.Z, 0,
                              a.Z * b.X, a.Z * b.Y, a.Z * b.Z, 0,
                              0, 0, 0, 1);
        }

        /// <summary>
        /// Computes the quaternion rotation between two normalized vectors.
        /// </summary>
        /// <param name="v1">First unit-length vector.</param>
        /// <param name="v2">Second unit-length vector.</param>
        /// <param name="q">Quaternion representing the rotation from v1 to v2.</param>
        public static void GetQuaternionBetweenNormalizedVectors(ref Vector3 v1, ref Vector3 v2, out Quaternion q)
        {
            float dot;
            Vector3.Dot(ref v1, ref v2, out dot);
            Vector3 axis;
            Vector3.Cross(ref v1, ref v2, out axis);
            //For non-normal vectors, the multiplying the axes length squared would be necessary:
            //float w = dot + (float)Math.Sqrt(v1.LengthSquared() * v2.LengthSquared());
            if (dot < -0.9999f) //parallel, opposing direction
                q = new Quaternion(-v1.Z, v1.Y, v1.X, 0);
            else
                q = new Quaternion(axis.X, axis.Y, axis.Z, dot + 1);
            q.Normalize();
        }

        /// <summary>
        /// Finds the transposed matrix of a vector.
        /// </summary>
        /// <param name="v">Vector to be transposed.</param>
        /// <returns>Transposed vector in matrix form.</returns>
        public static Matrix GetTransposedVector(Vector3 v)
        {
            return new Matrix(v.X, 0, 0, 0, v.Y, 0, 0, 0, v.Z, 0, 0, 0, 0, 0, 0, 0);
        }

        /// <summary>
        /// Finds the velocity of a point as if it were connected to the given entity.
        /// </summary>
        /// <param name="p">Location of point.</param>
        /// <param name="obj">Collidable for connection.</param>
        /// <returns>Acceleration of the point.</returns>
        public static Vector3 GetVelocityOfPoint(Vector3 p, Entity obj)
        {
            return obj.linearVelocity + Vector3.Cross(obj.angularVelocity, p - obj.position);
        }

        /// <summary>
        /// Compares elements in the lists, removing any duplicates from the lists.
        /// Two vectors will be considered duplicate if they are parallel, even if facing opposite directions.
        /// </summary>
        /// <param name="a">First set of vectors for comparison.</param>
        /// <param name="b">Second set of vectors for comparison.</param>
        public static void PruneDirectionalDuplicates(List<Vector3> a, List<Vector3> b)
        {
            var comparison = new List<Vector3>();
            var toRemoveA = new List<Vector3>();
            var toRemoveB = new List<Vector3>();
            foreach (Vector3 v in a)
            {
                if (!ContainsSimilarAxis(comparison, v))
                    comparison.Add(v);
                else
                    toRemoveA.Add(v);
            }
            foreach (Vector3 v in b)
            {
                if (!ContainsSimilarAxis(comparison, v))
                    comparison.Add(v);
                else
                    toRemoveB.Add(v);
            }
            foreach (Vector3 v in toRemoveA)
            {
                a.Remove(v);
            }
            foreach (Vector3 v in toRemoveB)
            {
                b.Remove(v);
            }
        }



        //TODO: RK4 integrators
        internal static void IntegrateAngularVelocity(Entity e, float dt, out Quaternion initial, out Quaternion final)
        {
            initial = e.orientation;
            Vector3 angVel;
            Vector3.Multiply(ref e.angularVelocity, dt * .5f, out angVel);
            var component = new Quaternion(angVel.X, angVel.Y, angVel.Z, 0);
            Quaternion.Multiply(ref component, ref initial, out final);
            Quaternion.Add(ref initial, ref final, out final);
            Quaternion.Normalize(ref final, out final);
        }

        internal static void IntegrateAngularVelocity(Entity a, Entity b, float dt, out Quaternion aInitial, out Quaternion bInitial, out Quaternion aFinal, out Quaternion bFinal)
        {
            aInitial = a.orientation;
            bInitial = b.orientation;

            Vector3 aAngVel, bAngVel;
            Vector3.Multiply(ref a.angularVelocity, dt * .5f, out aAngVel);
            Vector3.Multiply(ref b.angularVelocity, dt * .5f, out bAngVel);
            var aComponent = new Quaternion(aAngVel.X, aAngVel.Y, aAngVel.Z, 0);
            var bComponent = new Quaternion(bAngVel.X, bAngVel.Y, bAngVel.Z, 0);
            Quaternion.Multiply(ref aComponent, ref aInitial, out aFinal);
            Quaternion.Multiply(ref bComponent, ref bInitial, out bFinal);
            Quaternion.Add(ref aInitial, ref aFinal, out aFinal);
            Quaternion.Add(ref bInitial, ref bFinal, out bFinal);
            Quaternion.Normalize(ref aFinal, out aFinal);
            Quaternion.Normalize(ref bFinal, out bFinal);
        }

        internal static void IntegrateLinearVelocity(Entity e, float dt, out Vector3 initial, out Vector3 final)
        {
            initial = e.position;
            Vector3.Multiply(ref e.linearVelocity, dt, out final);
            Vector3.Add(ref initial, ref final, out final);
        }

        internal static void IntegrateLinearVelocity(Entity a, Entity b, float dt, out Vector3 aInitial, out Vector3 bInitial, out Vector3 aFinal, out Vector3 bFinal)
        {
            aInitial = a.position;
            bInitial = b.position;
            Vector3.Multiply(ref a.linearVelocity, dt, out aFinal);
            Vector3.Add(ref aInitial, ref aFinal, out aFinal);
            Vector3.Multiply(ref b.linearVelocity, dt, out bFinal);
            Vector3.Add(ref bInitial, ref bFinal, out bFinal);
        }

        /// <summary>
        /// Updates the quaternion using RK4 integration.
        /// </summary>
        /// <param name="q">Quaternion to update.</param>
        /// <param name="localInertiaTensorInverse">Local-space inertia tensor of the object being updated.</param>
        /// <param name="angularMomentum">Angular momentum of the object.</param>
        /// <param name="dt">Time since last frame, in seconds.</param>
        /// <param name="newOrientation">New orientation quaternion.</param>
        internal static void UpdateOrientationRK4(ref Quaternion q, ref Matrix3X3 localInertiaTensorInverse, ref Vector3 angularMomentum, float dt, out Quaternion newOrientation)
        {
            //TODO: This is a little goofy
            //Quaternion diff = differentiateQuaternion(ref q, ref localInertiaTensorInverse, ref angularMomentum);
            Quaternion d1;
            DifferentiateQuaternion(ref q, ref localInertiaTensorInverse, ref angularMomentum, out d1);
            Quaternion s2;
            Quaternion.Multiply(ref d1, dt * .5f, out s2);
            Quaternion.Add(ref q, ref s2, out s2);

            Quaternion d2;
            DifferentiateQuaternion(ref s2, ref localInertiaTensorInverse, ref angularMomentum, out d2);
            Quaternion s3;
            Quaternion.Multiply(ref d2, dt * .5f, out s3);
            Quaternion.Add(ref q, ref s3, out s3);

            Quaternion d3;
            DifferentiateQuaternion(ref s3, ref localInertiaTensorInverse, ref angularMomentum, out d3);
            Quaternion s4;
            Quaternion.Multiply(ref d3, dt, out s4);
            Quaternion.Add(ref q, ref s4, out s4);

            Quaternion d4;
            DifferentiateQuaternion(ref s4, ref localInertiaTensorInverse, ref angularMomentum, out d4);

            Quaternion.Multiply(ref d1, dt / 6, out d1);
            Quaternion.Multiply(ref d2, dt / 3, out d2);
            Quaternion.Multiply(ref d3, dt / 3, out d3);
            Quaternion.Multiply(ref d4, dt / 6, out d4);
            Quaternion added;
            Quaternion.Add(ref q, ref d1, out added);
            Quaternion.Add(ref added, ref d2, out added);
            Quaternion.Add(ref added, ref d3, out added);
            Quaternion.Add(ref added, ref d4, out added);
            Quaternion.Normalize(ref added, out newOrientation);
        }


        /// <summary>
        /// Finds the change in the rotation state quaternion provided the local inertia tensor and angular velocity.
        /// </summary>
        /// <param name="orientation">Orienatation of the object.</param>
        /// <param name="localInertiaTensorInverse">Local-space inertia tensor of the object being updated.</param>
        /// <param name="angularMomentum">Angular momentum of the object.</param>
        ///  <param name="orientationChange">Change in quaternion.</param>
        internal static void DifferentiateQuaternion(ref Quaternion orientation, ref Matrix3X3 localInertiaTensorInverse, ref Vector3 angularMomentum, out Quaternion orientationChange)
        {
            Quaternion normalizedOrientation;
            Quaternion.Normalize(ref orientation, out normalizedOrientation);
            Matrix3X3 tempRotMat;
            Matrix3X3.CreateFromQuaternion(ref normalizedOrientation, out tempRotMat);
            Matrix3X3 tempInertiaTensorInverse;
            Matrix3X3.MultiplyTransposed(ref tempRotMat, ref localInertiaTensorInverse, out tempInertiaTensorInverse);
            Matrix3X3.Multiply(ref tempInertiaTensorInverse, ref tempRotMat, out tempInertiaTensorInverse);
            Vector3 halfspin;
            Matrix3X3.Transform(ref angularMomentum, ref tempInertiaTensorInverse, out halfspin);
            Vector3.Multiply(ref halfspin, .5f, out halfspin);
            var halfspinQuaternion = new Quaternion(halfspin.X, halfspin.Y, halfspin.Z, 0);
            Quaternion.Multiply(ref halfspinQuaternion, ref normalizedOrientation, out orientationChange);
        }

        
        /// <summary>
        /// Gets the barycentric coordinates of the point with respect to a triangle's vertices.
        /// </summary>
        /// <param name="p">Point to compute the barycentric coordinates of.</param>
        /// <param name="a">First vertex in the triangle.</param>
        /// <param name="b">Second vertex in the triangle.</param>
        /// <param name="c">Third vertex in the triangle.</param>
        /// <param name="aWeight">Weight of the first vertex.</param>
        /// <param name="bWeight">Weight of the second vertex.</param>
        /// <param name="cWeight">Weight of the third vertex.</param>
        public static void GetBarycentricCoordinates(ref Vector3 p, ref Vector3 a, ref Vector3 b, ref Vector3 c, out float aWeight, out float bWeight, out float cWeight)
        {
            Vector3 ab, ac;
            Vector3.Subtract(ref b, ref a, out ab);
            Vector3.Subtract(ref c, ref a, out ac);
            Vector3 triangleNormal;
            Vector3.Cross(ref ab, ref ac, out triangleNormal);
            float x = triangleNormal.X < 0 ? -triangleNormal.X : triangleNormal.X;
            float y = triangleNormal.Y < 0 ? -triangleNormal.Y : triangleNormal.Y;
            float z = triangleNormal.Z < 0 ? -triangleNormal.Z : triangleNormal.Z;
            
            float numeratorU, numeratorV, denominator;
            if (x >= y && x >= z)
            {
                //The projection of the triangle on the YZ plane is the largest.
                numeratorU = (p.Y - b.Y) * (b.Z - c.Z) - (b.Y - c.Y) * (p.Z - b.Z); //PBC
                numeratorV = (p.Y - c.Y) * (c.Z - a.Z) - (c.Y - a.Y) * (p.Z - c.Z); //PCA
                denominator = triangleNormal.X;
            }
            else if (y >= z)
            {
                //The projection of the triangle on the XZ plane is the largest.
                numeratorU = (p.X - b.X) * (b.Z - c.Z) - (b.X - c.X) * (p.Z - b.Z); //PBC
                numeratorV = (p.X - c.X) * (c.Z - a.Z) - (c.X - a.X) * (p.Z - c.Z); //PCA
                denominator = -triangleNormal.Y;
            }
            else
            {
                //The projection of the triangle on the XY plane is the largest.
                numeratorU = (p.X - b.X) * (b.Y - c.Y) - (b.X - c.X) * (p.Y - b.Y); //PBC
                numeratorV = (p.X - c.X) * (c.Y - a.Y) - (c.X - a.X) * (p.Y - c.Y); //PCA
                denominator = triangleNormal.Z;
            }

            if (denominator < -1e-9 || denominator > 1e-9)
            {
                denominator = 1 / denominator;
                aWeight = numeratorU * denominator;
                bWeight = numeratorV * denominator;
                cWeight = 1 - aWeight - bWeight;
            }
            else
            {               
                //It seems to be a degenerate triangle.
                //In that case, pick one of the closest vertices.
                //MOST of the time, this will happen when the vertices
                //are all very close together (all three points form a single point).
                //Sometimes, though, it could be that it's more of a line.
                //If it's a little inefficient, don't worry- this is a corner case anyway.

                float distance1, distance2, distance3;
                Vector3.DistanceSquared(ref p, ref a, out distance1);
                Vector3.DistanceSquared(ref p, ref b, out distance2);
                Vector3.DistanceSquared(ref p, ref c, out distance3);
                if (distance1 < distance2 && distance1 < distance3)
                {
                    aWeight = 1;
                    bWeight = 0;
                    cWeight = 0;
                }
                else if (distance2 < distance3)
                {
                    aWeight = 0;
                    bWeight = 1;
                    cWeight = 0;
                }
                else
                {
                    aWeight = 0;
                    bWeight = 0;
                    cWeight = 1;
                }
            }

            //if (((a * aWeight + b * bWeight + c * cWeight) - p).LengthSquared() > .01f)
            //    return;
            
        }

       

        ///// <summary>
        ///// Gets the barycentric coordinates of the point with respect to a triangle's vertices.
        ///// </summary>
        ///// <param name="point">Point to compute the barycentric coordinates of.</param>
        ///// <param name="v1">First vertex in the triangle.</param>
        ///// <param name="v2">Second vertex in the triangle.</param>
        ///// <param name="v3">Third vertex in the triangle.</param>
        ///// <param name="v1Weight">Weight of the first vertex.</param>
        ///// <param name="v2Weight">Weight of the second vertex.</param>
        ///// <param name="v3Weight">Weight of the third vertex.</param>
        //public static void GetBarycentricCoordinates(ref Vector3 point, ref Vector3 v1, ref Vector3 v2, ref Vector3 v3, out float v1Weight, out float v2Weight, out float v3Weight)
        //{
        //    //Triangle edge vectors.   
        //    Vector3 e0, e1, e2;
        //    Vector3.Subtract(ref v3, ref v1, out e0);
        //    Vector3.Subtract(ref v2, ref v1, out e1);
        //    Vector3.Subtract(ref point, ref v1, out e2);

        //    //Set up precalculations
        //    float d00 = e0.LengthSquared();
        //    float d01;
        //    Vector3.Dot(ref e0, ref e1, out d01);
        //    float d02;
        //    Vector3.Dot(ref e0, ref e2, out d02);
        //    float d11 = e1.LengthSquared();
        //    float d12;
        //    Vector3.Dot(ref e1, ref e2, out d12);
        //    //Find barycoords
        //    float denom = (d00 * d11 - d01 * d01);
        //    if (denom < 1e-9f && denom > -1e-9f)
        //    {
        //        //It seems to be a degenerate triangle.
        //        //In that case, pick one of the closest vertices.
        //        //MOST of the time, this will happen when the vertices
        //        //are all very close together (all three points form a single point).
        //        //Sometimes, though, it could be that it's more of a line.
        //        //If it's a little inefficient, don't worry- this is a corner case anyway.

        //        float distance1, distance2, distance3;
        //        Vector3.DistanceSquared(ref point, ref v1, out distance1);
        //        Vector3.DistanceSquared(ref point, ref v2, out distance2);
        //        Vector3.DistanceSquared(ref point, ref v3, out distance3);
        //        if (distance1 < distance2 && distance1 < distance3)
        //        {
        //            v1Weight = 1;
        //            v2Weight = 0;
        //            v3Weight = 0;
        //        }
        //        else if (distance2 < distance3)
        //        {
        //            v1Weight = 0;
        //            v2Weight = 1;
        //            v3Weight = 0;
        //        }
        //        else
        //        {
        //            v1Weight = 0;
        //            v2Weight = 0;
        //            v3Weight = 1;
        //        }
        //    }
        //    else
        //    {
        //        denom = 1 / denom;
        //        v3Weight = (d11 * d02 - d01 * d12) * denom;
        //        v2Weight = (d00 * d12 - d01 * d02) * denom;
        //        v1Weight = 1 - v3Weight - v2Weight;
        //    }


        //    if (((v1 * v1Weight + v2 * v2Weight + v3 * v3Weight) - point).LengthSquared() > 1f)
        //        return;

        //}

        #endregion

        #region debug functions

        #endregion

        #region Shape-plane volume tests

        /// <summary>
        /// Calculates the volume of the sphere 'below' the given plane.
        /// </summary>
        /// <param name="spherePosition">Center of mass of the sphere.</param>
        /// <param name="sphereVolume">Precalculated total volume of the sphere (4/3 * pi * radius ^ 3).</param>
        /// <param name="radius">Radius of the sphere.</param>
        /// <param name="p">A point on the plane.</param>
        /// <param name="norm">Normal of the plane.</param>
        /// <param name="volume">Volume of the sphere which is opposite the normal direction.</param>
        public static void GetSphereVolumeSplitByPlane(Vector3 spherePosition, float sphereVolume, float radius, Vector3 p, Vector3 norm, out float volume)
        {
            Vector3 newP = GetPointProjectedOnPlane(spherePosition, norm, p);
            Vector3 diff = newP - spherePosition;

            float distance = diff.Length();
            //Strange but true!
            float h = radius - distance;
            float vol = (float)Math.PI * h * h * (radius - .3333333f * h);
            if (Vector3.Dot(diff, norm) > 0) //more than half taken up 
                volume = sphereVolume - vol;
            else
                volume = vol;
        }

        #endregion





    }
}