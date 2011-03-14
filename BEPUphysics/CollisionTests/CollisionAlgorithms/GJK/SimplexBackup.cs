/*using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionShapes;

namespace BEPUphysics.CollisionTests.CollisionAlgorithms
{
    public enum SimplexState : byte
    {
        Empty,
        Point,
        Segment,
        Triangle,
        Tetrahedron
    }

    public struct CachedSimplex
    {
        public ContributingShapeSimplex LocalSimplexA;
        public ContributingShapeSimplex LocalSimplexB;
        public SimplexState State;
    }

    public struct ContributingShapeSimplex
    {
        public Vector3 A;
        public Vector3 B;
        public Vector3 C;
        public Vector3 D;
    }

    public struct PairSimplex
    {
        public ContributingShapeSimplex SimplexA;
        public ContributingShapeSimplex SimplexB;
        public Vector3 A;
        public Vector3 B;
        public Vector3 C;
        public Vector3 D;
        public SimplexState State;
        public float U;
        public float V;
        public float W;
        public RigidTransform LocalTransformB;

        public PairSimplex(ref RigidTransform localTransformB)
        {
            LocalTransformB = localTransformB;
            //Warm up the simplex using the centroids.
            //Could also use the GetNewSimplexPoint if it had a Empty case, but test before choosing.
            State = SimplexState.Point;
            SimplexA = new ContributingShapeSimplex();
            SimplexB = new ContributingShapeSimplex();
            SimplexB.A = localTransformB.Position;
            //minkowski space support = shapeA-shapeB = 0,0,0 - positionB
            Vector3.Negate(ref localTransformB.Position, out A);
            B = new Vector3();
            C = new Vector3();
            D = new Vector3();
            U = 0;
            V = 0;
            W = 0;
        }

        private PairSimplex(ref CachedSimplex cachedSimplex, ref RigidTransform localTransformB)
        {
            //TODO: NOTE:
            //USING A CACHED SIMPLEX INVALIDATES ASSUMPTIONS THAT ALLOW SIMPLEX CASES TO BE IGNORED!
            //To get those assumptions back, either DO NOT USE CACHED SIMPLEXES, or 
            //VERIFY THE SIMPLEXES.
            //-A point requires no verification.
            //-A segment needs verification that the origin is in front of A in the direction of B.
            //-A triangle needs verification that the origin is within the edge planes and in the direction of C.
            //-A tetrahedron needs verification that the origin is within the edge planes of triangle ABC and is in the direction of D.
            //Furthermore, due to relative movement, the simplex may become degenerate.  Edges could become points, etc.  More verification.


            LocalTransformB = localTransformB;

            //Transform the SimplexB into the working space of the simplex and compute the working space simplex.
            State = cachedSimplex.State;
            SimplexA = cachedSimplex.LocalSimplexA;
            SimplexB = new ContributingShapeSimplex();
            U = 0;
            V = 0;
            W = 0;
            switch (State)
            {
                case SimplexState.Point:
                    Vector3.Transform(ref cachedSimplex.LocalSimplexB.A, ref LocalTransformB.Orientation, out SimplexB.A);
                    Vector3.Add(ref SimplexB.A, ref LocalTransformB.Position, out SimplexB.A);

                    Vector3.Subtract(ref SimplexA.A, ref SimplexB.A, out A);
                    B = new Vector3();
                    C = new Vector3();
                    D = new Vector3();
                    break;
                case SimplexState.Segment:
                    Vector3.Transform(ref cachedSimplex.LocalSimplexB.A, ref LocalTransformB.Orientation, out SimplexB.A);
                    Vector3.Transform(ref cachedSimplex.LocalSimplexB.B, ref LocalTransformB.Orientation, out SimplexB.B);
                    Vector3.Add(ref SimplexB.A, ref LocalTransformB.Position, out SimplexB.A);
                    Vector3.Add(ref SimplexB.B, ref LocalTransformB.Position, out SimplexB.B);

                    Vector3.Subtract(ref SimplexA.A, ref SimplexB.A, out A);
                    Vector3.Subtract(ref SimplexA.B, ref SimplexB.B, out B);
                    C = new Vector3();
                    D = new Vector3();
                    break;
                case SimplexState.Triangle:
                    Vector3.Transform(ref cachedSimplex.LocalSimplexB.A, ref LocalTransformB.Orientation, out SimplexB.A);
                    Vector3.Transform(ref cachedSimplex.LocalSimplexB.B, ref LocalTransformB.Orientation, out SimplexB.B);
                    Vector3.Transform(ref cachedSimplex.LocalSimplexB.C, ref LocalTransformB.Orientation, out SimplexB.C);
                    Vector3.Add(ref SimplexB.A, ref LocalTransformB.Position, out SimplexB.A);
                    Vector3.Add(ref SimplexB.B, ref LocalTransformB.Position, out SimplexB.B);
                    Vector3.Add(ref SimplexB.C, ref LocalTransformB.Position, out SimplexB.C);

                    Vector3.Subtract(ref SimplexA.A, ref SimplexB.A, out A);
                    Vector3.Subtract(ref SimplexA.B, ref SimplexB.B, out B);
                    Vector3.Subtract(ref SimplexA.C, ref SimplexB.C, out C);
                    D = new Vector3();
                    break;
                case SimplexState.Tetrahedron:
                    Vector3.Transform(ref cachedSimplex.LocalSimplexB.A, ref LocalTransformB.Orientation, out SimplexB.A);
                    Vector3.Transform(ref cachedSimplex.LocalSimplexB.B, ref LocalTransformB.Orientation, out SimplexB.B);
                    Vector3.Transform(ref cachedSimplex.LocalSimplexB.C, ref LocalTransformB.Orientation, out SimplexB.C);
                    Vector3.Transform(ref cachedSimplex.LocalSimplexB.D, ref LocalTransformB.Orientation, out SimplexB.D);
                    Vector3.Add(ref SimplexB.A, ref LocalTransformB.Position, out SimplexB.A);
                    Vector3.Add(ref SimplexB.B, ref LocalTransformB.Position, out SimplexB.B);
                    Vector3.Add(ref SimplexB.C, ref LocalTransformB.Position, out SimplexB.C);
                    Vector3.Add(ref SimplexB.D, ref LocalTransformB.Position, out SimplexB.D);

                    Vector3.Subtract(ref SimplexA.A, ref SimplexB.A, out A);
                    Vector3.Subtract(ref SimplexA.B, ref SimplexB.A, out B);
                    Vector3.Subtract(ref SimplexA.C, ref SimplexB.A, out C);
                    Vector3.Subtract(ref SimplexA.D, ref SimplexB.A, out D);
                    break;
                default:
                    A = new Vector3();
                    B = new Vector3();
                    C = new Vector3();
                    D = new Vector3();
                    break;
            }
        }

        public bool GetPointClosestToOrigin(out Vector3 point)
        {
            //This method finds the closest point on the simplex to the origin.
            //Barycentric coordinates are assigned to the MinimumNormCoordinates as necessary to perform the inclusion calculation.
            //If the simplex is a tetrahedron and found to be overlapping the origin, the function returns true to tell the caller to terminate.
            //Elements of the simplex that are not used to determine the point of minimum norm are removed from the simplex.

            switch (State)
            {

                case SimplexState.Point:
                    point = A;
                    U = 1;
                    break;
                case SimplexState.Segment:
                    GetPointOnSegmentClosestToOrigin(out point);
                    break;
                case SimplexState.Triangle:
                    GetPointOnTriangleClosestToOrigin(out point);
                    break;
                case SimplexState.Tetrahedron:
                    return GetPointOnTetrahedronClosestToOrigin(out point);
                default:
                    point = Toolbox.ZeroVector;
                    break;


            }
            return false;
        }


        public void GetPointOnSegmentClosestToOrigin(out Vector3 point)
        {
            Vector3 segmentDisplacement;
            Vector3.Subtract(ref B, ref A, out segmentDisplacement);
            float dotB;
            Vector3.Dot(ref segmentDisplacement, ref B, out dotB);
            if (dotB > 0)
            {
                //Inside segment.
                U = dotB / segmentDisplacement.LengthSquared();
                V = 1 - U;
                Vector3.Multiply(ref segmentDisplacement, V, out point);
                Vector3.Add(ref point, ref A, out point);

            }
            else
            {
                //TODO: Can it ever be outside B?  It should be possible in the closest point calculation.
                //It is not possible in a 'boolean' GJK, where it early outs as soon as a separating axis is found.

                //Outside B.
                //Remove current A; we're becoming a point.
                A = B;
                SimplexA.A = SimplexA.B;
                SimplexB.A = SimplexB.B;
                State = SimplexState.Point;

                U = 1;
                point = A;
            }
            //It can never be outside A! 
            //That would mean that the origin is LESS extreme along the search direction than our extreme point--- our search direction would not have picked that direction.
        }

        public void GetPointOnTriangleClosestToOrigin(out Vector3 point)
        {
            Vector3 ab, ac;
            Vector3.Subtract(ref B, ref A, out ab);
            Vector3.Subtract(ref C, ref A, out ac);
            //The point we are comparing against the triangle is 0,0,0, so instead of storing an "A->P" vector,
            //just use -A.
            //Same for B->, C->P...

            //CAN'T BE IN A'S REGION.

            //CAN'T BE IN B'S REGION.

            //CAN'T BE IN AB'S REGION.

            //Check to see if it's outside C.
            //TODO: Note that in a boolean-style GJK, it shouldn't be possible to be outside C.
            float d5, d6;
            Vector3.Dot(ref ab, ref C, out d5);
            Vector3.Dot(ref ac, ref C, out d6);
            d5 = -d5;
            d6 = -d6;
            if (d6 >= 0f && d5 <= d6)
            {
                //It is C!
                State = SimplexState.Point;
                A = C;
                SimplexA.A = SimplexA.C;
                SimplexB.A = SimplexB.C;
                U = 1;
                point = A;
                return;
            }

            //Check if it's outside AC.            
            float d1, d2;
            Vector3.Dot(ref ab, ref A, out d1);
            Vector3.Dot(ref ac, ref A, out d2);
            d1 = -d1;
            d2 = -d2;
            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 > 0f && d6 < 0f)//Note > instead of >= and < instead of <=; prevents bad denominator
            {
                //Get rid of B.  Compress C into B.
                State = SimplexState.Segment;
                B = C;
                SimplexA.B = SimplexA.C;
                SimplexB.B = SimplexB.C;
                V = d2 / (d2 - d6);
                U = 1 - V;
                Vector3.Multiply(ref ac, V, out point);
                Vector3.Add(ref point, ref A, out point);
                return;
            }

            //Check if it's outside BC.
            float d3, d4;
            Vector3.Dot(ref ab, ref B, out d3);
            Vector3.Dot(ref ac, ref B, out d4);
            d3 = -d3;
            d4 = -d4;
            float va = d3 * d6 - d5 * d4;
            float d3d4;
            float d6d5;
            if (va <= 0f && (d3d4 = d4 - d3) > 0f && (d6d5 = d5 - d6) > 0f)//Note > instead of >= and < instead of <=; prevents bad denominator
            {
                //Throw away A.  C->A.
                //TODO: Does B->A, C->B work better?
                State = SimplexState.Segment;
                A = C;
                SimplexA.A = SimplexA.C;
                SimplexB.A = SimplexB.C;
                U = d3d4 / (d3d4 + d6d5);
                V = 1 - U;

                Vector3 bc;
                Vector3.Subtract(ref C, ref B, out bc);
                Vector3.Multiply(ref bc, U, out point);
                Vector3.Add(ref point, ref B, out point);
                return;
            }


            //On the face of the triangle.
            float vc = d1 * d4 - d3 * d2;
            float denom = 1f / (va + vb + vc);
            V = vb * denom;
            W = vc * denom;
            U = 1 - V - W;
            Vector3.Multiply(ref ab, V, out point);
            Vector3 acw;
            Vector3.Multiply(ref ac, W, out acw);
            Vector3.Add(ref A, ref point, out point);
            Vector3.Add(ref point, ref acw, out point);

            //The origin is on the 'inside' side of this triangle's plane.
            //To verify winding, check the sign.  If it's positive, the normal is facing 'into' a possible tetrahedron, so flip the normal by flipping the order.
            //This new winding ensures that every triangle considered in the tetrahedral case will have a predictable winding.
            if (denom > 0)
            {
                //Flip the ordering.
                float temp = W;
                W = V;
                V = temp;

                D = B;
                B = C;
                C = D;

                SimplexA.D = SimplexA.B;
                SimplexA.B = SimplexA.C;
                SimplexA.C = SimplexA.D;

                SimplexB.D = SimplexB.B;
                SimplexB.B = SimplexB.C;
                SimplexB.C = SimplexB.D;
            }



        }

        public bool GetPointOnTetrahedronClosestToOrigin(out Vector3 point)
        {
            //Thanks to the fact that D is new and that we know that the origin is within the extruded
            //triangular prism of ABC (and on the "D" side of ABC),
            //we can immediately ignore voronoi regions:
            //A, B, C, AC, AB, BC, ABC
            //and only consider:
            //D, DA, DB, DC, DAC, DCB, DBA

            
            //VERTEX D
            float DdotDA, DdotDB, DdotDC;
            Vector3 DA, DB, DC;
            Vector3.Subtract(ref A, ref D, out DA);
            Vector3.Subtract(ref B, ref D, out DB);
            Vector3.Subtract(ref C, ref D, out DC);
            Vector3.Dot(ref D, ref DA, out DdotDA);
            Vector3.Dot(ref D, ref DB, out DdotDB);
            Vector3.Dot(ref D, ref DC, out DdotDC);

            if (DdotDA >= 0 && DdotDB >= 0 && DdotDC >= 0)
            {
                //Outside of D.  Reduce to single point.
                State = SimplexState.Point;
                A = D;
                SimplexA.A = SimplexA.D;
                SimplexB.A = SimplexB.D;
                U = 1;
                point = A;
                return false;
            }


            //EDGE DA
            float AdotDA;
            float BdotDA;
            float CdotDA;
            Vector3.Dot(ref A, ref DA, out AdotDA);
            Vector3.Dot(ref B, ref DA, out BdotDA);
            Vector3.Dot(ref C, ref DA, out CdotDA);
            float DAdotDA = AdotDA - DdotDA;
            float DBdotDA = BdotDA - DdotDA;
            float DCdotDA = CdotDA - DdotDA;
            if (DdotDA < 0 && AdotDA >= 0 && //Within edge planes
                DAdotDA * DdotDC - DdotDA * DCdotDA >= 0 && //Within DAC triangle plane
                DdotDB * DAdotDA - DBdotDA * DdotDA >= 0) //Within DBA triangle plane
            {
                //Compress into edge AB.  A->B, D->A.
                State = SimplexState.Segment;
                B = A;
                A = D;
                SimplexA.B = SimplexA.A;
                SimplexB.A = SimplexB.D;
                V = AdotDA / DAdotDA;
                U = 1 - V;
                Vector3.Multiply(ref DA, V, out point);
                Vector3.Add(ref point, ref A, out point);
                return false;
            }

            //EDGE DB
            float AdotDB;
            float BdotDB;
            float CdotDB;
            Vector3.Dot(ref A, ref DB, out AdotDB);
            Vector3.Dot(ref B, ref DB, out BdotDB);
            Vector3.Dot(ref C, ref DB, out CdotDB);
            float DBdotDB = BdotDB - DdotDB;
            float DCdotDB = CdotDB - DdotDB;
            if (DdotDB < 0 && BdotDB >= 0 && //Within edge planes
                DCdotDB * DdotDB - DdotDC * DBdotDB >= 0 && //Within DCB triangle plane
                DdotDB * DBdotDA - DBdotDB * DdotDA >= 0) //Within DBA triangle plane
            {
                //Compress into edge AB.  D->A.
                State = SimplexState.Segment;
                A = D;
                SimplexB.A = SimplexB.D;
                V = BdotDB / DBdotDB;
                U = 1 - V;
                Vector3.Multiply(ref DB, U, out point);
                Vector3.Add(ref point, ref A, out point);
                return false;
            }

            //EDGE DC
            float AdotDC;
            float BdotDC;
            float CdotDC;
            Vector3.Dot(ref A, ref DC, out AdotDC);
            Vector3.Dot(ref B, ref DC, out BdotDC);
            Vector3.Dot(ref C, ref DC, out CdotDC);
            float DCdotDC = CdotDC - DdotDC;
            if (DdotDC < 0 && CdotDC >= 0 && //Within edge planes
                DCdotDA * DdotDC - DdotDA * DCdotDC >= 0 && //Within DAC triangle plane
                DdotDC * DCdotDB - DCdotDC * DdotDB >= 0) //Within DCB triangle plane
            {
                //Compress into edge AB.  D->A, C->B.
                State = SimplexState.Segment;
                A = D;
                B = C;
                SimplexB.A = SimplexB.D;
                SimplexB.B = SimplexB.C;
                V = CdotDC / DCdotDC;
                U = 1 - V;
                Vector3.Multiply(ref DC, U, out point);
                Vector3.Add(ref point, ref A, out point);
                return false;
            }

            //TRIANGLE DAC
            float va = AdotDA * CdotDC - CdotDA * AdotDC;
            float vb = CdotDA * DdotDC - DdotDA * CdotDC;
            float vc = DdotDA * AdotDC - AdotDA * DdotDC;
            if (va > 0 && vb > 0 && vc > 0) //Can't be negative; point needs to be outside of the face.
            {
                //Inside triangle, compress C->B, A->C, D->A.  Must flip winding to DCA.  (The origin is outside this winding; the next update will make this 'outside' the 'inside.')
                State = SimplexState.Triangle;
                float denom = 1f / (va + vb + vc);
                B = C;
                C = A;
                A = D;

                SimplexA.B = SimplexA.C;
                SimplexA.C = SimplexA.A;
                SimplexA.A = SimplexA.D;

                SimplexB.B = SimplexB.C;
                SimplexB.C = SimplexB.A;
                SimplexB.A = SimplexB.D;

                W = vb * denom;
                V = vc * denom;
                U = 1 - V - W;

                //TODO: Verify this and the barycentric coordinates; could be problematic due to winding flip.
                Vector3.Multiply(ref DA, W, out point);
                Vector3 acw;
                Vector3.Multiply(ref DC, V, out acw);
                Vector3.Add(ref A, ref point, out point);
                Vector3.Add(ref point, ref acw, out point);
                return false;

            }

            //TRIANGLE DCB
            va = CdotDC * BdotDB - BdotDC * CdotDB;
            vb = BdotDC * DdotDB - DdotDC * BdotDB;
            vc = DdotDC * CdotDB - CdotDC * DdotDB;
            if (va > 0 && vb > 0 && vc > 0) //Can't be negative; point needs to be outside of the face.
            {
                //Inside triangle, compress D->A.  Must flip winding to DBC.
                State = SimplexState.Triangle;
                float denom = 1f / (va + vb + vc);
                A = D;
                SimplexA.A = SimplexA.D;
                SimplexB.A = SimplexB.D;

                W = vb * denom;
                V = vc * denom;
                U = 1 - V - W;

                //TODO: Verify this and the barycentric coordinates; could be problematic due to winding flip.
                Vector3.Multiply(ref DC, W, out point);
                Vector3 acw;
                Vector3.Multiply(ref DB, V, out acw);
                Vector3.Add(ref A, ref point, out point);
                Vector3.Add(ref point, ref acw, out point);
                return false;

            }

            //TRIANGLE DBA
            va = BdotDB * AdotDA - AdotDB * BdotDA;
            vb = AdotDB * DdotDA - DdotDB * AdotDA;
            vc = DdotDB * BdotDA - BdotDB * DdotDA;
            if (va > 0 && vb > 0 && vc > 0) //Can't be negative; point needs to be outside of the face.
            {
                //Inside triangle, compress D->A, A->C.  Must flip winding to DAB.
                State = SimplexState.Triangle;
                float denom = 1f / (va + vb + vc);
                C = B;
                B = A;
                A = D;
                SimplexA.C = SimplexA.B;
                SimplexA.B = SimplexA.A;
                SimplexA.A = SimplexA.D;

                SimplexB.C = SimplexB.B;
                SimplexB.B = SimplexB.A;
                SimplexB.A = SimplexB.D;
                W = vb * denom;
                V = vc * denom;
                U = 1 - V - W;

                //TODO: Verify this and the barycentric coordinates; could be problematic due to winding flip.
                Vector3.Multiply(ref DB, W, out point);
                Vector3 acw;
                Vector3.Multiply(ref DA, V, out acw);
                Vector3.Add(ref A, ref point, out point);
                Vector3.Add(ref point, ref acw, out point);
                return false;

            }

            //It's inside the tetrahedron.
            point = Toolbox.ZeroVector; //The origin is, in fact, the closest point...
            //Do not compute barycentric coordinates for this case.
            //Closest points are not returned in the intersecting case, so it's not necessary data.
            return true;


        }

        public bool GetNewSimplexPoint(ConvexShape shapeA, ConvexShape shapeB, ref Vector3 closestPoint)
        {
            Vector3 negativeDirection;
            Vector3.Negate(ref closestPoint, out negativeDirection);
            Vector3 sa, sb;
            shapeA.GetLocalExtremePointWithoutMargin(ref negativeDirection, out sa);
            shapeB.GetExtremePointWithoutMargin(closestPoint, ref LocalTransformB, out sb);
            Vector3 S;
            Vector3.Subtract(ref sa, ref sb, out S);
            //If S is not further towards the origin along negativeDirection than closestPoint, then we're done.
            float dotS, dotClosest;
            Vector3.Dot(ref S, ref negativeDirection, out dotS); //-P * S
            dotClosest = -closestPoint.LengthSquared(); // -P * P
            //TODO: NUMERICAL STABILITY ISSUES POSSIBLE HERE.
            if (dotS - dotClosest <= Toolbox.BigEpsilon)
                return true; //No meaningful progress, no intersection.

            //If "A" is the new point always, then the switch statement can be removed
            //in favor of just pushing three points up.
            switch (State)
            {
                case SimplexState.Point:
                    State = SimplexState.Segment;
                    B = S;
                    SimplexA.B = sa;
                    SimplexB.B = sb;
                    break;
                case SimplexState.Segment:
                    State = SimplexState.Triangle;
                    C = S;
                    SimplexA.C = sa;
                    SimplexB.C = sb;
                    break;
                case SimplexState.Triangle:
                    State = SimplexState.Tetrahedron;
                    D = S;
                    SimplexA.D = sa;
                    SimplexB.D = sb;
                    break;
            }
            return false;
        }

        public void GetClosestPoints(out Vector3 closestPointA, out Vector3 closestPointB)
        {
            //A * U + B * V + C * W
            switch (State)
            {
                case SimplexState.Point:
                    closestPointA = SimplexA.A;
                    closestPointB = SimplexB.A;
                    return;
                case SimplexState.Segment:
                    Vector3 temp;
                    Vector3.Multiply(ref SimplexA.A, U, out closestPointA);
                    Vector3.Multiply(ref SimplexA.B, V, out temp);
                    Vector3.Add(ref closestPointA, ref temp, out closestPointA);

                    Vector3.Multiply(ref SimplexB.A, U, out closestPointB);
                    Vector3.Multiply(ref SimplexB.B, V, out temp);
                    Vector3.Add(ref closestPointB, ref temp, out closestPointB);
                    return;
                case SimplexState.Triangle:
                    Vector3.Multiply(ref SimplexA.A, U, out closestPointA);
                    Vector3.Multiply(ref SimplexA.B, V, out temp);
                    Vector3.Add(ref closestPointA, ref temp, out closestPointA);
                    Vector3.Multiply(ref SimplexA.C, W, out temp);
                    Vector3.Add(ref closestPointA, ref temp, out closestPointA);

                    Vector3.Multiply(ref SimplexB.A, U, out closestPointB);
                    Vector3.Multiply(ref SimplexB.B, V, out temp);
                    Vector3.Add(ref closestPointB, ref temp, out closestPointB);
                    Vector3.Multiply(ref SimplexB.C, W, out temp);
                    Vector3.Add(ref closestPointB, ref temp, out closestPointB);
                    return;
            }
            closestPointA = Toolbox.ZeroVector;
            closestPointB = Toolbox.ZeroVector;

        }
    }

}
*/