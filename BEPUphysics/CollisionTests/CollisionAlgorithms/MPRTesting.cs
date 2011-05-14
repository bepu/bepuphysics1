using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;
using System.Diagnostics;

namespace BEPUphysics.CollisionTests.CollisionAlgorithms
{
    public static class MPRTesting
    {
        //1) Determine if the shapes are colliding.  Return false if they are ever found not to be.
        //2) If there exists a desired separating direction, perform a second raycast from the origin to the surface along the direction.
        //   If there does not exist a desired separating direction, continue progressing along the same direction until the surface is hit.
        //   

        public static Vector3 DEBUGlastRayDirection;
        public static Vector3 DEBUGlastNormal;
        public static Vector3 DEBUGlastPosition;
        public static float DEBUGlastDepth;
        public static float DEBUGlastRayT;
        public static Vector3 DEBUGlastV1;
        public static Vector3 DEBUGlastV2;
        public static Vector3 DEBUGlastV3;

        public static bool GetOverlapPosition(ConvexShape shapeA, ConvexShape shapeB, ref RigidTransform transformA, ref RigidTransform transformB, out Vector3 position)
        {
            RigidTransform localTransformB;
            MinkowskiToolbox.GetLocalTransform(ref transformA, ref transformB, out localTransformB);
            bool toReturn = GetLocalOverlapPosition(shapeA, shapeB, ref localTransformB, out position);
            RigidTransform.Transform(ref position, ref transformA, out position);
            return toReturn;

        }

        public static bool GetLocalOverlapPosition(ConvexShape shapeA, ConvexShape shapeB, ref RigidTransform localTransformB, out Vector3 position)
        {
            return GetLocalOverlapPosition(shapeA, shapeB, ref localTransformB.Position, ref localTransformB, out position);
        }

        //Sometimes we need to be able to define an origin ray which is not based on the transforms.
        public static bool GetLocalOverlapPosition(ConvexShape shapeA, ConvexShape shapeB, ref Vector3 originRay, ref RigidTransform localTransformB, out Vector3 position)
        {
            //Compute the origin ray.  This points from a point known to be inside the minkowski sum to the origin.
            //The centers of the shapes are used to create the interior point.

            //It's possible that the two objects' centers are overlapping, or very very close to it.  In this case, 
            //they are obviously colliding and we can immediately exit.
            if (originRay.LengthSquared() < Toolbox.Epsilon)
            {
                position = new Vector3();
                DEBUGlastPosition = position;
                return true;
            }

            Vector3 v0;
            Vector3.Negate(ref originRay, out v0); //Since we're in A's local space, A-B is just -B.



            //Now that the origin ray is known, create a portal through which the ray passes.
            //To do this, first guess a portal.
            //This implementation is similar to that of the original XenoCollide.
            //'n' will be the direction used to find supports throughout the algorithm.
            Vector3 n = originRay;
            Vector3 v1;
            Vector3 v1A, v1B; //extreme point contributions from each shape.  Used later to compute contact position; could be used to cache simplex too.
            MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v1A, out v1B, out v1);

            //Find another extreme point in a direction perpendicular to the previous.
            Vector3 v2;
            Vector3 v2A, v2B;
            Vector3.Cross(ref v1, ref v0, out n);
            if (n.LengthSquared() < Toolbox.Epsilon)
            {
                //v1 and v0 could be parallel.
                Vector3.Cross(ref v1, ref Toolbox.UpVector, out n);
                if (n.LengthSquared() < Toolbox.Epsilon)
                    Vector3.Cross(ref v1, ref Toolbox.RightVector, out n);
            }
            MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v2A, out v2B, out v2);

            Vector3 temp1, temp2;
            //Set n for the first iteration.
            Vector3.Subtract(ref v1, ref v0, out temp1);
            Vector3.Subtract(ref v2, ref v0, out temp2);
            Vector3.Cross(ref temp1, ref temp2, out n);


            Vector3 v3A, v3B, v3;
            int count = 0;
            while (true)
            {
                //Find a final extreme point using the normal of the plane defined by v0, v1, v2.
                MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v3A, out v3B, out v3);

                if (count > MPRToolbox.OuterIterationLimit)
                    break;
                count++;
                //By now, the simplex is a tetrahedron, but it is not known whether or not the origin ray found earlier actually passes through the portal
                //defined by v1, v2, v3.

                // If the origin is outside the plane defined by v1,v0,v3, then the portal is invalid.
                Vector3.Cross(ref v1, ref v3, out temp1);
                float dot;
                Vector3.Dot(ref temp1, ref v0, out dot);
                if (dot < 0)
                {
                    //Replace the point that was on the inside of the plane (v2) with the new extreme point.
                    v2 = v3;
                    v2A = v3A;
                    v2B = v3B;
                    // Calculate the normal of the plane that will be used to find a new extreme point.
                    Vector3.Subtract(ref v1, ref v0, out temp1);
                    Vector3.Subtract(ref v3, ref v0, out temp2);
                    Vector3.Cross(ref temp1, ref temp2, out n);
                    continue;
                }

                // If the origin is outside the plane defined by v3,v0,v2, then the portal is invalid.
                Vector3.Cross(ref v3, ref v2, out temp1);
                Vector3.Dot(ref temp1, ref v0, out dot);
                if (dot < 0)
                {
                    //Replace the point that was on the inside of the plane (v1) with the new extreme point.
                    v1 = v3;
                    v1A = v3A;
                    v1B = v3B;
                    // Calculate the normal of the plane that will be used to find a new extreme point.
                    Vector3.Subtract(ref v2, ref v0, out temp1);
                    Vector3.Subtract(ref v3, ref v0, out temp2);
                    Vector3.Cross(ref temp1, ref temp2, out n);
                    continue;
                }
                break;
            }

            if (!VerifySimplex(ref v0, ref v1, ref v2, ref v3, ref localTransformB.Position))
                Debug.WriteLine("Break.");


            // Refine the portal.
            while (true)
            {
                //Test the origin against the plane defined by v1, v2, v3.  If it's inside, we're done.
                //Compute the outward facing normal.
                Vector3.Subtract(ref v3, ref v2, out temp1);
                Vector3.Subtract(ref v1, ref v2, out temp2);
                Vector3.Cross(ref temp1, ref temp2, out n);
                float dot;
                Vector3.Dot(ref n, ref v1, out dot);
                if (dot > 0)
                {
                    Vector3 temp3;
                    //Compute the barycentric coordinates of the origin.
                    //This is done by computing the scaled volume (parallelepiped) of the tetrahedra 
                    //formed by each triangle of the v0v1v2v3 tetrahedron and the origin.
                    Vector3.Subtract(ref v1, ref v0, out temp1);
                    Vector3.Subtract(ref v2, ref v0, out temp2);
                    Vector3.Subtract(ref v3, ref v0, out temp3);

                    Vector3 cross;
                    Vector3.Cross(ref temp1, ref temp2, out cross);
                    float v0v1v2v3volume;
                    Vector3.Dot(ref cross, ref temp3, out v0v1v2v3volume);

                    Vector3.Cross(ref v1, ref v2, out cross);
                    float ov1v2v3volume;
                    Vector3.Dot(ref cross, ref v3, out ov1v2v3volume);

                    Vector3.Cross(ref originRay, ref temp2, out cross);
                    float v0ov2v3volume;
                    Vector3.Dot(ref cross, ref temp3, out v0ov2v3volume);

                    Vector3.Cross(ref temp1, ref originRay, out cross);
                    float v0v1ov3volume;
                    Vector3.Dot(ref cross, ref temp3, out v0v1ov3volume);


                    float inverseTotalVolume = 1 / v0v1v2v3volume;
                    float v0Weight = ov1v2v3volume * inverseTotalVolume;
                    float v1Weight = v0ov2v3volume * inverseTotalVolume;
                    float v2Weight = v0v1ov3volume * inverseTotalVolume;
                    float v3Weight = 1 - v0Weight - v1Weight - v2Weight;
                    position = v1Weight * v1A + v2Weight * v2A + v3Weight * v3A;
                    DEBUGlastPosition = position;
                    return true;
                }

                //We haven't yet found the origin.  Find the support point in the portal's outward facing direction.
                Vector3 v4, v4A, v4B;
                MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v4A, out v4B, out v4);
                //If the origin is further along the direction than the extreme point, it's not inside the shape.
                float dot2;
                Vector3.Dot(ref v4, ref n, out dot2);
                if (dot2 < 0)
                {
                    //The origin is outside!
                    position = new Vector3();
                    return false;
                }

                //If the plane which generated the normal is very close to the extreme point, then we're at the surface
                //and we have not found the origin; it's either just BARELY inside, or it is outside.  Assume it's outside.
                if (dot2 - dot < Toolbox.BigEpsilon || count > MPRToolbox.InnerIterationLimit) // TODO: Could use a dynamic epsilon for possibly better behavior.
                {
                    position = new Vector3();
                    DEBUGlastPosition = position;
                    return false;
                }
                count++;

                //Still haven't exited, so refine the portal.
                //Test origin against the three planes that separate the new portal candidates: (v1,v4,v0) (v2,v4,v0) (v3,v4,v0)
                Vector3.Cross(ref v4, ref v0, out temp1);
                Vector3.Dot(ref v1, ref temp1, out dot);
                if (dot >= 0)
                {
                    Vector3.Dot(ref v2, ref temp1, out dot);
                    if (dot >= 0)
                    {
                        v1 = v4; // Inside v1 & inside v2 ==> eliminate v1
                        v1A = v4A;
                        v1B = v4B;
                    }
                    else
                    {
                        v3 = v4; // Inside v1 & outside v2 ==> eliminate v3
                        v3A = v4A;
                        v3B = v4B;
                    }
                }
                else
                {
                    Vector3.Dot(ref v3, ref temp1, out dot);
                    if (dot >= 0)
                    {
                        v2 = v4; // Outside v1 & inside v3 ==> eliminate v2
                        v2A = v4A;
                        v2B = v4B;
                    }
                    else
                    {
                        v1 = v4; // Outside v1 & outside v3 ==> eliminate v1
                        v1A = v4A;
                        v1B = v4B;
                    }
                }

                if (!VerifySimplex(ref v0, ref v1, ref v2, ref v3, ref localTransformB.Position))
                    Debug.WriteLine("Break.");

            }


        }

        public static void LocalSurfaceCast(ConvexShape shapeA, ConvexShape shapeB, ref RigidTransform localTransformB, ref Vector3 direction, out float t, out Vector3 normal)
        {
            // Local surface cast is very similar to regular MPR.  However, instead of starting at an interior point and targeting the origin,
            // the ray starts at the origin (a point known to be in both shapeA and shapeB), and just goes towards the direction until the surface
            // is found.  The portal (v1, v2, v3) at termination defines the surface normal, and the distance from the origin to the portal along the direction is used as the 't' result.


            //'v0' is no longer explicitly tracked since it is simply the origin.


            //Now that the origin ray is known, create a portal through which the ray passes.
            //To do this, first guess a portal.
            //This implementation is similar to that of the original XenoCollide.
            //'n' will be the direction used to find supports throughout the algorithm.
            Vector3 n = direction;
            Vector3 v1;
            MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v1);
            //v1 could be zero in some degenerate cases.
            if (v1.LengthSquared() < Toolbox.Epsilon)
            {
                t = 0;
                normal = n;
                return;
            }

            //Find another extreme point in a direction perpendicular to the previous.
            Vector3 v2;
            Vector3.Cross(ref direction, ref v1, out n);
            if (n.LengthSquared() < Toolbox.Epsilon)
            {
                //direction and v1 could be parallel.
                Vector3.Cross(ref v1, ref Toolbox.UpVector, out n);
                if (n.LengthSquared() < Toolbox.Epsilon)
                    Vector3.Cross(ref v1, ref Toolbox.RightVector, out n);
            }
            MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v2);




            Vector3 temp1, temp2;
            //Set n for the first iteration.
            Vector3.Cross(ref v1, ref v2, out n);

            //It's possible that v1 and v2 were constructed in such a way that 'n' is not properly calibrated
            //relative to the direction vector.
            float dot;
            Vector3.Dot(ref n, ref direction, out dot);
            if (dot > 0)
            {
                //It's not properly calibrated.  Flip the winding (and the previously calculated normal).
                Vector3.Negate(ref n, out n);
                temp1 = v1;
                v1 = v2;
                v2 = temp1;
            }

            Vector3 v3;
            int count = 0;
            while (true)
            {
                //Find a final extreme point using the normal of the plane defined by v0, v1, v2.
                MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v3);

                if (count > MPRToolbox.OuterIterationLimit)
                {
                    //Can't enclose the origin! That's a bit odd; something is wrong.
                    t = float.MaxValue;
                    normal = Toolbox.UpVector;
                    return;
                }
                count++;

                //By now, the simplex is a tetrahedron, but it is not known whether or not the ray actually passes through the portal
                //defined by v1, v2, v3.

                // If the direction is outside the plane defined by v1,v0,v3, then the portal is invalid.
                Vector3.Cross(ref v1, ref v3, out temp1);
                Vector3.Dot(ref temp1, ref direction, out dot);
                if (dot < 0)
                {
                    //Replace the point that was on the inside of the plane (v2) with the new extreme point.
                    v2 = v3;
                    // Calculate the normal of the plane that will be used to find a new extreme point.
                    Vector3.Cross(ref v1, ref v3, out n);
                    continue;
                }

                // If the direction is outside the plane defined by v3,v0,v2, then the portal is invalid.
                Vector3.Cross(ref v3, ref v2, out temp1);
                Vector3.Dot(ref temp1, ref direction, out dot);
                if (dot < 0)
                {
                    //Replace the point that was on the inside of the plane (v1) with the new extreme point.
                    v1 = v3;
                    // Calculate the normal of the plane that will be used to find a new extreme point.
                    Vector3.Cross(ref v2, ref v3, out n);
                    continue;
                }
                break;
            }

            if (!VerifySimplex(ref Toolbox.ZeroVector, ref v1, ref v2, ref v3, ref direction))
                Debug.WriteLine("Break.");


            // Refine the portal.
            count = 0;
            while (true)
            {
                //Compute the outward facing normal.
                Vector3.Subtract(ref v1, ref v2, out temp1);
                Vector3.Subtract(ref v3, ref v2, out temp2);
                Vector3.Cross(ref temp1, ref temp2, out n);


                //Keep working towards the surface.  Find the next extreme point.
                Vector3 v4;
                MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v4);


                //If the plane which generated the normal is very close to the extreme point, then we're at the surface.
                Vector3.Dot(ref n, ref v1, out dot);
                float supportDot;
                Vector3.Dot(ref v4, ref n, out supportDot);

                if (supportDot - dot < Toolbox.BigEpsilon || count > MPRToolbox.InnerIterationLimit) // TODO: Could use a dynamic epsilon for possibly better behavior.
                {
                    //normal = n;
                    //float normalLengthInverse = 1 / normal.Length();
                    //Vector3.Multiply(ref normal, normalLengthInverse, out normal);
                    ////Find the distance from the origin to the plane.
                    //t = dot * normalLengthInverse;

                    Vector3.Normalize(ref n, out normal);
                    //The plane is very close to the surface, and the ray is known to pass through it.
                    //dot is the rate.
                    Vector3.Dot(ref normal, ref direction, out dot);
                    //supportDot is the distance to the plane.
                    Vector3.Dot(ref normal, ref v1, out supportDot);
                    t = supportDot / dot;

                    //DEBUG STUFF:

                    DEBUGlastRayT = t;
                    DEBUGlastRayDirection = direction;
                    DEBUGlastDepth = t;
                    DEBUGlastNormal = normal;
                    DEBUGlastV1 = v1;
                    DEBUGlastV2 = v2;
                    DEBUGlastV3 = v3;
                    return;
                }

                //Still haven't exited, so refine the portal.
                //Test direction against the three planes that separate the new portal candidates: (v1,v4,v0) (v2,v4,v0) (v3,v4,v0)



                //This may look a little weird at first.
                //'inside' here means 'on the positive side of the plane.'
                //There are three total planes being tested, one for each of v1, v2, and v3.
                //The planes are created from consistently wound vertices, so it's possible to determine
                //where the ray passes through the portal based upon its relationship to two of the three planes.
                //The third vertex which is found to be opposite the face which contains the ray is replaced with the extreme point.

                //This v4 x direction is just a minor reordering of a scalar triple product: (v1 x v4) * direction.
                //It eliminates the need for extra cross products for the inner if.
                Vector3.Cross(ref v4, ref direction, out temp1);
                Vector3.Dot(ref v1, ref temp1, out dot);
                if (dot >= 0)
                {
                    Vector3.Dot(ref v2, ref temp1, out dot);
                    if (dot >= 0)
                    {
                        v1 = v4; // Inside v1 & inside v2 ==> eliminate v1
                    }
                    else
                    {
                        v3 = v4; // Inside v1 & outside v2 ==> eliminate v3
                    }
                }
                else
                {
                    Vector3.Dot(ref v3, ref temp1, out dot);
                    if (dot >= 0)
                    {
                        v2 = v4; // Outside v1 & inside v3 ==> eliminate v2
                    }
                    else
                    {
                        v1 = v4; // Outside v1 & outside v3 ==> eliminate v1
                    }
                }

                count++;

                //Here's an unoptimized equivalent without the scalar triple product reorder.
                #region Equivalent refinement
                //Vector3.Cross(ref v1, ref v4, out temp1);
                //Vector3.Dot(ref temp1, ref direction, out dot);
                //if (dot > 0)
                //{
                //    Vector3.Cross(ref v2, ref v4, out temp2);
                //    Vector3.Dot(ref temp2, ref direction, out dot);
                //    if (dot > 0)
                //    {
                //        //Inside v1, v4, v0 and inside v2, v4, v0
                //        v1 = v4;
                //    }
                //    else
                //    {
                //        //Inside v1, v4, v0 and outside v2, v4, v0
                //        v3 = v4;
                //    }
                //}
                //else
                //{
                //    Vector3.Cross(ref v3, ref v4, out temp2);
                //    Vector3.Dot(ref temp2, ref direction, out dot);
                //    if (dot > 0)
                //    {
                //        //Outside v1, v4, v0 and inside v3, v4, v0
                //        v2 = v4;
                //    }
                //    else
                //    {
                //        //Outside v1, v4, v0 and outside v3, v4, v0
                //        v1 = v4;
                //    }
                //}
                #endregion

                if (!VerifySimplex(ref Toolbox.ZeroVector, ref v1, ref v2, ref v3, ref direction))
                    Debug.WriteLine("Break.");
            }
        }

        static bool VerifySimplex(ref Vector3 v0, ref Vector3 v1, ref Vector3 v2, ref Vector3 v3, ref Vector3 direction)
        {


            //v1, v0, v3
            Vector3 cross = Vector3.Cross(v0 - v1, v3 - v1);
            float planeProduct1 = Vector3.Dot(cross, direction);
            //v3, v0, v2
            cross = Vector3.Cross(v0 - v3, v2 - v3);
            float planeProduct2 = Vector3.Dot(cross, direction);
            //v2, v0, v1
            cross = Vector3.Cross(v0 - v2, v1 - v2);
            float planeProduct3 = Vector3.Dot(cross, direction);
            return (planeProduct1 <= 0 && planeProduct2 <= 0 && planeProduct3 <= 0) ||
                (planeProduct1 >= 0 && planeProduct2 >= 0 && planeProduct3 >= 0);
        }
        
        public static bool GetContact(ConvexShape shapeA, ConvexShape shapeB, ref RigidTransform transformA, ref RigidTransform transformB, ref Vector3 penetrationAxis, out ContactData contact)
        {
            Vector3 originRay;
            Vector3.Subtract(ref transformB.Position, ref transformA.Position, out originRay);

            return GetContact(shapeA, shapeB, ref transformA, ref transformB, ref originRay, ref penetrationAxis, out contact);
        }
        public static bool GetContact(ConvexShape shapeA, ConvexShape shapeB, ref RigidTransform transformA, ref RigidTransform transformB, ref Vector3 originRay, ref Vector3 penetrationAxis, out ContactData contact)
        {
            RigidTransform localTransformB;
            MinkowskiToolbox.GetLocalTransform(ref transformA, ref transformB, out localTransformB);
            if (MPRTesting.GetLocalOverlapPosition(shapeA, shapeB, ref originRay, ref localTransformB, out contact.Position))
            {
                //First, try to use the heuristically found direction.  This comes from either the GJK shallow contact separating axis or from the relative velocity.
                Vector3 rayCastDirection;
                float lengthSquared = penetrationAxis.LengthSquared();
                if (lengthSquared > Toolbox.Epsilon)
                {
                    Vector3.Divide(ref penetrationAxis, (float)Math.Sqrt(lengthSquared), out rayCastDirection);// (Vector3.Normalize(localDirection) + Vector3.Normalize(collidableB.worldTransform.Position - collidableA.worldTransform.Position)) / 2;
                    MPRTesting.LocalSurfaceCast(shapeA, shapeB, ref localTransformB, ref rayCastDirection, out contact.PenetrationDepth, out contact.Normal);
                }
                else
                {
                    contact.PenetrationDepth = float.MaxValue;
                    contact.Normal = Toolbox.UpVector;
                }
                //Try the offset between the origins as a second option.  Sometimes this is a better choice than the relative velocity.
                //TODO: Could use the position-finding MPR iteration to find the A-B direction hit by continuing even after the origin has been found (optimization).
                Vector3 normalCandidate;
                float depthCandidate;
                lengthSquared = originRay.LengthSquared();
                if (lengthSquared > Toolbox.Epsilon)
                {
                    Vector3.Divide(ref originRay, (float)Math.Sqrt(lengthSquared), out rayCastDirection);
                    MPRTesting.LocalSurfaceCast(shapeA, shapeB, ref localTransformB, ref rayCastDirection, out depthCandidate, out normalCandidate);
                    if (depthCandidate < contact.PenetrationDepth)
                    {
                        contact.Normal = normalCandidate;
                    }
                }


                //Correct the penetration depth.
                MPRTesting.LocalSurfaceCast(shapeA, shapeB, ref localTransformB, ref contact.Normal, out contact.PenetrationDepth, out rayCastDirection);


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
                //we're still in local space! transform it all back.
                Matrix3X3 orientation;
                Matrix3X3.CreateFromQuaternion(ref transformA.Orientation, out orientation);
                Matrix3X3.Transform(ref contact.Normal, ref orientation, out contact.Normal);
                //Vector3.Negate(ref contact.Normal, out contact.Normal);
                Matrix3X3.Transform(ref contact.Position, ref orientation, out contact.Position);
                Vector3.Add(ref contact.Position, ref transformA.Position, out contact.Position);
                return true;
            }
            contact = new ContactData();
            return false;
        }
    }
}


#region OldCode
//while (true)
//{
//    //In order for the origin ray to pass through the portal, the origin must be on the same side of all three
//    //of the portal's bounding planes.

//    //The calculations are simplified thanks to the fact that all comparisons are against the zero vector (the origin)
//    //and the windings that are consistently maintained.

//    //Try v0, v1, v2 first.
//    Vector3.Subtract(ref v1, ref v0, out temp1);
//    Vector3.Subtract(ref v2, ref v0, out temp2);
//    Vector3.Cross(ref temp1, ref temp2, out n); //'n' points away from the face v0, v1, v2
//    float dot;
//    Vector3.Dot(ref n, ref v0, out dot); // v0 is on the plane, so compare the origin against its distance.
//    if (dot > 0)
//    {
//        //Replace v3 with the extreme point in the direction of the plane in an attempt to contain the origin.
//        MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v3A, out v3B, out v3);
//        //Flip v1 and v2 to maintain winding.
//        temp1 = v2;
//        v2 = v1;
//        v1 = temp1;
//        temp1 = v2A;
//        v2A = v1A;
//        v1A = temp1;
//        temp1 = v2B;
//        v2B = v1B;
//        v1B = temp1;
//        continue;
//    }

//    //Try v0, v2, v3 second.
//    Vector3.Subtract(ref v2, ref v0, out temp1);
//    Vector3.Subtract(ref v3, ref v0, out temp2);
//    Vector3.Cross(ref temp1, ref temp2, out n); //'n' points away from the face v0, v2, v3
//    Vector3.Dot(ref n, ref v0, out dot); // v0 is on the plane, so compare the origin against its distance.
//    if (dot > 0)
//    {
//        //Replace v1 with the extreme point in the direction of the plane in an attempt to contain the origin.
//        MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v1A, out v1B, out v1);
//        //Flip v2 and v3 to maintain winding.
//        temp1 = v2;
//        v2 = v3;
//        v3 = temp1;
//        temp1 = v2A;
//        v2A = v3A;
//        v3A = temp1;
//        temp1 = v2B;
//        v2B = v3B;
//        v3B = temp1;
//        continue;
//    }

//    //Try v0, v3, v1 third.
//    Vector3.Subtract(ref v3, ref v0, out temp1);
//    Vector3.Subtract(ref v1, ref v0, out temp2);
//    Vector3.Cross(ref temp1, ref temp2, out n); //'n' points away from the face v0, v2, v3
//    Vector3.Dot(ref n, ref v0, out dot); // v0 is on the plane, so compare the origin against its distance.
//    if (dot > 0)
//    {
//        //Replace v2 with the extreme point in the direction of the plane in an attempt to contain the origin.
//        MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v2A, out v2B, out v2);
//        //Flip v3 and v1 to maintain winding.
//        temp1 = v3;
//        v3 = v1;
//        v1 = temp1;
//        temp1 = v3A;
//        v3A = v1A;
//        v1A = temp1;
//        temp1 = v3B;
//        v3B = v1B;
//        v1B = temp1;
//        continue;
//    }

//    //If we got here, that means the origin is contained! We can stop.
//    break;

//}

//TODO: Don't need to check all three planes every single time.
// Only two planes change each time.  The third is the same.
// Replace v3 with the new support and flip other vertices around to ensure winding.
// Can bundle initial v3 calculation into the verify loop.
#endregion