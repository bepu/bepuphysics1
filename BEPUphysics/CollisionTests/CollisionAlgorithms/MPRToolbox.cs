using System;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;

namespace BEPUphysics.CollisionTests.CollisionAlgorithms
{
    /// <summary>
    /// Contains MPR-based collision queries.
    /// </summary>
    public static class MPRToolbox
    {
        /// <summary>
        /// Number of iterations that the MPR system will run in its inner loop before giving up and returning with failure.
        /// </summary>
        public static int InnerIterationLimit = 15;
        /// <summary>
        /// Number of iterations that the MPR system will run in its outer loop before giving up and moving on to its inner loop.
        /// </summary>
        public static int OuterIterationLimit = 15;

        //TODO: Lots of nasty code repeat.

        /// <summary>
        /// Determines whether or not the given entities are colliding.
        /// </summary>
        /// <param name="shapeA">First shape of the pair.</param>
        /// <param name="shapeB">Second shape of the pair.</param>
        /// <param name="transformA">Transform to apply to shapeA for the test.</param>
        /// <param name="transformB">Transform to apply to shapeB for the test.</param>
        /// <returns>Whether or not the margin-expanded shapes are colliding.</returns>
        public static bool AreObjectsColliding(ConvexShape shapeA, ConvexShape shapeB, ref RigidTransform transformA, ref RigidTransform transformB)
        {
            RigidTransform localTransformB;
            MinkowskiToolbox.GetLocalTransform(ref transformA, ref transformB, out localTransformB);

            // v0 = center of Minkowski difference
            Vector3 v0;
            MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref Toolbox.ZeroVector, ref localTransformB, out v0);
            if (v0.LengthSquared() < Toolbox.Epsilon)
                return true; // v0 and origin overlap ==> hit

            // v1 = support in direction of origin
            Vector3 n = localTransformB.Position;
            Vector3 v1;
            MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v1);
            float dot;
            Vector3.Dot(ref v1, ref n, out dot);
            if (dot <= 0)
                return false; // origin outside v1 support plane ==> miss

            // v2 = support perpendicular to plane containing origin, v0 and v1
            Vector3.Cross(ref v1, ref v0, out n);
            if (n == Toolbox.ZeroVector) //(n.LengthSquared() < Toolbox.bigEpsilon)
                return true; // v0, v1 and origin colinear (and origin inside v1 support plane) == > hit
            Vector3 v2;
            MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v2);
            Vector3.Dot(ref v2, ref n, out dot);
            if (dot <= 0)
                return false; // origin outside v2 support plane ==> miss

            // v3 = support perpendicular to plane containing v0, v1 and v2
            Vector3 v0v1, v0v2;
            Vector3.Subtract(ref v1, ref v0, out v0v1);
            Vector3.Subtract(ref v2, ref v0, out v0v2);
            Vector3.Cross(ref v0v1, ref v0v2, out n);

            // If the origin is on the - side of the plane, reverse the direction of the plane
            Vector3.Dot(ref n, ref v0, out dot);
            if (dot > 0)
            {
                Vector3 temp = v1;
                v1 = v2;
                v2 = temp;
                Vector3.Negate(ref n, out n);
            }

            // Phase One: Find a valid portal
            int count = 0;
            while (true)
            {
                count++;
                // Obtain the next support point
                Vector3 v3;
                MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v3);
                Vector3.Dot(ref v3, ref n, out dot);
                if (dot <= 0)
                    return false; // origin outside v3 support plane ==> miss

                // If origin is outside (v1,v0,v3), then portal is invalid -- eliminate v2 and find new support outside face
                Vector3 cross;
                Vector3.Cross(ref v1, ref v3, out cross);
                Vector3.Dot(ref cross, ref v0, out dot);
                if (count < OuterIterationLimit && dot < 0)
                {
                    v2 = v3;
                    Vector3 v0v3;
                    Vector3.Subtract(ref v1, ref v0, out v0v1);
                    Vector3.Subtract(ref v3, ref v0, out v0v3);
                    Vector3.Cross(ref v0v1, ref v0v3, out n);
                    continue;
                }

                // If origin is outside (v3,v0,v2), then portal is invalid -- eliminate v1 and find new support outside face
                dot = Vector3.Dot(Vector3.Cross(v3, v2), v0);
                if (count < OuterIterationLimit && dot < 0)
                {
                    v1 = v3;
                    Vector3 v0v3;
                    Vector3.Subtract(ref v3, ref v0, out v0v3);
                    Vector3.Subtract(ref v2, ref v0, out v0v2);
                    Vector3.Cross(ref v0v3, ref v0v2, out n);
                    continue;
                }

                count = 0;
                // Phase Two: Refine the portal
                while (true)
                {
                    // Compute outward facing normal of the portal
                    Vector3 v1v2, v1v3;
                    Vector3.Subtract(ref v2, ref v1, out v1v2);
                    Vector3.Subtract(ref v3, ref v1, out v1v3);
                    Vector3.Cross(ref v1v2, ref v1v3, out n);

                    // If the origin is inside the portal, we have a hit
                    Vector3.Dot(ref n, ref v1, out dot);
                    if (dot >= 0)
                        return true;

                    // Find the support point in the direction of the portal's normal
                    Vector3 v4;
                    MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v4);

                    // If the origin is outside the support plane or the boundary is thin enough, we have a miss
                    float dot2;
                    Vector3.Dot(ref v4, ref n, out dot);
                    Vector3 v3v4;
                    Vector3.Subtract(ref v4, ref v3, out v3v4);
                    Vector3.Dot(ref v3v4, ref n, out dot2);
                    if (dot <= 0 || dot2 <= Toolbox.BigEpsilon ||
                        count > InnerIterationLimit) //Numerical problems
                        return false;

                    // Test origin against the three planes that separate the new portal candidates: (v1,v4,v0) (v2,v4,v0) (v3,v4,v0)
                    Vector3.Cross(ref v4, ref v0, out cross);
                    Vector3.Dot(ref v1, ref cross, out dot);
                    if (dot > 0)
                    {
                        Vector3.Dot(ref v2, ref cross, out dot);
                        if (dot > 0)
                            v1 = v4; // Inside v1 & inside v2 ==> eliminate v1
                        else
                            v3 = v4; // Inside v1 & outside v2 ==> eliminate v3
                    }
                    else
                    {
                        Vector3.Dot(ref v3, ref cross, out dot);
                        if (dot > 0)
                            v2 = v4; // Outside v1 & inside v3 ==> eliminate v2
                        else
                            v1 = v4; // Outside v1 & outside v3 ==> eliminate v1
                    }
                    count++;
                }
            }
        }


        /// <summary>
        /// Determines whether or not the given entities are colliding, and if so, returns collision data about them.
        /// </summary>
        /// <param name="shapeA">First shape to check.</param>
        /// <param name="shapeB">Second shape to check.</param>
        /// <param name="transformA">Transform to apply to shapeA for the test.</param>
        /// <param name="transformB">Transform to apply to shapeB for the test.</param>
        /// <param name="contact">Contact data generated by the test, if any.</param>
        /// <returns>Whether or not the shapes are colliding.</returns>
        public static bool AreObjectsColliding(ConvexShape shapeA, ConvexShape shapeB, ref RigidTransform transformA, ref RigidTransform transformB, out ContactData contact)
        {

            RigidTransform localTransformB;
            MinkowskiToolbox.GetLocalTransform(ref transformA, ref transformB, out localTransformB);

            contact.Position = Toolbox.NoVector;
            contact.Normal = Toolbox.NoVector;
            contact.PenetrationDepth = -float.MaxValue;
            contact.Id = -1;
            bool isColliding = false;


            // v0 = Some point within the minkowski sum.  Using the zero vector
            Vector3 v0A, v0B, v0;
            MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref Toolbox.ZeroVector, ref localTransformB, out v0A, out v0B, out v0);

            //The normal faces back towards the origin.
            Vector3 n;
            Vector3.Negate(ref v0, out n);
            if (n.LengthSquared() < Toolbox.Epsilon)
            {
                isColliding = true; // v0 and origin overlap ==> hit
                n = new Vector3(0, -Toolbox.BigEpsilon, 0);
            }

            // v1 = support in direction of origin
            Vector3 v1A, v1B, v1;
            MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v1A, out v1B, out v1);

            //Can we early out?
            if (!isColliding && Vector3.Dot(v1, n) <= 0)
                return false; // origin outside v1 support plane ==> miss

            Vector3.Cross(ref v1, ref v0, out n);

            //How about now?
            if (n.LengthSquared() < Toolbox.Epsilon)
            {
                Vector3 sub;
                Vector3.Subtract(ref v1, ref v0, out sub);
                Vector3.Normalize(ref sub, out n);
                contact.Normal = n;
                Vector3 add;
                Vector3.Add(ref v1A, ref v1B, out add);
                Vector3.Multiply(ref add, .5f, out contact.Position);
                Vector3.Dot(ref n, ref v1, out contact.PenetrationDepth); //TODO TODO
                Vector3.Transform(ref contact.Position, ref transformA.Orientation, out contact.Position);
                Vector3.Add(ref contact.Position, ref transformA.Position, out contact.Position);
                Vector3.Transform(ref contact.Normal, ref transformA.Orientation, out contact.Normal);
                return true;
            }

            // v2 = support perpendicular to plane containing origin, v0 and v1
            Vector3 v2A, v2B, v2;
            MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v2A, out v2B, out v2);

            //Can we early out NOW?
            if (!isColliding && Vector3.Dot(v2, n) <= 0)
                return false; // origin outside v2 support plane ==> miss

            // v3 = support perpendicular to plane containing v0, v1 and v2
            Vector3 v0v1, v0v2, v0v3, cross;
            Vector3.Subtract(ref v1, ref v0, out v0v1);
            Vector3.Subtract(ref v2, ref v0, out v0v2);
            Vector3.Cross(ref v0v1, ref v0v2, out n);

            // If the origin is on the - side of the plane, reverse the direction of the plane
            float dot;
            Vector3.Dot(ref n, ref v0, out dot);
            if (dot > 0)
            {
                Vector3 temp = v1;
                v1 = v2;
                v2 = temp;
                temp = v1A;
                v1A = v2A;
                v2A = temp;
                temp = v1B;
                v1B = v2B;
                v2B = temp;
                Vector3.Negate(ref n, out n);
            }
            // Phase One: Find a valid portal
            int count = 0;
            while (true)
            {
                count++;
                // Obtain the next support point
                Vector3 v3A, v3B, v3;
                MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v3A, out v3B, out v3);

                //Can we get out now?
                if (!isColliding && Vector3.Dot(v3, n) <= 0)
                    return false; // origin outside v3 support plane ==> miss

                // If origin is outside (v1,v0,v3), then portal is invalid -- eliminate v2 and find new support outside face
                Vector3.Cross(ref v1, ref v3, out cross);
                Vector3.Dot(ref cross, ref v0, out dot);
                if (dot < 0 && count < OuterIterationLimit)
                {
                    v2 = v3;
                    v2A = v3A;
                    v2B = v3B;
                    Vector3.Subtract(ref v1, ref v0, out v0v1);
                    Vector3.Subtract(ref v3, ref v0, out v0v3);
                    Vector3.Cross(ref v0v1, ref v0v3, out n);
                    continue;
                }

                // If origin is outside (v3,v0,v2), then portal is invalid -- eliminate v1 and find new support outside face
                Vector3.Cross(ref v3, ref v2, out cross);
                Vector3.Dot(ref cross, ref v0, out dot);
                if (dot < 0 && count < OuterIterationLimit)
                {
                    v1 = v3;
                    v1A = v3A;
                    v1B = v3B;
                    Vector3.Subtract(ref v2, ref v0, out v0v2);
                    Vector3.Subtract(ref v3, ref v0, out v0v3);
                    Vector3.Cross(ref v0v2, ref v0v3, out n);
                    continue;
                }

                // Phase Two: Refine the portal
                count = 0;
                while (true)
                {
                    Vector3 v1v2, v1v3;
                    // Compute outward facing normal of the portal
                    Vector3.Subtract(ref v2, ref v1, out v1v2);
                    Vector3.Subtract(ref v3, ref v1, out v1v3);
                    Vector3.Cross(ref v1v2, ref v1v3, out n);

                    // If the origin is inside the portal, we have a hit
                    if (!isColliding)
                    {
                        Vector3.Dot(ref n, ref v1, out dot);
                        isColliding = dot >= 0;
                    }

                    // Find the support point in the direction of the portal's normal

                    Vector3 v4A, v4B, v4;
                    MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v4A, out v4B, out v4);

                    // If the origin is outside the support plane
                    if ((!isColliding && -Vector3.Dot(v4, n) >= 0) ||
                        count > InnerIterationLimit) //Numerical problem
                        return false;

                    Vector3 v3v4;
                    Vector3.Subtract(ref v4, ref v3, out v3v4);
                    Vector3.Dot(ref v3v4, ref n, out dot);
                    if (dot <= MathHelper.Max(1, MathHelper.Max(v1.LengthSquared(), MathHelper.Max(v2.LengthSquared(), v3.LengthSquared()))) * Toolbox.Epsilon)
                    {
                        //We've reached the surface
                        if (isColliding)
                        {
                            Vector3.Normalize(ref n, out contact.Normal);
                            //contact point is the origin formed from barycentric coordinates.
                            //This is done by computing the scaled volume (parallelepiped) of the tetrahedra 
                            //formed by each triangle of the v0v1v2v3 tetrahedron and the origin.
                            Vector3.Subtract(ref v1, ref v0, out v0v1);
                            Vector3.Subtract(ref v2, ref v0, out v0v2);
                            Vector3.Subtract(ref v3, ref v0, out v0v3);

                            //float v0v1v2v3volume = Vector3.Dot(Vector3.Cross(v0v1, v0v2), v0v3);
                            //float ov1v2v3volume = Vector3.Dot(Vector3.Cross(v1, v2), v3);
                            //float v0ov2v3volume = Vector3.Dot(Vector3.Cross(-v0, v0v2), v0v3);
                            //float v0v1ov3volume = Vector3.Dot(Vector3.Cross(v0v1, -v0), v0v3);
                            Vector3.Cross(ref v0v1, ref v0v2, out cross);
                            float v0v1v2v3volume;
                            Vector3.Dot(ref cross, ref v0v3, out v0v1v2v3volume);

                            Vector3.Cross(ref v1, ref v2, out cross);
                            float ov1v2v3volume;
                            Vector3.Dot(ref cross, ref v3, out ov1v2v3volume);

                            Vector3 negV0;
                            Vector3.Negate(ref v0, out negV0);

                            Vector3.Cross(ref negV0, ref v0v2, out cross);
                            float v0ov2v3volume;
                            Vector3.Dot(ref cross, ref v0v3, out v0ov2v3volume);

                            Vector3.Cross(ref v0v1, ref negV0, out cross);
                            float v0v1ov3volume;
                            Vector3.Dot(ref cross, ref v0v3, out v0v1ov3volume);


                            float inverseTotalVolume = 1 / v0v1v2v3volume;
                            //Check signed volumes?
                            float v0Weight = ov1v2v3volume * inverseTotalVolume;
                            float v1Weight = v0ov2v3volume * inverseTotalVolume;
                            float v2Weight = v0v1ov3volume * inverseTotalVolume;
                            float v3Weight = 1 - v0Weight - v1Weight - v2Weight;
                            Vector3 contactLocationA = v0Weight * v0A + v1Weight * v1A + v2Weight * v2A + v3Weight * v3A;
                            Vector3 contactLocationB = v0Weight * v0B + v1Weight * v1B + v2Weight * v2B + v3Weight * v3B;
                            contact.Position = (contactLocationA + contactLocationB) / 2;
                            //Find depth by passing in normal to special normal-warmstarted depth finder.
                            contact.PenetrationDepth = FindPenetrationDepth(shapeA, shapeB, ref v0, ref localTransformB, ref contact.Normal);

                            //Transform the data back into world space.
                            Vector3.Transform(ref contact.Position, ref transformA.Orientation, out contact.Position);
                            Vector3.Add(ref contact.Position, ref transformA.Position, out contact.Position);
                            Vector3.Transform(ref contact.Normal, ref transformA.Orientation, out contact.Normal);
                            return true;
                        }
                        return false;
                    }

                    // Test origin against the three planes that separate the new portal candidates: (v1,v4,v0) (v2,v4,v0) (v3,v4,v0)
                    Vector3.Cross(ref v4, ref v0, out cross);
                    Vector3.Dot(ref v1, ref cross, out dot);
                    if (dot > 0)
                    {
                        Vector3.Dot(ref v2, ref cross, out dot);
                        if (dot > 0)
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
                        Vector3.Dot(ref v3, ref cross, out dot);
                        if (dot > 0)
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

                    count++;
                }
            }
        }

        
        /// <summary>
        /// Finds a estimate of the separation distance between the two bodies' core, non margin-expanded shapes.
        /// Will be less than or equal to the actual distance.
        /// </summary>
        /// <param name="shapeA">First shape to check.</param>
        /// <param name="shapeB">Second shape to check.</param>
        /// <param name="transformA">Transform to apply to shapeA for the test.</param>
        /// <param name="transformB">Transform to apply to shapeB for the test.</param>
        /// <param name="separatingDirection">Direction along which the distance is measured.</param>
        /// <returns>Conservative estimate of the distance between the objects.  0 if colliding.</returns>
        public static float FindConservativeDistanceEstimate(ConvexShape shapeA, ConvexShape shapeB, ref RigidTransform transformA, ref RigidTransform transformB,
                                                             out Vector3 separatingDirection)
        {
            RigidTransform localTransformB;
            MinkowskiToolbox.GetLocalTransform(ref transformA, ref transformB, out localTransformB);
            separatingDirection = Toolbox.NoVector;
            float dist;
            // v0 = some point in minkowski sum
            Vector3 v0;
            MinkowskiToolbox.GetLocalMinkowskiExtremePointWithoutMargin(shapeA, shapeB, ref Toolbox.ZeroVector, ref localTransformB, out v0);
            Vector3.Negate(ref localTransformB.Position, out v0);
            if (v0.LengthSquared() < Toolbox.Epsilon)
            {
                return 0; // v0 and origin overlap ==> hit
            }

            // v1 = support in direction of origin
            Vector3 n;
            Vector3.Negate(ref v0, out n);
            Vector3 v1;
            MinkowskiToolbox.GetLocalMinkowskiExtremePointWithoutMargin(shapeA, shapeB, ref n, ref localTransformB, out v1);
            float dot;
            Vector3.Dot(ref v1, ref n, out dot);
            if (dot <= .0001f)
            {
                float length = n.Length();
                n /= length;
                Vector3.Negate(ref n, out separatingDirection);
                if (dot > 0)
                {
                    return 0;
                }
                dist = -dot / length;
                //if (dist <= 0)
                //{
                //    dist *= -1;
                //    separatingDirection *= -1;
                //}
                return dist;

                //return false;	// origin outside v1 support plane ==> miss
            }
            // v2 = support perpendicular to plane containing origin, v0 and v1
            Vector3.Cross(ref v1, ref v0, out n);
            if (n.LengthSquared() < Toolbox.Epsilon)
            {
                //n = new Vector3(0, .0001f, 0);
                //separatingDirection = Vector3.Normalize(v1);
                return 0; // v0, v1 and origin colinear (and origin inside v1 support plane) == > hit
            }

            Vector3 v2;
            MinkowskiToolbox.GetLocalMinkowskiExtremePointWithoutMargin(shapeA, shapeB, ref n, ref localTransformB, out v2);
            Vector3.Dot(ref v2, ref n, out dot);
            if (dot <= 0)
            {
                n.Normalize();
                separatingDirection = n;
                Vector3.Dot(ref v2, ref n, out dist);
                if (dist < 0)
                {
                    dist *= -1;
                    Vector3.Negate(ref separatingDirection, out separatingDirection);
                }
                return dist;
                //return false;	// origin outside v2 support plane ==> miss
            }

            // v3 = support perpendicular to plane containing v0, v1 and v2
            Vector3 v0v1, v0v2, v0v3;
            Vector3.Subtract(ref v1, ref v0, out v0v1);
            Vector3.Subtract(ref v2, ref v0, out v0v2);
            Vector3.Cross(ref v0v1, ref v0v2, out n);
            //If this normal is inside, assume it's intersecting.
            if (n == Toolbox.ZeroVector)
                return 0;

            // If the origin is on the - side of the plane, reverse the direction of the plane
            Vector3.Dot(ref v0, ref n, out dot);
            if (dot > 0)
            {
                Vector3 temp = v1;
                v1 = v2;
                v2 = temp;
                Vector3.Negate(ref n, out n);
            }

            // Phase One: Find a valid portal
            int count = 0;
            while (true)
            {
                count++;
                //Debug.WriteLine("D");
                // Obtain the next support point
                Vector3 v3;
                MinkowskiToolbox.GetLocalMinkowskiExtremePointWithoutMargin(shapeA, shapeB, ref n, ref localTransformB, out v3);
                //Debug.WriteLine("superup: " + Vector3.Dot(v3, n));
                Vector3.Dot(ref v3, ref n, out dot);
                if (dot <= 0)
                {
                    //return false;	// origin outside v3 support plane ==> miss
                    n.Normalize();
                    separatingDirection = n;
                    Vector3.Dot(ref v3, ref n, out dist);
                    if (dist < 0)
                    {
                        dist *= -1;
                        Vector3.Negate(ref separatingDirection, out separatingDirection);
                    }
                    //Debug.WriteLine("end2");
                    return dist;
                }

                // If origin is outside (v1,v0,v3), then portal is invalid -- eliminate v2 and find new support outside face
                //Debug.WriteLine("up: " + Vector3.Dot(Vector3.Cross(v1, v3), v0));
                Vector3 cross;
                Vector3.Cross(ref v1, ref v3, out cross);
                Vector3.Dot(ref cross, ref v0, out dot);
                if (dot < -Toolbox.Epsilon && count < OuterIterationLimit)
                {
                    v2 = v3;
                    Vector3.Subtract(ref v1, ref v0, out v0v1);
                    Vector3.Subtract(ref v3, ref v0, out v0v3);
                    Vector3.Cross(ref v0v1, ref v0v3, out n);
                    continue;
                }

                //Debug.WriteLine("down: " + Vector3.Dot(Vector3.Cross(v3, v2), v0));
                // If origin is outside (v3,v0,v2), then portal is invalid -- eliminate v1 and find new support outside face
                Vector3.Cross(ref v3, ref v2, out cross);
                Vector3.Dot(ref cross, ref v0, out dot);
                if (dot < -Toolbox.Epsilon && count < OuterIterationLimit)
                {
                    v1 = v3;
                    Vector3.Subtract(ref v2, ref v0, out v0v2);
                    Vector3.Subtract(ref v3, ref v0, out v0v3);
                    Vector3.Cross(ref v0v2, ref v0v3, out n);
                    continue;
                }


                // Phase Two: Refine the portal
                count = 0;
                bool isNotColliding = false;
                while (true)
                {
                    Vector3 v1v2, v1v3, v3v4;
                    // Compute outward facing normal of the portal
                    Vector3.Subtract(ref v2, ref v1, out v1v2);
                    Vector3.Subtract(ref v3, ref v1, out v1v3);
                    Vector3.Cross(ref v1v2, ref v1v3, out n);
                    float nLengthSquared = n.LengthSquared();

                    // If the origin is inside the portal, we have a hit
                    if (!isNotColliding && Vector3.Dot(n, v1) >= 0 ||
                        count > InnerIterationLimit) //Numerical problems
                        return 0;

                    // Find the support point in the direction of the portal's normal
                    Vector3 v4;
                    MinkowskiToolbox.GetLocalMinkowskiExtremePointWithoutMargin(shapeA, shapeB, ref n, ref localTransformB, out v4);

                    Vector3.Dot(ref v4, ref n, out dot);
                    if (!isNotColliding && dot <= 0)
                    {
                        isNotColliding = dot <= 0;
                    }

                    // If the origin is outside the support plane or the boundary is thin enough, we have a miss
                    Vector3.Subtract(ref v4, ref v3, out v3v4);
                    Vector3.Dot(ref v3v4, ref n, out dot);
                    if (dot <= Math.Max(Toolbox.BigEpsilon * nLengthSquared, Toolbox.BigEpsilon))
                    {
                        if (nLengthSquared > 0)
                            n /= (float)Math.Sqrt(nLengthSquared);
                        separatingDirection = n;
                        Vector3.Dot(ref v3, ref n, out dist);
                        if (dist < 0)
                        {
                            dist *= -1;
                            Vector3.Negate(ref separatingDirection, out separatingDirection);
                        }
                        return dist;
                    }

                    // Test origin against the three planes that separate the new portal candidates: (v1,v4,v0) (v2,v4,v0) (v3,v4,v0)
                    Vector3.Cross(ref v4, ref v0, out cross);
                    Vector3.Dot(ref v1, ref cross, out dot);
                    if (dot > 0)
                    {
                        Vector3.Dot(ref v2, ref cross, out dot);
                        if (dot > 0)
                            v1 = v4; // Inside v1 & inside v2 ==> eliminate v1
                        else
                            v3 = v4; // Inside v1 & outside v2 ==> eliminate v3
                    }
                    else
                    {
                        Vector3.Dot(ref v3, ref cross, out dot);
                        if (dot > 0)
                            v2 = v4; // Outside v1 & inside v3 ==> eliminate v2
                        else
                            v1 = v4; // Outside v1 & outside v3 ==> eliminate v1
                    }
                    count++;
                }
            }
        }


        /// <summary>
        /// Determines the length of the interpenetrating area along the given normal between the given objects.
        /// Assumes the two entities are actually colliding.
        /// </summary>
        /// <param name="shapeA">First shape to check.</param>
        /// <param name="shapeB">Second shape to check.</param>
        /// <param name="v0">Normal to warmstart the penetration depth calculation.</param>
        /// <param name="localTransformB">Transform of shapeB in the local space of A.</param>
        /// <param name="normal">Direction along which to find the penetration depth.</param>
        /// <returns>Length of the interpenetrating area along the given normal.</returns>
        public static float FindPenetrationDepth(ConvexShape shapeA, ConvexShape shapeB, ref Vector3 v0, ref RigidTransform localTransformB, ref Vector3 normal)
        {
            // v0 = point in the minkowski difference.  Taken from the caller.

            // v1 = support in direction of origin
            Vector3 n = normal;
            Vector3 v1;
            MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v1);
            float dot;
            Vector3.Dot(ref v1, ref n, out dot);
            if (dot <= 0)
                return -float.MaxValue; // origin outside v1 support plane ==> miss

            // v2 = support perpendicular to plane containing origin, v0 and v1
            Vector3.Cross(ref v1, ref v0, out n);
            if (n == Toolbox.ZeroVector)
            {
                v0.X -= .001f;
                v0.Y -= .001f;
                v0.Z -= .001f;
                v1.X += .001f;
                v1.Y += .001f;
                v1.Z += .001f;
                Vector3.Cross(ref v1, ref v0, out n);
                if (n == Toolbox.ZeroVector)
                {
                    v0.X += .001f;
                    v0.Y += .002f;
                    v0.Z += .001f;
                    v1.X -= .001f;
                    v1.Y -= .002f;
                    v1.Z -= .001f;
                    v1 += new Vector3(-.001f, -.002f, -.001f);
                    Vector3.Cross(ref v1, ref v0, out n);
                }
            }
            Vector3 v2;
            MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v2);
            Vector3.Dot(ref v2, ref n, out dot);
            if (dot <= 0)
                return -float.MaxValue; // origin outside v2 support plane ==> miss

            // v3 = support perpendicular to plane containing v0, v1 and v2
            Vector3 v0v1, v0v2, v0v3;
            Vector3.Subtract(ref v1, ref v0, out v0v1);
            Vector3.Subtract(ref v2, ref v0, out v0v2);
            Vector3.Cross(ref v0v1, ref v0v2, out n);

            // If the origin is on the - side of the plane, reverse the direction of the plane
            Vector3.Dot(ref n, ref v0, out dot);
            if (dot > 0)
            {
                Vector3 temp = v1;
                v1 = v2;
                v2 = temp;
                Vector3.Negate(ref n, out n);
            }

            // Phase One: Find a valid portal
            int count = 0;
            while (true)
            {
                count++;
                // Obtain the next support point
                Vector3 v3;
                MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v3);
                Vector3.Dot(ref v3, ref n, out dot);
                if (dot <= 0)
                    return -float.MaxValue; // origin outside v3 support plane ==> miss

                // If origin is outside (v1,v0,v3), then portal is invalid -- eliminate v2 and find new support outside face
                Vector3 cross;
                Vector3.Cross(ref v1, ref v3, out cross);
                Vector3.Dot(ref cross, ref v0, out dot);
                if (dot < 0 && count < OuterIterationLimit)
                {
                    v2 = v3;
                    Vector3.Subtract(ref v1, ref v0, out v0v1);
                    Vector3.Subtract(ref v3, ref v0, out v0v3);
                    Vector3.Cross(ref v0v1, ref v0v3, out n);
                    continue;
                }

                // If origin is outside (v3,v0,v2), then portal is invalid -- eliminate v1 and find new support outside face
                Vector3.Cross(ref v3, ref v2, out cross);
                Vector3.Dot(ref cross, ref v0, out dot);
                if (dot < 0 && count < OuterIterationLimit)
                {
                    v1 = v3;
                    Vector3.Subtract(ref v3, ref v0, out v0v3);
                    Vector3.Subtract(ref v2, ref v0, out v0v2);
                    Vector3.Cross(ref v0v3, ref v0v2, out n);
                    continue;
                }

                // Phase Two: Refine the portal
                count = 0;
                //The tolerance is FIXED because the quantity being compared is fixed (due to the normalization above).
                float tolerance = Toolbox.Epsilon;// MathHelper.Max(1, MathHelper.Max(v1.LengthSquared(), MathHelper.Max(v2.LengthSquared(), v3.LengthSquared()))) * Toolbox.Epsilon;
                while (true)
                {
                    Vector3 v1v2, v1v3;
                    // Compute outward facing normal of the portal
                    Vector3.Subtract(ref v2, ref v1, out v1v2);
                    Vector3.Subtract(ref v3, ref v1, out v1v3);
                    Vector3.Cross(ref v1v2, ref v1v3, out n);
                    n.Normalize();
                    Vector3.Dot(ref n, ref normal, out dot);
                    dot = Math.Abs(dot);
                    
                    if (Math.Abs(dot - 1) < tolerance || count > InnerIterationLimit)// dot > .999f && dot < 1.001f)
                    {
                        //The normals are now similar enough to be considered 'converged.'
                        Vector3.Dot(ref v3, ref n, out dot);
                        if (dot < 0)
                            return -dot;
                        return dot;
                        //Return distance from origin to the contact patch.
                    }

                    // Find the support point in the direction of the portal's normal
                    Vector3 v4;
                    MinkowskiToolbox.GetLocalMinkowskiExtremePoint(shapeA, shapeB, ref n, ref localTransformB, out v4);

                    // If the origin is outside the support plane or the boundary is thin enough, we have a miss
                    Vector3.Dot(ref v4, ref n, out dot);
                    if (dot < 0) // || Vector3.Dot(v4 - v3, n) <= Toolbox.bigEpsilon)
                        return -float.MaxValue;
                    
                    //This test isn't strictly necessary.
                    //Vector3 v3v4;
                    //Vector3.Subtract(ref v4, ref v3, out v3v4);
                    //Vector3.Dot(ref v3v4, ref n, out dot);
                    //if (dot <= tolerance)
                    //{
                    //    Vector3.Dot(ref v3, ref n, out dot);
                    //    if (dot < 0)
                    //        return -dot;
                    //    return dot;
                    //}

                    // Test origin against the three planes that separate the new portal candidates: (v1,v4,v0) (v2,v4,v0) (v3,v4,v0)
                    Vector3.Cross(ref v4, ref v0, out cross);
                    Vector3.Dot(ref v1, ref cross, out dot);
                    if (dot > 0)
                    {
                        Vector3.Dot(ref v2, ref cross, out dot);
                        if (dot > 0)
                            v1 = v4; // Inside v1 & inside v2 ==> eliminate v1
                        else
                            v3 = v4; // Inside v1 & outside v2 ==> eliminate v3
                    }
                    else
                    {
                        Vector3.Dot(ref v3, ref cross, out dot);
                        if (dot > 0)
                            v2 = v4; // Outside v1 & inside v3 ==> eliminate v2
                        else
                            v1 = v4; // Outside v1 & outside v3 ==> eliminate v1
                    }

                    count++;
                }
            }
        }


        /// <summary>
        /// Determines whether or the point lies within the entity.
        /// </summary>
        /// <param name="p">Location to test for inclusion within the entity.</param>
        /// <param name="shape">Shape to test.</param>
        /// <param name="transform">Transform to use for the shape.</param>
        /// <returns>Whether or not the margin-expanded shape encloses the point.</returns>
        public static bool IsPointInsideShape(ref Vector3 p, ConvexShape shape, ref RigidTransform transform)
        {
            Vector3 localPosition;
            Quaternion conjugate;
            Quaternion.Conjugate(ref transform.Orientation, out conjugate);
            Vector3.Subtract(ref p, ref transform.Position, out localPosition);
            Vector3.Transform(ref localPosition, ref conjugate, out localPosition);

            //Shape is 'A', point is 'B'.
            //We're doing Minkowski difference of A - B, so shape - point.

            // v0 = center of Minkowski difference
            Vector3 v0;
            Vector3.Negate(ref localPosition, out v0);
            if (v0 == Toolbox.ZeroVector)
                return true; // v0 and origin overlap ==> hit

            // v1 = support in direction of origin
            Vector3 n = localPosition;
            Vector3 v1;
            shape.GetLocalExtremePoint(n, out v1);
            Vector3.Subtract(ref v1, ref localPosition, out v1);
            float dot;
            Vector3.Dot(ref v1, ref n, out dot);
            if (dot <= 0)
                return false; // origin outside v1 support plane ==> miss

            // v2 = support perpendicular to plane containing origin, v0 and v1
            Vector3.Cross(ref v1, ref v0, out n);
            if (n == Toolbox.ZeroVector) //(n.LengthSquared() < Toolbox.bigEpsilon)
                return true; // v0, v1 and origin colinear (and origin inside v1 support plane) == > hit
            Vector3 v2;
            shape.GetLocalExtremePoint(n, out v2);
            Vector3.Subtract(ref v2, ref localPosition, out v2);
            if (Vector3.Dot(v2, n) <= 0)
                return false; // origin outside v2 support plane ==> miss

            // v3 = support perpendicular to plane containing v0, v1 and v2
            Vector3 v0v1, v0v2;
            Vector3.Subtract(ref v1, ref v0, out v0v1);
            Vector3.Subtract(ref v2, ref v0, out v0v2);
            Vector3.Cross(ref v0v1, ref v0v2, out n);

            // If the origin is on the - side of the plane, reverse the direction of the plane
            Vector3.Dot(ref n, ref v0, out dot);
            if (dot > 0)
            {
                Vector3 temp = v1;
                v1 = v2;
                v2 = temp;
                n = -n;
            }
            else if (dot == 0)
                return true;

            // Phase One: Find a valid portal
            int count = 0;
            while (true)
            {
                count++;
                // Obtain the next support point
                Vector3 v3;
                shape.GetLocalExtremePoint(n, out v3);
                Vector3.Subtract(ref v3, ref localPosition, out v3);
                Vector3.Dot(ref v3, ref n, out dot);
                if (dot <= 0)
                    return false; // origin outside v3 support plane ==> miss

                // If origin is outside (v1,v0,v3), then portal is invalid -- eliminate v2 and find new support outside face
                Vector3 cross;
                Vector3.Cross(ref v1, ref v3, out cross);
                Vector3.Dot(ref cross, ref v0, out dot);
                if (count < OuterIterationLimit && dot < 0)
                {
                    v2 = v3;
                    Vector3 v0v3;
                    Vector3.Subtract(ref v1, ref v0, out v0v1);
                    Vector3.Subtract(ref v3, ref v0, out v0v3);
                    Vector3.Cross(ref v0v1, ref v0v3, out n);
                    continue;
                }

                // If origin is outside (v3,v0,v2), then portal is invalid -- eliminate v1 and find new support outside face
                Vector3.Cross(ref v3, ref v2, out cross);
                Vector3.Dot(ref cross, ref v0, out dot);
                if (count < OuterIterationLimit && dot < 0)
                {
                    v1 = v3;
                    Vector3 v0v3;
                    Vector3.Subtract(ref v3, ref v0, out v0v3);
                    Vector3.Subtract(ref v2, ref v0, out v0v2);
                    Vector3.Cross(ref v0v3, ref v0v2, out n);
                    continue;
                }

                count = 0;
                // Phase Two: Refine the portal
                while (true)
                {
                    // Compute outward facing normal of the portal
                    Vector3 v1v2, v1v3;
                    Vector3.Subtract(ref v2, ref v1, out v1v2);
                    Vector3.Subtract(ref v3, ref v1, out v1v3);
                    Vector3.Cross(ref v1v2, ref v1v3, out n);

                    // If the origin is inside the portal, we have a hit
                    Vector3.Dot(ref v1, ref n, out dot);
                    if (dot >= 0)
                        return true;

                    // Find the support point in the direction of the portal's normal
                    Vector3 v4;
                    shape.GetLocalExtremePoint(n, out v4);
                    Vector3.Subtract(ref v4, ref localPosition, out v4);

                    // If the origin is outside the support plane or the boundary is thin enough, we have a miss
                    float dot2;
                    Vector3.Dot(ref v4, ref n, out dot);
                    Vector3.Dot(ref v4, ref n, out dot);
                    Vector3 v3v4;
                    Vector3.Subtract(ref v4, ref v3, out v3v4);
                    Vector3.Dot(ref v3v4, ref n, out dot2);
                    if (dot <= 0 || dot2 <= Toolbox.BigEpsilon ||
                        count > InnerIterationLimit) //Numerical problem)
                        return false;

                    // Test origin against the three planes that separate the new portal candidates: (v1,v4,v0) (v2,v4,v0) (v3,v4,v0)
                    Vector3.Cross(ref v4, ref v0, out cross);
                    Vector3.Dot(ref v1, ref cross, out dot);
                    if (dot > 0)
                    {
                        Vector3.Dot(ref v2, ref cross, out dot);
                        if (dot > 0)
                            v1 = v4; // Inside v1 & inside v2 ==> eliminate v1
                        else
                            v3 = v4; // Inside v1 & outside v2 ==> eliminate v3
                    }
                    else
                    {
                        Vector3.Dot(ref v3, ref cross, out dot);
                        if (dot > 0)
                            v2 = v4; // Outside v1 & inside v3 ==> eliminate v2
                        else
                            v1 = v4; // Outside v1 & outside v3 ==> eliminate v1
                    }

                    count++;
                }
            }
        }



    }
}