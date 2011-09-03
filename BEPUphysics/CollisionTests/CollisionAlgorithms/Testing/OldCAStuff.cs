using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
 

namespace BEPUphysics.CollisionShapes
{
    class OldCAStuff
    {
        #region Support Mapping/Minkowski Sum Collision Detection Methods



        #region Conservative Advancement Methods

        /// <summary>
        /// Determines if the objects are colliding during the next frame.
        /// </summary>
        /// <param name="objA">First object to check.</param>
        /// <param name="objB">Second object to check.</param>
        /// <param name="dt">Time in seconds of physical logic to integrate over.</param>
        /// <param name="nextPositionA">The next non-penetrating center of mass position for object A.</param>
        /// <param name="nextPositionB">The next non-penetrating center of mass position for object B.</param>
        /// <param name="nextOrientationA">The next non-penetrating orientation state for object A.</param>
        /// <param name="nextOrientationB">The next non-penetrating orientation state for object B.</param>
        /// <param name="timeOfImpact">Time of impact, if any, between the objects.</param>
        /// <returns>Whether or not the core shapes of the objects have collided during the frame.
        /// If false, it does not mean that the margin-expanded shapes are not colliding, just that the cores are not.</returns>
        public static bool AreObjectsCollidingCA(Entity objA, Entity objB, float dt,
                                                 out Vector3 nextPositionA, out Vector3 nextPositionB, out Quaternion nextOrientationA, out Quaternion nextOrientationB, out float timeOfImpact)
        {
            Vector3 initialPositionA, initialPositionB, finalPositionA, finalPositionB;
            Quaternion initialOrientationA, initialOrientationB, finalOrientationA, finalOrientationB;
            IntegrateLinearVelocity(objA, objB, dt, out initialPositionA, out initialPositionB, out finalPositionA, out finalPositionB);
            IntegrateAngularVelocity(objA, objB, dt, out initialOrientationA, out initialOrientationB, out finalOrientationA, out finalOrientationB);
            return AreSweptObjectsCollidingCA(objA, objB, ref initialPositionA, ref initialPositionB, ref initialOrientationA, ref initialOrientationB,
                                              ref finalPositionA, ref finalPositionB, ref finalOrientationA, ref finalOrientationB,
                                              out nextPositionA, out nextPositionB, out nextOrientationA, out nextOrientationB, out timeOfImpact);
        }

        /// <summary>
        /// Determines if the objects are colliding during the next frame.
        /// </summary>
        /// <param name="objA">First object to check.</param>
        /// <param name="objB">Second object to check.</param>
        /// <param name="dt">Time in seconds of physical logic to integrate over.</param>
        /// <param name="nextPositionA">The next non-penetrating center of mass position for object A.</param>
        /// <param name="nextPositionB">The next non-penetrating center of mass position for object B.</param>
        /// <param name="nextOrientationA">The next non-penetrating orientation state for object A.</param>
        /// <param name="nextOrientationB">The next non-penetrating orientation state for object B.</param>
        /// <param name="timeOfImpact">Time of impact, if any, between the objects.</param>
        /// <param name="location">Location where the objects touch.</param>
        /// <param name="normal">Normal at the contact surface.</param>
        /// <returns>Whether or not the core shapes of the objects have collided during the frame.
        /// If false, it does not mean that the margin-expanded shapes are not colliding, just that the cores are not.</returns>
        public static bool AreSweptObjectsCollidingCA(Entity objA, Entity objB, float dt,
                                                      out Vector3 nextPositionA, out Vector3 nextPositionB, out Quaternion nextOrientationA, out Quaternion nextOrientationB,
                                                      out float timeOfImpact, out Vector3 location, out Vector3 normal)
        {
            Vector3 initialPositionA, initialPositionB, finalPositionA, finalPositionB;
            Quaternion initialOrientationA, initialOrientationB, finalOrientationA, finalOrientationB;
            IntegrateLinearVelocity(objA, objB, dt, out initialPositionA, out initialPositionB, out finalPositionA, out finalPositionB);
            IntegrateAngularVelocity(objA, objB, dt, out initialOrientationA, out initialOrientationB, out finalOrientationA, out finalOrientationB);
            if (AreSweptObjectsCollidingCA(objA, objB, ref initialPositionA, ref initialPositionB, ref initialOrientationA, ref initialOrientationB,
                                           ref finalPositionA, ref finalPositionB, ref finalOrientationA, ref finalOrientationB,
                                           out nextPositionA, out nextPositionB, out nextOrientationA, out nextOrientationB, out timeOfImpact))
            {
                float depth;
                MPRToolbox.AreObjectsColliding(objA, objB, ref nextPositionA, ref nextPositionB, ref nextOrientationA, ref nextOrientationB, objA.CollisionMargin, objB.CollisionMargin, out location,
                                                  out normal, out depth);
                return true;
            }
            location = NoVector;
            normal = NoVector;
            return false;
        }

        /// <summary>
        /// Determines if the objects are colliding during movement between the provided states.
        /// </summary>
        /// <param name="objA">First object to check.</param>
        /// <param name="objB">Second object to check.</param>
        /// <param name="originalPositionA">Initial center of mass position for entity A.</param>
        /// <param name="originalPositionB">Initial center of mass position for entity B.</param>
        /// <param name="originalOrientationA">Initial orientation for entity A.</param>
        /// <param name="originalOrientationB">Initial orientation for entity B.</param>
        /// <param name="finalPositionA">Final center of mass position for entity A.</param>
        /// <param name="finalPositionB">Final center of mass position for entity B.</param>
        /// <param name="finalOrientationA">Final orientation for entity A.</param>
        /// <param name="finalOrientationB">Final orientation for entity B.</param>
        /// <param name="nextPositionA">The next non-penetrating center of mass position for object A.</param>
        /// <param name="nextPositionB">The next non-penetrating center of mass position for object B.</param>
        /// <param name="nextOrientationA">The next non-penetrating orientation state for object A.</param>
        /// <param name="nextOrientationB">The next non-penetrating orientation state for object B.</param>
        /// <param name="timeOfImpact">Time of impact, if any, between the objects.</param>
        /// <param name="location">Location where the objects touch.</param>
        /// <param name="normal">Normal at the contact surface.</param>
        /// <returns>Whether or not the core shapes of the objects have collided during the frame.
        /// If false, it does not mean that the margin-expanded shapes are not colliding, just that the cores are not.</returns>
        public static bool AreSweptObjectsCollidingCA(Entity objA, Entity objB,
                                                      ref Vector3 originalPositionA, ref Vector3 originalPositionB, ref Quaternion originalOrientationA, ref Quaternion originalOrientationB,
                                                      ref Vector3 finalPositionA, ref Vector3 finalPositionB, ref Quaternion finalOrientationA, ref Quaternion finalOrientationB,
                                                      out Vector3 nextPositionA, out Vector3 nextPositionB, out Quaternion nextOrientationA, out Quaternion nextOrientationB,
                                                      out float timeOfImpact, out Vector3 location, out Vector3 normal)
        {
            if (AreSweptObjectsCollidingCA(objA, objB, ref originalPositionA, ref originalPositionB, ref originalOrientationA, ref originalOrientationB,
                                           ref finalPositionA, ref finalPositionB, ref finalOrientationA, ref finalOrientationB,
                                           out nextPositionA, out nextPositionB, out nextOrientationA, out nextOrientationB, out timeOfImpact))
            {
                float depth;
                MPRToolbox.AreObjectsCollidingMPR(objA, objB, ref nextPositionA, ref nextPositionB, ref nextOrientationA, ref nextOrientationB, objA.CollisionMargin, objB.CollisionMargin, out location,
                                                  out normal, out depth);
                return true;
            }
            location = NoVector;
            normal = NoVector;
            return false;
        }

        /// <summary>
        /// Determines if the objects are colliding during movement between the provided states.
        /// </summary>
        /// <param name="objA">First object to check.</param>
        /// <param name="objB">Second object to check.</param>
        /// <param name="originalPositionA">Initial center of mass position for entity A.</param>
        /// <param name="originalPositionB">Initial center of mass position for entity B.</param>
        /// <param name="originalOrientationA">Initial orientation for entity A.</param>
        /// <param name="originalOrientationB">Initial orientation for entity B.</param>
        /// <param name="finalPositionA">Final center of mass position for entity A.</param>
        /// <param name="finalPositionB">Final center of mass position for entity B.</param>
        /// <param name="finalOrientationA">Final orientation for entity A.</param>
        /// <param name="finalOrientationB">Final orientation for entity B.</param>
        /// <param name="nextPositionA">The next non-penetrating center of mass position for object A.</param>
        /// <param name="nextPositionB">The next non-penetrating center of mass position for object B.</param>
        /// <param name="nextOrientationA">The next non-penetrating orientation state for object A.</param>
        /// <param name="nextOrientationB">The next non-penetrating orientation state for object B.</param>
        /// <param name="timeOfImpact">Time of impact, if any, between the objects.</param>
        /// <returns>Whether or not the core shapes of the objects have collided during the frame.
        /// If false, it does not mean that the margin-expanded shapes are not colliding, just that the cores are not.</returns>
        public static bool AreSweptObjectsCollidingCA(Entity objA, Entity objB,
                                                      ref Vector3 originalPositionA, ref Vector3 originalPositionB, ref Quaternion originalOrientationA, ref Quaternion originalOrientationB,
                                                      ref Vector3 finalPositionA, ref Vector3 finalPositionB, ref Quaternion finalOrientationA, ref Quaternion finalOrientationB,
                                                      out Vector3 nextPositionA, out Vector3 nextPositionB, out Quaternion nextOrientationA, out Quaternion nextOrientationB,
                                                      out float timeOfImpact)
        {
            /*
             * Angle between quaternions:
             * angle = 2 * acos(|q1.q2|)
             * Angular velocity magnitude = angle / dt
             * (the time elapsed here is considered to be dt, so it works out nicely to = angle!)
             * Switch the CA method over to use initial/final inputs, and a 'next' output.
             * 
             */
            bool toReturn = true;
            float distance;
            Vector3 linearVelocityA;
            Vector3.Subtract(ref finalPositionA, ref originalPositionA, out linearVelocityA);
            float angularVelocityA = 2 * (float)Math.Acos(Math.Min(1, Math.Abs(Quaternion.Dot(originalOrientationA, finalOrientationA))));
            Vector3 linearVelocityB;
            Vector3.Subtract(ref finalPositionB, ref originalPositionB, out linearVelocityB);
            float angularVelocityB = 2 * (float)Math.Acos(Math.Min(1, Math.Abs(Quaternion.Dot(originalOrientationB, finalOrientationB))));

            Vector3 currentPositionA = originalPositionA;
            Vector3 currentPositionB = originalPositionB;

            Quaternion currentOrientationA = originalOrientationA;
            Quaternion currentOrientationB = originalOrientationB;
            Vector3 normal;
            timeOfImpact = 0;

            if (!AreObjectsCollidingMPR(objA, objB, ref currentPositionA, ref currentPositionB, ref currentOrientationA, ref currentOrientationB, out distance, out normal))
            {
                //if(objA is CompoundBody || objB is CompoundBody)
                //    upperBound = (linearVelocityA - linearVelocityB).Length() + angularVelocityA * objA.maximumRadius + angularVelocityB * objB.maximumRadius;
                //else
                Vector3 velocityDifference;
                Vector3.Subtract(ref linearVelocityA, ref linearVelocityB, out velocityDifference);
                float dot;
                Vector3.Dot(ref velocityDifference, ref normal, out dot);
                float upperBound = Math.Abs(dot + angularVelocityA * objA.maximumRadius + angularVelocityB * objB.maximumRadius);
                //float oldUpperBound = Math.Abs(Vector3.Dot(objA.myLinearVelocity * dt - objB.myLinearVelocity * dt, normal)) + (objA.myAngularVelocity - objB.myAngularVelocity).Length() * dt * (objA.maximumRadius + objB.maximumRadius);

                do
                {
                    //if (oldDistance < distance)
                    //    Debug.WriteLine("break.");
                    if (distance < .01f)
                    {
                        //Slightly 'Perfect' collision, must not have had much margin at all around the shapes.
                        //Help end the loop faster by giving it a boost, we'll accept a possible slight interpenetration.
                        //It's either that or the thing is having a little trouble converging.
                        distance = .01f;
                    }

                    timeOfImpact += distance / upperBound;
                    if (timeOfImpact >= 1)
                    {
                        //No collision.
                        //nextPositionA = finalPositionA;
                        //nextPositionB = finalPositionB;
                        //nextOrientationA = finalOrientationA;
                        //nextOrientationB = finalOrientationB;
                        timeOfImpact = 1;
                        toReturn = false;
                        break; //Shrunken shapes do not collide.
                    }
                    //Advance the states.
                    //float originalFraction = 1 - timeOfImpact;
                    //Advance the states.
                    Vector3.Lerp(ref originalPositionA, ref finalPositionA, timeOfImpact, out currentPositionA);
                    Vector3.Lerp(ref originalPositionB, ref finalPositionB, timeOfImpact, out currentPositionB);
                    Quaternion.Slerp(ref originalOrientationA, ref finalOrientationA, timeOfImpact, out currentOrientationA);
                    Quaternion.Slerp(ref originalOrientationB, ref finalOrientationB, timeOfImpact, out currentOrientationB);

                    /*currentPositionA = originalPositionA * originalFraction + finalPositionA * timeOfImpact;
                    currentPositionB = originalPositionB * originalFraction + finalPositionB * timeOfImpact;
                    Quaternion.Slerp(ref originalOrientationA, ref finalOrientationA, timeOfImpact, out currentOrientationA);
                    Quaternion.Slerp(ref originalOrientationB, ref finalOrientationB, timeOfImpact, out currentOrientationB);
                    currentGeometricPositionA = currentPositionA + Vector3.Transform(-objA.myCenterOfMassOffset, currentOrientationA);
                    currentGeometricPositionB = currentPositionB + Vector3.Transform(-objB.myCenterOfMassOffset, currentOrientationB);*/
                } while (!AreObjectsCollidingMPR(objA, objB, ref currentPositionA, ref currentPositionB, ref currentOrientationA, ref currentOrientationB, out distance, out normal));
                //Make sure the next state is not in penetration, but is within a margin.
                //float safePushBack;
                //if (upperBound > 0)
                //    safePushBack = .01f * (objA.margin + objB.margin) / (upperBound);
                //else
                //    safePushBack = 0;

                if (timeOfImpact <= 1)
                {
                    float newToi = timeOfImpact;
                    float originalFraction = 1 - newToi;
                    currentPositionA = originalPositionA * originalFraction + finalPositionA * newToi;
                    currentPositionB = originalPositionB * originalFraction + finalPositionB * newToi;
                    Quaternion.Slerp(ref originalOrientationA, ref finalOrientationA, newToi, out currentOrientationA);
                    Quaternion.Slerp(ref originalOrientationB, ref finalOrientationB, newToi, out currentOrientationB);
                }
            }

            //Find the closest points and try to generate contact data, or determine that the objects are too far apart to do so.
            //if (currentPositionA == originalPositionA)
            //    Debug.WriteLine("break.");
            nextPositionA = currentPositionA;
            nextPositionB = currentPositionB;
            nextOrientationA = currentOrientationA;
            nextOrientationB = currentOrientationB;

            #region old contact generation

            /*
            float totalMargin = objA.margin + objB.margin;
            if (distance < totalMargin)
            {
                //return areObjectsCollidingMPR(objA, objB, ref nextPositionA, ref nextPositionB, ref nextOrientationA, ref nextOrientationB, objA.margin, objB.margin, objA.margin, objB.margin, out contactLocation, out normal, out depth);

                Vector3 closestA, closestB;
                Vector3 ab = getClosestPointsBetweenObjects(objA, objB, ref nextPositionA, ref nextPositionB, ref nextOrientationA, ref nextOrientationB, 0, 0, out closestA, out closestB);
                float abLengthSquared = ab.LengthSquared();
                if (abLengthSquared < totalMargin * totalMargin)
                {
                    if (abLengthSquared > 0)
                    {
                        //Collision! generate contact data.
                        contactLocation = (closestA + closestB) / 2;
                        float abLength = (float)Math.Sqrt(abLengthSquared);
                        if (abLengthSquared > 0)
                            normal = ab / abLength;
                        else
                            normal = Vector3.Up; //Arbitrarily choose a direction.  Better a slight 'oops' than a complete explosion; should never happen anyway.
                        depth = totalMargin - abLength;
                        return true;
                    }
                    else //In the case that the objects are intersecting and GJK can't identify contact data, use MPR.
                        return areObjectsCollidingMPR(objA, objB, ref nextPositionA, ref nextPositionB, ref nextOrientationA, ref nextOrientationB, objA.margin, objB.margin, out contactLocation, out normal, out depth);

                }
                
                
            }
            //No collision.
            contactLocation = noVector;
            normal = noVector;
            depth = -float.MaxValue;
            timeOfImpact = 1;
            return false;*/

            #endregion

            return toReturn;
        }

        /// <summary>
        /// Determines if the objects are colliding during movement between the provided states.
        /// </summary>
        /// <param name="objA">First object to check.</param>
        /// <param name="objB">Second object to check.</param>
        /// <param name="originalPositionA">Initial center of mass position for entity A.</param>
        /// <param name="originalPositionB">Initial center of mass position for entity B.</param>
        /// <param name="originalOrientationA">Initial orientation for entity A.</param>
        /// <param name="originalOrientationB">Initial orientation for entity B.</param>
        /// <param name="finalPositionA">Final center of mass position for entity A.</param>
        /// <param name="finalPositionB">Final center of mass position for entity B.</param>
        /// <param name="nextPositionA">The next non-penetrating center of mass position for object A.</param>
        /// <param name="nextPositionB">The next non-penetrating center of mass position for object B.</param>
        /// <param name="timeOfImpact">Time of impact, if any, between the objects.</param>
        /// <returns>Whether or not the core shapes of the objects have collided during the frame.
        /// If false, it does not mean that the margin-expanded shapes are not colliding, just that the cores are not.</returns>
        public static bool AreSweptObjectsCollidingCA(Entity objA, Entity objB,
                                                      ref Vector3 originalPositionA, ref Vector3 originalPositionB, ref Quaternion originalOrientationA, ref Quaternion originalOrientationB,
                                                      ref Vector3 finalPositionA, ref Vector3 finalPositionB,
                                                      out Vector3 nextPositionA, out Vector3 nextPositionB,
                                                      out float timeOfImpact)
        {
            /*
             * Angle between quaternions:
             * angle = 2 * acos(|q1.q2|)
             * Angular velocity magnitude = angle / dt
             * (the time elapsed here is considered to be dt, so it works out nicely to = angle!)
             * Switch the CA method over to use initial/final inputs, and a 'next' output.
             * 
             */
            bool toReturn = true;
            float distance;
            Vector3 linearVelocityA;
            Vector3.Subtract(ref finalPositionA, ref originalPositionA, out linearVelocityA);
            Vector3 linearVelocityB;
            Vector3.Subtract(ref finalPositionB, ref originalPositionB, out linearVelocityB);

            Vector3 currentPositionA = originalPositionA;
            Vector3 currentPositionB = originalPositionB;

            Vector3 normal;
            timeOfImpact = 0;

            if (!AreObjectsCollidingMPR(objA, objB, ref currentPositionA, ref currentPositionB, ref originalOrientationA, ref originalOrientationB, out distance, out normal))
            {
                //if(objA is CompoundBody || objB is CompoundBody)
                //    upperBound = (linearVelocityA - linearVelocityB).Length() + angularVelocityA * objA.maximumRadius + angularVelocityB * objB.maximumRadius;
                //else
                Vector3 velocityDifference;
                Vector3.Subtract(ref linearVelocityA, ref linearVelocityB, out velocityDifference);
                float dot;
                Vector3.Dot(ref velocityDifference, ref normal, out dot);
                float upperBound = Math.Abs(dot);
                //float oldUpperBound = Math.Abs(Vector3.Dot(objA.myLinearVelocity * dt - objB.myLinearVelocity * dt, normal)) + (objA.myAngularVelocity - objB.myAngularVelocity).Length() * dt * (objA.maximumRadius + objB.maximumRadius);

                do
                {
                    //if (oldDistance < distance)
                    //    Debug.WriteLine("break.");
                    if (distance < .01f)
                    {
                        //Slightly 'Perfect' collision, must not have had much margin at all around the shapes.
                        //Help end the loop faster by giving it a boost, we'll accept a possible slight interpenetration.
                        //It's either that or the thing is having a little trouble converging.
                        distance = .01f;
                    }

                    timeOfImpact += distance / upperBound;
                    if (timeOfImpact >= 1)
                    {
                        //No collision.
                        //nextPositionA = finalPositionA;
                        //nextPositionB = finalPositionB;
                        timeOfImpact = 1;
                        toReturn = false;
                        break; //Shrunken shapes do not collide.
                    }
                    //Advance the states.
                    //float originalFraction = 1 - timeOfImpact;
                    //Advance the states.
                    Vector3.Lerp(ref originalPositionA, ref finalPositionA, timeOfImpact, out currentPositionA);
                    Vector3.Lerp(ref originalPositionB, ref finalPositionB, timeOfImpact, out currentPositionB);

                    /*currentPositionA = originalPositionA * originalFraction + finalPositionA * timeOfImpact;
                    currentPositionB = originalPositionB * originalFraction + finalPositionB * timeOfImpact;
                    Quaternion.Slerp(ref originalOrientationA, ref finalOrientationA, timeOfImpact, out currentOrientationA);
                    Quaternion.Slerp(ref originalOrientationB, ref finalOrientationB, timeOfImpact, out currentOrientationB);
                    currentGeometricPositionA = currentPositionA + Vector3.Transform(-objA.myCenterOfMassOffset, currentOrientationA);
                    currentGeometricPositionB = currentPositionB + Vector3.Transform(-objB.myCenterOfMassOffset, currentOrientationB);*/
                } while (!AreObjectsCollidingMPR(objA, objB, ref currentPositionA, ref currentPositionB, ref originalOrientationA, ref originalOrientationB, out distance, out normal));
                //Make sure the next state is not in penetration, but is within a margin.
                //float safePushBack;
                //if (upperBound > 0)
                //    safePushBack = .01f * (objA.margin + objB.margin) / (upperBound);
                //else
                //    safePushBack = 0;

                if (timeOfImpact <= 1)
                {
                    float newToi = timeOfImpact;
                    float originalFraction = 1 - newToi;
                    currentPositionA = originalPositionA * originalFraction + finalPositionA * newToi;
                    currentPositionB = originalPositionB * originalFraction + finalPositionB * newToi;
                }
            }
            nextPositionA = currentPositionA;
            nextPositionB = currentPositionB;


            return toReturn;
        }

        /// <summary>
        /// Determines the status of a collision between two entities.
        /// </summary>
        /// <param name="a">First entity involved in the collision.</param>
        /// <param name="b">Second entity involved in the collision.</param>
        /// <param name="distance">If the objects are not colliding, represents a conservative estimate of the distance between the two bodies' core, non-margin expanded shapes.</param>
        /// <param name="normal">If the objects are not colliding, represents a separating direction between the two bodies.</param>
        /// <returns>Whether or not the bodies' margin-expanded shapes are colliding.</returns>
        internal static bool AreObjectsCollidingMPR(Entity a, Entity b, out float distance, out Vector3 normal)
        {
            return AreObjectsCollidingMPR(a, b, ref a.position, ref b.position, ref a.orientation, ref b.orientation,
                                          out distance, out normal);
        }

        /// <summary>
        /// Determines the status of a collision between two entities.
        /// </summary>
        /// <param name="a">First entity involved in the collision.</param>
        /// <param name="b">Second entity involved in the collision.</param>
        /// <param name="positionA">Location to consider as the center of the first object.</param>
        /// <param name="positionB">Location to consider as the center of the second object.</param>
        /// <param name="orientationA">Orientation to use in lieu of the first object's rotation.</param>
        /// <param name="orientationB">Orientation to use in lieu of the second object's rotation.</param>
        /// <param name="distance">If the objects are not colliding, represents a conservative estimate of the distance between the two bodies' core, non-margin expanded shapes.</param>
        /// <param name="normal">If the objects are not colliding, represents a separating direction between the two bodies.</param>
        /// <returns>Whether or not the bodies' margin-expanded shapes are colliding.</returns>
        internal static bool AreObjectsCollidingMPR(Entity a, Entity b, ref Vector3 positionA, ref Vector3 positionB, ref Quaternion orientationA, ref Quaternion orientationB, out float distance,
                                                    out Vector3 normal)
        {
            distance = MPRToolbox.FindConservativeDistanceEstimate(a, b, ref positionA, ref positionB, ref orientationA, ref orientationB, 0, 0, out normal);

            //distance = getDistanceBetweenObjects(a, b, ref positionA, ref positionB, ref orientationA, ref orientationB, 0, 0, out normal);
            if (distance == 0)
            {
                normal = NoVector;

                return true;
            }

            return false;
        }

        #endregion



        #endregion
    }
}
