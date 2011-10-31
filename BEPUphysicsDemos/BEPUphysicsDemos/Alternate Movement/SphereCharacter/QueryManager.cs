using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.DataStructures;
using BEPUphysics.CollisionTests;
using Microsoft.Xna.Framework;
using BEPUphysics.MathExtensions;
using BEPUphysics.Collidables;
using BEPUphysics.NarrowPhaseSystems;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Settings;
using BEPUphysics;
using BEPUphysics.BroadPhaseSystems;

namespace BEPUphysicsDemos.AlternateMovement.SphereCharacter
{
    /// <summary>
    /// Helps a character identify supports by finding contacts and ray cast intersections with its immediate surroundings.
    /// </summary>
    public class QueryManager
    {
        //This QueryManager is not thread safe in any way, but it's only ever used by a single character at a time, so this isn't an issue.
        RawList<ContactData> contacts = new RawList<ContactData>();
        RawList<ContactData> supportContacts = new RawList<ContactData>();
        RawList<ContactData> tractionContacts = new RawList<ContactData>();
        RawList<ContactData> sideContacts = new RawList<ContactData>();
        RawList<ContactData> headContacts = new RawList<ContactData>();

        public RawList<ContactData> Contacts { get { return contacts; } }
        public RawList<ContactData> SupportContacts { get { return supportContacts; } }
        public RawList<ContactData> TractionContacts { get { return tractionContacts; } }

        EntityCollidable queryObject;
        SphereCharacterController character;

        /// <summary>
        /// Constructs the query manager for a character.
        /// </summary>
        /// <param name="character">Character to manage queries for.</param>
        public QueryManager(SphereCharacterController character)
        {
            this.character = character;
            //We can share the real shape with the 'current' query object.
            queryObject = new ConvexCollidable<SphereShape>(character.Body.CollisionInformation.Shape);
            //Share the collision rules between the main body and its query objects.  That way, the character's queries return valid results.
            queryObject.CollisionRules = character.Body.CollisionInformation.CollisionRules;


            SupportRayFilter = SupportRayFilterFunction;
        }


        Func<BroadPhaseEntry, bool> SupportRayFilter;
        bool SupportRayFilterFunction(BroadPhaseEntry entry)
        {
            //Only permit an object to be used as a support if it fully collides with the character.
            return CollisionRules.CollisionRuleCalculator(entry.CollisionRules, character.Body.CollisionInformation.CollisionRules) == CollisionRule.Normal;
        }

        /// <summary>
        /// Computes the intersection, if any, between a ray and the objects in the character's bounding box.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="length">Length of the ray to use in units of the ray's length.</param>
        /// <param name="earliestHit">Earliest intersection location and information.</param>
        /// <returns>Whether or not the ray hit anything.</returns>
        public bool RayCast(Ray ray, float length, out RayHit earliestHit)
        {
            earliestHit = new RayHit();
            earliestHit.T = float.MaxValue;
            foreach (var collidable in character.Body.CollisionInformation.OverlappedCollidables)
            {
                //Check to see if the collidable is hit by the ray.
                float? t = ray.Intersects(collidable.BoundingBox);
                if (t != null && t < length)
                {
                    //Is it an earlier hit than the current earliest?
                    RayHit hit;
                    if (collidable.RayCast(ray, length, SupportRayFilter, out hit) && hit.T < earliestHit.T)
                    {
                        earliestHit = hit;
                    }
                }
            }
            if (earliestHit.T == float.MaxValue)
                return false;
            return true;

        }

        /// <summary>
        /// Computes the intersection, if any, between a ray and the objects in the character's bounding box.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="length">Length of the ray to use in units of the ray's length.</param>
        /// <param name="earliestHit">Earliest intersection location and information.</param>
        /// <param name="hitObject">Collidable intersected by the ray, if any.</param>
        /// <returns>Whether or not the ray hit anything.</returns>
        public bool RayCast(Ray ray, float length, out RayHit earliestHit, out Collidable hitObject)
        {
            earliestHit = new RayHit();
            earliestHit.T = float.MaxValue;
            hitObject = null;
            foreach (var collidable in character.Body.CollisionInformation.OverlappedCollidables)
            {
                //Check to see if the collidable is hit by the ray.
                float? t = ray.Intersects(collidable.BoundingBox);
                if (t != null && t < length)
                {
                    //Is it an earlier hit than the current earliest?
                    RayHit hit;
                    if (collidable.RayCast(ray, length, SupportRayFilter, out hit) && hit.T < earliestHit.T)
                    {
                        earliestHit = hit;
                        hitObject = collidable;
                    }
                }
            }
            if (earliestHit.T == float.MaxValue)
                return false;
            return true;

        }

        /// <summary>
        /// Determines if a ray intersects any object in the character's bounding box.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="length">Length of the ray to use in units of the ray's length.</param>
        /// <returns>Whether or not the ray hit anything.</returns>
        public bool RayCastHitAnything(Ray ray, float length)
        {
            foreach (var collidable in character.Body.CollisionInformation.OverlappedCollidables)
            {
                //Check to see if the collidable is hit by the ray.
                float? t = ray.Intersects(collidable.BoundingBox);
                if (t != null && t < length)
                {
                    RayHit hit;
                    if (collidable.RayCast(ray, length, SupportRayFilter, out hit))
                    {
                        return true;
                    }
                }
            }
            return false;

        }

        private void ClearContacts()
        {
            contacts.Clear();
            supportContacts.Clear();
            tractionContacts.Clear();
            sideContacts.Clear();
            headContacts.Clear();
        }

        /// <summary>
        /// Tests a collision object with the same shape as the current character at the given position for contacts.
        /// Output data is stored in the query manager's supporting lists.
        /// </summary>
        /// <param name="position">Position to use for the query.</param>
        public void QueryContacts(Vector3 position)
        {
            QueryContacts(position, queryObject);
        }


        void QueryContacts(Vector3 position, EntityCollidable queryObject)
        {
            ClearContacts();

            //Update the position and orientation of the query object.
            RigidTransform transform;
            transform.Position = position;
            transform.Orientation = character.Body.Orientation;
            queryObject.UpdateBoundingBoxForTransform(ref transform, 0);

            foreach (var collidable in character.Body.CollisionInformation.OverlappedCollidables)
            {
                if (collidable.BoundingBox.Intersects(queryObject.BoundingBox))
                {
                    var pair = new CollidablePair(collidable, queryObject);
                    var pairHandler = NarrowPhaseHelper.GetPairHandler(ref pair);
                    if (pairHandler.CollisionRule == CollisionRule.Normal)
                    {
                        pairHandler.UpdateCollision(0);
                        foreach (var contact in pairHandler.Contacts)
                        {
                            //Must check per-contact collision rules, just in case
                            //the pair was actually a 'parent pair.'
                            if (contact.Pair.CollisionRule == CollisionRule.Normal)
                            {
                                ContactData contactData;
                                contactData.Position = contact.Contact.Position;
                                contactData.Normal = contact.Contact.Normal;
                                contactData.Id = contact.Contact.Id;
                                contactData.PenetrationDepth = contact.Contact.PenetrationDepth;
                                contacts.Add(contactData);
                            }
                        }
                    }
                    //TODO: It would be nice if this was a bit easier.
                    //Having to remember to clean up AND give it back is a bit weird, especially with the property-diving.
                    //No one would ever just guess this correctly.
                    //At least hide it behind a NarrowPhaseHelper function.
                    pairHandler.CleanUp();
                    pairHandler.Factory.GiveBack(pairHandler);
                }
            }

            CategorizeContacts(ref position);
        }

        void CategorizeContacts(ref Vector3 position)
        {
            Vector3 downDirection = character.Body.OrientationMatrix.Down;
            for (int i = 0; i < contacts.Count; i++)
            {
                float dot;
                Vector3 offset;
                Vector3.Subtract(ref contacts.Elements[i].Position, ref position, out offset);
                Vector3.Dot(ref contacts.Elements[i].Normal, ref offset, out dot);
                ContactData processed = contacts.Elements[i];
                if (dot < 0)
                {
                    //The normal should face outward.
                    dot = -dot;
                    Vector3.Negate(ref processed.Normal, out processed.Normal);
                }
                Vector3.Dot(ref processed.Normal, ref downDirection, out dot);
                if (dot > SupportFinder.SideContactThreshold)
                {
                    //It's a support.
                    supportContacts.Add(processed);
                    if (dot > character.SupportFinder.cosMaximumSlope)
                    {
                        //It's a traction contact.
                        tractionContacts.Add(processed);
                    }
                    else
                        sideContacts.Add(processed); //Considering the side contacts to be supports can help with upstepping.
                }
                else if (dot < -SupportFinder.SideContactThreshold)
                {
                    //It's a head contact.
                    headContacts.Add(processed);
                }
                else
                {
                    //It's a side contact.  These could obstruct the stepping.
                    sideContacts.Add(processed);
                }

            }
        }

        internal bool HasSupports(out bool hasTraction, out PositionState state, out ContactData supportContact)
        {
            float maxDepth = -float.MaxValue;
            int deepestIndex = -1;
            if (tractionContacts.Count > 0)
            {
                //It has traction!
                //But is it too deep?
                //Find the deepest contact.
                for (int i = 0; i < tractionContacts.Count; i++)
                {
                    if (tractionContacts.Elements[i].PenetrationDepth > maxDepth)
                    {
                        maxDepth = tractionContacts.Elements[i].PenetrationDepth;
                        deepestIndex = i;
                    }
                }
                hasTraction = true;
                supportContact = tractionContacts.Elements[deepestIndex];
            }
            else if (supportContacts.Count > 0)
            {
                //It has support!
                //But is it too deep?
                //Find the deepest contact.

                for (int i = 0; i < supportContacts.Count; i++)
                {
                    if (supportContacts.Elements[i].PenetrationDepth > maxDepth)
                    {
                        maxDepth = supportContacts.Elements[i].PenetrationDepth;
                        deepestIndex = i;
                    }
                }
                hasTraction = false;
                supportContact = supportContacts.Elements[deepestIndex];
            }
            else
            {
                hasTraction = false;
                state = PositionState.NoHit;
                supportContact = new ContactData();
                return false;
            }
            //Check the depth.
            if (maxDepth > CollisionDetectionSettings.AllowedPenetration)
            {
                //It's too deep.
                state = PositionState.TooDeep;
            }
            else if (maxDepth < 0)
            {
                //The depth is negative, meaning it's separated.  This shouldn't happen with the initial implementation of the character controller,
                //but this case could conceivably occur in other usages of a system like this (or in a future version of the character),
                //so go ahead and handle it.
                state = PositionState.NoHit;
            }
            else
            {
                //The deepest contact appears to be very nicely aligned with the ground!
                //It's fully supported.
                state = PositionState.Accepted;
            }
            return true;

        }
    }



    enum PositionState
    {
        Accepted,
        Rejected,
        TooDeep,
        Obstructed,
        HeadObstructed,
        NoHit
    }
}
