using System;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.Entities;
using BEPUutilities;
using BEPUutilities.DataStructures;
using BEPUphysics.NarrowPhaseSystems;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.Settings;

namespace BEPUphysics.Character
{
    /// <summary>
    /// Helps a character identify supports by finding contacts and ray cast intersections with its immediate surroundings.
    /// </summary>
    public class QueryManager
    {
        private Entity characterBody;
        private CharacterContactCategorizer contactCategorizer;


        /// <summary>
        /// Constructs the query manager for a character.
        /// </summary>
        public QueryManager(Entity characterBody, CharacterContactCategorizer contactCategorizer)
        {
            this.characterBody = characterBody;
            this.contactCategorizer = contactCategorizer;

            SupportRayFilter = SupportRayFilterFunction;
        }


        Func<BroadPhaseEntry, bool> SupportRayFilter;
        bool SupportRayFilterFunction(BroadPhaseEntry entry)
        {
            //Only permit an object to be used as a support if it fully collides with the character.
            return CollisionRules.CollisionRuleCalculator(entry, characterBody.CollisionInformation) == CollisionRule.Normal;
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
            foreach (var collidable in characterBody.CollisionInformation.OverlappedCollidables)
            {
                //Check to see if the collidable is hit by the ray.
                float t;
                if (ray.Intersects(ref collidable.boundingBox, out t) && t < length)
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
            foreach (var collidable in characterBody.CollisionInformation.OverlappedCollidables)
            {
                //Check to see if the collidable is hit by the ray.
                float t;
                if (ray.Intersects(ref collidable.boundingBox, out t) && t < length)
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
            foreach (var collidable in characterBody.CollisionInformation.OverlappedCollidables)
            {
                //Check to see if the collidable is hit by the ray.
                float t;
                if (ray.Intersects(ref collidable.boundingBox, out t) && t < length)
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




        /// <summary>
        /// Finds contacts between the query object and any intersected objects within the character's bounding box.
        /// </summary>
        /// <param name="queryObject">Collidable to query for contacts with.</param>
        /// <param name="tractionContacts">Output contacts that would provide traction.</param>
        /// <param name="supportContacts">Output contacts that would provide support.</param>
        /// <param name="sideContacts">Output contacts on the sides of the query object.</param>
        /// <param name="headContacts">Output contacts on the head of the query object.</param>
        public void QueryContacts(EntityCollidable queryObject,
            ref QuickList<CharacterContact> tractionContacts, ref QuickList<CharacterContact> supportContacts, ref QuickList<CharacterContact> sideContacts, ref QuickList<CharacterContact> headContacts)
        {
            var downDirection = characterBody.orientationMatrix.Down;

            tractionContacts.Clear();
            supportContacts.Clear();
            sideContacts.Clear();
            headContacts.Clear();

            foreach (var collidable in characterBody.CollisionInformation.OverlappedCollidables)
            {
                //The query object is assumed to have a valid bounding box.
                if (collidable.BoundingBox.Intersects(queryObject.BoundingBox))
                {
                    var pair = new CollidablePair(collidable, queryObject);
                    var pairHandler = NarrowPhaseHelper.GetPairHandler(ref pair);
                    if (pairHandler.CollisionRule == CollisionRule.Normal)
                    {
                        pairHandler.SuppressEvents = true;
                        pairHandler.UpdateCollision(0);
                        pairHandler.SuppressEvents = false;

                        contactCategorizer.CategorizeContacts(pairHandler, characterBody.CollisionInformation, ref downDirection,
                                                              ref tractionContacts, ref supportContacts, ref sideContacts, ref headContacts);
                    }
                    //TODO: It would be nice if this was a bit easier.
                    //Having to remember to clean up AND give it back is a bit weird, especially with the property-diving.
                    //No one would ever just guess this correctly.
                    //At least hide it behind a NarrowPhaseHelper function.
                    pairHandler.CleanUp();
                    pairHandler.Factory.GiveBack(pairHandler);
                }
            }

        }



        /// <summary>
        /// Analyzes the support state of the character based on the speculative input support and traction contacts.
        /// </summary>
        /// <param name="tractionContacts">Contacts providing the character with traction.</param>
        /// <param name="supportContacts">Contacts providing the character with support.</param>
        /// <param name="state">State of the contacts relative to the speculative character position.</param>
        /// <param name="supportContact">Representative contact to use, if any.</param>
        public void AnalyzeSupportState(ref QuickList<CharacterContact> tractionContacts, ref QuickList<CharacterContact> supportContacts,
                                        out CharacterContactPositionState state, out CharacterContact supportContact)
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
                    if (tractionContacts.Elements[i].Contact.PenetrationDepth > maxDepth)
                    {
                        maxDepth = tractionContacts.Elements[i].Contact.PenetrationDepth;
                        deepestIndex = i;
                    }
                }
                supportContact = tractionContacts.Elements[deepestIndex];
            }
            else if (supportContacts.Count > 0)
            {
                //It has support!
                //But is it too deep?
                //Find the deepest contact.

                for (int i = 0; i < supportContacts.Count; i++)
                {
                    if (supportContacts.Elements[i].Contact.PenetrationDepth > maxDepth)
                    {
                        maxDepth = supportContacts.Elements[i].Contact.PenetrationDepth;
                        deepestIndex = i;
                    }
                }
                supportContact = supportContacts.Elements[deepestIndex];
            }
            else
            {
                state = CharacterContactPositionState.NoHit;
                supportContact = new CharacterContact();
                return;
            }
            //Check the depth.
            if (maxDepth > CollisionDetectionSettings.AllowedPenetration)
            {
                //It's too deep.
                state = CharacterContactPositionState.TooDeep;
            }
            else if (maxDepth < 0)
            {
                //The depth is negative, meaning it's separated.  This shouldn't happen with the initial implementation of the character controller,
                //but this case could conceivably occur in other usages of a system like this (or in a future version of the character),
                //so go ahead and handle it.
                state = CharacterContactPositionState.NoHit;
            }
            else
            {
                //The deepest contact appears to be very nicely aligned with the ground!
                //It's fully supported.
                state = CharacterContactPositionState.Accepted;
            }
        }

    }




}
