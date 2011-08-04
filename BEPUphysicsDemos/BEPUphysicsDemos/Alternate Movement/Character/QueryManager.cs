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

namespace BEPUphysicsDemos.AlternateMovement.Character
{
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
        public RawList<ContactData> SideContacts { get { return sideContacts; } }
        public RawList<ContactData> HeadContacts { get { return headContacts; } }

        EntityCollidable standingQueryObject;
        EntityCollidable crouchingQueryObject;
        EntityCollidable currentQueryObject;
        CharacterController character;

        public QueryManager(CharacterController character)
        {
            this.character = character;
            //We can share the real shape with the 'current' query object.
            currentQueryObject = new ConvexCollidable<CylinderShape>(character.Body.CollisionInformation.Shape);
            standingQueryObject = new ConvexCollidable<CylinderShape>(new CylinderShape(character.StanceManager.StandingHeight, character.Body.Radius));
            crouchingQueryObject = new ConvexCollidable<CylinderShape>(new CylinderShape(character.StanceManager.CrouchingHeight, character.Body.Radius));
            //Share the collision rules between the main body and its query objects.  That way, the character's queries return valid results.
            currentQueryObject.CollisionRules = character.Body.CollisionInformation.CollisionRules;
            standingQueryObject.CollisionRules = character.Body.CollisionInformation.CollisionRules;
            crouchingQueryObject.CollisionRules = character.Body.CollisionInformation.CollisionRules;
        }

        private void ClearContacts()
        {
            contacts.Clear();
            supportContacts.Clear();
            tractionContacts.Clear();
            sideContacts.Clear();
            headContacts.Clear();
        }

        public void QueryContacts(Vector3 position)
        {
            QueryContacts(position, currentQueryObject);
        }

        public void QueryContacts(Vector3 position, Stance stance)
        {
            QueryContacts(position, stance == Stance.Standing ? standingQueryObject : crouchingQueryObject);
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
                    //Having to remember to clean up AND give it back is a bit weird, especially with the casting and property-diving.
                    //No one would ever just guess this correctly.
                    //At least hide it behind a NarrowPhaseHelper function.
                    pairHandler.CleanUp();
                    (pairHandler as INarrowPhasePair).Factory.GiveBack(pairHandler);
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
