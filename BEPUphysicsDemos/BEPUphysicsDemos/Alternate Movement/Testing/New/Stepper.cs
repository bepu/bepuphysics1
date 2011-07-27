using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.CollisionShapes.ConvexShapes;
using Microsoft.Xna.Framework;
using BEPUphysics.MathExtensions;
using BEPUphysics.NarrowPhaseSystems;
using BEPUphysics.Collidables;
using BEPUphysics.CollisionTests;
using BEPUphysics.DataStructures;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.Settings;
using BEPUphysics;
using System.Diagnostics;

namespace BEPUphysicsDemos.AlternateMovement.Testing.New
{
    public class Stepper
    {
        CharacterController character;
        public float MaximumStepHeight = 1;
        public float MinimumDownStepHeight = .1f;

        EntityCollidable queryObject;

        public Stepper(CharacterController character)
        {
            this.character = character;
            queryObject = new ConvexCollidable<CylinderShape>(character.Body.CollisionInformation.Shape);
            queryObject.CollisionRules.Personal = BEPUphysics.CollisionRuleManagement.CollisionRule.NoSolver;


        }

        public void QueryContacts(Vector3 position, RawList<ContactData> contacts)
        {
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
                    pairHandler.UpdateCollision(0);
                    foreach (var contact in pairHandler.Contacts)
                    {
                        ContactData contactData;
                        contactData.Position = contact.Contact.Position;
                        contactData.Normal = contact.Contact.Normal;
                        contactData.Id = contact.Contact.Id;
                        contactData.PenetrationDepth = contact.Contact.PenetrationDepth;
                        contacts.Add(contactData);
                    }
                    pairHandler.CleanUp();
                    (pairHandler as INarrowPhasePair).Factory.GiveBack(pairHandler);
                }
            }

        }

        public void CategorizeContacts(RawList<ContactData> contacts, RawList<ContactData> outputSupports, RawList<ContactData> outputTraction, RawList<ContactData> outputHeadContacts, RawList<ContactData> outputSideContacts)
        {
            Vector3 downDirection = character.Body.OrientationMatrix.Down;
            Vector3 position = character.Body.Position;
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
                    outputSupports.Add(processed);
                    if (dot > character.SupportFinder.cosMaximumSlope)
                    {
                        //It's a traction contact.
                        outputTraction.Add(processed);
                    }
                }
                else if (dot < SupportFinder.SideContactThreshold)
                {
                    //It's a head contact.
                    outputHeadContacts.Add(processed);
                }
                else
                {
                    //It's a side contact.  These could obstruct the stepping.
                    outputSideContacts.Add(processed);
                }

            }
        }

        public bool IsObstructed(RawList<ContactData> outputSideContacts)
        {
            //A contact is considered obstructive if its projected depth is deeper than any existing contact along the existing contacts' normals.
            for (int i = 0; i < outputSideContacts.Count; i++)
            {
                if (IsObstructive(ref outputSideContacts.Elements[i]))
                    return true;
            }
            return false;
        }

        bool IsObstructive(ref ContactData contact)
        {
            //If there weren't any contacts before, and the new contact is too deep, then it's obstructive.
            //If it is barely touching then it's not really an issue.
            if (character.SupportFinder.SideContacts.Count == 0 && contact.PenetrationDepth > CollisionDetectionSettings.AllowedPenetration)
            {
                return true;
            }
            //Go through side-facing contact and check to see if the new contact is deeper than any existing contact in the direction of the existing contact.
            //This is equivalent to considering the existing contacts to define planes and then comparing the new contact against those planes.
            //Since we already have the penetration depths, we don't need to use the positions of the contacts.
            foreach (var c in character.SupportFinder.SideContacts)
            {
                float dot = Vector3.Dot(contact.Normal, c.Contact.Normal);
                float depth = dot * c.Contact.PenetrationDepth;
                if (depth > c.Contact.PenetrationDepth)
                    return true;

            }
            return false;
        }

        bool HasSupports(RawList<ContactData> supportContacts, RawList<ContactData> tractionContacts, out bool hasTraction, out PositionState state, out ContactData supportContact)
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
            hasTraction = true;
            return true;

        }

        RawList<ContactData> contacts = new RawList<ContactData>();
        RawList<ContactData> supportContacts = new RawList<ContactData>();
        RawList<ContactData> tractionContacts = new RawList<ContactData>();
        RawList<ContactData> sideContacts = new RawList<ContactData>();
        RawList<ContactData> headContacts = new RawList<ContactData>();

        public bool TryToStepDown(out Vector3 newPosition)
        {
            //Don't bother trying to step down if we already have a support contact or if the support ray doesn't have traction.
            if (character.SupportFinder.supports.Count == 0 && character.SupportFinder.SupportRayData != null && character.SupportFinder.SupportRayData.Value.HasTraction &&
                character.SupportFinder.SupportRayData.Value.HitData.T - character.SupportFinder.RayLengthToBottom > MinimumDownStepHeight) //Don't do expensive stuff if it's, at most, a super tiny step that gravity will take care of.
            {
                //Predict a hit location based on the time of impact and the normal at the intersection.
                //Take into account the radius of the character (don't forget the collision margin!)
                Vector3 normal = character.SupportFinder.SupportRayData.Value.HitData.Normal;

                Vector3 down = character.Body.OrientationMatrix.Down;

                RigidTransform transform = character.Body.CollisionInformation.WorldTransform;

                //We know that the closest point to the plane will be the extreme point in the plane's direction.
                //Use it as the ray origin.
                Ray ray;
                character.Body.CollisionInformation.Shape.GetExtremePoint(normal, ref transform, out ray.Position);
                ray.Direction = down;

                //Intersect the ray against the plane defined by the support hit.
                Vector3 intersection;
                Plane plane = new Plane(normal, Vector3.Dot(character.SupportFinder.SupportRayData.Value.HitData.Location, normal));
                Vector3 candidatePosition;

                //Define the interval bounds to be used later.

                //The words 'highest' and 'lowest' here refer to the position relative to the character's body.
                //The ray cast points downward relative to the character's body.
                float highestBound = 0;
                float lowestBound = CollisionDetectionSettings.AllowedPenetration + character.SupportFinder.SupportRayData.Value.HitData.T - character.SupportFinder.RayLengthToBottom;
                float currentOffset = lowestBound;
                float hintOffset;

                //This guess may either win immediately, or at least give us a better idea of where to search.
                if (Toolbox.GetRayPlaneIntersection(ref ray, ref plane, out currentOffset, out intersection))
                {
                    if (currentOffset > lowestBound)
                    {
                        Debug.WriteLine("Breka.");
                    }
                    candidatePosition = character.Body.Position + down * currentOffset;
                    switch (TryPosition(ref candidatePosition, out hintOffset))
                    {
                        case PositionState.Accepted:
                            currentOffset += hintOffset;
                            //Only use the new position location if the movement distance was the right size.
                            if (currentOffset > MinimumDownStepHeight && currentOffset < MaximumStepHeight)
                            {
                                newPosition = character.Body.Position + currentOffset * down;
                                return true;
                            }
                            else
                            {
                                newPosition = new Vector3();
                                return false;
                            }
                        case PositionState.NoHit:
                            highestBound = currentOffset + hintOffset;
                            currentOffset = (lowestBound + currentOffset) * .5f;
                            break;
                        case PositionState.Obstructed:
                            lowestBound = currentOffset;
                            currentOffset = (highestBound + currentOffset) * .5f;
                            break;
                        case PositionState.TooDeep:
                            currentOffset += hintOffset;
                            lowestBound = currentOffset;
                            break;
                    }

                }

                //Our guesses failed.
                //Begin the regular process.  Start at the time of impact of the ray itself.
                //How about trying the time of impact of the ray itself?

                //Since we wouldn't be here unless there were no contacts at the body's current position,
                //testing the ray cast location gives us the second bound we need to do an informed binary search.



                int attempts = 0;
                //Don't keep querying indefinitely.  If we fail to reach it in a few informed steps, it's probably not worth continuing.
                //The bound size check prevents the system from continuing to search a meaninglessly tiny interval.
                while (attempts++ < 5 && lowestBound - highestBound > Toolbox.BigEpsilon)
                {
                    candidatePosition = character.Body.Position + currentOffset * down;
                    switch (TryPosition(ref candidatePosition, out hintOffset))
                    {
                        case PositionState.Accepted:
                            currentOffset += hintOffset;
                            //Only use the new position location if the movement distance was the right size.
                            if (currentOffset > MinimumDownStepHeight && currentOffset < MaximumStepHeight)
                            {
                                newPosition = character.Body.Position + currentOffset * down;
                                return true;
                            }
                            else
                            {
                                newPosition = new Vector3();
                                return false;
                            }
                        case PositionState.NoHit:
                            highestBound = currentOffset + hintOffset;
                            currentOffset = (lowestBound + highestBound) * .5f;
                            break;
                        case PositionState.Obstructed:
                            lowestBound = currentOffset;
                            currentOffset = (highestBound + lowestBound) * .5f;
                            break;
                        case PositionState.TooDeep:
                            currentOffset += hintOffset;
                            lowestBound = currentOffset;
                            break;
                    }
                }
                //Couldn't find a candidate.
                newPosition = new Vector3();
                return false;


            }
            else
            {
                newPosition = new Vector3();
                return false;
            }
        }

        PositionState TryPosition(ref Vector3 position, out float hintOffset)
        {
            hintOffset = 0;
            ClearContacts();
            QueryContacts(position, contacts);
            CategorizeContacts(contacts, supportContacts, tractionContacts, headContacts, sideContacts);
            bool hasTraction;
            PositionState supportState;
            ContactData supportContact;
            bool obstructed = IsObstructed(sideContacts);
            if (HasSupports(supportContacts, tractionContacts, out hasTraction, out supportState, out supportContact) && !obstructed)
            {
                if (supportState == PositionState.Accepted)
                {
                    //We're done! The guess found a good spot to stand on.
                    //The final state doesn't need to actually create contacts, so shove it up 
                    //just barely to the surface.
                    hintOffset = -Vector3.Dot(supportContact.Normal, character.Body.OrientationMatrix.Down) * supportContact.PenetrationDepth;
                    return PositionState.Accepted;
                }
                else if (supportState == PositionState.TooDeep)
                {
                    //Looks like we have to keep trying, but at least we found a good hint.
                    hintOffset = -.001f - Vector3.Dot(supportContact.Normal, character.Body.OrientationMatrix.Down) * supportContact.PenetrationDepth;
                    return PositionState.TooDeep;
                }
                else //if (supportState == SupportState.Separated)
                {
                    //It's not obstructed, but the support isn't quite right.
                    //It's got a negative penetration depth.
                    //We can use that as a hint.
                    hintOffset = -.001f - Vector3.Dot(supportContact.Normal, character.Body.OrientationMatrix.Down) * supportContact.PenetrationDepth;
                    return PositionState.NoHit;
                }
            }
            else if (obstructed)
            {
                return PositionState.Obstructed;
            }
            else
            {
                return PositionState.NoHit;
            }
        }

        private void ClearContacts()
        {
            contacts.Clear();
            supportContacts.Clear();
            tractionContacts.Clear();
            sideContacts.Clear();
            headContacts.Clear();
        }

        enum PositionState
        {
            Accepted,
            TooDeep,
            Obstructed,
            NoHit
        }

    }
}
