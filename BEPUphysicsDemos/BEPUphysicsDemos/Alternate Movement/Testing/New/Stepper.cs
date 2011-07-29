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
using BEPUphysics.CollisionRuleManagement;

namespace BEPUphysicsDemos.AlternateMovement.Testing.New
{
    public class Stepper
    {
        CharacterController character;
        public float MaximumStepHeight = 1;
        public float MinimumDownStepHeight = .1f;
        public float MinimumUpStepHeight = .1f;

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
                    pairHandler.CleanUp();
                    (pairHandler as INarrowPhasePair).Factory.GiveBack(pairHandler);
                }
            }

        }

        public void CategorizeContacts(ref Vector3 position, RawList<ContactData> contacts, RawList<ContactData> outputSupports, RawList<ContactData> outputTraction, RawList<ContactData> outputHeadContacts, RawList<ContactData> outputSideContacts)
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
                    outputSupports.Add(processed);
                    if (dot > character.SupportFinder.cosMaximumSlope)
                    {
                        //It's a traction contact.
                        outputTraction.Add(processed);
                    }
                    else
                        sideContacts.Add(processed); //Considering the side contacts to be supports can help with upstepping.
                }
                else if (dot < -SupportFinder.SideContactThreshold)
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

        public bool IsDownStepObstructed(RawList<ContactData> sideContacts)
        {
            //A contact is considered obstructive if its projected depth is deeper than any existing contact along the existing contacts' normals.
            for (int i = 0; i < sideContacts.Count; i++)
            {
                if (IsObstructiveToDownStepping(ref sideContacts.Elements[i]))
                    return true;
            }
            return false;
        }

        bool IsObstructiveToDownStepping(ref ContactData contact)
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
        RawList<ContactData> stepContacts = new RawList<ContactData>();

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
                float hitT;
                if (Toolbox.GetRayPlaneIntersection(ref ray, ref plane, out hitT, out intersection))
                {
                    currentOffset = hitT + CollisionDetectionSettings.AllowedPenetration;
                    candidatePosition = character.Body.Position + down * currentOffset;
                    switch (TryDownStepPosition(ref candidatePosition, out hintOffset))
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
                    switch (TryDownStepPosition(ref candidatePosition, out hintOffset))
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

        PositionState TryDownStepPosition(ref Vector3 position, out float hintOffset)
        {
            hintOffset = 0;
            ClearContacts();
            QueryContacts(position, contacts);
            CategorizeContacts(ref position, contacts, supportContacts, tractionContacts, headContacts, sideContacts);
            bool hasTraction;
            PositionState supportState;
            ContactData supportContact;
            bool obstructed = IsDownStepObstructed(sideContacts);
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
                    hintOffset = Math.Min(0, .001f - Vector3.Dot(supportContact.Normal, character.Body.OrientationMatrix.Down) * supportContact.PenetrationDepth);
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

        public bool TryToStepUp(out Vector3 newPosition)
        {
            //Can't step up if we don't have traction to begin with.
            if (character.SupportFinder.HasTraction)
            {
                //Further, we must test to see if the character has a side contact which is suitable for climbing.
                stepContacts.Clear();
                FindUpStepCandidates(stepContacts);
                //We must test every such contact until we find a step up or we run out of candidates.
                for (int i = 0; i < stepContacts.Count; i++)
                {
                    if (TryToStepUsingContact(ref stepContacts.Elements[i], out newPosition))
                    {
                        return true;
                    }
                }

            }
            newPosition = new Vector3();
            return false;
        }

        float upStepMargin = .1f;  //There's a little extra space above the maximum step height to start the obstruction and downcast test rays.  Helps when a step is very close to the max step height.
        void FindUpStepCandidates(RawList<ContactData> outputStepCandidates)
        {
            foreach (var c in character.SupportFinder.sideContacts)
            {
                //A 6DOF character will need to have a 3d movement direction.  It will replace this graduation of a 2d vector.
                Vector3 movementDirection = new Vector3()
                {
                    X = character.HorizontalMotionConstraint.MovementDirection.X,
                    Z = character.HorizontalMotionConstraint.MovementDirection.Y
                };
                //Check to see if the contact is sufficiently aligned with the movement direction to be considered for stepping.
                //TODO: This could behave a bit odd when encountering steps or slopes near the base of rounded collision margin.
                var contact = c.Contact;
                float dot;
                Vector3.Dot(ref contact.Normal, ref movementDirection, out dot);
                if (dot > 0)
                {
                    //It is! But is it low enough?
                    dot = Vector3.Dot(character.Body.OrientationMatrix.Down, c.Contact.Position - character.Body.Position);
                    //It must be between the bottom of the character and the maximum step height.
                    if (dot < character.Body.Height * .5f && dot > character.Body.Height * .5f - MaximumStepHeight - upStepMargin)
                    {
                        //It's a candidate!
                        //But wait, there's more! Do we already have a candidate that covers this direction?
                        bool shouldAdd = true;
                        for (int i = 0; i < outputStepCandidates.Count; i++)
                        {
                            Vector3.Dot(ref outputStepCandidates.Elements[i].Normal, ref contact.Normal, out dot);
                            if (dot > .99f)
                            {
                                shouldAdd = false; //Woops! This direction is already covered.  Don't bother.
                                break;
                            }
                        }
                        if (shouldAdd)
                            outputStepCandidates.Add(contact);
                    }
                }
            }

        }

        bool TryToStepUsingContact(ref ContactData contact, out Vector3 newPosition)
        {
            Vector3 down = character.Body.OrientationMatrix.Down;
            Vector3 position = character.Body.Position;
            //The normal of the contact may not be facing perfectly out to the side.
            //The detection process allows a bit of slop.
            //Correct it by removing any component of the normal along the local up vector.
            Vector3 normal = contact.Normal;
            float dot;
            Vector3.Dot(ref normal, ref down, out dot);
            Vector3 error;
            Vector3.Multiply(ref down, dot, out error);
            Vector3.Subtract(ref normal, ref error, out normal);
            normal.Normalize();

            //Now we need to ray cast out from the center of the character in the direction of this normal to check for obstructions.
            //Compute the ray origin location.  Fire it out of the top of the character; if we're stepping, this must be a valid location.
            //Putting it as high as possible helps to reject more invalid step geometry.
            Ray ray;
            float downRayLength = character.Body.Height;// MaximumStepHeight + upStepMargin;
            Vector3.Multiply(ref down, character.Body.Height * .5f - downRayLength, out ray.Position);
            Vector3.Add(ref ray.Position, ref position, out ray.Position);
            ray.Direction = normal;
            //Include a little margin in the length.
            //Technically, the character only needs to teleport horizontally by the complicated commented expression.
            //That puts it just far enough to have traction on the new surface.
            //In practice, the current contact refreshing approach used for many pair types causes contacts to persist horizontally a bit,
            //which can cause side effects for the character.
            float horizontalOffsetAmount = character.Body.CollisionInformation.Shape.CollisionMargin;// (float)((1 - character.SupportFinder.sinMaximumSlope) * character.Body.CollisionInformation.Shape.CollisionMargin + 0);
            float length = character.Body.Radius + horizontalOffsetAmount - contact.PenetrationDepth;

            foreach (var collidable in character.Body.CollisionInformation.OverlappedCollidables)
            {
                //Check to see if the collidable is hit by the ray.
                float? t = ray.Intersects(collidable.BoundingBox);
                if (t != null && t < length)
                {
                    RayHit hit;
                    if (collidable.RayCast(ray, length, character.SupportFinder.SupportRayFilter, out hit))
                    {
                        //The step is obstructed!
                        newPosition = new Vector3();
                        return false;
                    }
                }
            }

            //The down-cast ray origin has been verified by the previous ray cast.
            //Let's look for a support!
            Vector3 horizontalOffset;
            Vector3.Multiply(ref normal, length, out horizontalOffset);
            Vector3.Add(ref ray.Position, ref horizontalOffset, out ray.Position);
            ray.Direction = down;

            //Find the earliest hit, if any.
            RayHit earliestHit = new RayHit();
            earliestHit.T = float.MaxValue;
            foreach (var collidable in character.Body.CollisionInformation.OverlappedCollidables)
            {
                //Check to see if the collidable is hit by the ray.
                float? t = ray.Intersects(collidable.BoundingBox);
                if (t != null && t < downRayLength)
                {
                    //Is it an earlier hit than the current earliest?
                    RayHit hit;
                    if (collidable.RayCast(ray, downRayLength, character.SupportFinder.SupportRayFilter, out hit) && hit.T < earliestHit.T)
                    {
                        earliestHit = hit;
                    }
                }
            }
            if (earliestHit.T <= 0 || //Can't do anything if the hit was invalid.
                earliestHit.T == float.MaxValue || //Can't do anything if nothing was hit.
                earliestHit.T - downRayLength > -MinimumUpStepHeight || //Don't bother doing anything if the step is too small.
                earliestHit.T - downRayLength < -MaximumStepHeight - upStepMargin) //Can't do anything if the step is too tall.
            {
                //No valid hit was detected.
                newPosition = new Vector3();
                return false;
            }

            //Since contact queries are frequently expensive compared to ray cast tests,
            //do one more ray cast test.  This time, starting from the same position, cast upwards.
            //In order to step up, the previous down-ray hit must be at least a character height away from the result of the up-ray.
            Vector3.Negate(ref down, out ray.Direction);
            //Find the earliest hit, if any.
            RayHit earliestHitUp = new RayHit();
            earliestHitUp.T = float.MaxValue;
            float upLength = character.Body.Height - earliestHit.T;
            foreach (var collidable in character.Body.CollisionInformation.OverlappedCollidables)
            {
                //Check to see if the collidable is hit by the ray.
                float? t = ray.Intersects(collidable.BoundingBox);
                if (t != null && t < upLength)
                {
                    //Is it an earlier hit than the current earliest?
                    RayHit hit;
                    if (collidable.RayCast(ray, upLength, character.SupportFinder.SupportRayFilter, out hit) && hit.T < earliestHitUp.T)
                    {
                        earliestHitUp = hit;
                    }
                }
            }
            //If the sum of the up and down distances is less than the height, the character can't fit.
            if (earliestHitUp.T != float.MaxValue && earliestHitUp.T + earliestHit.T < character.Body.Height)
            {
                newPosition = new Vector3();
                return false;
            }

            //By now, a valid ray hit has been found.  Now we need to validate it using contact queries.
            //This process is very similar in concept to the down step verification, but it has some extra
            //requirements.

            //Predict a hit location based on the time of impact and the normal at the intersection.
            //Take into account the radius of the character (don't forget the collision margin!)
            Vector3 supportNormal;
            Vector3.Normalize(ref earliestHit.Normal, out supportNormal);
            //Calibrate the normal to face in the same direction as the down vector for consistency.
            Vector3.Dot(ref supportNormal, ref down, out dot);
            if (dot < 0)
            {
                Vector3.Negate(ref supportNormal, out supportNormal);
                dot = -dot;
            }

            //If the new surface does not have traction, do not attempt to step up.
            if (dot < character.SupportFinder.cosMaximumSlope)
            {
                newPosition = new Vector3();
                return false;
            }


            RigidTransform transform = character.Body.CollisionInformation.WorldTransform;
            //The transform must be modified to position the query body at the right location.
            //The horizontal offset of the queries ensures that a tractionable part of the character will be put onto the new support.
            Vector3.Multiply(ref normal, horizontalOffsetAmount, out horizontalOffset);
            Vector3.Add(ref transform.Position, ref horizontalOffset, out transform.Position);
            Vector3 verticalOffset;
            Vector3.Multiply(ref down, -downRayLength, out verticalOffset);
            Vector3.Add(ref transform.Position, ref verticalOffset, out transform.Position);

            //We know that the closest point to the plane will be the extreme point in the plane's direction.
            //Use it as the ray origin.
            Ray downRay;
            character.Body.CollisionInformation.Shape.GetExtremePoint(supportNormal, ref transform, out downRay.Position);
            downRay.Direction = down;

            //Intersect the ray against the plane defined by the support hit.
            Vector3 intersection;
            Vector3.Dot(ref earliestHit.Location, ref supportNormal, out dot);
            Plane plane = new Plane(supportNormal, dot);
            Vector3 candidatePosition;

            //Define the interval bounds to be used later.

            //The words 'highest' and 'lowest' here refer to the position relative to the character's body.
            //The ray cast points downward relative to the character's body.
            float highestBound = -MaximumStepHeight;
            float lowestBound = CollisionDetectionSettings.AllowedPenetration - downRayLength + earliestHit.T;
            float currentOffset = lowestBound;
            float hintOffset;



            //This guess may either win immediately, or at least give us a better idea of where to search.
            float hitT;
            if (Toolbox.GetRayPlaneIntersection(ref downRay, ref plane, out hitT, out intersection))
            {
                hitT = -downRayLength + hitT + CollisionDetectionSettings.AllowedPenetration;
                if (hitT < highestBound)
                {
                    //Don't try a location known to be too high.
                    hitT = highestBound;
                }
                currentOffset = hitT;
                if (currentOffset > lowestBound)
                    lowestBound = currentOffset;
                candidatePosition = character.Body.Position + down * currentOffset + horizontalOffset;
                switch (TryUpStepPosition(ref normal, ref candidatePosition, out hintOffset))
                {
                    case PositionState.Accepted:
                        currentOffset += hintOffset;
                        //Only use the new position location if the movement distance was the right size.
                        if (currentOffset < 0 && currentOffset > -MaximumStepHeight - CollisionDetectionSettings.AllowedPenetration)
                        {
                            //It's possible that we let a just-barely-too-high step occur, limited by the allowed penetration.
                            //Just clamp the overall motion and let it penetrate a bit.
                            newPosition = character.Body.Position + Math.Max(-MaximumStepHeight, currentOffset) * down + horizontalOffset;
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
                    case PositionState.HeadObstructed:
                        highestBound = currentOffset + hintOffset;
                        currentOffset = (lowestBound + currentOffset) * .5f;
                        break;
                    case PositionState.TooDeep:
                        currentOffset += hintOffset;
                        lowestBound = currentOffset;
                        break;

                }

            }//TODO: If the ray cast doesn't hit, that could be used to early out...  Then again, it pretty much can't happen.

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
                candidatePosition = character.Body.Position + currentOffset * down + horizontalOffset;
                switch (TryUpStepPosition(ref normal, ref candidatePosition, out hintOffset))
                {
                    case PositionState.Accepted:
                        currentOffset += hintOffset;
                        //Only use the new position location if the movement distance was the right size.
                        if (currentOffset < 0 && currentOffset > -MaximumStepHeight - CollisionDetectionSettings.AllowedPenetration)
                        {
                            //It's possible that we let a just-barely-too-high step occur, limited by the allowed penetration.
                            //Just clamp the overall motion and let it penetrate a bit.
                            newPosition = character.Body.Position + Math.Max(-MaximumStepHeight, currentOffset) * down + horizontalOffset;
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
                    case PositionState.HeadObstructed:
                        highestBound = currentOffset + hintOffset;
                        currentOffset = (lowestBound + currentOffset) * .5f;
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


        PositionState TryUpStepPosition(ref Vector3 sideNormal, ref Vector3 position, out float hintOffset)
        {
            hintOffset = 0;
            ClearContacts();
            QueryContacts(position, contacts);
            CategorizeContacts(ref position, contacts, supportContacts, tractionContacts, headContacts, sideContacts);
            bool hasTraction;
            PositionState supportState;
            ContactData supportContact;
            if (headContacts.Count > 0)
            {
                //The head is obstructed.  This will define a maximum bound.
                //Find the deepest contact on the head and use it to provide a hint.
                Vector3 up = character.Body.OrientationMatrix.Up;
                float dot;
                Vector3.Dot(ref up, ref headContacts.Elements[0].Normal, out dot);
                hintOffset = dot * headContacts.Elements[0].PenetrationDepth;
                for (int i = 1; i < headContacts.Count; i++)
                {
                    Vector3.Dot(ref up, ref headContacts.Elements[i].Normal, out dot);
                    dot *= headContacts.Elements[i].PenetrationDepth;
                    if (dot > hintOffset)
                    {
                        hintOffset = dot;
                    }
                }
                return PositionState.HeadObstructed;
            }
            bool obstructed = IsUpStepObstructed(ref sideNormal, sideContacts, headContacts);
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
                    hintOffset = Math.Min(0, .001f - Vector3.Dot(supportContact.Normal, character.Body.OrientationMatrix.Down) * supportContact.PenetrationDepth);
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

        public bool IsUpStepObstructed(ref Vector3 sideNormal, RawList<ContactData> sideContacts, RawList<ContactData> headContacts)
        {
            //A contact is considered obstructive if its projected depth is deeper than any existing contact along the existing contacts' normals.
            for (int i = 0; i < sideContacts.Count; i++)
            {
                if (IsObstructiveToUpStepping(ref sideNormal, ref sideContacts.Elements[i]))
                    return true;
            }
            return false;
        }

        bool IsObstructiveToUpStepping(ref Vector3 sideNormal, ref ContactData contact)
        {
            //Up stepping has slightly different rules than down stepping.
            //For contacts with normals aligned with the side normal that triggered the step,
            //only marginal (allowed penetration) obstruction is permitted.
            //Consider the side normal to define an implicit plane.
            float dot;
            Vector3.Dot(ref contact.Normal, ref sideNormal, out dot);
            if (dot * contact.PenetrationDepth > CollisionDetectionSettings.AllowedPenetration)
            {
                //It's too deep! Can't step.
                return true;
            }

            //Go through side-facing contact and check to see if the new contact is deeper than any existing contact in the direction of the existing contact.
            //This is equivalent to considering the existing contacts to define planes and then comparing the new contact against those planes.
            //Since we already have the penetration depths, we don't need to use the positions of the contacts.
            foreach (var c in character.SupportFinder.SideContacts)
            {
                dot = Vector3.Dot(contact.Normal, c.Contact.Normal);
                float depth = dot * c.Contact.PenetrationDepth;
                if (depth > c.Contact.PenetrationDepth)
                    return true;

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

        enum PositionState
        {
            Accepted,
            TooDeep,
            Obstructed,
            HeadObstructed,
            NoHit
        }

    }
}
