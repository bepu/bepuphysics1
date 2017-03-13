using System;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;
using BEPUutilities.DataStructures;
using BEPUphysics.CollisionTests;
using BEPUphysics.Settings;
using BEPUutilities.ResourceManagement;

namespace BEPUphysics.Character
{
    /// <summary>
    /// Checks to see if a character is capable of stepping up or down onto a new support object.
    /// </summary>
    public class StepManager
    {
        private Cylinder characterBody;
        ConvexCollidable<CylinderShape> currentQueryObject;

        float maximumStepHeight = 1f;
        /// <summary>
        /// Gets or sets the maximum height which the character is capable of stepping up or down onto.
        /// </summary>
        public float MaximumStepHeight
        {
            get
            {
                return maximumStepHeight;
            }
            set
            {
                if (maximumStepHeight < 0)
                    throw new ArgumentException("Value must be nonnegative.");
                maximumStepHeight = value;
            }
        }
        float minimumDownStepHeight = .1f;
        /// <summary>
        /// Gets or sets the minimum down step height.  Down steps which are smaller than this are simply ignored by the step system; instead, the character falls.
        /// If the new step location has traction, the intermediate falling will not remove traction from the character.  The only difference is that the character isn't
        /// teleported down when the step is too small.
        /// </summary>
        public float MinimumDownStepHeight
        {
            get
            {
                return minimumDownStepHeight;
            }
            set
            {
                if (minimumDownStepHeight < 0)
                    throw new ArgumentException("Value must be nonnegative.");
                minimumDownStepHeight = value;
            }
        }
        float minimumUpStepHeight;

        private SupportFinder SupportFinder { get; set; }
        private QueryManager QueryManager { get; set; }
        private CharacterContactCategorizer ContactCategorizer { get; set; }
        private HorizontalMotionConstraint HorizontalMotionConstraint { get; set; }

        /// <summary>
        /// Constructs a new step manager for a character.
        /// </summary>
        /// <param name="characterBody">The character's body.</param>
        /// <param name="contactCategorizer">Contact categorizer used by the character.</param>
        /// <param name="supportFinder">Support finder used by the character.</param>
        /// <param name="queryManager">Query provider to use in checking for obstructions.</param>
        /// <param name="horizontalMotionConstraint">Horizontal motion constraint used by the character. Source of 3d movement direction.</param>
        public StepManager(Cylinder characterBody, CharacterContactCategorizer contactCategorizer, SupportFinder supportFinder, QueryManager queryManager, HorizontalMotionConstraint horizontalMotionConstraint)
        {
            this.characterBody = characterBody;
            currentQueryObject = new ConvexCollidable<CylinderShape>(characterBody.CollisionInformation.Shape);
            ContactCategorizer = contactCategorizer;
            SupportFinder = supportFinder;
            QueryManager = queryManager;
            HorizontalMotionConstraint = horizontalMotionConstraint;
            //The minimum step height is just barely above where the character would generally find the ground.
            //This helps avoid excess tests.
            minimumUpStepHeight = CollisionDetectionSettings.AllowedPenetration * 1.1f;// Math.Max(0, -.01f + character.Body.CollisionInformation.Shape.CollisionMargin * (1 - character.SupportFinder.sinMaximumSlope));

        }

        bool IsDownStepObstructed(ref QuickList<CharacterContact> sideContacts)
        {
            //A contact is considered obstructive if its projected depth is deeper than any existing contact along the existing contacts' normals.
            for (int i = 0; i < sideContacts.Count; i++)
            {
                if (SupportFinder.IsSideContactObstructive(ref sideContacts.Elements[i].Contact))
                    return true;
            }
            return false;
        }
        


        RawList<ContactData> stepContacts = new RawList<ContactData>();

        /// <summary>
        /// Determines if a down step is possible, and if so, computes the location to which the character should teleport.
        /// </summary>
        /// <param name="newPosition">New position the character should teleport to if the down step is accepted.</param>
        /// <returns>Whether or not the character should attempt to step down.</returns>
        public bool TryToStepDown(out Vector3 newPosition)
        {
            //Don't bother trying to step down if we already have a support contact or if the support ray doesn't have traction.
            if (!(SupportFinder.Supports.Count == 0 && SupportFinder.SupportRayData != null && SupportFinder.SupportRayData.Value.HasTraction))
            {
                newPosition = new Vector3();
                return false;
            }
            if (!(SupportFinder.SupportRayData.Value.HitData.T - SupportFinder.BottomDistance > minimumDownStepHeight)) //Don't do expensive stuff if it's, at most, a super tiny step that gravity will take care of.
            {
                newPosition = new Vector3();
                return false;
            }
            //Predict a hit location based on the time of impact and the normal at the intersection.
            //Take into account the radius of the character (don't forget the collision margin!)
            Vector3 normal = SupportFinder.SupportRayData.Value.HitData.Normal;

            Vector3 down = characterBody.orientationMatrix.Down;

            RigidTransform transform = characterBody.CollisionInformation.WorldTransform;

            //We know that the closest point to the plane will be the extreme point in the plane's direction.
            //Use it as the ray origin.
            Ray ray;
            characterBody.CollisionInformation.Shape.GetExtremePoint(normal, ref transform, out ray.Position);
            ray.Direction = down;

            //Intersect the ray against the plane defined by the support hit.
            Vector3 intersection;
            Plane plane = new Plane(normal, Vector3.Dot(SupportFinder.SupportRayData.Value.HitData.Location, normal));
            Vector3 candidatePosition;

            //Define the interval bounds to be used later.

            //The words 'highest' and 'lowest' here refer to the position relative to the character's body.
            //The ray cast points downward relative to the character's body.
            float highestBound = 0;
            //The lowest possible distance is the ray distance plus the collision margin because the ray could theoretically be on the outskirts of the collision margin
            //where the shape would actually have to move more than the bottom distance difference would imply.
            //(Could compute the true lowest bound analytically based on the horizontal position of the ray...)
            float lowestBound = characterBody.CollisionInformation.Shape.CollisionMargin + SupportFinder.SupportRayData.Value.HitData.T - SupportFinder.BottomDistance;
            float currentOffset = lowestBound;
            float hintOffset;

            var tractionContacts = new QuickList<CharacterContact>(BufferPools<CharacterContact>.Thread);
            var supportContacts = new QuickList<CharacterContact>(BufferPools<CharacterContact>.Thread);
            var sideContacts = new QuickList<CharacterContact>(BufferPools<CharacterContact>.Thread);
            var headContacts = new QuickList<CharacterContact>(BufferPools<CharacterContact>.Thread);
            try
            {

                //This guess may either win immediately, or at least give us a better idea of where to search.
                float hitT;
                if (Toolbox.GetRayPlaneIntersection(ref ray, ref plane, out hitT, out intersection))
                {
                    currentOffset = hitT + CollisionDetectionSettings.AllowedPenetration * 0.5f;
                    candidatePosition = characterBody.Position + down * currentOffset;
                    switch (TryDownStepPosition(ref candidatePosition, ref down,
                                                ref tractionContacts, ref supportContacts, ref sideContacts, ref headContacts,
                                                out hintOffset))
                    {
                        case CharacterContactPositionState.Accepted:
                            currentOffset += hintOffset;
                            //Only use the new position location if the movement distance was the right size.
                            if (currentOffset > minimumDownStepHeight && currentOffset < maximumStepHeight)
                            {
                                newPosition = characterBody.Position + currentOffset * down;
                                return true;
                            }
                            else
                            {
                                newPosition = new Vector3();
                                return false;
                            }
                        case CharacterContactPositionState.NoHit:
                            highestBound = currentOffset + hintOffset;
                            currentOffset = (lowestBound + currentOffset) * .5f;
                            break;
                        case CharacterContactPositionState.Obstructed:
                            lowestBound = currentOffset;
                            currentOffset = (highestBound + currentOffset) * .5f;
                            break;
                        case CharacterContactPositionState.TooDeep:
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
                    candidatePosition = characterBody.Position + currentOffset * down;
                    switch (TryDownStepPosition(ref candidatePosition, ref down, ref tractionContacts, ref supportContacts, ref sideContacts, ref headContacts, out hintOffset))
                    {
                        case CharacterContactPositionState.Accepted:
                            currentOffset += hintOffset;
                            //Only use the new position location if the movement distance was the right size.
                            if (currentOffset > minimumDownStepHeight && currentOffset < maximumStepHeight)
                            {
                                newPosition = characterBody.Position + currentOffset * down;
                                return true;
                            }
                            else
                            {
                                newPosition = new Vector3();
                                return false;
                            }
                        case CharacterContactPositionState.NoHit:
                            highestBound = currentOffset + hintOffset;
                            currentOffset = (lowestBound + highestBound) * .5f;
                            break;
                        case CharacterContactPositionState.Obstructed:
                            lowestBound = currentOffset;
                            currentOffset = (highestBound + lowestBound) * .5f;
                            break;
                        case CharacterContactPositionState.TooDeep:
                            currentOffset += hintOffset;
                            lowestBound = currentOffset;
                            break;
                    }
                }
                //Couldn't find a candidate.
                newPosition = new Vector3();
                return false;
            }
            finally
            {
                tractionContacts.Dispose();
                supportContacts.Dispose();
                sideContacts.Dispose();
                headContacts.Dispose();
            }


        }

        CharacterContactPositionState TryDownStepPosition(ref Vector3 position, ref Vector3 down,
            ref QuickList<CharacterContact> tractionContacts, ref QuickList<CharacterContact> supportContacts, ref QuickList<CharacterContact> sideContacts, ref QuickList<CharacterContact> headContacts,
            out float hintOffset)
        {
            hintOffset = 0;
            PrepareQueryObject(ref position);
            QueryManager.QueryContacts(currentQueryObject, ref tractionContacts, ref supportContacts, ref sideContacts, ref headContacts);
            if (IsDownStepObstructed(ref sideContacts))
            {
                return CharacterContactPositionState.Obstructed;
            }
            //Note the use of traction contacts. We only want to step down if there are traction contacts to support us.
            if (tractionContacts.Count > 0)
            {
                CharacterContactPositionState supportState;
                CharacterContact supportContact;
                QueryManager.AnalyzeSupportState(ref tractionContacts, ref supportContacts, out supportState, out supportContact);
                if (supportState == CharacterContactPositionState.Accepted)
                {
                    //We're done! The guess found a good spot to stand on.
                    //The final state doesn't need to actually create contacts, so shove it up 
                    //just barely to the surface.
                    hintOffset = -Vector3.Dot(supportContact.Contact.Normal, down) * supportContact.Contact.PenetrationDepth;
                    return CharacterContactPositionState.Accepted;
                }
                else if (supportState == CharacterContactPositionState.TooDeep)
                {
                    //Looks like we have to keep trying, but at least we found a good hint.
                    hintOffset = Math.Min(0, .001f - Vector3.Dot(supportContact.Contact.Normal, down) * supportContact.Contact.PenetrationDepth);
                    return CharacterContactPositionState.TooDeep;
                }
                else //if (supportState == SupportState.Separated)
                {
                    //It's not obstructed, but the support isn't quite right.
                    //It's got a negative penetration depth.
                    //We can use that as a hint.
                    hintOffset = -.001f - Vector3.Dot(supportContact.Contact.Normal, down) * supportContact.Contact.PenetrationDepth;
                    return CharacterContactPositionState.NoHit;
                }
            }
            else
            {
                return CharacterContactPositionState.NoHit;
            }
        }

        /// <summary>
        /// Determines if an up step is possible, and if so, computes the location to which the character should teleport.
        /// </summary>
        /// <param name="newPosition">New position the character should teleport to if the up step is accepted.</param>
        /// <returns>Whether or not the character should attempt to step up.</returns>
        public bool TryToStepUp(out Vector3 newPosition)
        {
            //Can't step up if we don't have traction to begin with.
            var down = characterBody.orientationMatrix.Down;
            if (SupportFinder.HasTraction)
            {
                //Further, we must test to see if the character has a side contact which is suitable for climbing.
                stepContacts.Clear();
                FindUpStepCandidates(stepContacts, ref down);
                //We must test every such contact until we find a step up or we run out of candidates.
                for (int i = 0; i < stepContacts.Count; i++)
                {
                    if (TryToStepUsingContact(ref stepContacts.Elements[i], ref down, out newPosition))
                    {
                        return true;
                    }
                }

            }
            newPosition = new Vector3();
            return false;
        }

        float upStepMargin = .1f;  //There's a little extra space above the maximum step height to start the obstruction and downcast test rays.  Helps when a step is very close to the max step height.
        void FindUpStepCandidates(RawList<ContactData> outputStepCandidates, ref Vector3 down)
        {
            Vector3 movementDirection = HorizontalMotionConstraint.MovementDirection3d;
            foreach (var c in SupportFinder.SideContacts)
            {
                //Check to see if the contact is sufficiently aligned with the movement direction to be considered for stepping.
                //TODO: This could behave a bit odd when encountering steps or slopes near the base of rounded collision margin.
                var contact = c.Contact;
                float dot;
                Vector3.Dot(ref contact.Normal, ref movementDirection, out dot);
                if (dot > 0)
                {
                    //It is! But is it low enough?
                    dot = Vector3.Dot(down, c.Contact.Position - characterBody.Position);
                    //It must be between the bottom of the character and the maximum step height.
                    if (dot < characterBody.Height * .5f && dot > characterBody.Height * .5f - maximumStepHeight - upStepMargin)
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

        bool TryToStepUsingContact(ref ContactData contact, ref Vector3 down, out Vector3 newPosition)
        {
            Vector3 position = characterBody.Position;
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
            float downRayLength = characterBody.Height;// MaximumStepHeight + upStepMargin;
            Vector3.Multiply(ref down, characterBody.Height * .5f - downRayLength, out ray.Position);
            Vector3.Add(ref ray.Position, ref position, out ray.Position);
            ray.Direction = normal;
            //Include a little margin in the length.
            //Technically, the character only needs to teleport horizontally by the complicated commented expression.
            //That puts it just far enough to have traction on the new surface.
            //In practice, the current contact refreshing approach used for many pair types causes contacts to persist horizontally a bit,
            //which can cause side effects for the character.
            float horizontalOffsetAmount = characterBody.CollisionInformation.Shape.CollisionMargin;// (float)((1 - character.SupportFinder.sinMaximumSlope) * character.Body.CollisionInformation.Shape.CollisionMargin + 0);
            float length = characterBody.Radius + horizontalOffsetAmount;// -contact.PenetrationDepth;


            if (QueryManager.RayCastHitAnything(ray, length))
            {
                //The step is obstructed!
                newPosition = new Vector3();
                return false;
            }

            //The down-cast ray origin has been verified by the previous ray cast.
            //Let's look for a support!
            Vector3 horizontalOffset;
            Vector3.Multiply(ref normal, length, out horizontalOffset);
            Vector3.Add(ref ray.Position, ref horizontalOffset, out ray.Position);
            ray.Direction = down;

            //Find the earliest hit, if any.
            RayHit earliestHit;
            if (!QueryManager.RayCast(ray, downRayLength, out earliestHit) || //Can't do anything if it didn't hit.
                earliestHit.T <= 0 || //Can't do anything if the hit was invalid.
                earliestHit.T - downRayLength > -minimumUpStepHeight || //Don't bother doing anything if the step is too small.
                earliestHit.T - downRayLength < -maximumStepHeight - upStepMargin) //Can't do anything if the step is too tall.
            {
                //No valid hit was detected.
                newPosition = new Vector3();
                return false;
            }

            //Ensure the candidate surface supports traction.
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
            if (dot < ContactCategorizer.TractionThreshold)
            {
                newPosition = new Vector3();
                return false;
            }

            //Since contact queries are frequently expensive compared to ray cast tests,
            //do one more ray cast test.  This time, starting from the same position, cast upwards.
            //In order to step up, the previous down-ray hit must be at least a character height away from the result of the up-ray.
            Vector3.Negate(ref down, out ray.Direction);
            //Find the earliest hit, if any.
            //RayHit earliestHitUp = new RayHit();
            //earliestHitUp.T = float.MaxValue;
            float upLength = characterBody.Height - earliestHit.T;

            //If the sum of the up and down distances is less than the height, the character can't fit.
            if (QueryManager.RayCastHitAnything(ray, upLength))
            {
                newPosition = new Vector3();
                return false;
            }

            //By now, a valid ray hit has been found.  Now we need to validate it using contact queries.
            //This process is very similar in concept to the down step verification, but it has some extra
            //requirements.

            //Predict a hit location based on the time of impact and the normal at the intersection.
            //Take into account the radius of the character (don't forget the collision margin!)



            RigidTransform transform = characterBody.CollisionInformation.WorldTransform;
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
            characterBody.CollisionInformation.Shape.GetExtremePoint(supportNormal, ref transform, out downRay.Position);
            downRay.Direction = down;

            //Intersect the ray against the plane defined by the support hit.
            Vector3 intersection;
            Vector3.Dot(ref earliestHit.Location, ref supportNormal, out dot);
            Plane plane = new Plane(supportNormal, dot);
            Vector3 candidatePosition;

            //Define the interval bounds to be used later.

            //The words 'highest' and 'lowest' here refer to the position relative to the character's body.
            //The ray cast points downward relative to the character's body.
            float highestBound = -maximumStepHeight;
            float lowestBound = characterBody.CollisionInformation.Shape.CollisionMargin - downRayLength + earliestHit.T;
            float currentOffset = lowestBound;
            float hintOffset;

            var tractionContacts = new QuickList<CharacterContact>(BufferPools<CharacterContact>.Thread);
            var supportContacts = new QuickList<CharacterContact>(BufferPools<CharacterContact>.Thread);
            var sideContacts = new QuickList<CharacterContact>(BufferPools<CharacterContact>.Thread);
            var headContacts = new QuickList<CharacterContact>(BufferPools<CharacterContact>.Thread);
            try
            {
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
                    candidatePosition = characterBody.Position + down * currentOffset + horizontalOffset;
                    switch (TryUpStepPosition(ref normal, ref candidatePosition, ref down,
                                              ref tractionContacts, ref supportContacts, ref sideContacts, ref headContacts,
                                              out hintOffset))
                    {
                        case CharacterContactPositionState.Accepted:
                            currentOffset += hintOffset;
                            //Only use the new position location if the movement distance was the right size.
                            if (currentOffset < 0 && currentOffset > -maximumStepHeight - CollisionDetectionSettings.AllowedPenetration)
                            {
                                //It's possible that we let a just-barely-too-high step occur, limited by the allowed penetration.
                                //Just clamp the overall motion and let it penetrate a bit.
                                newPosition = characterBody.Position + Math.Max(-maximumStepHeight, currentOffset) * down + horizontalOffset;
                                return true;
                            }
                            else
                            {
                                newPosition = new Vector3();
                                return false;
                            }
                        case CharacterContactPositionState.Rejected:
                            newPosition = new Vector3();
                            return false;
                        case CharacterContactPositionState.NoHit:
                            highestBound = currentOffset + hintOffset;
                            currentOffset = (lowestBound + currentOffset) * .5f;
                            break;
                        case CharacterContactPositionState.Obstructed:
                            lowestBound = currentOffset;
                            currentOffset = (highestBound + currentOffset) * .5f;
                            break;
                        case CharacterContactPositionState.HeadObstructed:
                            highestBound = currentOffset + hintOffset;
                            currentOffset = (lowestBound + currentOffset) * .5f;
                            break;
                        case CharacterContactPositionState.TooDeep:
                            currentOffset += hintOffset;
                            lowestBound = currentOffset;
                            break;

                    }

                } //TODO: If the ray cast doesn't hit, that could be used to early out...  Then again, it pretty much can't happen.

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
                    candidatePosition = characterBody.Position + currentOffset * down + horizontalOffset;
                    switch (TryUpStepPosition(ref normal, ref candidatePosition, ref down,
                                              ref tractionContacts, ref supportContacts, ref sideContacts, ref headContacts,
                                              out hintOffset))
                    {
                        case CharacterContactPositionState.Accepted:
                            currentOffset += hintOffset;
                            //Only use the new position location if the movement distance was the right size.
                            if (currentOffset < 0 && currentOffset > -maximumStepHeight - CollisionDetectionSettings.AllowedPenetration)
                            {
                                //It's possible that we let a just-barely-too-high step occur, limited by the allowed penetration.
                                //Just clamp the overall motion and let it penetrate a bit.
                                newPosition = characterBody.Position + Math.Max(-maximumStepHeight, currentOffset) * down + horizontalOffset;
                                return true;
                            }
                            else
                            {
                                newPosition = new Vector3();
                                return false;
                            }
                        case CharacterContactPositionState.Rejected:
                            newPosition = new Vector3();
                            return false;
                        case CharacterContactPositionState.NoHit:
                            highestBound = currentOffset + hintOffset;
                            currentOffset = (lowestBound + highestBound) * .5f;
                            break;
                        case CharacterContactPositionState.Obstructed:
                            lowestBound = currentOffset;
                            currentOffset = (highestBound + lowestBound) * .5f;
                            break;
                        case CharacterContactPositionState.HeadObstructed:
                            highestBound = currentOffset + hintOffset;
                            currentOffset = (lowestBound + currentOffset) * .5f;
                            break;
                        case CharacterContactPositionState.TooDeep:
                            currentOffset += hintOffset;
                            lowestBound = currentOffset;
                            break;
                    }
                }
            }
            finally
            {
                tractionContacts.Dispose();
                supportContacts.Dispose();
                sideContacts.Dispose();
                headContacts.Dispose();
            }
            //Couldn't find a candidate.
            newPosition = new Vector3();
            return false;



        }

        private void PrepareQueryObject(ref Vector3 position)
        {
            RigidTransform transform;
            transform.Position = position;
            transform.Orientation = characterBody.Orientation;
            currentQueryObject.UpdateBoundingBoxForTransform(ref transform, 0);
        }

        CharacterContactPositionState TryUpStepPosition(ref Vector3 sideNormal, ref Vector3 position, ref Vector3 down,
            ref QuickList<CharacterContact> tractionContacts, ref QuickList<CharacterContact> supportContacts, ref QuickList<CharacterContact> sideContacts, ref QuickList<CharacterContact> headContacts,
            out float hintOffset)
        {
            hintOffset = 0;
            PrepareQueryObject(ref position);
            QueryManager.QueryContacts(currentQueryObject, ref tractionContacts, ref supportContacts, ref sideContacts, ref headContacts);
            if (headContacts.Count > 0)
            {
                //The head is obstructed.  This will define a maximum bound.
                //Find the deepest contact on the head and use it to provide a hint.
                float dot;
                Vector3.Dot(ref down, ref headContacts.Elements[0].Contact.Normal, out dot);
                hintOffset = -dot * headContacts.Elements[0].Contact.PenetrationDepth;
                for (int i = 1; i < headContacts.Count; i++)
                {
                    Vector3.Dot(ref down, ref headContacts.Elements[i].Contact.Normal, out dot);
                    dot *= -headContacts.Elements[i].Contact.PenetrationDepth;
                    if (dot > hintOffset)
                    {
                        hintOffset = dot;
                    }
                }
                return CharacterContactPositionState.HeadObstructed;
            }
            bool obstructed = IsUpStepObstructedBySideContacts(ref sideNormal, ref sideContacts);
            if (!obstructed && supportContacts.Count > 0)
            {
                CharacterContactPositionState supportState;
                CharacterContact supportContact;
                QueryManager.AnalyzeSupportState(ref tractionContacts, ref supportContacts, out supportState, out supportContact);
                if (supportState == CharacterContactPositionState.Accepted)
                {
                    if (tractionContacts.Count > 0)
                    {
                        //We're done! The guess found a good spot to stand on.
                        //Unlike down stepping, upstepping DOES need good contacts in the final state.
                        //Push it up if necessary, but don't push it too far.
                        //Putting it into the middle of the allowed penetration makes it very likely that it will properly generate contacts.
                        //Choosing something smaller than allowed penetration ensures that the search makes meaningful progress forward when the sizes get really tiny;
                        //we wouldn't want it edging every closer to AllowedPenetration and then exit because too many queries were made.
                        hintOffset = Math.Min(0, Vector3.Dot(supportContact.Contact.Normal, down) * (CollisionDetectionSettings.AllowedPenetration * .5f - supportContact.Contact.PenetrationDepth));
                        return CharacterContactPositionState.Accepted;
                    }
                    else
                    {
                        //No traction... Before we give up and reject the step altogether, let's try one last thing.  It's possible that the character is trying to step up onto the side of a ramp or something.
                        //In this scenario, the top-down ray cast detects a perfectly walkable slope.  However, the contact queries will find a contact with a normal necessarily
                        //steeper than the one found by the ray cast because it is an edge contact.  Not being able to step up in this scenario doesn't feel natural to the player
                        //even if it is technically consistent.

                        //So, let's try to ray cast down to the a point just barely beyond the contact (to ensure we don't land right on the edge, which would invite numerical issues).
                        //Note that this is NOT equivalent to the ray cast we performed earlier to test for an initial step height and surface normal.
                        //This one is based on the QUERY state and the QUERY's contact position.

                        //Find the down test ray's position.
                        Ray downRay;
                        downRay.Position = supportContact.Contact.Position + sideNormal * .001f;
                        float verticalOffset = Vector3.Dot(downRay.Position - position, down);
                        verticalOffset = characterBody.Height * .5f + verticalOffset;
                        downRay.Position -= verticalOffset * down;
                        downRay.Direction = down;

                        //First, we must ensure that the ray cast test origin is not obstructed.  Starting very close to the very top of the character is safe because the process has already validated
                        //this location as accepted, just without traction.
                        Ray obstructionTestRay;
                        obstructionTestRay.Position = position - down * (characterBody.Height * .5f);
                        obstructionTestRay.Direction = downRay.Position - obstructionTestRay.Position;

                        if (!QueryManager.RayCastHitAnything(obstructionTestRay, 1))
                        {
                            //Okay! it's safe to cast down, then.
                            RayHit hit;
                            if (QueryManager.RayCast(downRay, characterBody.Height, out hit))
                            {
                                //Got a hit!
                                if (characterBody.Height - maximumStepHeight < hit.T)
                                {
                                    //It's in range!                   
                                    float dot;
                                    hit.Normal.Normalize();
                                    Vector3.Dot(ref hit.Normal, ref down, out dot);
                                    if (Math.Abs(dot) > ContactCategorizer.TractionThreshold)
                                    {
                                        //Slope is shallow enough to stand on!
                                        hintOffset = Math.Min(0, Vector3.Dot(supportContact.Contact.Normal, down) * (CollisionDetectionSettings.AllowedPenetration * .5f - supportContact.Contact.PenetrationDepth));
                                        //ONE MORE thing to check.  The new position of the center ray must be able to touch the ground!
                                        downRay.Position = position;
                                        if (QueryManager.RayCast(downRay, characterBody.Height * .5f + maximumStepHeight, out hit))
                                        {
                                            //It hit.. almost there!
                                            hit.Normal.Normalize();
                                            Vector3.Dot(ref hit.Normal, ref down, out dot);
                                            if (Math.Abs(dot) > ContactCategorizer.TractionThreshold)
                                            {
                                                //It has traction! We can step!
                                                return CharacterContactPositionState.Accepted;
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        //If it didn't have traction, and this was the most valid location we could find, then there is no support.
                        return CharacterContactPositionState.Rejected;
                    }
                }
                else if (supportState == CharacterContactPositionState.TooDeep)
                {
                    //Looks like we have to keep trying, but at least we found a good hint.
                    hintOffset = Math.Min(0, Vector3.Dot(supportContact.Contact.Normal, down) * (CollisionDetectionSettings.AllowedPenetration * .5f - supportContact.Contact.PenetrationDepth));
                    return CharacterContactPositionState.TooDeep;
                }
                else //if (supportState == SupportState.Separated)
                {
                    //It's not obstructed, but the support isn't quite right.
                    //It's got a negative penetration depth.
                    //We can use that as a hint.
                    hintOffset = -.001f - Vector3.Dot(supportContact.Contact.Normal, down) * supportContact.Contact.PenetrationDepth;
                    return CharacterContactPositionState.NoHit;
                }
            }
            else if (obstructed)
            {
                return CharacterContactPositionState.Obstructed;
            }
            else
            {
                return CharacterContactPositionState.NoHit;
            }
        }

        bool IsUpStepObstructedBySideContacts(ref Vector3 sideNormal, ref QuickList<CharacterContact> sideContacts)
        {
            //A contact is considered obstructive if its projected depth is deeper than any existing contact along the existing contacts' normals.
            for (int i = 0; i < sideContacts.Count; i++)
            {
                if (IsObstructiveToUpStepping(ref sideNormal, ref sideContacts.Elements[i].Contact))
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
            foreach (var c in SupportFinder.SideContacts)
            {
                dot = Vector3.Dot(contact.Normal, c.Contact.Normal);
                float depth = dot * c.Contact.PenetrationDepth;
                if (depth > Math.Max(c.Contact.PenetrationDepth, CollisionDetectionSettings.AllowedPenetration))
                    return true;

            }
            return false;
        }




    }
}
