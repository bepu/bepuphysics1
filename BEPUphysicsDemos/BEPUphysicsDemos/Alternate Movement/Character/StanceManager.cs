using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.CollisionTests;
using BEPUphysics.DataStructures;
using BEPUphysics.Settings;
using BEPUphysics;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.AlternateMovement.Character
{
    /// <summary>
    /// Handles a character's stances, like standing or crouching, and their transitions.
    /// </summary>
    public class StanceManager
    {
        /// <summary>
        /// Gets the height of the character while standing.
        /// </summary>
        public float StandingHeight
        {
            get;
            private set;
        }

        /// <summary>
        /// Gets the height of the character while crouching.
        /// </summary>
        public float CrouchingHeight
        {
            get;
            private set;
        }

        /// <summary>
        /// Gets the current stance of the character.
        /// </summary>
        public Stance CurrentStance
        {
            get;
            private set;
        }

        /// <summary>
        /// Gets or sets the stance that the character is trying to move into.
        /// </summary>
        public Stance DesiredStance
        {
            get;
            set;
        }

        CharacterController character;
        /// <summary>
        /// Constructs a stance manager for a character.
        /// </summary>
        /// <param name="character">Character governed by the manager.</param>
        public StanceManager(CharacterController character)
        {
            this.character = character;
            StandingHeight = character.Body.Height;
            CrouchingHeight = StandingHeight * .7f;
        }

        /// <summary>
        /// Attempts to change the stance of the character if possible.
        /// </summary>
        /// <returns>Whether or not the character was able to change its stance.</returns>
        public bool UpdateStance(out Vector3 newPosition)
        {
            newPosition = new Vector3();
            if (CurrentStance != DesiredStance)
            {
                if (CurrentStance == Stance.Standing && DesiredStance == Stance.Crouching)
                {
                    //Crouch.  There's no complicated logic to crouching; you don't need to validate
                    //a crouch before doing it.
                    //You do, however, do a different kind of crouch if you're airborne.
                    if (character.SupportFinder.HasSupport)
                    {
                        //Move the character towards the ground.
                        newPosition = character.Body.Position + character.Body.OrientationMatrix.Down * ((StandingHeight - CrouchingHeight) * .5f);
                        character.Body.Height = CrouchingHeight;
                        CurrentStance = Stance.Crouching;
                    }
                    else
                    {
                        //We're in the air, so we don't have to change the position at all- just change the height.
                        //No queries needed since we're only shrinking.
                        newPosition = character.Body.Position;
                        character.Body.Height = CrouchingHeight;
                        CurrentStance = Stance.Crouching;
                    }
                    return true;
                }
                else if (CurrentStance == Stance.Crouching && DesiredStance == Stance.Standing)
                {
                    //Attempt to stand.
                    if (character.SupportFinder.HasSupport)
                    {
                        //Standing requires a query to verify that the new state is safe.
                        newPosition = character.Body.Position - character.Body.OrientationMatrix.Down * ((StandingHeight - CrouchingHeight) * .5f);
                        character.QueryManager.QueryContacts(newPosition, Stance.Standing);
                        if (IsObstructed(character.QueryManager.SideContacts, character.QueryManager.HeadContacts))
                        {
                            //Can't stand up if something is in the way!
                            return false;
                        }
                        character.Body.Height = StandingHeight;
                        CurrentStance = Stance.Standing;
                        return true;
                    }
                    else
                    {
                        //This is a complicated case.  We must perform a semi-downstep query.
                        //It's different than a downstep because the head may be obstructed as well.

                        float highestBound = 0;
                        float lowestBound = (StandingHeight - CrouchingHeight) * .5f;
                        float currentOffset = lowestBound;
                        float maximum = lowestBound;

                        int attempts = 0;
                        //Don't keep querying indefinitely.  If we fail to reach it in a few informed steps, it's probably not worth continuing.
                        //The bound size check prevents the system from continuing to search a meaninglessly tiny interval.
                        Vector3 down = character.Body.OrientationMatrix.Down;
                        while (attempts++ < 5 && lowestBound - highestBound > Toolbox.BigEpsilon)
                        {
                            Vector3 candidatePosition = character.Body.Position + currentOffset * down;
                            float hintOffset;
                            switch (TrySupportLocation(ref candidatePosition, out hintOffset))
                            {
                                case PositionState.Accepted:
                                    currentOffset += hintOffset;
                                    //Only use the new position location if the movement distance was the right size.
                                    if (currentOffset > 0 && currentOffset < maximum)
                                    {
                                        newPosition = character.Body.Position + currentOffset * down;
                                        character.Body.Height = StandingHeight;
                                        CurrentStance = Stance.Standing;
                                        return true;
                                    }
                                    else
                                    {
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
                        //Couldn't find a hit.  Go ahead and stand!
                        newPosition = character.Body.Position;
                        character.Body.Height = StandingHeight;
                        CurrentStance = Stance.Standing;
                        return true;
                    }
                }
            }

            return false;
        }

        bool IsObstructed(RawList<ContactData> sideContacts, RawList<ContactData> headContacts)
        {
            //No head contacts can exist!
            if (headContacts.Count > 0)
                return true;
            //A contact is considered obstructive if its projected depth is deeper than any existing contact along the existing contacts' normals.
            for (int i = 0; i < sideContacts.Count; i++)
            {
                if (IsObstructive(ref sideContacts.Elements[i]))
                    return true;
            }
            return false;
        }

        bool IsObstructive(ref ContactData contact)
        {
            //Can't stand up if there are new side contacts that are too deep.
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
                if (depth > Math.Max(c.Contact.PenetrationDepth, CollisionDetectionSettings.AllowedPenetration))
                    return true;

            }
            return false;
        }

        PositionState TrySupportLocation(ref Vector3 position, out float hintOffset)
        {
            hintOffset = 0;
            character.QueryManager.QueryContacts(position, Stance.Standing);
            bool hasTraction;
            PositionState supportState;
            ContactData supportContact;
            bool obstructed = IsObstructed(character.QueryManager.SideContacts, character.QueryManager.HeadContacts);
            if (character.QueryManager.HasSupports(out hasTraction, out supportState, out supportContact) && !obstructed)
            {
                if (supportState == PositionState.Accepted)
                {
                    //We're done! The guess found a good spot to stand on.
                    //We need to have fairly good contacts after this process, so only push it up a bit.
                    hintOffset = Math.Min(0, Vector3.Dot(supportContact.Normal, character.Body.OrientationMatrix.Down) * (CollisionDetectionSettings.AllowedPenetration * .5f - supportContact.PenetrationDepth));
                    return PositionState.Accepted;
                }
                else if (supportState == PositionState.TooDeep)
                {
                    //Looks like we have to keep trying, but at least we found a good hint.
                    hintOffset = Math.Min(0, Vector3.Dot(supportContact.Normal, character.Body.OrientationMatrix.Down) * (CollisionDetectionSettings.AllowedPenetration * .5f - supportContact.PenetrationDepth));
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

    }

    //The StanceManager, as is, is semi-extensible.  Technically additional states can be added.
    //However, there are parts that are hard coded for the sake of initial implementation simplicity,
    //so it won't be a cakewalk.
    public enum Stance
    {
        Standing,
        Crouching
    }
}
