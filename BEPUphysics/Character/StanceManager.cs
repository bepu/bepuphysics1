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
    /// Handles a cylindrical character's stances, like standing or crouching, and their transitions.
    /// </summary>
    public class StanceManager
    {
        /// <summary>
        /// This is a direct reference to the 'true' character collidable. The others are query proxies that share the same shape.
        /// </summary>
        private Cylinder characterBody;

        ConvexCollidable<CylinderShape> standingQueryObject;
        ConvexCollidable<CylinderShape> crouchingQueryObject;
        ConvexCollidable<CylinderShape> currentQueryObject;

        /// <summary>
        /// Updates the query objects to match the character controller's current state.  Called when BodyRadius, StanceManager.StandingHeight, or StanceManager.CrouchingHeight is set.
        /// </summary>
        public void UpdateQueryShapes()
        {
            standingQueryObject.Shape.Radius = characterBody.Radius;
            standingQueryObject.Shape.Height = StandingHeight;
            crouchingQueryObject.Shape.Radius = characterBody.Radius;
            crouchingQueryObject.Shape.Height = CrouchingHeight;
        }

        private float standingHeight;
        /// <summary>
        /// Gets or sets the height of the character while standing.  To avoid resizing-related problems, use this only when the character is not being actively simulated or is not currently standing.
        /// </summary>
        public float StandingHeight
        {
            get { return standingHeight; }
            set
            {
                if (value <= 0 || value < CrouchingHeight)
                    throw new ArgumentException("Standing height must be positive and greater than the crouching height.");
                standingHeight = value;
                UpdateQueryShapes();
                if (CurrentStance == Stance.Standing)
                {
                    //If we're currently standing, then the current shape must be modified as well.
                    //This isn't entirely safe, but dynamic resizing generally isn't.
                    characterBody.Height = standingHeight;
                }
            }
        }

        private float crouchingHeight;
        /// <summary>
        /// Gets or sets the height of the character while crouching.  Must be less than the standing height.  To avoid resizing-related problems, use this only when the character is not being actively simulated or is not currently crouching.
        /// </summary>
        public float CrouchingHeight
        {
            get { return crouchingHeight; }
            set
            {
                if (value <= 0 || value > StandingHeight)
                    throw new ArgumentException("Crouching height must be positive and less than the standing height.");
                crouchingHeight = value;
                UpdateQueryShapes();

                if (CurrentStance == Stance.Crouching)
                {
                    //If we're currently crouching, then the current shape must be modified as well.
                    //This isn't entirely safe, but dynamic resizing generally isn't.
                    characterBody.Height = crouchingHeight;
                }
            }
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

        private QueryManager QueryManager { get; set; }
        private SupportFinder SupportFinder { get; set; }

        /// <summary>
        /// Constructs a stance manager for a character.
        /// </summary>
        /// <param name="characterBody">The character's body entity.</param>
        /// <param name="crouchingHeight">Crouching height of the character.</param>
        /// <param name="queryManager">Provider of queries used by the stance manager to test if it is okay to change stances.</param>
        /// <param name="supportFinder">Support finder used by the character.</param>
        public StanceManager(Cylinder characterBody, float crouchingHeight, QueryManager queryManager, SupportFinder supportFinder)
        {
            this.QueryManager = queryManager;
            this.SupportFinder = supportFinder;
            this.characterBody = characterBody;
            standingHeight = characterBody.Height;
            if (crouchingHeight < standingHeight)
                this.crouchingHeight = StandingHeight * .7f;
            else
                throw new ArgumentException("Crouching height must be less than standing height.");

            //We can share the real shape with the query objects.
            currentQueryObject = new ConvexCollidable<CylinderShape>(characterBody.CollisionInformation.Shape);
            standingQueryObject = new ConvexCollidable<CylinderShape>(new CylinderShape(StandingHeight, characterBody.Radius));
            crouchingQueryObject = new ConvexCollidable<CylinderShape>(new CylinderShape(CrouchingHeight, characterBody.Radius));
            //Share the collision rules between the main body and its query objects.  That way, the character's queries return valid results.
            currentQueryObject.CollisionRules = characterBody.CollisionInformation.CollisionRules;
            standingQueryObject.CollisionRules = characterBody.CollisionInformation.CollisionRules;
            crouchingQueryObject.CollisionRules = characterBody.CollisionInformation.CollisionRules;
        }

        private void PrepareQueryObject(EntityCollidable queryObject, ref Vector3 position)
        {
            RigidTransform transform;
            transform.Position = position;
            transform.Orientation = characterBody.Orientation;
            queryObject.UpdateBoundingBoxForTransform(ref transform, 0);
        }

        /// <summary>
        /// Attempts to change the stance of the character if possible.
        /// </summary>
        /// <returns>Whether or not the character was able to change its stance.</returns>
        public bool UpdateStance(out Vector3 newPosition)
        {
            var currentPosition = characterBody.position;
            var down = characterBody.orientationMatrix.Down;
            newPosition = new Vector3();
            if (CurrentStance != DesiredStance)
            {
                if (CurrentStance == Stance.Standing && DesiredStance == Stance.Crouching)
                {
                    //Crouch.  There's no complicated logic to crouching; you don't need to validate
                    //a crouch before doing it.
                    //You do, however, do a different kind of crouch if you're airborne.
                    if (SupportFinder.HasSupport)
                    {
                        //Move the character towards the ground.
                        newPosition = currentPosition + down * ((StandingHeight - CrouchingHeight) * .5f);
                        characterBody.Height = CrouchingHeight;
                        CurrentStance = Stance.Crouching;
                    }
                    else
                    {
                        //We're in the air, so we don't have to change the position at all- just change the height.
                        //No queries needed since we're only shrinking.
                        newPosition = currentPosition;
                        characterBody.Height = CrouchingHeight;
                        CurrentStance = Stance.Crouching;
                    }
                    return true;
                }
                else if (CurrentStance == Stance.Crouching && DesiredStance == Stance.Standing)
                {
                    var tractionContacts = new QuickList<CharacterContact>(BufferPools<CharacterContact>.Thread);
                    var supportContacts = new QuickList<CharacterContact>(BufferPools<CharacterContact>.Thread);
                    var sideContacts = new QuickList<CharacterContact>(BufferPools<CharacterContact>.Thread);
                    var headContacts = new QuickList<CharacterContact>(BufferPools<CharacterContact>.Thread);
                    try
                    {
                        //Attempt to stand.
                        if (SupportFinder.HasSupport)
                        {
                            //Standing requires a query to verify that the new state is safe.
                            //TODO: State queries can be expensive if the character is crouching beneath something really detailed.
                            //There are some situations where you may want to do an upwards-pointing ray cast first.  If it hits something,
                            //there's no need to do the full query.
                            newPosition = currentPosition - down * ((StandingHeight - CrouchingHeight) * .5f);
                            PrepareQueryObject(standingQueryObject, ref newPosition);
                            QueryManager.QueryContacts(standingQueryObject, ref tractionContacts, ref supportContacts, ref sideContacts, ref headContacts);
                            if (IsObstructed(ref sideContacts, ref headContacts))
                            {
                                //Can't stand up if something is in the way!
                                return false;
                            }
                            characterBody.Height = StandingHeight;
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
                            while (attempts++ < 5 && lowestBound - highestBound > Toolbox.BigEpsilon)
                            {
                                Vector3 candidatePosition = currentPosition + currentOffset * down;
                                float hintOffset;
                                switch (TrySupportLocation(ref candidatePosition, out hintOffset, ref tractionContacts, ref supportContacts, ref sideContacts, ref headContacts))
                                {
                                    case CharacterContactPositionState.Accepted:
                                        currentOffset += hintOffset;
                                        //Only use the new position location if the movement distance was the right size.
                                        if (currentOffset > 0 && currentOffset < maximum)
                                        {
                                            newPosition = currentPosition + currentOffset * down;
                                            characterBody.Height = StandingHeight;
                                            CurrentStance = Stance.Standing;
                                            return true;
                                        }
                                        else
                                        {
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
                            //Couldn't find a hit.  Go ahead and stand!
                            newPosition = currentPosition;
                            characterBody.Height = StandingHeight;
                            CurrentStance = Stance.Standing;
                            return true;
                        }
                    }
                    finally
                    {
                        tractionContacts.Dispose();
                        supportContacts.Dispose();
                        sideContacts.Dispose();
                        headContacts.Dispose();
                    }
                }
            }

            return false;
        }

        bool IsObstructed(ref QuickList<CharacterContact> sideContacts, ref QuickList<CharacterContact> headContacts)
        {
            //No head contacts can exist!
            if (headContacts.Count > 0)
                return true;
            //A contact is considered obstructive if its projected depth is deeper than any existing contact along the existing contacts' normals.
            for (int i = 0; i < sideContacts.Count; i++)
            {
                if (IsObstructive(ref sideContacts.Elements[i].Contact))
                    return true;
            }
            return false;
        }

        bool IsObstructive(ref ContactData contact)
        {
            //Can't stand up if there are new side contacts that are too deep.
            if (SupportFinder.SideContacts.Count == 0 && contact.PenetrationDepth > CollisionDetectionSettings.AllowedPenetration)
            {
                return true;
            }

            //Go through side-facing contact and check to see if the new contact is deeper than any existing contact in the direction of the existing contact.
            //This is equivalent to considering the existing contacts to define planes and then comparing the new contact against those planes.
            //Since we already have the penetration depths, we don't need to use the positions of the contacts.
            foreach (var c in SupportFinder.SideContacts)
            {
                float dot = Vector3.Dot(contact.Normal, c.Contact.Normal);
                float depth = dot * c.Contact.PenetrationDepth;
                if (depth > Math.Max(c.Contact.PenetrationDepth, CollisionDetectionSettings.AllowedPenetration))
                    return true;

            }
            return false;
        }

        CharacterContactPositionState TrySupportLocation(ref Vector3 position, out float hintOffset,
            ref QuickList<CharacterContact> tractionContacts, ref QuickList<CharacterContact> supportContacts, ref QuickList<CharacterContact> sideContacts, ref QuickList<CharacterContact> headContacts)
        {
            hintOffset = 0;
            PrepareQueryObject(standingQueryObject, ref position);
            QueryManager.QueryContacts(standingQueryObject, ref tractionContacts, ref supportContacts, ref sideContacts, ref headContacts);

            bool obstructed = IsObstructed(ref sideContacts, ref headContacts);
            if (obstructed)
            {
                return CharacterContactPositionState.Obstructed;
            }
            if (supportContacts.Count > 0)
            {
                CharacterContactPositionState supportState;
                CharacterContact supportContact;
                QueryManager.AnalyzeSupportState(ref tractionContacts, ref supportContacts, out supportState, out supportContact);
                var down = characterBody.orientationMatrix.Down;
                //Note that traction is not tested for; it isn't important for the stance manager.
                if (supportState == CharacterContactPositionState.Accepted)
                {
                    //We're done! The guess found a good spot to stand on.
                    //We need to have fairly good contacts after this process, so only push it up a bit.
                    hintOffset = Math.Min(0, Vector3.Dot(supportContact.Contact.Normal, down) * (CollisionDetectionSettings.AllowedPenetration * .5f - supportContact.Contact.PenetrationDepth));
                    return CharacterContactPositionState.Accepted;
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
            else //Not obstructed, but no support.
            {
                return CharacterContactPositionState.NoHit;
            }
        }



    }

    //The StanceManager, as is, is semi-extensible.  Technically additional states can be added.
    //However, there are parts that are hard coded for the sake of initial implementation simplicity,
    //so it won't be a cakewalk.
    /// <summary>
    /// Stance of a cylindrical character.
    /// </summary>
    public enum Stance
    {
        Standing,
        Crouching
    }
}
