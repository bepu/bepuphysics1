using System;
using System.Collections.Generic;
using BEPUphysics.Character;
using BEPUphysics.CollisionTests;
using BEPUutilities;
using BEPUutilities.DataStructures;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.CollisionRuleManagement;

namespace BEPUphysicsDemos.AlternateMovement.Character
{
    /// <summary>
    /// Analyzes the contacts on the character's body to find supports.
    /// </summary>
    public class SupportFinder
    {
        private RawList<CharacterContact> supportContacts = new RawList<CharacterContact>();
        private RawList<CharacterContact> tractionContacts = new RawList<CharacterContact>();
        private RawList<CharacterContact> sideContacts = new RawList<CharacterContact>();
        private RawList<CharacterContact> headContacts = new RawList<CharacterContact>();

        /// <summary>
        /// Gets the contact categorizer used by the support finder.
        /// </summary>
        public CharacterContactCategorizer ContactCategorizer { get; private set; }

        /// <summary>
        /// Gets the query manager used by the support finder.
        /// </summary>
        public QueryManager QueryManager { get; private set; }

        float bottomHeight;
        /// <summary>
        /// Gets the length from the ray start to the bottom of the character.
        /// </summary>
        public float RayLengthToBottom
        {
            get
            {
                return bottomHeight;
            }
        }

        float maximumAssistedDownStepHeight = 1;
        /// <summary>
        /// Gets or sets the maximum distance from the character's center to the support that will be assisted by downstepping.
        /// If the character walks off a step with height less than this value, the character will retain traction despite
        /// being temporarily airborne according to its contacts.
        /// </summary>
        public float MaximumAssistedDownStepHeight
        {
            get
            {
                return maximumAssistedDownStepHeight;
            }
            set
            {
                maximumAssistedDownStepHeight = Math.Max(value, 0);
            }
        }

        /// <summary>
        /// Computes a combined support contact from all available supports (contacts or ray).
        /// </summary>
        public SupportData? SupportData
        {
            get
            {
                if (supports.Count > 0)
                {
                    SupportData toReturn = new SupportData()
                    {
                        Position = supports.Elements[0].Contact.Position,
                        Normal = supports.Elements[0].Contact.Normal
                    };
                    for (int i = 1; i < supports.Count; i++)
                    {
                        Vector3.Add(ref toReturn.Position, ref supports.Elements[i].Contact.Position, out toReturn.Position);
                        Vector3.Add(ref toReturn.Normal, ref supports.Elements[i].Contact.Normal, out toReturn.Normal);
                    }
                    if (supports.Count > 1)
                    {
                        Vector3.Multiply(ref toReturn.Position, 1f / supports.Count, out toReturn.Position);
                        float length = toReturn.Normal.LengthSquared();
                        if (length < Toolbox.Epsilon)
                        {
                            //It's possible that the normals have cancelled each other out- that would be bad!
                            //Just use an arbitrary support's normal in that case.
                            toReturn.Normal = supports.Elements[0].Contact.Normal;
                        }
                        else
                        {
                            Vector3.Multiply(ref toReturn.Normal, 1 / (float)Math.Sqrt(length), out toReturn.Normal);
                        }
                    }
                    //Now that we have the normal, cycle through all the contacts again and find the deepest projected depth.
                    //Use that object as our support too.
                    float depth = -float.MaxValue;
                    Collidable supportObject = null;
                    for (int i = 0; i < supports.Count; i++)
                    {
                        float dot;
                        Vector3.Dot(ref supports.Elements[i].Contact.Normal, ref toReturn.Normal, out dot);
                        dot = dot * supports.Elements[i].Contact.PenetrationDepth;
                        if (dot > depth)
                        {
                            depth = dot;
                            supportObject = supports.Elements[i].Support;
                        }
                    }
                    toReturn.Depth = depth;
                    toReturn.SupportObject = supportObject;
                    return toReturn;
                }
                else
                {
                    //No support contacts; fall back to the raycast result...
                    if (SupportRayData != null)
                    {
                        return new SupportData
                        {
                            Position = SupportRayData.Value.HitData.Location,
                            Normal = SupportRayData.Value.HitData.Normal,
                            HasTraction = SupportRayData.Value.HasTraction,
                            Depth = Vector3.Dot(character.Down, SupportRayData.Value.HitData.Normal) * (bottomHeight - SupportRayData.Value.HitData.T),
                            SupportObject = SupportRayData.Value.HitObject
                        };
                    }
                    else
                    {
                        return null;
                    }
                }
            }
        }

        /// <summary>
        /// Computes a combined traction contact from all available supports with traction (contacts or ray).
        /// </summary>
        public SupportData? TractionData
        {
            get
            {
                if (supports.Count > 0)
                {
                    SupportData toReturn = new SupportData();
                    int withTraction = 0;
                    for (int i = 0; i < supports.Count; i++)
                    {
                        if (supports.Elements[i].HasTraction)
                        {
                            withTraction++;
                            Vector3.Add(ref toReturn.Position, ref supports.Elements[i].Contact.Position, out toReturn.Position);
                            Vector3.Add(ref toReturn.Normal, ref supports.Elements[i].Contact.Normal, out toReturn.Normal);
                        }
                    }
                    if (withTraction > 1)
                    {
                        Vector3.Multiply(ref toReturn.Position, 1f / withTraction, out toReturn.Position);
                        float length = toReturn.Normal.LengthSquared();
                        if (length < Toolbox.BigEpsilon)
                        {
                            //It's possible that the normals have cancelled each other out- that would be bad!
                            //Just use an arbitrary support's normal in that case.
                            toReturn.Normal = supports.Elements[0].Contact.Normal;
                        }
                        else
                        {
                            Vector3.Multiply(ref toReturn.Normal, 1 / (float)Math.Sqrt(length), out toReturn.Normal);
                        }
                    }
                    if (withTraction > 0)
                    {
                        //Now that we have the normal, cycle through all the contacts again and find the deepest projected depth.
                        float depth = -float.MaxValue;
                        Collidable supportObject = null;
                        for (int i = 0; i < supports.Count; i++)
                        {
                            if (supports.Elements[i].HasTraction)
                            {
                                float dot;
                                Vector3.Dot(ref supports.Elements[i].Contact.Normal, ref toReturn.Normal, out dot);
                                dot = dot * supports.Elements[i].Contact.PenetrationDepth;
                                if (dot > depth)
                                {
                                    depth = dot;
                                    supportObject = supports.Elements[i].Support;
                                }
                            }
                        }
                        toReturn.Depth = depth;
                        toReturn.SupportObject = supportObject;
                        toReturn.HasTraction = true;
                        return toReturn;
                    }
                }
                //No support contacts; fall back to the raycast result...
                if (SupportRayData != null && SupportRayData.Value.HasTraction)
                {
                    return new SupportData()
                    {
                        Position = SupportRayData.Value.HitData.Location,
                        Normal = SupportRayData.Value.HitData.Normal,
                        HasTraction = true,
                        Depth = Vector3.Dot(character.Down, SupportRayData.Value.HitData.Normal) * (bottomHeight - SupportRayData.Value.HitData.T),
                        SupportObject = SupportRayData.Value.HitObject
                    };
                }
                else
                {
                    return null;
                }

            }
        }

        public bool GetTractionInDirection(ref Vector3 movementDirection, out SupportData supportData)
        {

            if (HasTraction)
            {
                int greatestIndex = -1;
                float greatestDot = -float.MaxValue;
                for (int i = 0; i < supports.Count; i++)
                {
                    if (supports.Elements[i].HasTraction)
                    {
                        float dot;
                        Vector3.Dot(ref movementDirection, ref supports.Elements[i].Contact.Normal, out dot);
                        if (dot > greatestDot)
                        {
                            greatestDot = dot;
                            greatestIndex = i;
                        }
                    }
                }
                if (greatestIndex != -1)
                {
                    supportData.Position = supports.Elements[greatestIndex].Contact.Position;
                    supportData.Normal = supports.Elements[greatestIndex].Contact.Normal;
                    supportData.SupportObject = supports.Elements[greatestIndex].Support;
                    supportData.HasTraction = true;

                    float depth = -float.MaxValue;
                    for (int i = 0; i < supports.Count; i++)
                    {
                        if (supports.Elements[i].HasTraction)
                        {
                            float dot;
                            Vector3.Dot(ref supports.Elements[i].Contact.Normal, ref supportData.Normal, out dot);
                            dot = dot * supports.Elements[i].Contact.PenetrationDepth;
                            if (dot > depth)
                            {
                                depth = dot;
                            }
                        }
                    }
                    supportData.Depth = depth;

                    return true;
                }
                //Okay, try the ray cast result then.
                if (SupportRayData != null && SupportRayData.Value.HasTraction)
                {
                    supportData.Position = SupportRayData.Value.HitData.Location;
                    supportData.Normal = SupportRayData.Value.HitData.Normal;
                    supportData.Depth = Vector3.Dot(character.Down, SupportRayData.Value.HitData.Normal) * (bottomHeight - SupportRayData.Value.HitData.T);
                    supportData.SupportObject = SupportRayData.Value.HitObject;
                    supportData.HasTraction = true;
                    return true;
                }
                //Well that's strange!
                supportData = new SupportData();
                return false;
            }
            else
            {
                supportData = new SupportData();
                return false;
            }
        }

        /// <summary>
        /// Gets whether or not at least one of the character's body's contacts provide support to the character.
        /// </summary>
        public bool HasSupport { get; private set; }

        /// <summary>
        /// Gets whether or not at least one of the character's supports, if any, are flat enough to allow traction.
        /// </summary>
        public bool HasTraction { get; private set; }

        /// <summary>
        /// Gets the data about the supporting ray, if any.
        /// </summary>
        public SupportRayData? SupportRayData { get; private set; }

        /// <summary>
        /// Gets the character's supports.
        /// </summary>
        public ReadOnlyList<CharacterContact> Supports
        {
            get
            {
                return new ReadOnlyList<CharacterContact>(supportContacts);
            }
        }

        /// <summary>
        /// Gets the contacts on the side of the character.
        /// </summary>
        public ReadOnlyList<CharacterContact> SideContacts
        {
            get
            {
                return new ReadOnlyList<CharacterContact>(sideContacts);
            }
        }

        /// <summary>
        /// Gets the contacts on the top of the character.
        /// </summary>
        public ReadOnlyList<CharacterContact> HeadContacts
        {
            get
            {
                return new ReadOnlyList<CharacterContact>(headContacts);
            }
        }

        /// <summary>
        /// Gets a collection of the character's supports that provide traction.
        /// Traction means that the surface's slope is flat enough to stand on normally.
        /// </summary>
        public ReadOnlyList<CharacterContact> TractionSupports
        {
            get { return new ReadOnlyList<CharacterContact>(tractionContacts); }
        }
        
        /// <summary>
        /// Constructs a new support finder.
        /// </summary>
        /// <param name="contactCategorizer">Contact categorizer to use.</param>
        public SupportFinder(QueryManager queryManager, CharacterContactCategorizer contactCategorizer)
        {
            QueryManager = queryManager;
            ContactCategorizer = contactCategorizer;
        }

        /// <summary>
        /// Updates the collection of supporting contacts.
        /// </summary>
        public void UpdateSupports()
        {
            bool hadTraction = HasTraction;

            //Reset traction/support.
            HasTraction = false;
            HasSupport = false;

            var body = character.Body;
            Vector3 downDirection = character.Down;


            tractionContacts.Clear();
            supportContacts.Clear();
            sideContacts.Clear();
            headContacts.Clear();
            //Analyze the cylinder's contacts to see if we're supported.
            //Anything that can be a support will have a normal that's off horizontal.
            //That could be at the top or bottom, so only consider points on the bottom half of the shape.
            Vector3 position = character.Body.Position;

            foreach (var pair in character.Body.CollisionInformation.Pairs)
            {
                //Don't stand on things that aren't really colliding fully.
                if (pair.CollisionRule != CollisionRule.Normal)
                    continue;
                ContactCategorizer.CategorizeContacts(pair, character.Body.CollisionInformation, ref downDirection, tractionContacts, supportContacts, sideContacts, headContacts);
            }


            //Start the ray halfway between the center of the shape and the bottom of the shape.  That extra margin prevents it from getting stuck in the ground and returning t = 0 unhelpfully.
            SupportRayData = null;
            bottomHeight = body.Height * .25f;
            //If the contacts aren't available to support the character, raycast down to find the ground.
            if (!HasTraction && hadTraction)
            {

                //TODO: could also require that the character has a nonzero movement direction in order to use a ray cast.  Questionable- would complicate the behavior on edges.
                float length = bottomHeight + character.StepManager.MaximumStepHeight;
                Ray ray = new Ray(body.Position + downDirection * body.Height * .25f, downDirection);

                bool hasTraction;
                SupportRayData data;
                if (TryDownCast(ref ray, length, out hasTraction, out data))
                {
                    SupportRayData = data;
                    HasTraction = data.HasTraction;
                    HasSupport = true;
                }
            }

            //If contacts and the center ray cast failed, try a ray offset in the movement direction.
            Vector3 movementDirection;
            character.HorizontalMotionConstraint.GetMovementDirectionIn3D(out movementDirection);
            bool tryingToMove = movementDirection.LengthSquared() > 0;
            if (!HasTraction && hadTraction && tryingToMove)
            {
                Ray ray = new Ray(
                    body.Position +
                    movementDirection * (character.Body.Radius - character.Body.CollisionInformation.Shape.CollisionMargin) +
                    downDirection * body.Height * .25f, downDirection);

                //Have to test to make sure the ray doesn't get obstructed.  This could happen if the character is deeply embedded in a wall; we wouldn't want it detecting things inside the wall as a support!
                Ray obstructionRay;
                obstructionRay.Position = body.Position + downDirection * body.Height * .25f;
                obstructionRay.Direction = ray.Position - obstructionRay.Position;
                if (!QueryManager.RayCastHitAnything(obstructionRay, 1))
                {
                    //The origin isn't obstructed, so now ray cast down.
                    float length = bottomHeight + character.StepManager.MaximumStepHeight;
                    bool hasTraction;
                    SupportRayData data;
                    if (TryDownCast(ref ray, length, out hasTraction, out data))
                    {
                        if (SupportRayData == null || data.HitData.T < SupportRayData.Value.HitData.T)
                        {
                            //Only replace the previous support ray if we now have traction or we didn't have a support ray at all before,
                            //or this hit is a better (sooner) hit.
                            if (hasTraction)
                            {
                                SupportRayData = data;
                                HasTraction = true;
                            }
                            else if (SupportRayData == null)
                                SupportRayData = data;
                            HasSupport = true;
                        }
                    }
                }
            }

            //If contacts, center ray, AND forward ray failed to find traction, try a side ray created from down x forward.
            if (!HasTraction && hadTraction && tryingToMove)
            {
                //Compute the horizontal offset direction.  Down direction and the movement direction are normalized and perpendicular, so the result is too.
                Vector3 horizontalOffset;
                Vector3.Cross(ref movementDirection, ref downDirection, out horizontalOffset);
                Vector3.Multiply(ref horizontalOffset, character.Body.Radius - character.Body.CollisionInformation.Shape.CollisionMargin, out horizontalOffset);
                Ray ray = new Ray(body.Position + horizontalOffset + downDirection * body.Height * .25f, downDirection);

                //Have to test to make sure the ray doesn't get obstructed.  This could happen if the character is deeply embedded in a wall; we wouldn't want it detecting things inside the wall as a support!
                Ray obstructionRay;
                obstructionRay.Position = body.Position + downDirection * body.Height * .25f;
                obstructionRay.Direction = ray.Position - obstructionRay.Position;
                if (!QueryManager.RayCastHitAnything(obstructionRay, 1))
                {
                    //The origin isn't obstructed, so now ray cast down.
                    float length = bottomHeight + character.StepManager.MaximumStepHeight;
                    bool hasTraction;
                    SupportRayData data;
                    if (TryDownCast(ref ray, length, out hasTraction, out data))
                    {
                        if (SupportRayData == null || data.HitData.T < SupportRayData.Value.HitData.T)
                        {
                            //Only replace the previous support ray if we now have traction or we didn't have a support ray at all before,
                            //or this hit is a better (sooner) hit.
                            if (hasTraction)
                            {
                                SupportRayData = data;
                                HasTraction = true;
                            }
                            else if (SupportRayData == null)
                                SupportRayData = data;
                            HasSupport = true;
                        }
                    }
                }
            }

            //If contacts, center ray, forward ray, AND the first side ray failed to find traction, try a side ray created from forward x down.
            if (!HasTraction && hadTraction && tryingToMove)
            {
                //Compute the horizontal offset direction.  Down direction and the movement direction are normalized and perpendicular, so the result is too.
                Vector3 horizontalOffset;
                Vector3.Cross(ref downDirection, ref movementDirection, out horizontalOffset);
                Vector3.Multiply(ref horizontalOffset, character.Body.Radius - character.Body.CollisionInformation.Shape.CollisionMargin, out horizontalOffset);
                Ray ray = new Ray(body.Position + horizontalOffset + downDirection * body.Height * .25f, downDirection);

                //Have to test to make sure the ray doesn't get obstructed.  This could happen if the character is deeply embedded in a wall; we wouldn't want it detecting things inside the wall as a support!
                Ray obstructionRay;
                obstructionRay.Position = body.Position + downDirection * body.Height * .25f;
                obstructionRay.Direction = ray.Position - obstructionRay.Position;
                if (!QueryManager.RayCastHitAnything(obstructionRay, 1))
                {
                    //The origin isn't obstructed, so now ray cast down.
                    float length = bottomHeight + character.StepManager.MaximumStepHeight;
                    bool hasTraction;
                    SupportRayData data;
                    if (TryDownCast(ref ray, length, out hasTraction, out data))
                    {
                        if (SupportRayData == null || data.HitData.T < SupportRayData.Value.HitData.T)
                        {
                            //Only replace the previous support ray if we now have traction or we didn't have a support ray at all before,
                            //or this hit is a better (sooner) hit.
                            if (hasTraction)
                            {
                                SupportRayData = data;
                                HasTraction = true;
                            }
                            else if (SupportRayData == null)
                                SupportRayData = data;
                            HasSupport = true;
                        }
                    }
                }
            }

        }

        bool TryDownCast(ref Ray ray, float length, out bool hasTraction, out SupportRayData supportRayData)
        {
            RayHit earliestHit;
            Collidable earliestHitObject;
            supportRayData = new SupportRayData();
            hasTraction = false;
            if (QueryManager.RayCast(ray, length, out earliestHit, out earliestHitObject))
            {
                float lengthSquared = earliestHit.Normal.LengthSquared();
                if (lengthSquared < Toolbox.Epsilon)
                {
                    //Don't try to continue if the support ray is stuck in something.
                    return false;
                }
                Vector3.Divide(ref earliestHit.Normal, (float)Math.Sqrt(lengthSquared), out earliestHit.Normal);
                //A collidable was hit!  It's a support, but does it provide traction?
                earliestHit.Normal.Normalize();
                float dot;
                Vector3.Dot(ref ray.Direction, ref earliestHit.Normal, out dot);
                if (dot < 0)
                {
                    //Calibrate the normal so it always faces the same direction relative to the body.
                    Vector3.Negate(ref earliestHit.Normal, out earliestHit.Normal);
                    dot = -dot;
                }
                //This down cast is only used for finding supports and traction, not for finding side contacts.
                //If the detected normal is too steep, toss it out.
                if (dot > cosMaximumSlope)
                {
                    //It has traction!
                    hasTraction = true;
                    supportRayData = new SupportRayData() { HitData = earliestHit, HitObject = earliestHitObject, HasTraction = true };
                }
                else if (dot > SideContactThreshold)
                    supportRayData = new SupportRayData() { HitData = earliestHit, HitObject = earliestHitObject };
                else
                    return false; //Too steep! Toss it out.
                return true;
            }
            return false;
        }



        /// <summary>
        /// Cleans up the support finder.
        /// </summary>
        internal void ClearSupportData()
        {
            HasSupport = false;
            HasTraction = false;
            supports.Clear();
            SupportRayData = null;

        }

    }


    /// <summary>
    /// A contact generated between a character and a stored collidable.
    /// </summary>
    public struct CharacterContact
    {
        /// <summary>
        /// Core information about the contact.
        /// </summary>
        public ContactData Contact;
        /// <summary>
        /// Object that created this contact with the character.
        /// </summary>
        public Collidable Collidable;
    }

    /// <summary>
    /// Result of a ray cast which acts as a support for the character controller.
    /// </summary>
    public struct SupportRayData
    {
        /// <summary>
        /// Ray hit information of the support.
        /// </summary>
        public RayHit HitData;
        /// <summary>
        /// Object hit by the ray.
        /// </summary>
        public Collidable HitObject;
        /// <summary>
        /// Whether or not the support has traction.
        /// </summary>
        public bool HasTraction;
    }

    /// <summary>
    /// Contact which acts as a support for the character controller.
    /// </summary>
    public struct SupportData
    {
        /// <summary>
        /// Position of the support.
        /// </summary>
        public Vector3 Position;
        /// <summary>
        /// Normal of the support.
        /// </summary>
        public Vector3 Normal;
        /// <summary>
        /// Whether or not the contact was found to have traction.
        /// </summary>
        public bool HasTraction;
        /// <summary>
        /// Depth of the supporting location.
        /// Can be negative in the case of raycast supports.
        /// </summary>
        public float Depth;
        /// <summary>
        /// The object which the character is standing on.
        /// </summary>
        public Collidable SupportObject;
    }
}
