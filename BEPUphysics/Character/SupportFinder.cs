using System;
using System.Diagnostics;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionTests;
using BEPUphysics.Entities;
using BEPUutilities;
using BEPUutilities.DataStructures;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.Settings;

namespace BEPUphysics.Character
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
        /// Gets the vertical distance from the center of the character to the bottom of the character.
        /// </summary>
        public float BottomDistance { get; private set; }

        private SupportData supportData;
        /// <summary>
        /// Computes a combined support contact from all available supports (contacts or ray).
        /// </summary>
        public SupportData SupportData
        {
            get { return supportData; }
        }

        /// <summary>
        /// Computes representative support information based on the character's current traction contacts, support contacts, and ray contacts.
        /// </summary>
        /// <param name="down">Down direction of the character.</param>
        private void UpdateSupportData(ref Vector3 down)
        {
            //Choose which set of contacts to use.
            RawList<CharacterContact> contacts;
            if (tractionContacts.Count > 0)
            {
                contacts = tractionContacts;
            }
            else if (supportContacts.Count > 0)
            {
                contacts = supportContacts;
            }
            else
            {
                //No contacts provide support!
                //Fall back to the ray cast result.
                if (SupportRayData != null)
                {
                    supportData = new SupportData
                    {
                        Position = SupportRayData.Value.HitData.Location,
                        Normal = SupportRayData.Value.HitData.Normal,
                        Depth = Vector3.Dot(down, SupportRayData.Value.HitData.Normal) * (BottomDistance - SupportRayData.Value.HitData.T),
                        SupportObject = SupportRayData.Value.HitObject
                    };
                }
                else
                {
                    supportData = new SupportData();
                }
                return;
            }

            //Compute a representative support from the set of contacts.

            supportData.Position = contacts.Elements[0].Contact.Position;
            supportData.Normal = contacts.Elements[0].Contact.Normal;

            for (int i = 1; i < contacts.Count; i++)
            {
                Vector3.Add(ref supportData.Position, ref contacts.Elements[i].Contact.Position, out supportData.Position);
                Vector3.Add(ref supportData.Normal, ref contacts.Elements[i].Contact.Normal, out supportData.Normal);
            }
            if (contacts.Count > 1)
            {
                Vector3.Divide(ref supportData.Position, contacts.Count, out supportData.Position);
                float length = supportData.Normal.LengthSquared();
                if (length < Toolbox.Epsilon)
                {
                    //It's possible that the normals have cancelled each other out- that would be bad!
                    //Just use an arbitrary support's normal in that case.
                    supportData.Normal = contacts.Elements[0].Contact.Normal;
                }
                else
                {
                    Vector3.Multiply(ref supportData.Normal, 1 / (float)Math.Sqrt(length), out supportData.Normal);
                }
            }
            //Now that we have the normal, cycle through all the contacts again and find the deepest projected depth.
            //Use that object as our support too.
            float depth = -float.MaxValue;
            Collidable supportObject = null;
            for (int i = 0; i < contacts.Count; i++)
            {
                float dot;
                Vector3.Dot(ref contacts.Elements[i].Contact.Normal, ref supportData.Normal, out dot);
                dot = dot * contacts.Elements[i].Contact.PenetrationDepth;
                if (dot > depth)
                {
                    depth = dot;
                    supportObject = contacts.Elements[i].Collidable;
                }
            }
            supportData.Depth = depth;
            supportData.SupportObject = supportObject;
        }

        private SupportData verticalSupportData;
        /// <summary>
        /// Gets the support data of the character biased towards the character's movement direction.
        /// Excludes contacts that might otherwise cause the vertical motion constraint to fight the character's movement.
        /// </summary>
        public SupportData VerticalSupportData
        {
            get { return verticalSupportData; }
        }

        /// <summary>
        /// Computes a traction contact using a movement direction. This is helpful for the vertical motion constraint.
        /// By biasing the search in the movement direction, contacts on the character's butt (which might otherwise hold the character back via the vertical motion constraint) are ignored.
        /// </summary>
        /// <param name="down">Down direction of the character.</param>
        /// <param name="movementDirection">Movement direction of the character.</param>
        private void UpdateVerticalSupportData(ref Vector3 down, ref Vector3 movementDirection)
        {
            if (HasTraction)
            {
                if (tractionContacts.Count > 0)
                {
                    //Find the traction-providing contact which is furthest in the direction of the movement direction.
                    int greatestIndex = -1;
                    float greatestDot = -float.MaxValue;
                    for (int i = 0; i < tractionContacts.Count; i++)
                    {
                        float dot;
                        Vector3.Dot(ref movementDirection, ref tractionContacts.Elements[i].Contact.Normal, out dot);
                        if (dot > greatestDot)
                        {
                            greatestDot = dot;
                            greatestIndex = i;
                        }
                    }

                    verticalSupportData.Position = tractionContacts.Elements[greatestIndex].Contact.Position;
                    verticalSupportData.Normal = tractionContacts.Elements[greatestIndex].Contact.Normal;
                    verticalSupportData.SupportObject = tractionContacts.Elements[greatestIndex].Collidable;

                    //Project all other contact depths onto the chosen normal, keeping the largest one.
                    //This lets the vertical motion constraint relax when objects are penetrating deeply.
                    float depth = -float.MaxValue;
                    for (int i = 0; i < tractionContacts.Count; i++)
                    {
                        float dot;
                        Vector3.Dot(ref tractionContacts.Elements[i].Contact.Normal, ref verticalSupportData.Normal, out dot);
                        dot = dot * tractionContacts.Elements[i].Contact.PenetrationDepth;
                        if (dot > depth)
                        {
                            depth = dot;
                        }
                    }
                    verticalSupportData.Depth = depth;

                    return;
                }
                //There were no traction providing contacts, so check the support ray.
                Debug.Assert(SupportRayData != null, "If the character has traction but there are no contacts, there must be a ray cast with traction.");
                verticalSupportData.Position = SupportRayData.Value.HitData.Location;
                verticalSupportData.Normal = SupportRayData.Value.HitData.Normal;
                verticalSupportData.Depth = Vector3.Dot(down, SupportRayData.Value.HitData.Normal) * (BottomDistance - SupportRayData.Value.HitData.T);
                verticalSupportData.SupportObject = SupportRayData.Value.HitObject;
                return;
            }
            verticalSupportData = new SupportData();
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

        private Entity characterBody;
        /// <summary>
        /// Gets the contact categorizer used by the support finder.
        /// </summary>
        private CharacterContactCategorizer ContactCategorizer { get; set; }

        /// <summary>
        /// Gets the query manager used by the support finder.
        /// </summary>
        private QueryManager QueryManager { get; set; }

        /// <summary>
        /// Constructs a new support finder.
        /// </summary>
        /// <param name="characterBody">Body entity used by the character.</param>
        /// <param name="queryManager">Query provider used by the character. Used to perform ray cast tests against the character's near environment.</param>
        /// <param name="contactCategorizer">Contact categorizer to use.</param>
        public SupportFinder(Entity characterBody, QueryManager queryManager, CharacterContactCategorizer contactCategorizer)
        {
            this.characterBody = characterBody;
            QueryManager = queryManager;
            ContactCategorizer = contactCategorizer;
        }

        /// <summary>
        /// Updates the collection of supporting contacts.
        /// </summary>
        public void UpdateSupports(ref Vector3 movementDirection)
        {
            bool hadTraction = HasTraction;

            //Reset traction/support.
            HasTraction = false;
            HasSupport = false;

            Vector3 downDirection = characterBody.orientationMatrix.Down;
            Vector3 bodyPosition = characterBody.position;

            //Compute the character's radius, minus a little margin. We want the rays to originate safely within the character's body.
            //Assume vertical rotational invariance. Spheres, cylinders, and capsules don't have varying horizontal radii.
            Vector3 extremePoint;
            var convexShape = characterBody.CollisionInformation.Shape as ConvexShape;
            Debug.Assert(convexShape != null, "Character bodies must be convex.");

            //Find the lowest point on the collision shape.
            convexShape.GetLocalExtremePointWithoutMargin(ref Toolbox.DownVector, out extremePoint);
            BottomDistance = -extremePoint.Y + convexShape.collisionMargin;

            convexShape.GetLocalExtremePointWithoutMargin(ref Toolbox.RightVector, out extremePoint);
            float rayCastInnerRadius = Math.Max((extremePoint.X + convexShape.collisionMargin) * 0.8f, extremePoint.X);

            //Vertically, the rays will start at the same height as the character's center.
            //While they could be started lower on a cylinder, that wouldn't always work for a sphere or capsule: the origin might end up outside of the shape!

            tractionContacts.Clear();
            supportContacts.Clear();
            sideContacts.Clear();
            headContacts.Clear();

            foreach (var pair in characterBody.CollisionInformation.Pairs)
            {
                //Don't stand on things that aren't really colliding fully.
                if (pair.CollisionRule != CollisionRule.Normal)
                    continue;
                ContactCategorizer.CategorizeContacts(pair, characterBody.CollisionInformation, ref downDirection, ref tractionContacts, ref supportContacts, ref sideContacts, ref headContacts);
            }

            HasSupport = supportContacts.Count > 0;
            HasTraction = tractionContacts.Count > 0;

            //Only perform ray casts if the character has fully left the surface, and only if the previous frame had traction.
            //(If ray casts are allowed when support contacts still exist, the door is opened for climbing surfaces which should not be climbable.
            //Consider a steep slope. If the character runs at it, the character will likely be wedged off of the ground, making it lose traction while still having a support contact with the slope.
            //If ray tests are allowed when support contacts exist, the character will maintain traction despite climbing the wall.
            //The VerticalMotionConstraint can stop the character from climbing in many cases, but it's nice not to have to rely on it.
            //Disallowing ray tests when supports exist does have a cost, though. For example, consider rounded steps.
            //If the character walks off a step such that it is still in contact with the step but is far enough down that the slope is too steep for traction, 
            //the ray test won't recover traction. This situation just isn't very common.)
            if (!HasSupport && hadTraction)
            {
                float supportRayLength = maximumAssistedDownStepHeight + BottomDistance;
                SupportRayData = null;
                //If the contacts aren't available to support the character, raycast down to find the ground.
                if (!HasTraction)
                {
                    //TODO: could also require that the character has a nonzero movement direction in order to use a ray cast.  Questionable- would complicate the behavior on edges.
                    Ray ray = new Ray(bodyPosition, downDirection);

                    bool hasTraction;
                    SupportRayData data;
                    if (TryDownCast(ref ray, supportRayLength, out hasTraction, out data))
                    {
                        SupportRayData = data;
                        HasTraction = data.HasTraction;
                        HasSupport = true;
                    }
                }

                //If contacts and the center ray cast failed, try a ray offset in the movement direction.
                bool tryingToMove = movementDirection.LengthSquared() > 0;
                if (!HasTraction && tryingToMove)
                {
                    Ray ray = new Ray(
                        characterBody.Position +
                        movementDirection * rayCastInnerRadius, downDirection);

                    //Have to test to make sure the ray doesn't get obstructed.  This could happen if the character is deeply embedded in a wall; we wouldn't want it detecting things inside the wall as a support!
                    Ray obstructionRay;
                    obstructionRay.Position = characterBody.Position;
                    obstructionRay.Direction = ray.Position - obstructionRay.Position;
                    if (!QueryManager.RayCastHitAnything(obstructionRay, 1))
                    {
                        //The origin isn't obstructed, so now ray cast down.
                        bool hasTraction;
                        SupportRayData data;
                        if (TryDownCast(ref ray, supportRayLength, out hasTraction, out data))
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
                if (!HasTraction && tryingToMove)
                {
                    //Compute the horizontal offset direction.  Down direction and the movement direction are normalized and perpendicular, so the result is too.
                    Vector3 horizontalOffset;
                    Vector3.Cross(ref movementDirection, ref downDirection, out horizontalOffset);
                    Vector3.Multiply(ref horizontalOffset, rayCastInnerRadius, out horizontalOffset);
                    Ray ray = new Ray(bodyPosition + horizontalOffset, downDirection);

                    //Have to test to make sure the ray doesn't get obstructed.  This could happen if the character is deeply embedded in a wall; we wouldn't want it detecting things inside the wall as a support!
                    Ray obstructionRay;
                    obstructionRay.Position = bodyPosition;
                    obstructionRay.Direction = ray.Position - obstructionRay.Position;
                    if (!QueryManager.RayCastHitAnything(obstructionRay, 1))
                    {
                        //The origin isn't obstructed, so now ray cast down.
                        bool hasTraction;
                        SupportRayData data;
                        if (TryDownCast(ref ray, supportRayLength, out hasTraction, out data))
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
                if (!HasTraction && tryingToMove)
                {
                    //Compute the horizontal offset direction.  Down direction and the movement direction are normalized and perpendicular, so the result is too.
                    Vector3 horizontalOffset;
                    Vector3.Cross(ref downDirection, ref movementDirection, out horizontalOffset);
                    Vector3.Multiply(ref horizontalOffset, rayCastInnerRadius, out horizontalOffset);
                    Ray ray = new Ray(bodyPosition + horizontalOffset, downDirection);

                    //Have to test to make sure the ray doesn't get obstructed.  This could happen if the character is deeply embedded in a wall; we wouldn't want it detecting things inside the wall as a support!
                    Ray obstructionRay;
                    obstructionRay.Position = bodyPosition;
                    obstructionRay.Direction = ray.Position - obstructionRay.Position;
                    if (!QueryManager.RayCastHitAnything(obstructionRay, 1))
                    {
                        //The origin isn't obstructed, so now ray cast down.
                        bool hasTraction;
                        SupportRayData data;
                        if (TryDownCast(ref ray, supportRayLength, out hasTraction, out data))
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

            UpdateSupportData(ref downDirection);
            UpdateVerticalSupportData(ref downDirection, ref movementDirection);

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
                if (dot > ContactCategorizer.TractionThreshold)
                {
                    //It has traction!
                    hasTraction = true;
                    supportRayData = new SupportRayData { HitData = earliestHit, HitObject = earliestHitObject, HasTraction = true };
                }
                else if (dot > ContactCategorizer.SupportThreshold)
                    supportRayData = new SupportRayData { HitData = earliestHit, HitObject = earliestHitObject };
                else
                    return false; //Too steep! Toss it out.
                return true;
            }
            return false;
        }


        internal bool IsSideContactObstructive(ref ContactData contact)
        {
            //Can't stand up or step down if there are new side contacts that are too deep.
            //If the contact has less than the allowed penetration depth, allow it.
            if (contact.PenetrationDepth <= CollisionDetectionSettings.AllowedPenetration)
            {
                return false;
            }
            //If there is already a contact that is deeper than the new contact, then allow it. It won't make things worse.
            //Adding this extra permission avoids situations where the character can't stand up because it's just slightly pushed up against a wall.
            foreach (var c in SideContacts)
            {
                //An existing contact is considered 'deeper' if its normal-adjusted depth is greater than the new contact.
                float dot = Vector3.Dot(contact.Normal, c.Contact.Normal);
                float depth = dot * c.Contact.PenetrationDepth + Toolbox.BigEpsilon;
                if (depth >= contact.PenetrationDepth)
                    return false;

            }
            return true;

        }



        /// <summary>
        /// Clears out the support finder's cache, removing all support from the character.
        /// </summary>
        public void ClearSupportData()
        {
            HasSupport = false;
            HasTraction = false;
            tractionContacts.Clear();
            supportContacts.Clear();
            sideContacts.Clear();
            headContacts.Clear();
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
    /// Description of a support for a character controller.
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
