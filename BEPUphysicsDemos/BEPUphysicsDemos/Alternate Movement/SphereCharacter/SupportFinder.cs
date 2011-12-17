using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.CollisionTests;
using BEPUphysics.DataStructures;
using Microsoft.Xna.Framework;
using BEPUphysics;
using BEPUphysics.Collidables;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.BroadPhaseSystems;
using System.Diagnostics;

namespace BEPUphysicsDemos.AlternateMovement.SphereCharacter
{
    /// <summary>
    /// Analyzes the contacts on the character's body to find supports.
    /// </summary>
    public class SupportFinder
    {
        internal static float SideContactThreshold = .01f;

        internal RawList<SupportContact> supports = new RawList<SupportContact>();

        float maximumAssistedDownStepHeight = 1;
        /// <summary>
        /// Gets or sets the maximum distance from the character to the support that will be assisted by downstepping.
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
                        return new SupportData()
                        {
                            Position = SupportRayData.Value.HitData.Location,
                            Normal = SupportRayData.Value.HitData.Normal,
                            HasTraction = SupportRayData.Value.HasTraction,
                            Depth = Vector3.Dot(character.Body.OrientationMatrix.Down, SupportRayData.Value.HitData.Normal) * (bottomHeight - SupportRayData.Value.HitData.T),
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
                        Depth = Vector3.Dot(character.Body.OrientationMatrix.Down, SupportRayData.Value.HitData.Normal) * (bottomHeight - SupportRayData.Value.HitData.T),
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
                    supportData.Depth = Vector3.Dot(character.Body.OrientationMatrix.Down, SupportRayData.Value.HitData.Normal) * (bottomHeight - SupportRayData.Value.HitData.T);
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
        public ReadOnlyList<SupportContact> Supports
        {
            get
            {
                return new ReadOnlyList<SupportContact>(supports);
            }
        }

        /// <summary>
        /// Gets a collection of the character's supports that provide traction.
        /// Traction means that the surface's slope is flat enough to stand on normally.
        /// </summary>
        public TractionSupportCollection TractionSupports
        {
            get
            {
                return new TractionSupportCollection(supports);
            }
        }

        SphereCharacterController character;

        internal float sinMaximumSlope = (float)Math.Sin(MathHelper.PiOver4 + .01f);
        internal float cosMaximumSlope = (float)Math.Cos(MathHelper.PiOver4 + .01f);
        /// <summary>
        /// Gets or sets the maximum slope on which the character will have traction.
        /// </summary>
        public float MaximumSlope
        {
            get
            {
                return (float)Math.Acos(MathHelper.Clamp(cosMaximumSlope, -1, 1));
            }
            set
            {
                cosMaximumSlope = (float)Math.Cos(value);
                sinMaximumSlope = (float)Math.Sin(value);
            }
        }

        /// <summary>
        /// Constructs a new support finder.
        /// </summary>
        /// <param name="character">Character to analyze.</param>
        public SupportFinder(SphereCharacterController character)
        {
            this.character = character;
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
            Vector3 downDirection = character.Body.OrientationMatrix.Down; //For a cylinder orientation-locked to the Up axis, this is always {0, -1, 0}.  Keeping it generic doesn't cost much.


            supports.Clear();
            //Analyze the cylinder's contacts to see if we're supported.
            //Anything that can be a support will have a normal that's off horizontal.
            //That could be at the top or bottom, so only consider points on the bottom half of the shape.
            Vector3 position = character.Body.Position;

            foreach (var pair in character.Body.CollisionInformation.Pairs)
            {
                //Don't stand on things that aren't really colliding fully.
                if (pair.CollisionRule != CollisionRule.Normal)
                    continue;
                foreach (var c in pair.Contacts)
                {
                    //It's possible that a subpair has a non-normal collision rule, even if the parent pair is normal.
                    //Note that only contacts with nonnegative penetration depths are used.
                    //Negative depth contacts are 'speculative' in nature.
                    //If we were to use such a speculative contact for support, the character would find supports
                    //in situations where it should not.
                    //This can actually be useful in some situations, but keep it disabled by default.
                    if (c.Pair.CollisionRule != CollisionRule.Normal || c.Contact.PenetrationDepth < 0)
                        continue;
                    //Compute the offset from the position of the character's body to the contact.
                    Vector3 contactOffset;
                    Vector3.Subtract(ref c.Contact.Position, ref position, out contactOffset);


                    //Calibrate the normal of the contact away from the center of the object.
                    float dot;
                    Vector3 normal;
                    Vector3.Dot(ref contactOffset, ref c.Contact.Normal, out dot);
                    normal = c.Contact.Normal;
                    if (dot < 0)
                    {
                        Vector3.Negate(ref normal, out normal);
                        dot = -dot;
                    }

                    //Support contacts are all contacts on the feet of the character- a set that include contacts that support traction and those which do not.

                    Vector3.Dot(ref normal, ref downDirection, out dot);
                    if (dot > SideContactThreshold)
                    {
                        HasSupport = true;

                        //It is a support contact!
                        //Don't include a reference to the actual contact object.
                        //It's safer to return copies of a contact data struct.
                        var supportContact = new SupportContact()
                            {
                                Contact = new ContactData()
                                {
                                    Position = c.Contact.Position,
                                    Normal = normal,
                                    PenetrationDepth = c.Contact.PenetrationDepth,
                                    Id = c.Contact.Id
                                },
                                Support = pair.BroadPhaseOverlap.EntryA != body.CollisionInformation ? (Collidable)pair.BroadPhaseOverlap.EntryA : (Collidable)pair.BroadPhaseOverlap.EntryB
                            };

                        //But is it a traction contact?
                        //Traction contacts are contacts where the surface normal is flat enough to stand on.
                        //We want to check if slope < maxslope so:
                        //Acos(normal dot down direction) < maxSlope => normal dot down direction > cos(maxSlope)
                        if (dot > cosMaximumSlope)
                        {
                            //The slope is shallow enough that there is traction.
                            supportContact.HasTraction = true;
                            HasTraction = true;
                        }

                        supports.Add(supportContact);
                    }
                }

            }


            //Cast a ray straight down.
            SupportRayData = null;
            bottomHeight = character.Body.Radius;
            //If the contacts aren't available to support the character, raycast down to find the ground.
            if (!HasTraction && hadTraction)
            {

                //TODO: could also require that the character has a nonzero movement direction in order to use a ray cast.  Questionable- would complicate the behavior on edges.
                float length = hadTraction ? bottomHeight + maximumAssistedDownStepHeight : bottomHeight;
                Ray ray = new Ray(body.Position, downDirection);

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
            bool tryingToMove = character.HorizontalMotionConstraint.MovementDirection.LengthSquared() > 0;
            if (!HasTraction && hadTraction && tryingToMove)
            {

                Ray ray = new Ray(body.Position +
                    new Vector3(character.HorizontalMotionConstraint.MovementDirection.X, 0, character.HorizontalMotionConstraint.MovementDirection.Y) * (character.Body.Radius), downDirection);

                //Have to test to make sure the ray doesn't get obstructed.  This could happen if the character is deeply embedded in a wall; we wouldn't want it detecting things inside the wall as a support!
                Ray obstructionRay;
                obstructionRay.Position = body.Position;
                obstructionRay.Direction = ray.Position - obstructionRay.Position;
                if (!character.QueryManager.RayCastHitAnything(obstructionRay, 1))
                {
                    //The origin isn't obstructed, so now ray cast down.
                    float length = hadTraction ? bottomHeight + maximumAssistedDownStepHeight : bottomHeight;
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
                Vector3 horizontalOffset = new Vector3(character.HorizontalMotionConstraint.MovementDirection.X, 0, character.HorizontalMotionConstraint.MovementDirection.Y);
                Vector3.Cross(ref horizontalOffset, ref downDirection, out horizontalOffset);
                Vector3.Multiply(ref horizontalOffset, character.Body.Radius, out horizontalOffset);
                Ray ray = new Ray(body.Position + horizontalOffset, downDirection);

                //Have to test to make sure the ray doesn't get obstructed.  This could happen if the character is deeply embedded in a wall; we wouldn't want it detecting things inside the wall as a support!
                Ray obstructionRay;
                obstructionRay.Position = body.Position + downDirection * body.Radius * .5f;
                obstructionRay.Direction = ray.Position - obstructionRay.Position;
                if (!character.QueryManager.RayCastHitAnything(obstructionRay, 1))
                {
                    //The origin isn't obstructed, so now ray cast down.
                    float length = hadTraction ? bottomHeight + maximumAssistedDownStepHeight : bottomHeight;
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
                Vector3 horizontalOffset = new Vector3(character.HorizontalMotionConstraint.MovementDirection.X, 0, character.HorizontalMotionConstraint.MovementDirection.Y);
                Vector3.Cross(ref downDirection, ref horizontalOffset, out horizontalOffset);
                Vector3.Multiply(ref horizontalOffset, character.Body.Radius, out horizontalOffset);
                Ray ray = new Ray(body.Position + horizontalOffset, downDirection);

                //Have to test to make sure the ray doesn't get obstructed.  This could happen if the character is deeply embedded in a wall; we wouldn't want it detecting things inside the wall as a support!
                Ray obstructionRay;
                obstructionRay.Position = body.Position + downDirection * body.Radius * .5f;
                obstructionRay.Direction = ray.Position - obstructionRay.Position;
                if (!character.QueryManager.RayCastHitAnything(obstructionRay, 1))
                {
                    //The origin isn't obstructed, so now ray cast down.
                    float length = hadTraction ? bottomHeight + maximumAssistedDownStepHeight : bottomHeight;
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
            if (character.QueryManager.RayCast(ray, length, out earliestHit, out earliestHitObject))
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
                if (dot > cosMaximumSlope)
                {
                    //It has traction!
                    hasTraction = true;
                    supportRayData = new SupportRayData() { HitData = earliestHit, HitObject = earliestHitObject, HasTraction = true };
                }
                else
                    supportRayData = new SupportRayData() { HitData = earliestHit, HitObject = earliestHitObject };
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
    /// Convenience collection for finding the subset of contacts in a supporting contact list that have traction.
    /// </summary>
    public struct TractionSupportCollection : IEnumerable<ContactData>
    {
        RawList<SupportContact> supports;

        public TractionSupportCollection(RawList<SupportContact> supports)
        {
            this.supports = supports;
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(supports);
        }

        IEnumerator<ContactData> IEnumerable<ContactData>.GetEnumerator()
        {
            return new Enumerator(supports);
        }

        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()
        {
            return new Enumerator(supports);
        }

        /// <summary>
        /// Enumerator type for the TractionSupportCollection.
        /// </summary>
        public struct Enumerator : IEnumerator<ContactData>
        {
            int currentIndex;
            RawList<SupportContact> supports;

            /// <summary>
            /// Constructs the enumerator.
            /// </summary>
            /// <param name="supports">Support list to enumerate.</param>
            public Enumerator(RawList<SupportContact> supports)
            {
                currentIndex = -1;
                this.supports = supports;
            }

            /// <summary>
            /// Gets the current contact data.
            /// </summary>
            public ContactData Current
            {
                get { return supports.Elements[currentIndex].Contact; }
            }

            public void Dispose()
            {
            }

            object System.Collections.IEnumerator.Current
            {
                get { return Current; }
            }

            /// <summary>
            /// Moves to the next traction contact.  It skips contacts with normals that cannot provide traction.
            /// </summary>
            /// <returns></returns>
            public bool MoveNext()
            {
                while (++currentIndex < supports.Count)
                {
                    if (supports.Elements[currentIndex].HasTraction)
                        return true;
                }
                return false;
            }

            public void Reset()
            {
                currentIndex = -1;
            }
        }
    }

    /// <summary>
    /// Contact which acts as a support for the character controller.
    /// </summary>
    public struct SupportContact
    {
        /// <summary>
        /// Contact information at the support.
        /// </summary>
        public ContactData Contact;
        /// <summary>
        /// Object that created this contact with the character.
        /// </summary>
        public Collidable Support;
        /// <summary>
        /// Whether or not the contact was found to have traction.
        /// </summary>
        public bool HasTraction;
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
