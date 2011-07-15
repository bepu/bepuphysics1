using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.CollisionTests;
using BEPUphysics.DataStructures;
using Microsoft.Xna.Framework;
using BEPUphysics;
using BEPUphysics.Collidables;

namespace BEPUphysicsDemos
{
    /// <summary>
    /// Analyzes the contacts on the character's body to find supports.
    /// </summary>
    public class SupportFinder
    {
        RawList<SupportContact> supports = new RawList<SupportContact>();

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
                        float factor = 1f / supports.Count;
                        Vector3.Multiply(ref toReturn.Position, factor, out toReturn.Position);
                        float length = toReturn.Normal.LengthSquared();
                        if (length < Toolbox.BigEpsilon)
                        {
                            //It's possible that the normals have cancelled each other out- that would be bad!
                            //Just use an arbitrary support's normal in that case.
                            toReturn.Normal = supports.Elements[0].Contact.Normal;
                        }
                        else
                        {
                            Vector3.Multiply(ref toReturn.Normal, factor / (float)Math.Sqrt(length), out toReturn.Normal);
                        }
                    }
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
                            HasTraction = SupportRayData.Value.HasTraction
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
                        float factor = 1f / withTraction;
                        Vector3.Multiply(ref toReturn.Position, factor, out toReturn.Position);
                        float length = toReturn.Normal.LengthSquared();
                        if (length < Toolbox.BigEpsilon)
                        {
                            //It's possible that the normals have cancelled each other out- that would be bad!
                            //Just use an arbitrary support's normal in that case.
                            toReturn.Normal = supports.Elements[0].Contact.Normal;
                        }
                        else
                        {
                            Vector3.Multiply(ref toReturn.Normal, factor / (float)Math.Sqrt(length), out toReturn.Normal);
                        }
                    }
                    if (withTraction > 0)
                        return toReturn;
                }
                //No support contacts; fall back to the raycast result...
                if (SupportRayData != null && SupportRayData.Value.HasTraction)
                {
                    return new SupportData()
                    {
                        Position = SupportRayData.Value.HitData.Location,
                        Normal = SupportRayData.Value.HitData.Normal,
                        HasTraction = SupportRayData.Value.HasTraction
                    };
                }
                else
                {
                    return null;
                }

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

        CharacterController character;

        internal float cosMaximumSlope = (float)Math.Cos(MathHelper.PiOver4);
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
            }
        }

        /// <summary>
        /// Constructs a new support finder.
        /// </summary>
        /// <param name="character">Character to analyze.</param>
        public SupportFinder(CharacterController character)
        {
            this.character = character;
        }

        /// <summary>
        /// Updates the collection of supporting contacts.
        /// </summary>
        public void UpdateSupports()
        {


            //First, raycast down to find the ground.
            //Start the ray halfway between the center of the shape and the bottom of the shape.  That extra margin prevents it from getting stuck in the ground and returning t = 0 unhelpfully.
            var body = character.Body;
            float length = HasTraction ? body.Height * .25f + body.CollisionInformation.Shape.CollisionMargin + character.StepHeight : body.Height * .25f + body.CollisionInformation.Shape.CollisionMargin;
            Vector3 downDirection = character.Body.OrientationMatrix.Down; //For a cylinder orientation-locked to the Up axis, this is always {0, -1, 0}.  Keeping it generic doesn't cost much.
            Ray ray = new Ray(body.Position + downDirection * body.Height * .25f, downDirection);

            //Reset traction/support.
            HasTraction = false;
            HasSupport = false;

            BoundingBox boundingBox = body.CollisionInformation.BoundingBox;
            RayHit earliestHit = new RayHit() { T = float.MaxValue };
            Collidable earliestHitObject = null;
            foreach (var collidable in body.CollisionInformation.OverlappedCollidables)
            {
                //Check to see if the collidable is hit by the ray.
                bool intersects;
                collidable.BoundingBox.Intersects(ref boundingBox, out intersects);
                if (intersects)
                {
                    //Is it an earlier hit than the current earliest?
                    RayHit hit;
                    if (collidable.RayCast(ray, length, out hit) && hit.T < earliestHit.T)
                    {
                        earliestHit = hit;
                        earliestHitObject = collidable;
                    }
                }
            }
            if (earliestHit.T != float.MaxValue)
            {
                //A collidable was hit!  It's a support, but does it provide traction?
                HasSupport = true;
                earliestHit.Normal.Normalize();
                float dot;
                Vector3.Dot(ref downDirection, ref earliestHit.Normal, out dot);
                if (dot > cosMaximumSlope || dot < -cosMaximumSlope)
                {
                    //It has traction!
                    HasTraction = true;
                    SupportRayData = new SupportRayData() { HitData = earliestHit, HitObject = earliestHitObject, HasTraction = true };
                }
                else
                    SupportRayData = new SupportRayData() { HitData = earliestHit, HitObject = earliestHitObject };
            }
            else
                SupportRayData = null;


            supports.Clear();
            //Analyze the cylinder's contacts to see if we're supported.
            //Anything that can be a support will have a normal that's off horizontal.
            //That could be at the top or bottom, so only consider points on the bottom half of the shape.
            Vector3 position = character.Body.Position;

            foreach (var pair in character.Body.CollisionInformation.Pairs)
            {
                foreach (var c in pair.Contacts)
                {
                    //Compute the offset from the position of the character's body to the contact.
                    Vector3 contactOffset;
                    Vector3.Subtract(ref c.Contact.Position, ref position, out contactOffset);
                    //Compare that offset with the down vector.  If the dot product is positive, the offset points in the same direction as the down direction, which means it's on the bottom half of the shape.
                    float dot;
                    Vector3.Dot(ref contactOffset, ref downDirection, out dot);
                    if (dot > 0)
                    {
                        //This contact is on the bottom half!  Check to see if its normal is suitable for a 'support' contact.
                        //Support contacts are all contacts on the feet of the character- a set that include contacts that support traction and those which do not.
                        Vector3.Dot(ref c.Contact.Normal, ref downDirection, out dot);
                        if (dot < -.01f || dot > .01f) //Check both signs; the normal could point either direction since it hasn't been calibrated.
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
                                        Normal = c.Contact.Normal,
                                        PenetrationDepth = c.Contact.PenetrationDepth,
                                        Id = c.Contact.Id
                                    },
                                    Support = pair.BroadPhaseOverlap.EntryA != body.CollisionInformation ? (Collidable)pair.BroadPhaseOverlap.EntryA : (Collidable)pair.BroadPhaseOverlap.EntryB
                                };

                            //But is it a traction contact?
                            //Traction contacts are contacts where the surface normal is flat enough to stand on.
                            //We want to check if slope < maxslope so:
                            //Acos(normal dot down direction) < maxSlope => normal dot down direction > cos(maxSlope)
                            if (dot > cosMaximumSlope || dot < -cosMaximumSlope)
                            {
                                //The slope is shallow enought hat 
                                supportContact.HasTraction = true;
                                HasTraction = true;
                            }

                            supports.Add(supportContact);
                        }
                    }
                }
            }
        }


        /// <summary>
        /// Cleans up the support finder, preparing it for inactivity.
        /// </summary>
        internal void CleanUp()
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
    }
}
