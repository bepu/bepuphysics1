using System;
using System.Collections.Generic;
using System.Diagnostics;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.CollisionTests;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUutilities;

namespace BEPUphysics.Character
{
    /// <summary>
    /// Used by characters to figure out how to interpret a character's collisions for the purposes of movement.
    /// </summary>
    public class CharacterContactCategorizer
    {
        private float tractionThreshold;
        private float supportThreshold;
        private float headThreshold;
        /// <summary>
        /// Gets or sets the value compared against the result of dot(outward facing contact normal, character down direction) to determine if a character has traction because of a contact.
        /// A value near 1 implies that the character will have traction only when the support normal is almost aligned with the character's vertical axis.
        /// A value near epsilon implies that the character can walk up extremely steep slopes with traction.
        /// </summary>
        public float TractionThreshold
        {
            get { return tractionThreshold; }
            set
            {
                if (value < Toolbox.BigEpsilon || value > 1)
                    throw new ArgumentException("Traction threshold values must range from " + Toolbox.BigEpsilon + " to 1, inclusive.");
                tractionThreshold = value;
            }
        }

        /// <summary>
        /// Gets or sets the value compared against the result of dot(outward facing contact normal, character down direction) to determine if a character has support because of a contact.
        /// A value near 1 implies that the character will have support only when the support normal is almost aligned with the character's vertical axis.
        /// A value near epsilon implies that the character will consider very steep surfaces as supports.
        /// Should be less than or equal to the TractionThreshold.
        /// </summary>
        public float SupportThreshold
        {
            get { return supportThreshold; }
            set
            {
                if (value < Toolbox.BigEpsilon || value > 1)
                    throw new ArgumentException("Support threshold values must range from " + Toolbox.BigEpsilon + " to 1, inclusive.");
                supportThreshold = value;
            }
        }

        /// <summary>
        /// Gets or sets the value compared against the result of dot(outward facing contact normal, character down direction) to determine if a contact is on top of the character.
        /// A value near -1 implies that the contact will only be considered a 'head' contact when the support normal is almost aligned with the character's vertical axis.
        /// A value near -epsilon implies that almost all upper contacts will be considered head contacts.
        /// </summary>
        public float HeadThreshold
        {
            get { return headThreshold; }
            set
            {
                if (value < -1 || value > -Toolbox.BigEpsilon)
                    throw new ArgumentException("Head threshold values must range from -1 to " + -Toolbox.BigEpsilon + ", inclusive.");
                headThreshold = value;
            }
        }

        /// <summary>
        /// Gets or sets the maximum slope that a character can have traction on.
        /// </summary>
        public float MaximumTractionSlope
        {
            get { return (float)Math.Acos(TractionThreshold); }
            set { TractionThreshold = (float)Math.Cos(value); }
        }
        /// <summary>
        /// Gets or sets the maximum slope that a character can be supported by.
        /// </summary>
        public float MaximumSupportSlope
        {
            get { return (float)Math.Acos(SupportThreshold); }
            set { SupportThreshold = (float)Math.Cos(value); }
        }

        /// <summary>
        /// Creates a new character contact categorizer.
        /// </summary>
        /// <param name="maximumTractionSlope">Maximum slope that a character can have traction on.</param>
        /// <param name="maximumSupportSlope">Maximum slope that a character can be supported by.</param>
        /// <param name="headThreshold">Value compared against the result of dot(outward facing contact normal, character down direction) to determine if a contact is on top of the character.
        /// A value near -1 implies that the contact will only be considered a 'head' contact when the support normal is almost aligned with the character's vertical axis.
        /// A value near -epsilon implies that almost all upper contacts will be considered head contacts.</param>
        public CharacterContactCategorizer(float maximumTractionSlope, float maximumSupportSlope, float headThreshold = -.01f)
        {
            MaximumTractionSlope = maximumTractionSlope;
            MaximumSupportSlope = maximumSupportSlope;
            HeadThreshold = headThreshold;
            Debug.Assert(SupportThreshold <= TractionThreshold, "The character's support threshold should be no higher than the traction threshold for the traction threshold to be meaningful.");
        }

        /// <summary>
        /// Takes a collision pair and distributes its contacts into categories according to the categorizer's thresholds.
        /// </summary>
        /// <param name="pair">Source of the contacts to categorize.</param>
        /// <param name="characterCollidable">Collidable associated with the character.</param>
        /// <param name="downDirection">Down direction of the character.</param>
        /// <param name="tractionContacts">List to contain the traction contacts found in the input contacts list.</param>
        /// <param name="supportContacts">List to contain the support contacts found in the input contacts list.</param>
        /// <param name="sideContacts">List to contain the side contacts found in the input contacts list.</param>
        /// <param name="headContacts">List to contain the head contacts found in the input contacts list.</param>
        /// <typeparam name="TOutputContacts">List type used to store the output character contact structs.</typeparam>
        public void CategorizeContacts<TOutputContacts>(CollidablePairHandler pair, EntityCollidable characterCollidable, ref Vector3 downDirection, 
                                                        ref TOutputContacts tractionContacts, ref TOutputContacts supportContacts, ref TOutputContacts sideContacts, ref TOutputContacts headContacts)
                                                            where TOutputContacts : IList<CharacterContact>
        {
            var contactCollection = pair.Contacts;
            for (int i = pair.ContactCount - 1; i >= 0; --i)
            {
                var contactInfo = contactCollection[i];
                CharacterContact characterContact;
                characterContact.Contact = new ContactData(contactInfo.Contact);
                characterContact.Collidable = pair.CollidableA == characterCollidable ? pair.CollidableB : pair.CollidableA;

                //It's possible that a subpair has a non-normal collision rule, even if the parent pair is normal.
                //Note that only contacts with nonnegative penetration depths are used.
                //Negative depth contacts are 'speculative' in nature.
                //If we were to use such a speculative contact for support, the character would find supports
                //in situations where it should not.
                //This can actually be useful in some situations, but keep it disabled by default.
                if (contactInfo.Pair.CollisionRule != CollisionRule.Normal || characterContact.Contact.PenetrationDepth < 0)
                    continue;

                float dot;
                Vector3 offset;
                Vector3.Subtract(ref characterContact.Contact.Position, ref characterCollidable.worldTransform.Position, out offset);
                Vector3.Dot(ref characterContact.Contact.Normal, ref offset, out dot);
                if (dot < 0)
                {
                    //The normal should face outward.
                    Vector3.Negate(ref characterContact.Contact.Normal, out characterContact.Contact.Normal);
                }

                Vector3.Dot(ref characterContact.Contact.Normal, ref downDirection, out dot);
                if (dot > SupportThreshold)
                {
                    //It's a support.
                    supportContacts.Add(characterContact);
                    if (dot > TractionThreshold)
                    {
                        //It's a traction contact.
                        tractionContacts.Add(characterContact);
                    }
                    else
                    {
                        //Considering the side contacts to be supports can help with upstepping.
                        sideContacts.Add(characterContact);
                    }
                }
                else if (dot < HeadThreshold)
                {
                    //It's a head contact.
                    headContacts.Add(characterContact);
                }
                else
                {
                    //It's a side contact.  These could obstruct the stepping.
                    sideContacts.Add(characterContact);

                }
            }
        }
    }
}
