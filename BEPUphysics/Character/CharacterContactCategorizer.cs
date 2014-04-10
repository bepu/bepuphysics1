using System;
using System.Collections.Generic;
using System.Diagnostics;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.CollisionTests;
using BEPUphysicsDemos.AlternateMovement.Character;
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
        /// Takes all contacts associated with a particular collidable and distributes them into their associated categories according to the categorizer's thresholds.
        /// </summary>
        /// <param name="collidable">Collidable that generated the contacts with the character.</param>
        /// <param name="downDirection">Down direction of the character.</param>
        /// <param name="bodyPosition">Position of the character's body to use for calibration.</param>
        /// <param name="tractionContacts">List to contain the traction contacts found in the input contacts list.</param>
        /// <param name="supportContacts">List to contain the support contacts found in the input contacts list.</param>
        /// <param name="sideContacts">List to contain the side contacts found in the input contacts list.</param>
        /// <param name="headContacts">List to contain the head contacts found in the input contacts list.</param>
        /// <typeparam name="TOutputContacts">List type used to store the output character contact structs.</typeparam>
        public void CategorizeContacts<TOutputContacts>(Collidable collidable, ref Vector3 downDirection, ref Vector3 bodyPosition,
                                                        TOutputContacts tractionContacts, TOutputContacts supportContacts, TOutputContacts sideContacts, TOutputContacts headContacts)
                                                            where TOutputContacts : IList<CharacterContact>
        {
            CategorizeContacts(collidable.conta
        }

        /// <summary>
        /// Takes a list of contacts and distributes them into their associated categories according to the categorizer's thresholds.
        /// </summary>
        /// <param name="contacts">Contacts to categorize.</param>
        /// <param name="collidable">Collidable that generated the contacts with the character.</param>
        /// <param name="downDirection">Down direction of the character.</param>
        /// <param name="bodyPosition">Position of the character's body to use for calibration.</param>
        /// <param name="tractionContacts">List to contain the traction contacts found in the input contacts list.</param>
        /// <param name="supportContacts">List to contain the support contacts found in the input contacts list.</param>
        /// <param name="sideContacts">List to contain the side contacts found in the input contacts list.</param>
        /// <param name="headContacts">List to contain the head contacts found in the input contacts list.</param>
        /// <typeparam name="TInputContacts">List type containing the input contact structs.</typeparam>
        /// <typeparam name="TOutputContacts">List type used to store the output character contact structs.</typeparam>
        public void CategorizeContacts<TInputContacts, TOutputContacts>(ref TInputContacts contacts, Collidable collidable, ref Vector3 downDirection, ref Vector3 bodyPosition,
                                                                        TOutputContacts tractionContacts, TOutputContacts supportContacts, TOutputContacts sideContacts, TOutputContacts headContacts)
            where TInputContacts : IList<ContactData>
            where TOutputContacts : IList<CharacterContact>
        {
            CharacterContact characterContact;
            characterContact.Collidable = collidable;
            for (int i = contacts.Count - 1; i >= 0; --i)
            {
                characterContact.Contact = contacts[i];

                float dot;
                Vector3 offset;
                Vector3.Subtract(ref characterContact.Contact.Position, ref bodyPosition, out offset);
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
