using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.UpdateableSystems;
using BEPUphysics;
using Microsoft.Xna.Framework;
using BEPUphysics.MathExtensions;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.Materials;
using BEPUphysics.PositionUpdating;
using BEPUphysics.DataStructures;
using System.Diagnostics;

namespace BEPUphysicsDemos
{
    public class CharacterController : Updateable, IBeforePositionUpdateUpdateable, IDuringForcesUpdateable, IEndOfTimeStepUpdateable
    {
        public Capsule Body { get; private set; }

        public Vector2 MovementDirection;
        public float Speed = 20;
        float cosMaximumTractionSlope = (float)Math.Cos(MathHelper.PiOver4);
        /// <summary>
        /// Gets or sets the maximum slope on which the character will have traction.
        /// </summary>
        public float MaximumTractionSlope
        {
            get
            {
                return (float)Math.Acos(MathHelper.Clamp(cosMaximumTractionSlope, -1, 1));
            }
            set
            {
                cosMaximumTractionSlope = (float)Math.Cos(value);
            }
        }

        /// <summary>
        /// Gets or sets the maximum change in speed that the character will apply in order to stay connected to the ground.
        /// </summary>
        public float GlueSpeed { get; set; }

        public CharacterController()
        {
            Body = new Capsule(Vector3.Zero, 1.7f, .3f, 10);
            Body.PositionUpdateMode = PositionUpdateMode.Continuous;
            Body.LocalInertiaTensorInverse = new Matrix3X3();
            Body.CollisionInformation.Events.CreatingPair += RemoveFriction;
            GlueSpeed = 20;
        }

        void RemoveFriction(EntityCollidable sender, BroadPhaseEntry other, INarrowPhasePair pair)
        {
            var collidablePair = pair as CollidablePairHandler;
            if (collidablePair != null)
            {
                //The default values for InteractionProperties is all zeroes- zero friction, zero bounciness.
                //That's exactly how we want the character to behave when hitting objects.
                collidablePair.UpdateMaterialProperties(new InteractionProperties());
            }
        }

        void IDuringForcesUpdateable.Update(float dt)
        {
            Body.LinearVelocity = new Vector3(Speed * MovementDirection.X, Body.LinearVelocity.Y, Speed * MovementDirection.Y);
        }

        RawList<ContactInformation> supportingContacts = new RawList<ContactInformation>();
        RawList<ContactInformation> tractionContacts = new RawList<ContactInformation>();

        void IBeforePositionUpdateUpdateable.Update(float dt)
        {
            //The solver has run.  Sanitize its output to ensure that the character has the velocities we want before allowing the position updater to run.
            //Scan the traction contacts.  Check their relative velocities along the contact normal.  Compute the minimum velocity change necessary
            //to ensure that the contact does not separate.  If it's less than the 'glue speed,' apply the velocity change.
            //Technically, 'glue' impulses could de-glue other contacts.  Iterating could help a bit.

            //First, collect and classify the contacts the body has with its environment.
            supportingContacts.Clear();
            tractionContacts.Clear();
            Vector3 down = Body.OrientationMatrix.Down;
            Vector3 bodyPosition = Body.Position;
            float halfLength = Body.Length * .5f;
            foreach (var pair in Body.CollisionInformation.Pairs)
            {
                for (int i = 0; i < pair.Contacts.Count; i++)
                {
                    var contactData = pair.Contacts[i];
                    Vector3 offsetToContact;
                    Vector3.Subtract(ref contactData.Contact.Position, ref bodyPosition, out offsetToContact);
                    float dot;
                    Vector3.Dot(ref offsetToContact, ref down, out dot);
                    if (dot > halfLength)
                    {
                        //The contact is on the bottom cap of the capsule.  It's a 'supporting' contact!
                        supportingContacts.Add(contactData);
                        //Now, check to see if it's also a traction contact.
                        Vector3 calibratedNormal;
                        Vector3.Dot(ref offsetToContact, ref contactData.Contact.Normal, out dot);
                        if (dot < 0)
                            Vector3.Negate(ref contactData.Contact.Normal, out calibratedNormal);
                        else
                            calibratedNormal = contactData.Contact.Normal;
                        //The normal is facing out of the capsule.  Find the angle of the slope relative to the down vector.
                        Vector3.Dot(ref calibratedNormal, ref down, out dot);
                        //Instead of taking Math.Acos(dot) to find an angle, we can just compare the dot product directly to the cosine of the maximum slope.
                        if (dot > cosMaximumTractionSlope)
                            tractionContacts.Add(contactData);
                    }
                }
            }

            //Look at all the traction contacts.  Find the shallowest one.
            //We have to find the shallowest one because performing glue procedures on all of the contacts would result in the character being sucked through geometry as they walked 
            //across intersecting geometry.

            if (tractionContacts.Count > 0)
            {
                int shallowestIndex = 0;
                for (int i = 1; i < tractionContacts.Count; i++)
                {
                    if (tractionContacts.Elements[i].Contact.PenetrationDepth < tractionContacts.Elements[shallowestIndex].Contact.PenetrationDepth)
                        shallowestIndex = i;
                }

                //Analyze their velocities to see if they are separating along the normal.  If they are, apply a linear velocity to the character to prevent the
                //objects from separating, if possible.

                var shallowestTraction = tractionContacts.Elements[shallowestIndex];
                //Compute the normal velocity.
                float normalVelocity;
                Vector3.Dot(ref shallowestTraction.Contact.Normal, ref shallowestTraction.RelativeVelocity, out normalVelocity);
                normalVelocity *= -1;
                if (normalVelocity > 0 && normalVelocity < GlueSpeed)
                {
                    //The normal velocity is correctable, and at least superficially separating.
                    //Check the penetration at the contact.  We don't want to remove all the penetration recovery speed, just enough to ensure that the character isn't going to pop out of the surface in a single frame.
                    //Bias the separation up a bit; we'd like to keep the objects just barely in contact.
                    float excessSeparation = normalVelocity * dt - shallowestTraction.Contact.PenetrationDepth + .001f;

                    if (excessSeparation > 0)
                    {
                        //Get rid of the excess!
                        float velocityChange = excessSeparation / dt;
                        //Calibrated normal could be cached from the previous stage to eliminate this part.
                        Vector3 calibratedNormal;
                        Vector3 offsetToContact;
                        Vector3.Subtract(ref shallowestTraction.Contact.Position, ref bodyPosition, out offsetToContact);
                        float dot;
                        Vector3.Dot(ref offsetToContact, ref shallowestTraction.Contact.Normal, out dot);
                        if (dot < 0)
                            Vector3.Negate(ref shallowestTraction.Contact.Normal, out calibratedNormal);
                        else
                            calibratedNormal = shallowestTraction.Contact.Normal;
                        //Normal now faces out of the capsule.
                        //Apply the velocity change!
                        Body.LinearVelocity += calibratedNormal * velocityChange;

                    }
                }

            }
        }

        void IEndOfTimeStepUpdateable.Update(float dt)
        {
            //throw new NotImplementedException();
        }

        bool stop;

        public void Jump()
        {
            stop = true;
            Body.LinearVelocity = Body.OrientationMatrix.Up * 5;
        }

        public override void OnAdditionToSpace(ISpace newSpace)
        {
            //Add any supplements to the space too.
            newSpace.Add(Body);
        }
        public override void OnRemovalFromSpace(ISpace oldSpace)
        {
            //Remove any supplements from the space too.
            oldSpace.Remove(Body);
        }

    }
}
