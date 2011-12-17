using System;
using System.Collections.Generic;
using BEPUphysics;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Collidables.MobileCollidables;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.SampleCode
{
    /// <summary>
    /// Handles radial impulse applications on nearby objects when activated.
    /// </summary>
    public class Explosion
    {
        /// <summary>
        /// Re-used list of entities hit by the explosion.
        /// </summary>
        private readonly List<BroadPhaseEntry> affectedEntries = new List<BroadPhaseEntry>();

        /// <summary>
        /// Constructs an explosion.
        /// </summary>
        /// <param name="pos">Initial position of the explosion.</param>
        /// <param name="explosionMagnitude">Base strength of the blast as applied in units of impulse.</param>
        /// <param name="maxDist">Maximum radius of effect.</param>
        /// <param name="containingSpace">Space in which the explosion resides.</param>
        public Explosion(Vector3 pos, float explosionMagnitude, float maxDist, Space containingSpace)
        {
            Position = pos;
            Magnitude = explosionMagnitude;
            MaxDistance = maxDist;
            Space = containingSpace;
        }

        /// <summary>
        /// Gets or sets the current position of the explosion.
        /// </summary>
        public Vector3 Position { get; set; }

        /// <summary>
        /// Gets or sets the base strength of the blast.
        /// </summary>
        public float Magnitude { get; set; }

        /// <summary>
        /// Gets or sets the maximum distance that the explosion will affect.
        /// </summary>
        public float MaxDistance { get; set; }

        /// <summary>
        /// Gets or sets the space that the explosion will explode in.
        /// </summary>
        public Space Space { get; set; }

        /// <summary>
        /// Detonates the explosion, applying impulses to applicable physically simulated entities.
        /// </summary>
        public void Explode()
        {
            //Ask the broadphase system which entities are in the explosion region.
            Space.BroadPhase.QueryAccelerator.GetEntries(new BoundingSphere(Position, MaxDistance), affectedEntries);

            foreach (BroadPhaseEntry entry in affectedEntries)
            {
                var entityCollision = entry as EntityCollidable;
                if (entityCollision != null)
                {
                    var e = entityCollision.Entity;
                    //Don't bother applying impulses to kinematic entities; they have infinite inertia.
                    if (e.IsDynamic)
                    {
                        Vector3 offset = e.Position - Position;
                        float distanceSquared = offset.LengthSquared();
                        if (distanceSquared > Toolbox.Epsilon) //Be kind to the engine and don't give it a value divided by zero.
                        {
                            var distance = (float)Math.Sqrt(distanceSquared);
                            //This applies a force inversely proportional to the distance.
                            //Note the extra distance term in the denominator.  This normalizes the
                            //offset, resulting in a quadratic explosion falloff.
                            //A linear falloff could be accomplished by not including the extra distance term.
                            e.LinearMomentum += (offset * (Magnitude / (distanceSquared * distance)));
                            //The above only applies a linear impulse, which is quick and usually sufficient to look like an explosion.
                            //If you want some extra chaotic spinning, try applying an angular impulse.
                            //Using e.ApplyImpulse with an appropriate impulse location or e.applyAngularImpulse will do the job.

                        }
                        else
                        {
                            e.LinearMomentum += (new Vector3(0, Magnitude, 0));
                        }
                    }
                }
            }

            affectedEntries.Clear();
        }
    }
}