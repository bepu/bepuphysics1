using System;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionRuleManagement;

namespace BEPUphysics.BroadPhaseSystems
{
    /// <summary>
    /// Superclass of all objects which live inside the broad phase.
    /// The BroadPhase will generate pairs between BroadPhaseEntries.
    /// </summary>
    public abstract class BroadPhaseEntry : ICollisionRulesOwner, IBoundingBoxOwner
    {
        internal int hashCode;
        protected BroadPhaseEntry()
        {
            CollisionRules = new CollisionRules();
            collisionRulesUpdatedDelegate = CollisionRulesUpdated;

            hashCode = (int)(base.GetHashCode() * 0xd8163841);
        }

        /// <summary>
        /// Gets the object's hash code.
        /// </summary>
        /// <returns>Hash code for the object.</returns>
        public override int GetHashCode()
        {
            return hashCode;
        }

        private Action collisionRulesUpdatedDelegate;
        protected abstract void CollisionRulesUpdated();

        protected internal BoundingBox boundingBox;
        /// <summary>
        /// Gets the bounding box of the entry.
        /// </summary>
        public BoundingBox BoundingBox
        {
            get { return boundingBox; }
        }

        protected internal abstract bool IsActive { get; }

        internal CollisionRules collisionRules;
        /// <summary>
        /// Gets the entry's collision rules.
        /// </summary>
        public CollisionRules CollisionRules
        {
            get { return collisionRules; }
            set
            {
                if (collisionRules != value)
                {
                    if (collisionRules != null)
                        collisionRules.CollisionRulesChanged -= collisionRulesUpdatedDelegate;
                    collisionRules = value;
                    if (collisionRules != null)
                        collisionRules.CollisionRulesChanged += collisionRulesUpdatedDelegate;
                    CollisionRulesUpdated();
                }
            }
        }

        /// <summary>
        /// Tests a ray against the entry.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length, in units of the ray's direction's length, to test.</param>
        /// <param name="rayHit">Hit location of the ray on the entry, if any.</param>
        /// <returns>Whether or not the ray hit the entry.</returns>
        public abstract bool RayCast(Ray ray, float maximumLength, out RayHit rayHit);

        
        /// <summary>
        /// Gets or sets the user data associated with this entry.
        /// </summary>
        public object Tag { get; set; }


    }

}
