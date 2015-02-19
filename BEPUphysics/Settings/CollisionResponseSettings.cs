namespace BEPUphysics.Settings
{
    ///<summary>
    /// Contains global settings relating to the collision response system.
    ///</summary>
    public static class CollisionResponseSettings
    {
        /// <summary>
        /// Impact velocity above which the full bounciness of a material pair is taken into account.  Below the threshold, the extra energy introduced by the bounce is reduced. The lower the velocity is relative to the threshold, the lower the effective bounciness.
        /// Defaults to 1.
        /// </summary>
        public static float BouncinessVelocityThreshold = 1;

        /// <summary>
        /// Maximum speed at which interpenetrating objects will attempt to undo any overlap.
        /// Defaults to 2.
        /// </summary>
        public static float MaximumPenetrationRecoverySpeed = 2;

        /// <summary>
        /// Fraction of position error to convert into corrective momentum.
        /// Defaults to 0.2.
        /// </summary>
        public static float PenetrationRecoveryStiffness = 0.2f;

        /// <summary>
        /// Magnitude of relative velocity at a contact point below which staticFriction is used.
        /// dynamicFriction is used when velocity exceeds this threshold.
        /// Defaults to 0.2.
        /// </summary>
        public static float StaticFrictionVelocityThreshold = 0.2f;

        /// <summary>
        /// Value by which a collision pair's friction coefficient will be multiplied to get the twist friction coefficient.
        /// Defaults to 1.
        /// </summary>
        public static float TwistFrictionFactor = 1f;

        /// <summary>
        /// <para>Softness multiplier used by collision penetration constraints. Higher softness values allow more velocity error and make things look 'squishier'. Defaults to 0.001.</para>
        /// <para>Note that this value is not used directly by constraints; it is first scaled by the raw inverse effective mass. This allows consistent behavior across objects with different masses.</para>
        /// </summary>
        public static float Softness = .001f;


    }
}
