using BEPUphysics.PositionUpdating;

namespace BEPUphysics.Settings
{
    ///<summary>
    /// Contains global settings about motion updating.
    ///</summary>
    public static class MotionSettings
    {
        ///<summary>
        /// Whether or not to use RK4 angular integration.  This can improve simulation quality sometimes, but not always.
        /// It has a slight performance impact.  Enabling this when ConserveAngularMomentum is set to true may be helpful.
        ///</summary>
        public static bool UseRk4AngularIntegration;
        ///<summary>
        /// Whether or not to conserve angular momentum.
        /// This produces slightly more realistic angular behavior, but can reduce stability.
        /// Consider using a smaller timestep, enabling RK4 angular integration, or both.
        ///</summary>
        public static bool ConserveAngularMomentum;
        ///<summary>
        /// The scaling to apply to the core shapes used for continuous collision detection tests.
        /// Values should be between 0 and 1.  The smaller the value, the smaller the shapes used
        /// to perform CCD are, and more collisions are missed.  It is not recommended to set this value
        /// to 1, however; not all collisions need to be caught.  In fact, catching too many may result in jerky
        /// motion.
        ///</summary>
        public static float CoreShapeScaling = .8f;
        /// <summary>
        /// The default position updating mode used by position updateables.
        /// </summary>
        public static PositionUpdateMode DefaultPositionUpdateMode = PositionUpdateMode.Discrete;
    }
}
