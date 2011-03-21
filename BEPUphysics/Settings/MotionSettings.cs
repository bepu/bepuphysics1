using BEPUphysics.PositionUpdating;
using System;
using Microsoft.Xna.Framework;

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
        /// Values should be between 0 and 0.99f.  The smaller the value, the smaller the shapes used
        /// to perform CCD are, and more collisions are missed.
        ///</summary>
        public static float CoreShapeScaling
        {
            get
            {
                return coreShapeScaling;
            }
            set
            {
                //The reason why it doesn't allow up to 1.0 is there exist systems that require a small margin between the full minimum radius and the core shape.
                coreShapeScaling = MathHelper.Clamp(value, 0, .99f);
            }
        }
        static float coreShapeScaling = .8f;
        /// <summary>
        /// The default position updating mode used by position updateables.
        /// </summary>
        public static PositionUpdateMode DefaultPositionUpdateMode = PositionUpdateMode.Discrete;
    }
}
