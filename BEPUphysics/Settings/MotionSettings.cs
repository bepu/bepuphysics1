using BEPUphysics.PositionUpdating;
using System;
using BEPUutilities;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.Entities;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.BroadPhaseEntries;

namespace BEPUphysics.Settings
{
    ///<summary>
    /// Contains global settings about motion updating.
    ///</summary>
    public static class MotionSettings
    {
        ///<summary>
        /// The scaling to apply to the core shapes used for continuous collision detection tests.
        /// Values should be between 0 and 0.99f.  The smaller the value, the smaller the shapes used
        /// to perform CCD are, and more collisions are missed.
        /// Defaults to .8f.
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
        /// Defaults to Discrete.
        /// </summary>
        public static PositionUpdateMode DefaultPositionUpdateMode = PositionUpdateMode.Discrete;
        /// <summary>
        /// It is possible for an object in danger of being hit by a moving object to have a bounding box which
        /// does not contain the resulting motion, and CCD will fail to detect a secondary collision.
        /// Setting this to true will take into account nearby objects' velocities and use them to enlarge the 
        /// bounding box so that secondary collisions are not missed.
        /// The larger size of bounding boxes can cause an increase in collision pairs during stressful situations,
        /// which can harm performance.
        /// Defaults to false.
        /// </summary>
        public static bool UseExtraExpansionForContinuousBoundingBoxes;



        /// <summary>
        /// Delegate which determines if a given pair should be allowed to run continuous collision detection.
        /// This is only called for entities which are continuous and colliding with other objects.
        /// By default, this prevents CCD from being used in any pair where the pair's CollisionRule stops collision response.
        /// </summary>
        public static CCDFilter CCDFilter;

       
        static bool DefaultCCDFilter(CollidablePairHandler pair)
        {
            return pair.broadPhaseOverlap.collisionRule < CollisionRule.NoSolver;
        }

        static MotionSettings()
        {
            CCDFilter = DefaultCCDFilter;
        }

    }

    /// <summary>
    /// Delegate which determines if a given pair should be allowed to run continuous collision detection.
    /// This is only called for entities which are continuous and colliding with other objects.
    /// </summary>
    public delegate bool CCDFilter(CollidablePairHandler pair);
}
