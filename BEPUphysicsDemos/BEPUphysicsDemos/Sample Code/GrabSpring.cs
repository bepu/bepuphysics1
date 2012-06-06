using BEPUphysics.Entities;
using BEPUphysics.UpdateableSystems;
using Microsoft.Xna.Framework;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.SampleCode
{
    /// <summary>
    /// Grabs an entity at a specified location and applies corrective impulses to keep the grabbed location near the goal location.
    /// </summary>
    public class GrabSpring : Updateable, IDuringForcesUpdateable, IEndOfFrameUpdateable
    {
        /*
         * This class is a cleaned up version of the GrabSpring present in v0.11.0 and previous versions.
         * It's been moved into the demos as an example (and to clean up the main library).
         * 
         */

        /// <summary>
        /// Offset from the center of the entity to the grabbed location in local space.
        /// </summary>
        private Vector3 localOffset;


        /// <summary>
        /// Constructs a grab constraint.
        /// </summary>
        /// <param name="correctiveStrength">Factor of the position error to use in corrective impulses each frame.</param>
        /// <param name="linearDamp">Damping to apply to the grabbed entity's linear momentum.</param>
        /// <param name="angularDamp">Damping to apply to the grabbed entity's angular momentum.</param>
        public GrabSpring(float correctiveStrength, float linearDamp, float angularDamp)
        {
            CorrectionFactor = correctiveStrength;
            LinearDamping = linearDamp;
            AngularDamping = angularDamp;
        }

        /// <summary>
        /// Gets the grabbed entity.
        /// </summary>
        public Entity Entity { get; private set; }

        /// <summary>
        /// Gets the location that the entity will be pulled towards.
        /// </summary>
        public Vector3 GoalPosition { get; set; }

        /// <summary>
        /// Gets or sets the factor of the position error to use in corrective impulses each frame.
        /// </summary>
        public float CorrectionFactor { get; set; }

        /// <summary>
        /// Getes or sets damping to apply to the grabbed entity's linear momentum.
        /// </summary>
        public float LinearDamping { get; set; }

        /// <summary>
        /// Gets or sets damping to apply to the grabbed entity's angular momentum.
        /// </summary>
        public float AngularDamping { get; set; }

        /// <summary>
        /// Gets the last updated position of the grab location on the surface of the entity.
        /// </summary>
        public Vector3 GrabbedPosition { get; private set; }

        /// <summary>
        /// Reinitializes the grabbing constraint with new information.
        /// </summary>
        /// <param name="e">Entity to grab.</param>
        /// <param name="grabLocation">Location on the entity being grabbed in world space.</param>
        public void Setup(Entity e, Vector3 grabLocation)
        {
            Entity = e;
            localOffset = Vector3.Transform(grabLocation - e.Position, Quaternion.Conjugate(e.Orientation));
            GoalPosition = grabLocation;
        }


        /// <summary>
        /// Applies an appropriate impulse to correct the position of the entity.
        /// Called automatically by the space.
        /// </summary>
        /// <param name="dt">Time since last frame in simulation seconds.</param>
        void IDuringForcesUpdateable.Update(float dt)
        {
            if (Entity.IsDynamic)
            {
                Vector3 grabbedLocation = Matrix3X3.Transform(localOffset, Entity.OrientationMatrix) + Entity.Position;
                Vector3 positionError = GoalPosition - grabbedLocation;

                //Apply the force.
                Entity.ApplyImpulse(grabbedLocation, positionError * CorrectionFactor * dt * Entity.Mass);

                //Entities provide a little helper for damping.
                //This isn't exactly 'physically correct,' but
                //for what the grab spring is used for, it works fine.
                Entity.ModifyLinearDamping(LinearDamping);
                Entity.ModifyAngularDamping(AngularDamping);
            }
        }


        /// <summary>
        /// Updates the grab constraint's grab position after the end of a frame.
        /// </summary>
        /// <param name="dt">Time since last frame in simulation seconds.</param>
        void IEndOfFrameUpdateable.Update(float dt)
        {
            //Since the grabbed position is usually examined graphically, 
            //it's good to use the interpolated positions in case the 
            //engine is using internal time stepping and interpolation.
            GrabbedPosition = Matrix3X3.Transform(localOffset, Entity.BufferedStates.InterpolatedStates.OrientationMatrix) + Entity.BufferedStates.InterpolatedStates.Position;
        }
    }
}