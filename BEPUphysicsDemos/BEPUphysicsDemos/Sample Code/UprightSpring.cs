using System;
using BEPUphysics.Entities;
using BEPUphysics.UpdateableSystems;
using Microsoft.Xna.Framework;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.SampleCode
{
    /// <summary>
    /// Spring that attempts to keep a target entity oriented in a given direction by applying corrective torques.
    /// </summary>
    public class UprightSpring : Updateable, IDuringForcesUpdateable
    {
        /*
         * This class is a cleaned up version of the UprightConstraint present in v0.11.0 and previous versions.
         * It's been moved into the demos as an example (and to clean up the main library).
         * 
         * This could be relatively easily re-implemented as a SingleEntityConstraint.  The SingleEntityAngularMotor
         * can already be used to do something fairly similar, but this spring has angle limits as well.
         * 
         */


        private Vector3 myLocalUpVector;
        private float myMaximumAngle;
        private float myMinimumAngle;

        private Vector3 myWorldUpVector;

        /// <summary>
        /// Constructs a constraint to keep entities upright.
        /// </summary>
        /// <param name="entity">Entity to try to keep upright.</param>
        /// <param name="upVector">Direction to try to orient the entity with.</param>
        /// <param name="minimumAngle">Minimum angle between the car's transformed up vector and the actual up vector
        /// before the constraint begins to apply forces.</param>
        /// <param name="maximumAngle">Maximum angle between the car's transformed up vector and the actual up vector
        /// past which the constraint 'gives up' and lets the entity tumble.</param>
        /// <param name="correctionFactor">Factor of the orientation error to apply in angular velocity each frame.</param>
        public UprightSpring(Entity entity, Vector3 upVector, float minimumAngle, float maximumAngle, float correctionFactor)
        {
            this.Entity = entity;
            this.UpVector = upVector;
            this.MinimumAngle = minimumAngle;
            this.MaximumAngle = maximumAngle;
            this.CorrectionFactor = correctionFactor;
        }

        /// <summary>
        /// Gets or sets the entity to try to keep upright.
        /// </summary>
        public Entity Entity { get; set; }

        /// <summary>
        /// Gets or sets the up vector attached to the entity in its local space.
        /// </summary>
        public Vector3 LocalUpVector
        {
            get { return myWorldUpVector; }
            set
            {
                myLocalUpVector = Vector3.Normalize(value);
                myWorldUpVector = Matrix3X3.Transform(myLocalUpVector, Entity.OrientationMatrix);
            }
        }

        /// <summary>
        /// Gets or sets the up vector attached to the entity in world space.
        /// </summary>
        public Vector3 UpVector
        {
            get { return myWorldUpVector; }
            set
            {
                myWorldUpVector = Vector3.Normalize(value);
                myLocalUpVector = Matrix3X3.TransformTranspose(myWorldUpVector, Entity.OrientationMatrix);
            }
        }

        /// <summary>
        /// Gets or sets the minimum angle between the car's transformed up vector and the actual up vector
        /// before the constraint begins to apply forces.
        /// </summary>
        public float MinimumAngle
        {
            get { return myMinimumAngle; }
            set
            {
                myMinimumAngle = MathHelper.Clamp(value, 0, MathHelper.Pi);
                if (myMinimumAngle > myMaximumAngle)
                    MaximumAngle = myMinimumAngle;
            }
        }

        /// <summary>
        /// Gets or sets the maximum angle between the car's transformed up vector and the actual up vector
        /// past which the constraint 'gives up' and lets the entity tumble.
        /// </summary>
        public float MaximumAngle
        {
            get { return myMaximumAngle; }
            set
            {
                myMaximumAngle = MathHelper.Clamp(value, 0, MathHelper.Pi);
                if (myMaximumAngle < myMinimumAngle)
                    MinimumAngle = myMaximumAngle;
            }
        }

        /// <summary>
        /// Gets or sets the factor of the orientation error to apply in torque each frame.
        /// </summary>
        public float CorrectionFactor { get; set; }


        /// <summary>
        /// Updates the upright constraint.
        /// Called automatically by its owning space.
        /// </summary>
        /// <param name="dt">Time since last frame in simulation seconds.</param>
        void IDuringForcesUpdateable.Update(float dt)
        {
            myWorldUpVector = Matrix3X3.Transform(myLocalUpVector, Entity.OrientationMatrix);

            //Compute the axis and angle 
            Vector3 axis = Vector3.Cross(myWorldUpVector, Vector3.Up);
            var angle = (float) Math.Acos(Vector3.Dot(Vector3.Up, myWorldUpVector));

            if (angle > MinimumAngle && angle < MaximumAngle)
            {
                angle = angle - MinimumAngle;
                axis.Normalize();
                Entity.AngularMomentum += (axis * (angle * CorrectionFactor * dt));
            }
        }
    }
}