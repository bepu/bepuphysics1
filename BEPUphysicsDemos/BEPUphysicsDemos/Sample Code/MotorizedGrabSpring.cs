using BEPUphysics;
using BEPUphysics.Constraints.SingleEntity;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Entities;
using BEPUphysics.UpdateableSystems;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.SampleCode
{
    /// <summary>
    /// Grabs an entity at a specified location and applies corrective impulses to keep the grabbed location near the goal location.
    /// </summary>
    public class MotorizedGrabSpring : Updateable, IEndOfFrameUpdateable
    {

        SingleEntityLinearMotor linearMotor;
        SingleEntityAngularMotor angularMotor;


        /// <summary>
        /// Constructs a grab constraint.
        /// </summary>
        public MotorizedGrabSpring()
        {
            //Note that when the motor is created using the empty constructor, 
            //it starts deactivated.  This prevents explosions from attempting
            //to update it without being configured.
            linearMotor = new SingleEntityLinearMotor();
            angularMotor = new SingleEntityAngularMotor();
            linearMotor.Settings.Mode = MotorMode.Servomechanism;

            //The stiffness, damping, and maximum force could be assigned during setup if the motor
            //needs to behave similarly for entities of varying masses.  When using a fixed configuration,
            //the grabspring will behave weakly when trying to move extremely heavy objects, while staying
            //very responsive for sufficiently light objects.

            IsUpdating = false;
        }

        /// <summary>
        /// Gets the grabbed entity.
        /// </summary>
        public Entity Entity
        {
            get
            {
                return linearMotor.Entity;
            }
            private set
            {
                if (linearMotor.Entity != value) //Don't bother changing the entity if it is the same.
                {
                    linearMotor.Entity = value;
                    angularMotor.Entity = value;
                    //The motors can only be on while the entity isn't null.
                    if (value != null)
                    {
                        linearMotor.IsActive = true;
                        angularMotor.IsActive = true;
                        IsUpdating = true;
                    }
                    else
                    {
                        linearMotor.IsActive = false;
                        angularMotor.IsActive = false;
                        IsUpdating = false;
                    }
                }
            }
        }

        /// <summary>
        /// Gets the location that the entity will be pulled towards.
        /// </summary>
        public Vector3 GoalPosition
        {
            get
            {
                return linearMotor.Settings.Servo.Goal;
            }
            set
            {
                linearMotor.Settings.Servo.Goal = value;
            }
        }

        /// <summary>
        /// Gets the offset from the entity to the grabbed location in its local space.
        /// </summary>
        public Vector3 LocalOffset
        {
            get
            {
                return linearMotor.LocalPoint;
            }
            private set
            {
                linearMotor.LocalPoint = value;
            }
        }

        /// <summary>
        /// Gets the last updated position of the grab location on the surface of the entity.
        /// </summary>
        public Vector3 GrabbedPosition { get; private set; }

        /// <summary>
        /// Gets whether or not the grabber is currently grabbing something.
        /// </summary>
        public bool IsGrabbing
        {
            get
            {
                return Entity != null;
            }
        }

        /// <summary>
        /// Reinitializes the grabbing constraint with new information.
        /// </summary>
        /// <param name="e">Entity to grab.</param>
        /// <param name="grabLocation">Location on the entity being grabbed in world space.</param>
        public void Setup(Entity e, Vector3 grabLocation)
        {
            //You can configure the stiffness and damping of the corrective springs like so.
            //For this example, the motors will be just be the nearly rigid default.
            linearMotor.Settings.Servo.SpringSettings.StiffnessConstant = 60000 * e.Mass;
            linearMotor.Settings.Servo.SpringSettings.DampingConstant = 9000 * e.Mass;

            angularMotor.Settings.VelocityMotor.Softness = .1f / e.Mass;
            //An unlimited motor will gladly push the entity through other objects.
            //Putting a limit on the strength of the motor will prevent it from doing so.
            linearMotor.Settings.MaximumForce = 1000 * e.Mass;

            Entity = e;
            LocalOffset = Vector3.Transform(grabLocation - e.Position, Quaternion.Conjugate(e.Orientation));
            angularMotor.Settings.Servo.Goal = e.Orientation;
            GoalPosition = grabLocation;


        }

        /// <summary>
        /// Releases the entity being held by the grab spring.
        /// </summary>
        public void Release()
        {
            Entity = null;
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
            GrabbedPosition = Matrix3X3.Transform(LocalOffset, Entity.BufferedStates.InterpolatedStates.OrientationMatrix) + Entity.BufferedStates.InterpolatedStates.Position;
        }

        public override void OnAdditionToSpace(ISpace newSpace)
        {
            newSpace.Add(linearMotor);
            newSpace.Add(angularMotor);
        }

        public override void OnRemovalFromSpace(ISpace oldSpace)
        {
            oldSpace.Remove(linearMotor);
            oldSpace.Remove(angularMotor);
        }
    }
}