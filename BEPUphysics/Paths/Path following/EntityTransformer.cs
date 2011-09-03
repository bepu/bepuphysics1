//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
// 
//using BEPUphysics.Constraints;
//using BEPUphysics.Entities;

//namespace BEPUphysics.Curves.PathFollowing
//{
//    /// <summary>
//    /// Pushes an entity around according to goal positions and orientations.
//    /// </summary>
//    public class EntityMover : Updateable
//    {
//        /// <summary>
//        /// Gets the angular motor used by the entity mover.
//        /// When the affected entity is dynamic, it is pushed by motors.
//        /// This ensures that its interactions and collisions with
//        /// other entities remain stable.
//        /// </summary>
//        public SingleEntityAngularMotor angularMotor { get; private set; }
//        /// <summary>
//        /// Gets the linear motor used by the entity mover.
//        /// When the affected entity is dynamic, it is pushed by motors.
//        /// This ensures that its interactions and collisions with
//        /// other entities remain stable.
//        /// </summary>
//        public SingleEntityLinearMotor linearMotor { get; private set; }

//        /// <summary>
//        /// Gets or sets the target location of the entity mover.
//        /// </summary>
//        public Vector3 targetPosition { get; set; }
//        /// <summary>
//        /// Gets or sets the target orientation of the entity mover.
//        /// </summary>
//        public Quaternion targetOrientation { get; set; }

//        Entity myEntity;
//        /// <summary>
//        /// Gets or sets the entity being pushed by the entity mover.
//        /// </summary>
//        public Entity entity
//        {
//            get
//            {
//                return myEntity;
//            }
//            set
//            {
//                myEntity = value;
//                angularMotor.entity = value;
//                linearMotor.entity = value;
//            }
//        }

//        /// <summary>
//        /// Constructs a new EntityMover.
//        /// </summary>
//        /// <param name="e">Entity to move.</param>
//        public EntityMover(Entity e)
//        {
//            linearMotor = new SingleEntityLinearMotor(e, e.position);
//            angularMotor = new SingleEntityAngularMotor(e);
//            this.entity = e;

//            linearMotor.settings.mode = MotorMode.servomechanism;
//            angularMotor.settings.mode = MotorMode.servomechanism;
//            targetPosition = e.position;
//            targetOrientation = e.internalOrientationQuaternion;
//        }

//        /// <summary>
//        /// Constructs a new EntityMover.
//        /// </summary>
//        /// <param name="e">Entity to move.</param>
//        /// <param name="linearMotor">Motor to use for linear motion if the entity is dynamic.</param>
//        /// <param name="angularMotor">Motor to use for angular motion if the entity is dynamic.</param>
//        public EntityMover(Entity e, SingleEntityLinearMotor linearMotor, SingleEntityAngularMotor angularMotor)
//        {
//            this.linearMotor = linearMotor;
//            this.angularMotor = angularMotor;
//            this.entity = e;

//            linearMotor.entity = entity;
//            angularMotor.entity = entity;
//            linearMotor.settings.mode = MotorMode.servomechanism;
//            angularMotor.settings.mode = MotorMode.servomechanism;
//            targetPosition = e.position;
//            targetOrientation = e.internalOrientationQuaternion;
//        }

//        /// <summary>
//        /// Called automatically by the space.
//        /// </summary>
//        /// <param name="dt">Simulation timestep.</param>
//        /// <param name="timeScale">Time speed multiplier.</param>
//        /// <param name="timeSinceLastFrame">Time since the last frame in game time.</param>
//        public override void updateDuringForces(float dt, float timeScale, float timeSinceLastFrame)
//        {
//            if (entity != linearMotor.entity || entity != angularMotor.entity)
//                throw new InvalidOperationException("EntityMover's entity differs from EntityMover's motors' entities.  Ensure that the moved entity is only changed by setting the EntityMover's entity property.");
//            if (entity.space != space)
//                throw new InvalidOperationException("EntityMover's entity must belong to the EntityMover's space.  Ensure the entity has been added to the space.");
//            if (entity.isDynamic)
//            {
//                linearMotor.isActive = true;
//                angularMotor.isActive = true;
//                linearMotor.settings.servo.goal = targetPosition;
//                angularMotor.settings.servo.goal = targetOrientation;

//                //if (!entity.isAlwaysActive &&
//                //    (entity.position - targetPosition).LengthSquared() > Toolbox.bigEpsilon ||
//                //    angularMotor.error.LengthSquared() > Toolbox.bigEpsilon))
//                //    entity.isActive = true;
//            }
//            else
//            {
//                linearMotor.isActive = false;
//                angularMotor.isActive = false;

//                entity.internalLinearVelocity = getLinearVelocity(entity.position, targetPosition, dt);
//                entity.internalAngularVelocity = getAngularVelocity(entity.internalOrientationQuaternion, targetOrientation, dt);
//            }
//            base.updateDuringForces(dt, timeScale, timeSinceLastFrame);
//        }

//        /// <summary>
//        /// Adds the motors to the space.  Called automatically.
//        /// </summary>
//        /// <param name="newSpace">Space the mover is being added to.</param>
//        public override void onAdditionToSpace(Space newSpace)
//        {
//            if (linearMotor.space != newSpace)
//            {
//                if (linearMotor.space != null)
//                    linearMotor.space.remove(linearMotor);
//                newSpace.add(linearMotor);
//            }
//            if (angularMotor.space != newSpace)
//            {
//                if (angularMotor.space != null)
//                    angularMotor.space.remove(angularMotor);
//                newSpace.add(angularMotor);
//            }
//            base.onAdditionToSpace(newSpace);
//        }

//        /// <summary>
//        /// Removes the motors from the space.  Called automatically.
//        /// </summary>
//        public override void onRemovalFromSpace()
//        {
//            if (linearMotor.space == space)
//            {
//                space.remove(linearMotor);
//            }
//            if (angularMotor.space == space)
//            {
//                space.remove(angularMotor);
//            }
//            base.onRemovalFromSpace();
//        }


//        /// <summary>
//        /// Gets the angular velocity necessary to change an entity's orientation from
//        /// the starting quaternion to the ending quaternion over time dt.
//        /// </summary>
//        /// <param name="start">Initial orientation.</param>
//        /// <param name="end">Final orientation.</param>
//        /// <param name="dt">Time over which the angular velocity is to be applied.</param>
//        /// <returns>Angular velocity to reach the goal in time.</returns>
//        public static Vector3 getAngularVelocity(Quaternion start, Quaternion end, float dt)
//        {

//            //Compute the relative orientation R' between R and the target relative orientation.
//            Quaternion errorOrientation;
//            Quaternion.Conjugate(ref start, out errorOrientation);
//            Quaternion.Multiply(ref errorOrientation, ref end, out errorOrientation);

//            Vector3 axis;
//            float angle;
//            //Turn this into an axis-angle representation.
//            Toolbox.getAxisAngleFromQuaternion(ref errorOrientation, out axis, out angle);
//            Vector3.Multiply(ref axis, angle / dt, out axis);
//            return axis;
//        }

//        /// <summary>
//        /// Gets the angular velocity necessary to change an entity's orientation from
//        /// the starting quaternion to the ending quaternion over time dt.
//        /// </summary>
//        /// <param name="start">Initial position.</param>
//        /// <param name="end">Final position.</param>
//        /// <param name="dt">Time over which the angular velocity is to be applied.</param>
//        /// <returns>Angular velocity to reach the goal in time.</returns>
//        public static Vector3 getLinearVelocity(Vector3 start, Vector3 end, float dt)
//        {
//            Vector3 offset;
//            Vector3.Subtract(ref end, ref start, out offset);
//            Vector3.Divide(ref offset, dt, out offset);
//            return offset;
//        }
//    }
//}
