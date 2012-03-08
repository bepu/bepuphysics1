using BEPUphysics;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Entities;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.MathExtensions;
using System;

namespace BEPUphysicsDemos
{
    /// <summary>
    /// Simple camera class for moving around the demos area.
    /// </summary>
    public class Camera
    {
        /// <summary>
        /// Gets or sets the position of the camera.
        /// </summary>
        public Vector3 Position { get; set; }

        private float yaw;
        private float pitch;

        /// <summary>
        /// Gets or sets the yaw rotation of the camera.
        /// </summary>
        public float Yaw
        {
            get { return yaw; }
            set { yaw = MathHelper.WrapAngle(value); }
        }

        /// <summary>
        /// Gets or sets the pitch rotation of the camera.
        /// </summary>
        public float Pitch
        {
            get { return pitch; }
            set
            {
                pitch = value;
                if (pitch > MathHelper.PiOver2 * .99f)
                    pitch = MathHelper.PiOver2 * .99f;
                else if (pitch < -MathHelper.PiOver2 * .99f)
                    pitch = -MathHelper.PiOver2 * .99f;
            }
        }

        /// <summary>
        /// Gets or sets the speed at which the camera moves.
        /// </summary>
        public float Speed { get; set; }

        /// <summary>
        /// Gets the view matrix of the camera.
        /// </summary>
        public Matrix ViewMatrix { get; private set; }

        /// <summary>
        /// Gets or sets the projection matrix of the camera.
        /// </summary>
        public Matrix ProjectionMatrix { get; set; }

        /// <summary>
        /// Gets the world transformation of the camera.
        /// </summary>
        public Matrix WorldMatrix { get; private set; }

        /// <summary>
        /// Whether or not to use the default free-flight camera controls.
        /// Set to false when using vehicles or character controllers.
        /// </summary>
        public bool UseMovementControls = true;

        #region Chase Camera Mode

        //The following are used for the chase camera only.
        /// <summary>
        /// Entity to follow around and point at.
        /// </summary>
        private Entity entityToChase;

        /// <summary>
        /// Offset vector from the center of the target chase entity to look at.
        /// </summary>
        private Vector3 offsetFromChaseTarget;

        /// <summary>
        /// Whether or not to transform the offset vector with the rotation of the entity.
        /// </summary>
        private bool transformOffset;

        /// <summary>
        /// Distance away from the target entity to try to maintain.  The distance will be shorter at times if the ray hits an object.
        /// </summary>
        private float distanceToTarget;

        /// <summary>
        /// Whether or not the camera is currently in chase camera mode.
        /// </summary>
        private bool isChaseCameraMode;

        /// <summary>
        /// Sets up all the information required by the chase camera.
        /// </summary>
        /// <param name="target">Target to follow.</param>
        /// <param name="offset">Offset from the center of the entity target to point at.</param>
        /// <param name="transform">Whether or not to transform the offset with the target entity's rotation.</param>
        /// <param name="distance">Distance from the target position to try to maintain.</param>
        public void ActivateChaseCameraMode(Entity target, Vector3 offset, bool transform, float distance)
        {
            entityToChase = target;
            offsetFromChaseTarget = offset;
            transformOffset = transform;
            distanceToTarget = distance;
            isChaseCameraMode = true;
        }

        /// <summary>
        /// Disable the chase camera mode, returning it to first person perspective.
        /// </summary>
        public void DeactivateChaseCameraMode()
        {
            isChaseCameraMode = false;
        }

        #endregion

        /// <summary>
        /// Creates a camera.
        /// </summary>
        /// <param name="pos">Initial position of the camera.</param>
        /// <param name="s">Speed of the camera per second.</param>
        /// <param name="p">Initial pitch angle of the camera.</param>
        /// <param name="y">Initial yaw value of the camera.</param>
        /// <param name="projMatrix">Projection matrix used.</param>
        public Camera(Vector3 pos, float s, float p, float y, Matrix projMatrix)
        {
            Position = pos;
            Speed = s;
            Yaw = y;
            Pitch = p;
            ProjectionMatrix = projMatrix;

            rayCastFilter = RayCastFilter;
        }
        //The raycast filter limits the results retrieved from the Space.RayCast while in chase camera mode.
        Func<BroadPhaseEntry, bool> rayCastFilter;
        bool RayCastFilter(BroadPhaseEntry entry)
        {
            return entry != entityToChase.CollisionInformation && (entry.CollisionRules.Personal <= CollisionRule.Normal);
        }

        /// <summary>
        /// Moves the camera forward.
        /// </summary>
        /// <param name="distance">Distance to move.</param>
        public void MoveForward(float distance)
        {
            Position += WorldMatrix.Forward * distance;
        }

        /// <summary>
        /// Moves the camera to the right.
        /// </summary>
        /// <param name="distance">Distance to move.</param>
        public void MoveRight(float distance)
        {
            Position += WorldMatrix.Right * distance;
        }

        /// <summary>
        /// Moves the camera up.
        /// </summary>
        /// <param name="distance">Distanec to move.</param>
        public void MoveUp(float distance)
        {
            Position += new Vector3(0, distance, 0);
        }


        /// <summary>
        /// Updates the state of the camera.
        /// </summary>
        /// <param name="dt">Time since the last frame in seconds.</param>
        /// <param name="keyboardInput">Input for this frame from the keyboard.</param>
        /// <param name="oldMouseInput">Input from the previous frame from the mouse.</param>
        /// <param name="mouseInput">Input for this frame from the mouse.</param>
        /// <param name="gamePadInput">Input for this frame from the game pad.</param>
#if !WINDOWS
        public void Update(float dt, KeyboardState keyboardInput, GamePadState gamePadInput)
        {
            Yaw += gamePadInput.ThumbSticks.Right.X * -1.5f * dt;
            Pitch += gamePadInput.ThumbSticks.Right.Y * 1.5f * dt;

#else
        public void Update(float dt, KeyboardState keyboardInput, MouseState mouseInput, GamePadState gamePadInput)
        {
            Yaw += (200 - mouseInput.X) * dt * .12f;
            Pitch += (200 - mouseInput.Y) * dt * .12f;
#endif

            WorldMatrix = Matrix.CreateFromAxisAngle(Vector3.Right, Pitch) * Matrix.CreateFromAxisAngle(Vector3.Up, Yaw);

            if (isChaseCameraMode)
            {
                Vector3 offset;
                if (transformOffset)
                    offset = Matrix3X3.Transform(offsetFromChaseTarget, entityToChase.BufferedStates.InterpolatedStates.OrientationMatrix);
                else
                    offset = offsetFromChaseTarget;
                Vector3 lookAt = entityToChase.BufferedStates.InterpolatedStates.Position + offset;
                Vector3 backwards = WorldMatrix.Backward;

                //Find the earliest ray hit that isn't the chase target to position the camera appropriately.
                RayCastResult result;
                if (entityToChase.Space.RayCast(new Ray(lookAt, backwards), distanceToTarget, rayCastFilter, out result))
                {
                    Position = lookAt + (result.HitData.T) * backwards; //Put the camera just before any hit spot.
                }
                else
                    Position = lookAt + (distanceToTarget) * backwards;
            }
            else if (UseMovementControls)
            {
                //Only move around if the camera has control over its own position.
                float distance = Speed * dt;
#if !WINDOWS
                MoveForward(gamePadInput.ThumbSticks.Left.Y * distance);
                MoveRight(gamePadInput.ThumbSticks.Left.X * distance);
                if (gamePadInput.IsButtonDown(Buttons.LeftStick))
                    MoveUp(distance);
                if (gamePadInput.IsButtonDown(Buttons.RightStick))
                    MoveUp(-distance);
#endif

                if (keyboardInput.IsKeyDown(Keys.E))
                    MoveForward(distance);
                if (keyboardInput.IsKeyDown(Keys.D))
                    MoveForward(-distance);
                if (keyboardInput.IsKeyDown(Keys.S))
                    MoveRight(-distance);
                if (keyboardInput.IsKeyDown(Keys.F))
                    MoveRight(distance);
                if (keyboardInput.IsKeyDown(Keys.A))
                    MoveUp(distance);
                if (keyboardInput.IsKeyDown(Keys.Z))
                    MoveUp(-distance);
            }

            WorldMatrix = WorldMatrix * Matrix.CreateTranslation(Position);
            ViewMatrix = Matrix.Invert(WorldMatrix);
        }
    }
}