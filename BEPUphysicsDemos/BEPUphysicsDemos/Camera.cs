using System;
using BEPUphysics;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.Entities;
using BEPUutilities;
using Microsoft.Xna.Framework.Input;

namespace BEPUphysicsDemos
{
    /// <summary>
    /// Simple camera class for moving around the demos area.
    /// </summary>
    public class Camera
    {
        /// <summary>
        /// Gets the game associated with the camera.
        /// </summary>
        public DemosGame Game
        {
            get;
            private set;
        }

        /// <summary>
        /// Gets or sets the position of the camera.
        /// </summary>
        public Vector3 Position { get; set; }

        /// <summary>
        /// Gets or sets the speed at which the camera moves.
        /// </summary>
        public float Speed { get; set; }


        /// <summary>
        /// Gets or sets the projection matrix of the camera.
        /// </summary>
        public Matrix ProjectionMatrix { get; set; }


        /// <summary>
        /// Gets the view matrix of the camera.
        /// </summary>
        public Matrix ViewMatrix
        {
            get { return Matrix.CreateViewRH(Position, viewDirection, lockedUp); }
        }

        /// <summary>
        /// Gets the world transformation of the camera.
        /// </summary>
        public Matrix WorldMatrix
        {
            get { return Matrix.CreateWorldRH(Position, viewDirection, lockedUp); }
        }

        private Vector3 viewDirection;
        /// <summary>
        /// Gets or sets the view direction of the camera.
        /// </summary>
        public Vector3 ViewDirection
        {
            get { return viewDirection; }
            set
            {
                Vector3.Normalize(ref value, out value);
                //Validate the input. A temporary violation of the maximum pitch is permitted as it will be fixed as the user looks around.
                //However, we cannot allow a view direction parallel to the locked up direction.
                float dot;
                Vector3.Dot(ref value, ref lockedUp, out dot);
                if (Math.Abs(dot) > 1 - Toolbox.BigEpsilon)
                    throw new ArgumentException("The view direction must not be aligned with the locked up direction.");
                viewDirection = value;
            }
        }

        private float maximumPitch = MathHelper.PiOver2 * 0.99f;
        /// <summary>
        /// Gets or sets how far the camera can look up or down in radians.
        /// </summary>
        public float MaximumPitch
        {
            get { return maximumPitch; }
            set
            {
                if (value < 0)
                    throw new ArgumentException("Maximum pitch corresponds to pitch magnitude; must be positive.");
                if (value >= MathHelper.PiOver2)
                    throw new ArgumentException("Maximum pitch must be less than Pi/2.");
                maximumPitch = value;
            }
        }

        private Vector3 lockedUp;
        /// <summary>
        /// Gets or sets the current locked up vector of the camera.
        /// </summary>
        public Vector3 LockedUp
        {
            get { return lockedUp; }
            set
            {
                var oldUp = lockedUp;
                Vector3.Normalize(ref value, out lockedUp);
                //Move the view direction with the transform. This helps guarantee that the view direction won't end up aligned with the up vector.
                Quaternion rotation;
                Quaternion.GetQuaternionBetweenNormalizedVectors(ref oldUp, ref lockedUp, out rotation);
                Quaternion.Transform(ref viewDirection, ref rotation, out viewDirection);

            }
        }

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

        private const float chaseCameraMargin = 1f;

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
        /// <param name="game">Game used with the camera.</param>
        /// <param name="position">Initial position of the camera.</param>
        /// <param name="speed">Speed of the camera per second.</param>
        /// <param name="pitch">Initial pitch angle of the camera.</param>
        /// <param name="yaw">Initial yaw value of the camera.</param>
        /// <param name="projectionMatrix">Projection matrix used.</param>
        public Camera(DemosGame game, Vector3 position, float speed, float pitch, float yaw, Matrix projectionMatrix)
        {
            this.Game = game;
            Position = position;
            Speed = speed;
            Yaw(yaw);
            Pitch(pitch);
            ProjectionMatrix = projectionMatrix;

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
        /// Rotates the camera around its locked up vector.
        /// </summary>
        /// <param name="radians">Amount to rotate.</param>
        public void Yaw(float radians)
        {
            //Rotate around the up vector.
            Matrix3x3 rotation;
            Matrix3x3.CreateFromAxisAngle(ref lockedUp, radians, out rotation);
            Matrix3x3.Transform(ref viewDirection, ref rotation, out viewDirection);

            //Avoid drift by renormalizing.
            viewDirection.Normalize();
        }

        /// <summary>
        /// Rotates the view direction up or down relative to the locked up vector.
        /// </summary>
        /// <param name="radians">Amount to rotate.</param>
        public void Pitch(float radians)
        {
            //Do not allow the new view direction to violate the maximum pitch.
            float dot;
            Vector3.Dot(ref viewDirection, ref lockedUp, out dot);

            //While this could be rephrased in terms of dot products alone, converting to actual angles can be more intuitive.
            //Consider +Pi/2 to be up, and -Pi/2 to be down.
            float currentPitch = (float)Math.Acos(MathHelper.Clamp(-dot, -1, 1)) - MathHelper.PiOver2;
            //Compute our new pitch by clamping the current + change.
            float newPitch = MathHelper.Clamp(currentPitch + radians, -maximumPitch, maximumPitch);
            float allowedChange = newPitch - currentPitch;

            //Compute and apply the rotation.
            Vector3 pitchAxis;
            Vector3.Cross(ref viewDirection, ref lockedUp, out pitchAxis);
            //This is guaranteed safe by all interaction points stopping viewDirection from being aligned with lockedUp.
            pitchAxis.Normalize();
            Matrix3x3 rotation;
            Matrix3x3.CreateFromAxisAngle(ref pitchAxis, allowedChange, out rotation);
            Matrix3x3.Transform(ref viewDirection, ref rotation, out viewDirection);

            //Avoid drift by renormalizing.
            viewDirection.Normalize();
        }

        /// <summary>
        /// Updates the state of the camera.
        /// </summary>
        /// <param name="dt">Time since the last frame in seconds.</param>
        /// <param name="keyboardInput">Input for this frame from the keyboard.</param>
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
            //Only turn if the mouse is controlled by the game.
            if (!Game.IsMouseVisible)
            {
                Yaw((200 - mouseInput.X) * dt * .12f);
                Pitch((200 - mouseInput.Y) * dt * .12f);
            }
#endif

            if (isChaseCameraMode)
            {
                Vector3 offset;
                if (transformOffset)
                    offset = Matrix3x3.Transform(offsetFromChaseTarget, entityToChase.BufferedStates.InterpolatedStates.OrientationMatrix);
                else
                    offset = offsetFromChaseTarget;
                Vector3 lookAt = entityToChase.BufferedStates.InterpolatedStates.Position + offset;
                Vector3 backwards = -viewDirection;

                //Find the earliest ray hit that isn't the chase target to position the camera appropriately.
                RayCastResult result;
                if (entityToChase.Space.RayCast(new Ray(lookAt, backwards), distanceToTarget, rayCastFilter, out result))
                {
                    Position = lookAt + (Math.Max(result.HitData.T - chaseCameraMargin, 0)) * backwards; //Put the camera just before any hit spot.
                }
                else
                    Position = lookAt + (Math.Max(distanceToTarget - chaseCameraMargin, 0)) * backwards;
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

        }
    }
}