using BEPUphysics;
using BEPUphysics.Character;
using Microsoft.Xna.Framework.Input;
using System;
using System.Diagnostics;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using ConversionHelper;
using BEPUutilities;

namespace BEPUphysicsDemos.AlternateMovement
{
    /// <summary>
    /// Handles input and movement of a character in the game.
    /// Acts as a simple 'front end' for the bookkeeping and math of the character controller.
    /// </summary>
    public class CharacterControllerInput
    {
        /// <summary>
        /// Gets the camera to use for input.
        /// </summary>
        public Camera Camera { get; private set; }

        /// <summary>
        /// Physics representation of the character.
        /// </summary>
        public CharacterController CharacterController;

        /// <summary>
        /// Gets the camera control scheme used by the character input.
        /// </summary>
        public CharacterCameraControlScheme CameraControlScheme { get; private set; }

        /// <summary>
        /// Gets whether the character controller's input management is being used.
        /// </summary>
        public bool IsActive { get; private set; }

        /// <summary>
        /// Owning space of the character.
        /// </summary>
        public Space Space { get; private set; }


        /// <summary>
        /// Constructs the character and internal physics character controller.
        /// </summary>
        /// <param name="owningSpace">Space to add the character to.</param>
        /// <param name="camera">Camera to attach to the character.</param>
        /// <param name="game">The running game.</param>
        public CharacterControllerInput(Space owningSpace, Camera camera, DemosGame game)
        {
            CharacterController = new CharacterController();
            Camera = camera;
            CameraControlScheme = new CharacterCameraControlScheme(CharacterController, camera, game);

            Space = owningSpace;
        }

        /// <summary>
        /// Gives the character control over the Camera and movement input.
        /// </summary>
        public void Activate()
        {
            if (!IsActive)
            {
                IsActive = true;
                Space.Add(CharacterController);
                //Offset the character start position from the camera to make sure the camera doesn't shift upward discontinuously.
                CharacterController.Body.Position = Camera.Position - new Vector3(0, CameraControlScheme.StandingCameraOffset, 0);
            }
        }

        /// <summary>
        /// Returns input control to the Camera.
        /// </summary>
        public void Deactivate()
        {
            if (IsActive)
            {
                IsActive = false;
                Space.Remove(CharacterController);
            }
        }


        /// <summary>
        /// Handles the input and movement of the character.
        /// </summary>
        /// <param name="dt">Time since last frame in simulation seconds.</param>
        /// <param name="previousKeyboardInput">The last frame's keyboard state.</param>
        /// <param name="keyboardInput">The current frame's keyboard state.</param>
        /// <param name="previousGamePadInput">The last frame's gamepad state.</param>
        /// <param name="gamePadInput">The current frame's keyboard state.</param>
        public void Update(float dt, KeyboardState previousKeyboardInput, KeyboardState keyboardInput, GamePadState previousGamePadInput, GamePadState gamePadInput)
        {
            if (IsActive)
            {
                //Note that the character controller's update method is not called here; this is because it is handled within its owning space.
                //This method's job is simply to tell the character to move around.

                CameraControlScheme.Update(dt);

                Vector2 totalMovement = Vector2.Zero;

#if XBOX360
                totalMovement += new Vector2(gamePadInput.ThumbSticks.Left.X, gamePadInput.ThumbSticks.Left.Y);

                CharacterController.HorizontalMotionConstraint.SpeedScale = Math.Min(totalMovement.Length(), 1); //Don't trust the game pad to output perfectly normalized values.
                CharacterController.HorizontalMotionConstraint.MovementDirection = totalMovement;
                
                CharacterController.StanceManager.DesiredStance = gamePadInput.IsButtonDown(Buttons.RightStick) ? Stance.Crouching : Stance.Standing;

                //Jumping
                if (previousGamePadInput.IsButtonUp(Buttons.LeftStick) && gamePadInput.IsButtonDown(Buttons.LeftStick))
                {
                    CharacterController.Jump();
                }
#else

                //Collect the movement impulses.

                if (keyboardInput.IsKeyDown(Keys.E))
                {
                    totalMovement += new Vector2(0, 1);
                }
                if (keyboardInput.IsKeyDown(Keys.D))
                {
                    totalMovement += new Vector2(0, -1);
                }
                if (keyboardInput.IsKeyDown(Keys.S))
                {
                    totalMovement += new Vector2(-1, 0);
                }
                if (keyboardInput.IsKeyDown(Keys.F))
                {
                    totalMovement += new Vector2(1, 0);
                }
                if (totalMovement == Vector2.Zero)
                    CharacterController.HorizontalMotionConstraint.MovementDirection = Vector2.Zero;
                else
                    CharacterController.HorizontalMotionConstraint.MovementDirection = Vector2.Normalize(totalMovement);


                if (keyboardInput.IsKeyDown(Keys.X))
                    CharacterController.StanceManager.DesiredStance = Stance.Prone;
                else if (keyboardInput.IsKeyDown(Keys.Z))
                    CharacterController.StanceManager.DesiredStance = Stance.Crouching;
                else
                    CharacterController.StanceManager.DesiredStance = Stance.Standing;

                //Jumping
                if (previousKeyboardInput.IsKeyUp(Keys.A) && keyboardInput.IsKeyDown(Keys.A))
                {
                    CharacterController.Jump();
                }
#endif
                CharacterController.ViewDirection = Camera.WorldMatrix.Forward;

            }
        }
    }
}