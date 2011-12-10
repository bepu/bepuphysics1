using BEPUphysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;
using System;
using System.Diagnostics;
using BEPUphysics.Collidables.MobileCollidables;

namespace BEPUphysicsDemos.AlternateMovement.SphereCharacter
{
    /// <summary>
    /// Handles input and movement of a character in the game.
    /// Acts as a simple 'front end' for the bookkeeping and math of the character controller.
    /// </summary>
    public class SphereCharacterControllerInput
    {
        /// <summary>
        /// Camera to use for input.
        /// </summary>
        public Camera Camera;

        /// <summary>
        /// Offset from the position of the character to the 'eyes'.
        /// </summary>
        public float CameraOffset = .7f;


        /// <summary>
        /// Physics representation of the character.
        /// </summary>
        public SphereCharacterController CharacterController;

        /// <summary>
        /// Whether or not to use the character controller's input.
        /// </summary>
        public bool IsActive = true;

        /// <summary>
        /// Owning space of the character.
        /// </summary>
        public Space Space { get; private set; }


        /// <summary>
        /// Constructs the character and internal physics character controller.
        /// </summary>
        /// <param name="owningSpace">Space to add the character to.</param>
        /// <param name="CameraToUse">Camera to attach to the character.</param>
        public SphereCharacterControllerInput(Space owningSpace, Camera CameraToUse)
        {
            CharacterController = new SphereCharacterController();

            Space = owningSpace;
            Space.Add(CharacterController);


            Camera = CameraToUse;
            Deactivate();
        }

        /// <summary>
        /// Gives the character control over the Camera and movement input.
        /// </summary>
        public void Activate()
        {
            if (!IsActive)
            {
                IsActive = true;
                Camera.UseMovementControls = false;
                Space.Add(CharacterController);
                CharacterController.Body.Position = (Camera.Position - new Vector3(0, CameraOffset, 0));
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
                Camera.UseMovementControls = true;
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
                //This method's job is simply to tell the character to move around based on the Camera and input.

                ////Rotate the camera of the character based on the support velocity, if a support with velocity exists.
                ////This can be very disorienting in some cases; that's why it is off by default!
                //if (CharacterController.SupportFinder.HasSupport)
                //{
                //    SupportData? data;
                //    if (CharacterController.SupportFinder.HasTraction)
                //        data = CharacterController.SupportFinder.TractionData;
                //    else
                //        data = CharacterController.SupportFinder.SupportData;
                //    EntityCollidable support = data.Value.SupportObject as EntityCollidable;
                //    if (support != null && !support.Entity.IsDynamic) //Having the view turned by dynamic entities is extremely confusing for the most part.
                //    {
                //        float dot = Vector3.Dot(support.Entity.AngularVelocity, CharacterController.Body.OrientationMatrix.Up);
                //        Camera.Yaw += dot * dt;
                //    }
                //}


                Camera.Position = CharacterController.Body.Position + CameraOffset * CharacterController.Body.OrientationMatrix.Up;


                Vector2 totalMovement = Vector2.Zero;

#if XBOX360
                Vector3 forward = Camera.WorldMatrix.Forward;
                forward.Y = 0;
                forward.Normalize();
                Vector3 right = Camera.WorldMatrix.Right;
                totalMovement += gamePadInput.ThumbSticks.Left.Y * new Vector2(forward.X, forward.Z);
                totalMovement += gamePadInput.ThumbSticks.Left.X * new Vector2(right.X, right.Z);
                CharacterController.HorizontalMotionConstraint.MovementDirection = Vector2.Normalize(totalMovement);

                //Jumping
                if (previousGamePadInput.IsButtonUp(Buttons.LeftStick) && gamePadInput.IsButtonDown(Buttons.LeftStick))
                {
                    CharacterController.Jump();
                }
#else

                //Collect the movement impulses.

                Vector3 movementDir;

                if (keyboardInput.IsKeyDown(Keys.E))
                {
                    movementDir = Camera.WorldMatrix.Forward;
                    totalMovement += Vector2.Normalize(new Vector2(movementDir.X, movementDir.Z));
                }
                if (keyboardInput.IsKeyDown(Keys.D))
                {
                    movementDir = Camera.WorldMatrix.Forward;
                    totalMovement -= Vector2.Normalize(new Vector2(movementDir.X, movementDir.Z));
                }
                if (keyboardInput.IsKeyDown(Keys.S))
                {
                    movementDir = Camera.WorldMatrix.Left;
                    totalMovement += Vector2.Normalize(new Vector2(movementDir.X, movementDir.Z));
                }
                if (keyboardInput.IsKeyDown(Keys.F))
                {
                    movementDir = Camera.WorldMatrix.Right;
                    totalMovement += Vector2.Normalize(new Vector2(movementDir.X, movementDir.Z));
                }
                if (totalMovement == Vector2.Zero)
                    CharacterController.HorizontalMotionConstraint.MovementDirection = Vector2.Zero;
                else
                    CharacterController.HorizontalMotionConstraint.MovementDirection = Vector2.Normalize(totalMovement);


                //Jumping
                if (previousKeyboardInput.IsKeyUp(Keys.A) && keyboardInput.IsKeyDown(Keys.A))
                {
                    CharacterController.Jump();
                }
#endif

            }
        }
    }
}