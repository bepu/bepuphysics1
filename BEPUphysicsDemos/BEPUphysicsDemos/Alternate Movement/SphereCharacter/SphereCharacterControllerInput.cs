using BEPUphysics;
using BEPUutilities;
using Microsoft.Xna.Framework.Input;

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
        /// <param name="cameraToUse">Camera to attach to the character.</param>
        public SphereCharacterControllerInput(Space owningSpace, Camera cameraToUse)
        {
            CharacterController = new SphereCharacterController();

            Space = owningSpace;
            Space.Add(CharacterController);


            Camera = cameraToUse;
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
                CharacterController.Body.Position = Camera.Position - new Vector3(0, CameraOffset, 0);
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