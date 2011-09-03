using BEPUphysics;
using Microsoft.Xna.Framework.Input;
using ConversionHelper;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.AlternateMovement.Testing.ConvexCast
{
    /// <summary>
    /// Handles input and movement of a character in the game.
    /// Acts as a simple 'front end' for the bookkeeping and math of the character controller.
    /// </summary>
    public class CharacterControllerConvexCastInput
    {
        /// <summary>
        /// Camera to use for input.
        /// </summary>
        public Camera Camera;

        /// <summary>
        /// Current offset from the position of the character to the 'eyes.'
        /// </summary>
        public Vector3 CameraOffset = new Vector3(0, .7f, 0);

        /// <summary>
        /// Physics representation of the character.
        /// </summary>
        public CharacterControllerConvexCast CharacterController;

        /// <summary>
        /// Whether or not to use the character controller's input.
        /// </summary>
        public bool IsActive = true;

        /// <summary>
        /// Owning space of the character.
        /// </summary>
        public Space Space;


        /// <summary>
        /// Constructs the character and internal physics character controller.
        /// </summary>
        /// <param name="owningSpace">Space to add the character to.</param>
        /// <param name="CameraToUse">Camera to attach to the character.</param>
        public CharacterControllerConvexCastInput(Space owningSpace, Camera CameraToUse)
        {
            CharacterController = new CharacterControllerConvexCast();

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
                CharacterController.Body.Position = MathConverter.Convert(Camera.Position);
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

                //Puts the Camera at eye level.
                Camera.Position = MathConverter.Convert(CharacterController.Body.Position + CameraOffset);
                Vector2 totalMovement = Vector2.Zero;


                //Collect the movement impulses.

                Vector3 movementDir;

                if (keyboardInput.IsKeyDown(Keys.E))
                {
                    movementDir = MathConverter.Convert(Camera.WorldMatrix.Forward);
                    totalMovement += Vector2.Normalize(new Vector2(movementDir.X, movementDir.Z));
                }
                if (keyboardInput.IsKeyDown(Keys.D))
                {
                    movementDir = MathConverter.Convert(Camera.WorldMatrix.Forward);
                    totalMovement -= Vector2.Normalize(new Vector2(movementDir.X, movementDir.Z));
                }
                if (keyboardInput.IsKeyDown(Keys.S))
                {
                    movementDir = MathConverter.Convert(Camera.WorldMatrix.Left);
                    totalMovement += Vector2.Normalize(new Vector2(movementDir.X, movementDir.Z));
                }
                if (keyboardInput.IsKeyDown(Keys.F))
                {
                    movementDir = MathConverter.Convert(Camera.WorldMatrix.Right);
                    totalMovement += Vector2.Normalize(new Vector2(movementDir.X, movementDir.Z));
                }
                if (totalMovement == Vector2.Zero)
                    CharacterController.MovementDirection = Vector2.Zero;
                else
                    CharacterController.MovementDirection = Vector2.Normalize(totalMovement);

                //Jumping
                if (previousKeyboardInput.IsKeyUp(Keys.A) && keyboardInput.IsKeyDown(Keys.A))
                {
                    CharacterController.Jump();
                }

            }
        }
    }
}