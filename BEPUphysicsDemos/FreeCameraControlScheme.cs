using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework.Input;

namespace BEPUphysicsDemos
{
    public class FreeCameraControlScheme : CameraControlScheme
    {
        /// <summary>
        /// Gets or sets the speed at which the camera moves.
        /// </summary>
        public float Speed { get; set; }

        public FreeCameraControlScheme(float speed, Camera camera, DemosGame game)
            : base(camera, game)
        {
            Speed = speed;
        }

        public override void Update(float dt)
        {
            base.Update(dt);

            //Only move around if the camera has control over its own position.
            float distance = Speed * dt;
#if XBOX360
            MoveForward(Game.GamePadInput.ThumbSticks.Left.Y * distance);
            MoveRight(gamePadInput.ThumbSticks.Left.X * distance);
            if (Game.GamePadInput.IsButtonDown(Buttons.LeftStick))
                MoveUp(distance);
            if (Game.GamePadInput.IsButtonDown(Buttons.RightStick))
                MoveUp(-distance);
#endif

            if (Game.KeyboardInput.IsKeyDown(Keys.E))
                Camera.MoveForward(distance);
            if (Game.KeyboardInput.IsKeyDown(Keys.D))
                Camera.MoveForward(-distance);
            if (Game.KeyboardInput.IsKeyDown(Keys.S))
                Camera.MoveRight(-distance);
            if (Game.KeyboardInput.IsKeyDown(Keys.F))
                Camera.MoveRight(distance);
            if (Game.KeyboardInput.IsKeyDown(Keys.A))
                Camera.MoveUp(distance);
            if (Game.KeyboardInput.IsKeyDown(Keys.Z))
                Camera.MoveUp(-distance);
        }
    }
}
