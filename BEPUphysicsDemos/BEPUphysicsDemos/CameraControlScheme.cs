using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysicsDemos
{
    /// <summary>
    /// Superclass of implementations which control the behavior of a camera.
    /// </summary>
    public abstract class CameraControlScheme
    {
        /// <summary>
        /// Gets the game associated with the camera.
        /// </summary>
        public DemosGame Game { get; private set; }

        /// <summary>
        /// Gets the camera controlled by this control scheme.
        /// </summary>
        public Camera Camera { get; private set; }

        protected CameraControlScheme(Camera camera, DemosGame game)
        {
            Camera = camera;
            Game = game;
        }

        /// <summary>
        /// Updates the camera state according to the control scheme.
        /// </summary>
        /// <param name="dt">Time elapsed since previous frame.</param>
        public virtual void Update(float dt)
        {
#if XBOX360
            Yaw += Game.GamePadInput.ThumbSticks.Right.X * -1.5f * dt;
            Pitch += Game.GamePadInput.ThumbSticks.Right.Y * 1.5f * dt;
#else
            //Only turn if the mouse is controlled by the game.
            if (!Game.IsMouseVisible)
            {
                Camera.Yaw((200 - Game.MouseInput.X) * dt * .12f);
                Camera.Pitch((200 - Game.MouseInput.Y) * dt * .12f);
            }
#endif
        }
    }
}
