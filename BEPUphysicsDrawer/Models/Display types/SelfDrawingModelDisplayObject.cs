

using Microsoft.Xna.Framework;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Superclass of display types that can draw themselves.
    /// These types do not use any instancing techniques.
    /// </summary>
    public abstract class SelfDrawingModelDisplayObject
    {
        protected SelfDrawingModelDisplayObject(ModelDrawer modelDrawer)
        {
            ModelDrawer = modelDrawer;
        }

        /// <summary>
        /// Gets the drawer that manages this display object.
        /// </summary>
        public ModelDrawer ModelDrawer { get; private set; }

        /// <summary>
        /// Updates the display object.
        /// </summary>
        public abstract void Update();

        /// <summary>
        /// Draws the display object.
        /// </summary>
        /// <param name="viewMatrix">Current view matrix.</param>
        /// <param name="projectionMatrix">Current projection matrix.</param>
        public abstract void Draw(Matrix viewMatrix, Matrix projectionMatrix);
    }
}