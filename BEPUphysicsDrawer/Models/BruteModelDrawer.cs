using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Manages and draws non-instanced models.
    /// </summary>
    public class BruteModelDrawer : ModelDrawer
    {
        private readonly List<BruteDisplayObjectEntry> displayObjects = new List<BruteDisplayObjectEntry>();
        private readonly BasicEffect effect;

        private VertexDeclaration vertexDeclaration;

        public BruteModelDrawer(Game game)
            : base(game)
        {
            effect = new BasicEffect(game.GraphicsDevice);
            effect.PreferPerPixelLighting = true;
            effect.LightingEnabled = true;
            effect.DirectionalLight0.Enabled = true;
            effect.DirectionalLight0.Direction = Vector3.Normalize(new Vector3(.8f, -1.5f, -1.2f));
            effect.DirectionalLight0.DiffuseColor = new Vector3(.66f, .66f, .66f);
            effect.DirectionalLight1.Enabled = true;
            effect.DirectionalLight1.Direction = Vector3.Normalize(new Vector3(-.8f, 1.5f, 1.2f));
            effect.DirectionalLight1.DiffuseColor = new Vector3(.3f, .3f, .5f);
            effect.AmbientLightColor = new Vector3(.5f, .5f, .5f);

            effect.TextureEnabled = true;

            vertexDeclaration = VertexPositionNormalTexture.VertexDeclaration;
        }

        protected override void Add(ModelDisplayObjectBase displayObject)
        {
            displayObjects.Add(new BruteDisplayObjectEntry(this, displayObject));
        }

        protected override void Remove(ModelDisplayObjectBase displayObject)
        {
            for (int i = 0; i < displayObjects.Count; i++)
            {
                if (displayObjects[i].displayObject == displayObject)
                {
                    displayObjects.RemoveAt(i);
                    break;
                }
            }
        }

        protected override void ClearManagedModels()
        {
            displayObjects.Clear();
        }

        protected override void UpdateManagedModels()
        {
            foreach (BruteDisplayObjectEntry entry in displayObjects)
            {
                entry.displayObject.Update();
            }
        }


        /// <summary>
        /// Draws the models managed by the drawer.
        /// </summary>
        /// <param name="viewMatrix">View matrix to use to draw the objects.</param>
        /// <param name="projectionMatrix">Projection matrix to use to draw the objects.</param>
        protected override void DrawManagedModels(Matrix viewMatrix, Matrix projectionMatrix)
        {
            effect.View = viewMatrix;
            effect.Projection = projectionMatrix;

            for (int i = 0; i < effect.CurrentTechnique.Passes.Count; i++)
            {
                foreach (BruteDisplayObjectEntry entry in displayObjects)
                {
                    entry.Draw(textures, Game.GraphicsDevice, effect, effect.CurrentTechnique.Passes[i]);
                }
            }
        }
    }
}