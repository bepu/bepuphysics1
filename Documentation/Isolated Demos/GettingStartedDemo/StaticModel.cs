
using ConversionHelper;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Matrix = BEPUutilities.Matrix;

namespace GettingStartedDemo
{
    /// <summary>
    /// Component that draws a model.
    /// </summary>
    public class StaticModel : DrawableGameComponent
    {
        Model model;
        /// <summary>
        /// Base transformation to apply to the model.
        /// </summary>
        public Matrix Transform;
        Microsoft.Xna.Framework.Matrix[] boneTransforms;


        /// <summary>
        /// Creates a new StaticModel.
        /// </summary>
        /// <param name="model">Graphical representation to use for the entity.</param>
        /// <param name="transform">Base transformation to apply to the model before moving to the entity.</param>
        /// <param name="game">Game to which this component will belong.</param>
        public StaticModel(Model model, Matrix transform, Game game)
            : base(game)
        {
            this.model = model;
            this.Transform = transform;

            //Collect any bone transformations in the model itself.
            //The default cube model doesn't have any, but this allows the StaticModel to work with more complicated shapes.
            boneTransforms = new Microsoft.Xna.Framework.Matrix[model.Bones.Count];
            foreach (ModelMesh mesh in model.Meshes)
            {
                foreach (BasicEffect effect in mesh.Effects)
                {
                    effect.EnableDefaultLighting();
                }
            }
        }

        public override void Draw(GameTime gameTime)
        {
            model.CopyAbsoluteBoneTransformsTo(boneTransforms);
            foreach (ModelMesh mesh in model.Meshes)
            {
                foreach (BasicEffect effect in mesh.Effects)
                {
                    effect.World = boneTransforms[mesh.ParentBone.Index] * MathConverter.Convert(Transform);
                    effect.View = MathConverter.Convert((Game as GettingStartedGame).Camera.ViewMatrix);
                    effect.Projection = MathConverter.Convert((Game as GettingStartedGame).Camera.ProjectionMatrix);
                }
                mesh.Draw();
            }
            base.Draw(gameTime);
        }
    }
}
