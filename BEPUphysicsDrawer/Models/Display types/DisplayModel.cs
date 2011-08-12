using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Model which just sits in space.
    /// </summary>
    public class DisplayModel : SelfDrawingModelDisplayObject
    {
        private Model model;
        
        /// <summary>
        /// Bone transformations of meshes in the model.
        /// </summary>
        private Matrix[] transforms;

        /// <summary>
        /// Constructs a new display model.
        /// </summary>
        /// <param name="model">Model to draw on the entity.</param>
        /// <param name="modelDrawer">Model drawer to use.</param>
        public DisplayModel(Model model, ModelDrawer modelDrawer)
            : base(modelDrawer)
        {
            Model = model;
        }

        /// <summary>
        /// Gets or sets the model to display.
        /// </summary>
        public Model Model
        {
            get { return model; }
            set
            {
                model = value;
                transforms = new Matrix[model.Bones.Count];
                for (int i = 0; i < Model.Meshes.Count; i++)
                {
                    for (int j = 0; j < Model.Meshes[i].Effects.Count; j++)
                    {
                        var effect = Model.Meshes[i].Effects[j] as BasicEffect;
                        if (effect != null)
                            effect.EnableDefaultLighting();
                    }
                }
            }
        }

        /// <summary>
        /// Gets or sets the texture drawn on this model.
        /// </summary>
        public Texture2D Texture
        {
            get;
            set;
        }


        /// <summary>
        /// Gets or sets the world transformation applied to the model.
        /// </summary>
        public Matrix WorldTransform { get; set; }

        /// <summary>
        /// Updates the display object.
        /// </summary>
        public override void Update()
        {
        }

        /// <summary>
        /// Draws the display object.
        /// </summary>
        /// <param name="viewMatrix">Current view matrix.</param>
        /// <param name="projectionMatrix">Current projection matrix.</param>
        public override void Draw(Matrix viewMatrix, Matrix projectionMatrix)
        {
            //This is not a particularly fast method of drawing.
            //It's used very rarely in the demos.
            model.CopyAbsoluteBoneTransformsTo(transforms);
            for (int i = 0; i < Model.Meshes.Count; i++)
            {
                for (int j = 0; j < Model.Meshes[i].Effects.Count; j++)
                {
                    var effect = Model.Meshes[i].Effects[j] as BasicEffect;
                    if (effect != null)
                    {
                        effect.World = transforms[Model.Meshes[i].ParentBone.Index] * WorldTransform;
                        effect.View = viewMatrix;
                        effect.Projection = projectionMatrix;

                        if (Texture != null)
                        {
                            effect.TextureEnabled = true;
                            effect.Texture = Texture;
                        }
                        else
                            effect.TextureEnabled = false;
                    }
                }
                Model.Meshes[i].Draw();
            }
        }
    }
}