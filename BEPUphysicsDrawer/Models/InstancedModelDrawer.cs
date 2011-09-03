using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Manages and draws instanced models.
    /// </summary>
    public class InstancedModelDrawer : ModelDrawer
    {
        //'Instanced' may be a little bit of a misnomer, since the vertex buffers contain the local space vertices for every single 
        //display object.  This is due to the fact that each model being drawn can be completely unique.
        //Even two Capsules could require more than a simple scaling transform to be identical.
        //Instead of using a single set of vertex data and redrawing it, this 'instancing' system 
        //will get the GPU to do the transformation heavy lifting.

        private readonly List<ModelDisplayObjectBatch> batches = new List<ModelDisplayObjectBatch>();
        private readonly Effect instancingEffect;
        private readonly EffectParameter projectionParameter;

        private readonly EffectParameter textureIndicesParameter;
        private readonly EffectParameter viewParameter;
        private readonly EffectParameter worldTransformsParameter;
        internal VertexDeclaration instancingVertexDeclaration;

        public InstancedModelDrawer(Game game)
            : base(game)
        {
            instancingEffect = game.Content.Load<Effect>("InstancedEffect");

            worldTransformsParameter = instancingEffect.Parameters["WorldTransforms"];
            textureIndicesParameter = instancingEffect.Parameters["TextureIndices"];
            viewParameter = instancingEffect.Parameters["View"];
            projectionParameter = instancingEffect.Parameters["Projection"];

            instancingEffect.Parameters["LightDirection1"].SetValue(Vector3.Normalize(new Vector3(.8f, -1.5f, -1.2f)));
            instancingEffect.Parameters["DiffuseColor1"].SetValue(new Vector3(.66f, .66f, .66f));
            instancingEffect.Parameters["LightDirection2"].SetValue(Vector3.Normalize(new Vector3(-.8f, 1.5f, 1.2f)));
            instancingEffect.Parameters["DiffuseColor2"].SetValue(new Vector3(.3f, .3f, .5f));
            instancingEffect.Parameters["AmbientAmount"].SetValue(.5f);

            instancingEffect.Parameters["Texture0"].SetValue(textures[0]);
            instancingEffect.Parameters["Texture1"].SetValue(textures[1]);
            instancingEffect.Parameters["Texture2"].SetValue(textures[2]);
            instancingEffect.Parameters["Texture3"].SetValue(textures[3]);
            instancingEffect.Parameters["Texture4"].SetValue(textures[4]);
            instancingEffect.Parameters["Texture5"].SetValue(textures[5]);
            instancingEffect.Parameters["Texture6"].SetValue(textures[6]);
            instancingEffect.Parameters["Texture7"].SetValue(textures[7]);

            //This vertex declaration could be compressed or made more efficient, but such optimizations weren't critical.
            instancingVertexDeclaration = new VertexDeclaration(new[] {new VertexElement(0, VertexElementFormat.Single, VertexElementUsage.TextureCoordinate, 1)});
        }

        protected override void Add(ModelDisplayObjectBase displayObject)
        {
            foreach (ModelDisplayObjectBatch batch in batches)
            {
                if (batch.Add(displayObject, this))
                    return;
            }
            //If we made it here, that means there was no batch that could handle the display object.
            //This could be because the batches were full, or it could be that this
            //display object is just really large (like a terrain) and violated the 
            //primitive count limit.

            //So throw the object in a new batch.
            //This will also allow a display object that is too large (like a 512x512 terrain) to be added to a batch,
            //so long as it is alone.

            var batchToAdd = new ModelDisplayObjectBatch(Game.GraphicsDevice);
            batches.Add(batchToAdd);
            batchToAdd.Add(displayObject, this);
        }

        protected override void Remove(ModelDisplayObjectBase displayObject)
        {
            ModelDisplayObjectBatch batch = displayObject.BatchInformation.Batch;
            batch.Remove(displayObject, this);
            //If it's an empty batch, just throw it away.
            if (batch.DisplayObjects.Count == 0)
                batches.Remove(batch);
        }

        protected override void ClearManagedModels()
        {
            batches.Clear();
        }

        //This drawer isn't really optimized for the frequent addition and removal of objects.
        //It could use a compaction system that makes sure the batches stay full.
        //Also, right now it allocates pretty willy-nilly when it needs a new batch.
        //This could be reduced a little bit by keeping the object instances handy,
        //but it wasn't really worth the effort for the things this drawer is going to be
        //used for.

        protected override void UpdateManagedModels()
        {
            foreach (ModelDisplayObjectBatch batch in batches)
            {
                batch.Update();
            }
        }

        /// <summary>
        /// Draws the models managed by the drawer.
        /// </summary>
        /// <param name="viewMatrix">View matrix to use to draw the objects.</param>
        /// <param name="projectionMatrix">Projection matrix to use to draw the objects.</param>
        protected override void DrawManagedModels(Matrix viewMatrix, Matrix projectionMatrix)
        {
            viewParameter.SetValue(viewMatrix);
            projectionParameter.SetValue(projectionMatrix);

            for (int i = 0; i < instancingEffect.CurrentTechnique.Passes.Count; i++)
            {
                foreach (ModelDisplayObjectBatch batch in batches)
                {
                    batch.Draw(instancingEffect, worldTransformsParameter, textureIndicesParameter, instancingEffect.CurrentTechnique.Passes[i]);
                }
            }
        }
    }
}