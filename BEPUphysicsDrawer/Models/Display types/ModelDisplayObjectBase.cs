
using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Base class of ModelDisplayObjects.
    /// </summary>
    public abstract class ModelDisplayObjectBase
    {
        protected static Random Random = new Random();


        protected ModelDisplayObjectBase(ModelDrawer drawer)
        {
            Drawer = drawer;
            BatchInformation = new BatchInformation();
            TextureIndex = Random.Next(8);
        }

        /// <summary>
        /// Gets or sets the world transform of the display object.
        /// </summary>
        public Matrix WorldTransform { get; set; }

        /// <summary>
        /// Gets the batch information associated with this display object if it belongs to a batch.
        /// </summary>
        public BatchInformation BatchInformation { get; private set; }

        /// <summary>
        /// Gets the drawer that this display object belongs to.
        /// </summary>
        public ModelDrawer Drawer { get; private set; }

        /// <summary>
        /// Gets or sets the texture index used for this display object.
        /// </summary>
        public int TextureIndex { get; set; }

        /// <summary>
        /// When a ModelDisplayObject is removed from a batch, the batch information is cleaned up.
        /// </summary>
        public void ClearBatchReferences()
        {
            BatchInformation.Batch = null;
            BatchInformation.BaseVertexBufferIndex = 0;
            BatchInformation.BaseIndexBufferIndex = 0;
            BatchInformation.BatchListIndex = 0;
            BatchInformation.VertexCount = 0;
            BatchInformation.IndexCount = 0;
        }

        /// <summary>
        /// Collects the local space vertex data of the model.
        /// </summary>
        /// <param name="vertices">List of vertices to be filled with the model vertices.</param>
        /// <param name="indices">List of indices to be filled with the model indices.</param>
        /// <param name="batch">Batch that the display object is being added to.</param>
        /// <param name="baseVertexBufferIndex">Index in the batch's vertex buffer where this display object's vertices start.</param>
        /// <param name="baseIndexBufferIndex">Index in the batch's index buffer where this display object's vertices start.</param>
        /// <param name="batchListIndex">Index in the batch's display object list where this display object will be inserted.</param>
        public void GetVertexData(List<VertexPositionNormalTexture> vertices, List<ushort> indices,
                                  ModelDisplayObjectBatch batch, ushort baseVertexBufferIndex, int baseIndexBufferIndex, int batchListIndex)
        {
            BatchInformation.Batch = batch;
            BatchInformation.BaseVertexBufferIndex = baseVertexBufferIndex;
            BatchInformation.BaseIndexBufferIndex = baseIndexBufferIndex;
            BatchInformation.BatchListIndex = batchListIndex;
            GetMeshData(vertices, indices);
            //Modify the indices.
            for (int i = 0; i < indices.Count; i++)
            {
                indices[i] += baseVertexBufferIndex;
            }
            BatchInformation.VertexCount = vertices.Count;
            BatchInformation.IndexCount = indices.Count;
        }

        /// <summary>
        /// Gets the approximate number of triangles that will be used by the display object if added.
        /// </summary>
        /// <returns>Approximate number of triangles that the display object will use if added.</returns>
        public abstract int GetTriangleCountEstimate();

        /// <summary>
        /// Collects the local space vertex data of the model.
        /// </summary>
        /// <param name="vertices">List of vertices to be filled with the model vertices.</param>
        /// <param name="indices">List of indices to be filled with the model indices.</param>
        public abstract void GetMeshData(List<VertexPositionNormalTexture> vertices, List<ushort> indices);

        /// <summary>
        /// Updates the display object and reports the world transform.
        /// </summary>
        public abstract void Update();
    }
}