using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Manages batching of display models and their drawing.
    /// </summary>
    public class ModelDisplayObjectBatch
    {
        /// <summary>
        /// Maximum number of display objects that can be lumped into a single display batch.
        /// </summary>
        public const int MaximumObjectsPerBatch = 81;

        /// <summary>
        /// Maximum number of primitives that can be batched together in a single draw call.
        /// </summary>
        public const int MaximumPrimitiveCountPerBatch = 65535;
        public const int MaximumIndexCount = MaximumPrimitiveCountPerBatch * 3;

        private readonly GraphicsDevice graphicsDevice;
        private readonly List<ushort> indexList = new List<ushort>();
        private readonly List<ModelDisplayObject> displayObjects = new List<ModelDisplayObject>();
        private readonly ReadOnlyCollection<ModelDisplayObject> myDisplayObjectsReadOnly;

        /// <summary>
        /// List of textures associated with display objects in the batch.
        /// </summary>
        private readonly float[] textureIndices = new float[MaximumObjectsPerBatch];

        //These lists are used to collect temporary data from display objects.
        private readonly List<VertexPositionNormalTexture> vertexList = new List<VertexPositionNormalTexture>();

        /// <summary>
        /// List of all world transforms associated with display objects in the batch.
        /// </summary>
        private readonly Matrix[] worldTransforms = new Matrix[MaximumObjectsPerBatch];

        private IndexBuffer indexBuffer;
        int indexCount;
        private ushort[] indices;

        /// <summary>
        /// Contains instancing data to be fed into the second stream.
        /// </summary>
        private VertexBuffer instancedBuffer;

        int vertexCount;
        private InstancedVertex[] instancedVertices;
        private VertexBuffer vertexBuffer;
        private VertexPositionNormalTexture[] vertices;
        private VertexBufferBinding[] bindings;

        //Has VertexBuffer, IndexBuffer, second stream for indices.
        //Why second stream?  Creating it will be easier since can addRange on lists gotten from ModelDisplayObject, then toss in
        //the appropriate indices separately.


        /// <summary>
        /// Constructs a new batch.
        /// </summary>
        /// <param name="graphicsDevice">Device to use when drawing.</param>
        public ModelDisplayObjectBatch(GraphicsDevice graphicsDevice)
        {
            this.graphicsDevice = graphicsDevice;
            myDisplayObjectsReadOnly = new ReadOnlyCollection<ModelDisplayObject>(displayObjects);
            instancedVertices = new InstancedVertex[MaximumIndexCount];
            vertices = new VertexPositionNormalTexture[MaximumIndexCount];
            indices = new ushort[MaximumIndexCount];
            vertexBuffer = new VertexBuffer(graphicsDevice, VertexPositionNormalTexture.VertexDeclaration, MaximumIndexCount, BufferUsage.WriteOnly);
            instancedBuffer = new VertexBuffer(graphicsDevice, InstancedVertex.VertexDeclaration, MaximumIndexCount, BufferUsage.WriteOnly);
            indexBuffer = new IndexBuffer(graphicsDevice, IndexElementSize.SixteenBits, MaximumIndexCount, BufferUsage.WriteOnly);
            bindings = new[] { new VertexBufferBinding(vertexBuffer), new VertexBufferBinding(instancedBuffer) };
        }

        /// <summary>
        /// Gets the display objects in this batch.
        /// </summary>
        public ReadOnlyCollection<ModelDisplayObject> DisplayObjects
        {
            get { return myDisplayObjectsReadOnly; }
        }



        /// <summary>
        /// Adds a display object to the batch.
        /// </summary>
        /// <param name="displayObject">Display object to add.</param>
        /// <param name="drawer">Drawer of the batch.</param>
        public unsafe bool Add(ModelDisplayObject displayObject, InstancedModelDrawer drawer)
        {
            //In theory, don't need to test for duplicate entries since batch.Add
            //should only be called through a InstancedModelDrawer's add (which checks beforehand).
            if (displayObjects.Count == MaximumObjectsPerBatch ||
                ((indexCount / 3 + displayObject.GetTriangleCountEstimate()) > MaximumPrimitiveCountPerBatch &&
                 displayObjects.Count > 0))
                return false;
            displayObjects.Add(displayObject);
            int instanceIndex = displayObjects.Count - 1;
            var textureIndex = displayObject.TextureIndex;
            //OUegheogh, this could just directly write into the batch's vertex/index cache rather than going through a list and recopy.
            displayObject.GetVertexData(vertexList, indexList, this, (ushort)vertexCount, indexCount, instanceIndex);
            vertexList.CopyTo(vertices, vertexCount);
            indexList.CopyTo(indices, indexCount);

            var newIndexCount = indexCount + indexList.Count;
            var newVertexCount = vertexCount + vertexList.Count;
            for (int i = vertexCount; i < newVertexCount; i++)
                instancedVertices[i] = new InstancedVertex { InstanceIndex = instanceIndex, TextureIndex = textureIndex };

            vertexBuffer.SetData(sizeof(VertexPositionNormalTexture) * vertexCount, vertices, vertexCount, vertexList.Count, sizeof(VertexPositionNormalTexture));
            instancedBuffer.SetData(sizeof(InstancedVertex) * vertexCount, instancedVertices, vertexCount, vertexList.Count, sizeof(InstancedVertex));
            indexBuffer.SetData(sizeof(ushort) * indexCount, indices, indexCount, indexList.Count);

            vertexCount = newVertexCount;
            indexCount = newIndexCount;

            vertexList.Clear();
            indexList.Clear();
            return true;
        }

        /// <summary>
        /// Removes a display object from the batch.
        /// </summary>
        /// <param name="displayObject">Display object to remove.</param>
        /// <param name="drawer">Instanced model drawer doing the removal.</param>
        public unsafe void Remove(ModelDisplayObject displayObject, InstancedModelDrawer drawer)
        {
            //Copy the end of the list over the top of the  part back (after the display object)
            var vertexCopySource = displayObject.BatchInformation.BaseVertexBufferIndex + displayObject.BatchInformation.VertexCount;
            var vertexCopyTarget = displayObject.BatchInformation.BaseVertexBufferIndex;
            var vertexCopyLength = vertexCount - vertexCopySource;
            Array.Copy(vertices, vertexCopySource, vertices, vertexCopyTarget, vertexCopyLength);
            Array.Copy(instancedVertices, vertexCopySource, instancedVertices, vertexCopyTarget, vertexCopyLength);
            vertexCount -= displayObject.BatchInformation.VertexCount;

            //Copy the first part back (before the display object)     
            var indexCopySource = displayObject.BatchInformation.BaseIndexBufferIndex + displayObject.BatchInformation.IndexCount;
            var indexCopyTarget = displayObject.BatchInformation.BaseIndexBufferIndex;
            var indexCopyLength = indexCount - indexCopySource;
            Array.Copy(indices, indexCopySource, indices, indexCopyTarget, indexCopyLength);
            indexCount -= displayObject.BatchInformation.IndexCount;

            //The index buffer's data is now wrong. We deleted a bunch of vertices.
            //So go through the index buffer starting at the point of deletion and decrease the values appropriately.
            for (int i = displayObject.BatchInformation.BaseIndexBufferIndex; i < indexCount; i++)
                indices[i] -= (ushort)displayObject.BatchInformation.VertexCount;

            //Like with the index buffer, go through the buffer starting at the point of deletion and decrease the instance index values.
            for (int i = displayObject.BatchInformation.BaseVertexBufferIndex; i < vertexCount; i++)
                instancedVertices[i].InstanceIndex--;

            displayObjects.Remove(displayObject);
            //Move the subsequent display objects list indices and base vertices/indices.
            for (int i = displayObject.BatchInformation.BatchListIndex; i < DisplayObjects.Count; i++)
            {
                DisplayObjects[i].BatchInformation.BatchListIndex--;
                DisplayObjects[i].BatchInformation.BaseVertexBufferIndex -= displayObject.BatchInformation.VertexCount;
                DisplayObjects[i].BatchInformation.BaseIndexBufferIndex -= displayObject.BatchInformation.IndexCount;
            }
            // Tell ModelDisplayObject that it got removed (batch, indices).
            var batchListIndex = displayObject.BatchInformation.BatchListIndex;
            displayObject.ClearBatchReferences();

            if (displayObjects.Count > 0 && batchListIndex < displayObjects.Count)
            {
                vertexBuffer.SetData(sizeof(VertexPositionNormalTexture) * vertexCopyTarget, vertices, vertexCopyTarget, vertexCopyLength, sizeof(VertexPositionNormalTexture));
                instancedBuffer.SetData(sizeof(InstancedVertex) * vertexCopyTarget, instancedVertices, vertexCopyTarget, vertexCopyLength, sizeof(InstancedVertex));
                indexBuffer.SetData(sizeof(ushort) * indexCopyTarget, indices, indexCopyTarget, indexCopyLength);
            }
        }

        /// <summary>
        /// Clears the batch of all display objects.
        /// </summary>
        public void Clear()
        {
            vertexCount = 0;
            indexCount = 0;
            foreach (var displayObject in displayObjects)
                displayObject.ClearBatchReferences();
            displayObjects.Clear();
        }

        /// <summary>
        /// Updates the display objects within the batch and collects transforms.
        /// </summary>
        public void Update()
        {
            for (int i = 0; i < displayObjects.Count; i++)
            {
                displayObjects[i].Update();
                worldTransforms[i] = displayObjects[i].WorldTransform;
                textureIndices[i] = displayObjects[i].TextureIndex;
            }
        }

        /// <summary>
        /// Draws the models managed by the batch.
        /// </summary>
        public void Draw(Effect effect, EffectParameter worldTransformsParameter, EffectParameter textureIndicesParameter, EffectPass pass)
        {
            if (vertexCount > 0)
            {
                graphicsDevice.SetVertexBuffers(bindings);
                graphicsDevice.Indices = indexBuffer;
                worldTransformsParameter.SetValue(worldTransforms);
                textureIndicesParameter.SetValue(textureIndices);
                pass.Apply();

                graphicsDevice.DrawIndexedPrimitives(PrimitiveType.TriangleList,
                                                     0, 0, indexCount / 3);
            }
        }

        internal struct InstancedVertex
        {
            /// <summary>
            /// Index of the instance to which this vertex belongs.
            /// </summary>
            public float InstanceIndex;
            /// <summary>
            /// Index of the texture used by the vertex.
            /// </summary>
            public float TextureIndex;

            public static readonly VertexDeclaration VertexDeclaration = new VertexDeclaration(new[]
            {
                new VertexElement(0, VertexElementFormat.Single, VertexElementUsage.TextureCoordinate, 1),
                new VertexElement(4, VertexElementFormat.Single, VertexElementUsage.TextureCoordinate, 2)
            });
        }

        bool disposed;
        public void Dispose()
        {
            if(!disposed)
            {
                disposed = true;
                vertexBuffer.Dispose();
                indexBuffer.Dispose();
                instancedBuffer.Dispose();
            }
        }
        
    }
}