using System.Collections.Generic;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Stores vertex and index data for the brute force renderer.
    /// </summary>
    internal class BruteDisplayObjectEntry
    {
        internal ModelDisplayObjectBase displayObject;
        internal BruteModelDrawer drawer;
        internal IndexBuffer indexBuffer;

        internal ushort[] indices;

        internal VertexBuffer vertexBuffer;
        internal VertexPositionNormalTexture[] vertices;

        internal BruteDisplayObjectEntry(BruteModelDrawer drawer, ModelDisplayObjectBase displayObject)
        {
            this.drawer = drawer;
            this.displayObject = displayObject;

            var tempVertices = new List<VertexPositionNormalTexture>();
            var tempIndices = new List<ushort>();
            displayObject.GetMeshData(tempVertices, tempIndices);
            vertices = new VertexPositionNormalTexture[tempVertices.Count];
            indices = new ushort[tempIndices.Count];
            tempVertices.CopyTo(vertices);
            tempIndices.CopyTo(indices);

            vertexBuffer = new VertexBuffer(displayObject.Drawer.Game.GraphicsDevice, typeof (VertexPositionNormalTexture), vertices.Length, BufferUsage.WriteOnly);
            indexBuffer = new IndexBuffer(displayObject.Drawer.Game.GraphicsDevice, IndexElementSize.SixteenBits, indices.Length, BufferUsage.WriteOnly);

            vertexBuffer.SetData(vertices);
            indexBuffer.SetData(indices);
        }

        internal void Draw(Texture2D[] textures, GraphicsDevice device, BasicEffect effect, EffectPass pass)
        {
            effect.World = displayObject.WorldTransform;
            effect.Texture = textures[displayObject.TextureIndex];
            device.SetVertexBuffer(vertexBuffer);
            device.Indices = indexBuffer;
            pass.Apply();

            device.DrawIndexedPrimitives(PrimitiveType.TriangleList, 0, 0, vertices.Length, 0, indices.Length / 3);
        }
    }
}