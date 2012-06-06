using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.CollisionShapes.ConvexShapes;
using System;
using BEPUphysics.CollisionShapes;
using BEPUphysics.Collidables.MobileCollidables;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Helper class that can create shape mesh data.
    /// </summary>
    public static class DisplayBox
    {

        public static void GetShapeMeshData(EntityCollidable collidable, List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            var boxShape = collidable.Shape as BoxShape;
            if (boxShape == null)
                throw new ArgumentException("Wrong shape type.");

            var boundingBox = new BoundingBox(
                new Vector3(-boxShape.HalfWidth,
                            -boxShape.HalfHeight,
                            -boxShape.HalfLength),
                new Vector3(boxShape.HalfWidth,
                            boxShape.HalfHeight,
                            boxShape.HalfLength));


            Vector3[] corners = boundingBox.GetCorners();

            var textureCoords = new Vector2[4];
            textureCoords[0] = new Vector2(0, 0);
            textureCoords[1] = new Vector2(1, 0);
            textureCoords[2] = new Vector2(1, 1);
            textureCoords[3] = new Vector2(0, 1);

            vertices.Add(new VertexPositionNormalTexture(corners[0], Vector3.Backward, textureCoords[0]));
            vertices.Add(new VertexPositionNormalTexture(corners[1], Vector3.Backward, textureCoords[1]));
            vertices.Add(new VertexPositionNormalTexture(corners[2], Vector3.Backward, textureCoords[2]));
            vertices.Add(new VertexPositionNormalTexture(corners[3], Vector3.Backward, textureCoords[3]));
            indices.Add(0);
            indices.Add(1);
            indices.Add(2);
            indices.Add(0);
            indices.Add(2);
            indices.Add(3);

            vertices.Add(new VertexPositionNormalTexture(corners[1], Vector3.Right, textureCoords[0]));
            vertices.Add(new VertexPositionNormalTexture(corners[2], Vector3.Right, textureCoords[3]));
            vertices.Add(new VertexPositionNormalTexture(corners[5], Vector3.Right, textureCoords[1]));
            vertices.Add(new VertexPositionNormalTexture(corners[6], Vector3.Right, textureCoords[2]));
            indices.Add(4);
            indices.Add(6);
            indices.Add(7);
            indices.Add(4);
            indices.Add(7);
            indices.Add(5);

            vertices.Add(new VertexPositionNormalTexture(corners[4], Vector3.Forward, textureCoords[1]));
            vertices.Add(new VertexPositionNormalTexture(corners[5], Vector3.Forward, textureCoords[0]));
            vertices.Add(new VertexPositionNormalTexture(corners[6], Vector3.Forward, textureCoords[3]));
            vertices.Add(new VertexPositionNormalTexture(corners[7], Vector3.Forward, textureCoords[2]));
            indices.Add(9);
            indices.Add(8);
            indices.Add(11);
            indices.Add(9);
            indices.Add(11);
            indices.Add(10);

            vertices.Add(new VertexPositionNormalTexture(corners[0], Vector3.Left, textureCoords[1]));
            vertices.Add(new VertexPositionNormalTexture(corners[3], Vector3.Left, textureCoords[2]));
            vertices.Add(new VertexPositionNormalTexture(corners[4], Vector3.Left, textureCoords[0]));
            vertices.Add(new VertexPositionNormalTexture(corners[7], Vector3.Left, textureCoords[3]));
            indices.Add(14);
            indices.Add(12);
            indices.Add(13);
            indices.Add(14);
            indices.Add(13);
            indices.Add(15);

            vertices.Add(new VertexPositionNormalTexture(corners[0], Vector3.Up, textureCoords[2]));
            vertices.Add(new VertexPositionNormalTexture(corners[1], Vector3.Up, textureCoords[3]));
            vertices.Add(new VertexPositionNormalTexture(corners[4], Vector3.Up, textureCoords[1]));
            vertices.Add(new VertexPositionNormalTexture(corners[5], Vector3.Up, textureCoords[0]));
            indices.Add(16);
            indices.Add(19);
            indices.Add(17);
            indices.Add(16);
            indices.Add(18);
            indices.Add(19);

            vertices.Add(new VertexPositionNormalTexture(corners[2], Vector3.Down, textureCoords[1]));
            vertices.Add(new VertexPositionNormalTexture(corners[3], Vector3.Down, textureCoords[0]));
            vertices.Add(new VertexPositionNormalTexture(corners[6], Vector3.Down, textureCoords[2]));
            vertices.Add(new VertexPositionNormalTexture(corners[7], Vector3.Down, textureCoords[3]));
            indices.Add(21);
            indices.Add(20);
            indices.Add(22);
            indices.Add(21);
            indices.Add(22);
            indices.Add(23);
        }
    }
}