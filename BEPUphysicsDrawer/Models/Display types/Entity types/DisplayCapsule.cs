using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionShapes;
using BEPUphysics.Collidables.MobileCollidables;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Helper class that can create shape mesh data.
    /// </summary>
    public static class DisplayCapsule
    {
        /// <summary>
        /// Number of sides to build geometry with.
        /// </summary>
        public static int NumSides = 24;


        public static void GetShapeMeshData(EntityCollidable collidable, List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            var capsuleShape = collidable.Shape as CapsuleShape;
            if (capsuleShape == null)
                throw new ArgumentException("Wrong shape type.");

            var n = new Vector3();
            var offset = new Vector3(0, capsuleShape.Length / 2, 0);
            float angleBetweenFacets = MathHelper.TwoPi / NumSides;
            float radius = capsuleShape.Radius;

            //Create the vertex list
            //Top
            vertices.Add(new VertexPositionNormalTexture(new Vector3(0, radius + capsuleShape.Length / 2, 0), Vector3.Up, Vector2.Zero));
            //Upper hemisphere
            for (int i = 1; i <= NumSides / 4; i++)
            {
                float phi = MathHelper.PiOver2 - i * angleBetweenFacets;
                var sinPhi = (float)Math.Sin(phi);
                var cosPhi = (float)Math.Cos(phi);

                for (int j = 0; j < NumSides; j++)
                {
                    float theta = j * angleBetweenFacets;

                    n.X = (float)Math.Cos(theta) * cosPhi;
                    n.Y = sinPhi;
                    n.Z = (float)Math.Sin(theta) * cosPhi;

                    vertices.Add(new VertexPositionNormalTexture(n * radius + offset, n, Vector2.Zero));
                }
            }
            //Lower hemisphere
            for (int i = NumSides / 4; i < NumSides / 2; i++)
            {
                float phi = MathHelper.PiOver2 - i * angleBetweenFacets;
                var sinPhi = (float)Math.Sin(phi);
                var cosPhi = (float)Math.Cos(phi);

                for (int j = 0; j < NumSides; j++)
                {
                    float theta = j * angleBetweenFacets;

                    n.X = (float)Math.Cos(theta) * cosPhi;
                    n.Y = sinPhi;
                    n.Z = (float)Math.Sin(theta) * cosPhi;

                    vertices.Add(new VertexPositionNormalTexture(n * radius - offset, n, Vector2.Zero));
                }
            }
            //Bottom
            vertices.Add(new VertexPositionNormalTexture(new Vector3(0, -radius - capsuleShape.Length / 2, 0), Vector3.Down, Vector2.Zero));


            //Create the index list
            for (int i = 0; i < NumSides; i++)
            {
                indices.Add((ushort)(vertices.Count - 1));
                indices.Add((ushort)(vertices.Count - 2 - i));
                indices.Add((ushort)(vertices.Count - 2 - (i + 1) % NumSides));
            }

            for (int i = 0; i < NumSides / 2 - 1; i++)
            {
                for (int j = 0; j < NumSides; j++)
                {
                    int nextColumn = (j + 1) % NumSides;

                    indices.Add((ushort)(i * NumSides + nextColumn + 1));
                    indices.Add((ushort)(i * NumSides + j + 1));
                    indices.Add((ushort)((i + 1) * NumSides + j + 1));

                    indices.Add((ushort)((i + 1) * NumSides + nextColumn + 1));
                    indices.Add((ushort)(i * NumSides + nextColumn + 1));
                    indices.Add((ushort)((i + 1) * NumSides + j + 1));
                }
            }

            for (int i = 0; i < NumSides; i++)
            {
                indices.Add(0);
                indices.Add((ushort)(i + 1));
                indices.Add((ushort)((i + 1) % NumSides + 1));
            }
        }
    }
}