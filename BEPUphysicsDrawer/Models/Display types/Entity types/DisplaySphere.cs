
using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Collidables.MobileCollidables;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Helper class that can create shape mesh data.
    /// </summary>
    public static class DisplaySphere
    {
        /// <summary>
        /// Number of sides to build geometry with.
        /// </summary>
        public static int NumSides = 24;


        public static void GetShapeMeshData(EntityCollidable collidable, List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            var sphereShape = collidable.Shape as SphereShape;
            if (sphereShape == null)
                throw new ArgumentException("Wrong shape type");

            var n = new Vector3();
            float angleBetweenFacets = MathHelper.TwoPi / NumSides;
            float radius = sphereShape.Radius;

            //Create the vertex list
            vertices.Add(new VertexPositionNormalTexture(new Vector3(0, radius, 0), Vector3.Up, Vector2.Zero));
            for (int i = 1; i < NumSides / 2; i++)
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

                    vertices.Add(new VertexPositionNormalTexture(n * radius, n, Vector2.Zero));
                }
            }
            vertices.Add(new VertexPositionNormalTexture(new Vector3(0, -radius, 0), Vector3.Down, Vector2.Zero));


            //Create the index list
            for (int i = 0; i < NumSides; i++)
            {
                indices.Add((ushort)(vertices.Count - 1));
                indices.Add((ushort)(vertices.Count - 2 - i));
                indices.Add((ushort)(vertices.Count - 2 - (i + 1) % NumSides));
            }

            for (int i = 0; i < NumSides / 2 - 2; i++)
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