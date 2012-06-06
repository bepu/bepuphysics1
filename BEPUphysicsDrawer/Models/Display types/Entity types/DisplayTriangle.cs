
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
    public static class DisplayTriangle
    {


        public static void GetShapeMeshData(EntityCollidable collidable, List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            var triangleShape = collidable.Shape as TriangleShape;
            if(triangleShape == null)
                throw new ArgumentException("Wrong shape type.");
            Vector3 normal = triangleShape.GetLocalNormal();
            vertices.Add(new VertexPositionNormalTexture(triangleShape.VertexA, -normal, new Vector2(0, 0)));
            vertices.Add(new VertexPositionNormalTexture(triangleShape.VertexB, -normal, new Vector2(0, 1)));
            vertices.Add(new VertexPositionNormalTexture(triangleShape.VertexC, -normal, new Vector2(1, 0)));

            vertices.Add(new VertexPositionNormalTexture(triangleShape.VertexA, normal, new Vector2(0, 0)));
            vertices.Add(new VertexPositionNormalTexture(triangleShape.VertexB, normal, new Vector2(0, 1)));
            vertices.Add(new VertexPositionNormalTexture(triangleShape.VertexC, normal, new Vector2(1, 0)));

            indices.Add(0);
            indices.Add(1);
            indices.Add(2);

            indices.Add(3);
            indices.Add(5);
            indices.Add(4);
        }
    }
}