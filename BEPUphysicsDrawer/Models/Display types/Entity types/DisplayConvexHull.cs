

using System.Collections.Generic;
using BEPUphysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using System;
using BEPUphysics.Collidables.MobileCollidables;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Helper class that can create shape mesh data.
    /// </summary>
    public static class DisplayConvexHull
    {

        public static void GetShapeMeshData(EntityCollidable collidable, List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            var convexHullShape = collidable.Shape as ConvexHullShape;
            if (convexHullShape == null)
                throw new ArgumentException("Wrong shape type.");

            var hullTriangleVertices = new List<Vector3>();
            var hullTriangleIndices = new List<int>();
            Toolbox.GetConvexHull(convexHullShape.Vertices, hullTriangleIndices, hullTriangleVertices);
            //The hull triangle vertices are used as a dummy to get the unnecessary hull vertices, which are cleared afterwards.
            hullTriangleVertices.Clear();
            foreach (int i in hullTriangleIndices)
            {
                hullTriangleVertices.Add(convexHullShape.Vertices[i]);
            }

            var toReturn = new VertexPositionNormalTexture[hullTriangleVertices.Count];
            Vector3 normal;
            for (ushort i = 0; i < hullTriangleVertices.Count; i += 3)
            {
                normal = Vector3.Normalize(Vector3.Cross(hullTriangleVertices[i + 2] - hullTriangleVertices[i], hullTriangleVertices[i + 1] - hullTriangleVertices[i]));
                vertices.Add(new VertexPositionNormalTexture(hullTriangleVertices[i], normal, new Vector2(0, 0)));
                vertices.Add(new VertexPositionNormalTexture(hullTriangleVertices[i + 1], normal, new Vector2(1, 0)));
                vertices.Add(new VertexPositionNormalTexture(hullTriangleVertices[i + 2], normal, new Vector2(0, 1)));
                indices.Add(i);
                indices.Add((ushort)(i + 1));
                indices.Add((ushort)(i + 2));
            }
        }
    }
}