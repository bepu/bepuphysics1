using System.Collections.Generic;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.CollisionShapes;
using System;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Helper class that can create shape mesh data.
    /// </summary>
    public static class DisplayCompoundBody
    {

        public static void GetShapeMeshData(CollisionShape shape, List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            var compoundShape = shape as CompoundShape;
            if (compoundShape == null)
                throw new ArgumentException("Wrong shape type.");
            var tempIndices = new List<ushort>();
            var tempVertices = new List<VertexPositionNormalTexture>();
            for (int i = 0; i < compoundShape.Shapes.Count; i++)
            {
                CompoundShapeEntry entry = compoundShape.Shapes[i];
                ModelDrawer.ShapeMeshGetter shapeMeshGetter;
                if (ModelDrawer.ShapeMeshGetters.TryGetValue(entry.Shape.GetType(), out shapeMeshGetter))
                {
                    shapeMeshGetter(entry.Shape, tempVertices, tempIndices);

                    for (int j = 0; j < tempIndices.Count; j++)
                    {
                        indices.Add((ushort)(tempIndices[j] + vertices.Count));
                    }
                    for (int j = 0; j < tempVertices.Count; j++)
                    {
                        VertexPositionNormalTexture vertex = tempVertices[j];
                        RigidTransform.Transform(ref vertex.Position, ref entry.LocalTransform, out vertex.Position);
                        vertex.Normal = Vector3.Transform(vertex.Normal, entry.LocalTransform.Orientation);
                        vertices.Add(vertex);
                    }

                    tempVertices.Clear();
                    tempIndices.Clear();
                }
            }
        }
    }
}