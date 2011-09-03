using System.Collections.Generic;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.CollisionShapes;
using System;
using BEPUphysics.Collidables.MobileCollidables;
using ConversionHelper;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Helper class that can create shape mesh data.
    /// </summary>
    public static class DisplayCompoundBody
    {

        public static void GetShapeMeshData(EntityCollidable collidable, List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            var compoundCollidable = collidable as CompoundCollidable;
            if (compoundCollidable == null)
                throw new ArgumentException("Wrong shape type.");
            var tempIndices = new List<ushort>();
            var tempVertices = new List<VertexPositionNormalTexture>();
            for (int i = 0; i < compoundCollidable.Children.Count; i++)
            {
                var child = compoundCollidable.Children[i];
                ModelDrawer.ShapeMeshGetter shapeMeshGetter;
                if (ModelDrawer.ShapeMeshGetters.TryGetValue(child.CollisionInformation.GetType(), out shapeMeshGetter))
                {
                    shapeMeshGetter(child.CollisionInformation, tempVertices, tempIndices);

                    for (int j = 0; j < tempIndices.Count; j++)
                    {
                        indices.Add((ushort)(tempIndices[j] + vertices.Count));
                    }
                    var localTransform = child.Entry.LocalTransform;
                    var localPosition = MathConverter.Convert(child.CollisionInformation.LocalPosition);
                    var orientation = MathConverter.Convert(localTransform.Orientation);
                    var position = MathConverter.Convert(localTransform.Position);
                    for (int j = 0; j < tempVertices.Count; j++)
                    {
                        VertexPositionNormalTexture vertex = tempVertices[j];
                        Microsoft.Xna.Framework.Vector3.Add(ref vertex.Position, ref localPosition, out vertex.Position);
                        Microsoft.Xna.Framework.Vector3.Transform(ref vertex.Position, ref orientation, out vertex.Position);
                        Microsoft.Xna.Framework.Vector3.Add(ref vertex.Position, ref position, out vertex.Position);
                        Microsoft.Xna.Framework.Vector3.Transform(ref vertex.Normal, ref orientation, out vertex.Normal);
                        vertices.Add(vertex);
                    }

                    tempVertices.Clear();
                    tempIndices.Clear();
                }
            }
        }
    }
}