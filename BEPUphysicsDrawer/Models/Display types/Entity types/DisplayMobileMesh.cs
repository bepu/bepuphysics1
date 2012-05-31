using System.Collections.Generic;
using BEPUphysics.Collidables;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.CollisionShapes;
using BEPUphysics.Collidables.MobileCollidables;
using ConversionHelper;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Simple display object for triangles.
    /// </summary>
    public class DisplayMobileMesh
    {
        public static void GetShapeMeshData(EntityCollidable collidable, List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            MobileMeshShape shape = collidable.Shape as MobileMeshShape;
            var tempVertices = new VertexPositionNormalTexture[shape.TriangleMesh.Data.Vertices.Length];
            for (int i = 0; i < shape.TriangleMesh.Data.Vertices.Length; i++)
            {
                BEPUphysics.MathExtensions.Vector3 position;
                shape.TriangleMesh.Data.GetVertexPosition(i, out position);
                tempVertices[i] = new VertexPositionNormalTexture(
                    MathConverter.Convert(position),
                    Vector3.Zero, 
                    Vector2.Zero);
            }

            for (int i = 0; i < shape.TriangleMesh.Data.Indices.Length; i++)
            {
                indices.Add((ushort)shape.TriangleMesh.Data.Indices[i]);
            }
            for (int i = 0; i < indices.Count; i += 3)
            {
                int a = indices[i];
                int b = indices[i + 1];
                int c = indices[i + 2];
                Vector3 normal = Vector3.Normalize(Vector3.Cross(
                    tempVertices[c].Position - tempVertices[a].Position,
                    tempVertices[b].Position - tempVertices[a].Position));
                tempVertices[a].Normal += normal;
                tempVertices[b].Normal += normal;
                tempVertices[c].Normal += normal;
            }

            for (int i = 0; i < tempVertices.Length; i++)
            {
                tempVertices[i].Normal.Normalize();
                vertices.Add(tempVertices[i]);
            }
        }

    }
}