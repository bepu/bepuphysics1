using System.Collections.Generic;
using BEPUphysics.DataStructures;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Simple display object for triangles.
    /// </summary>
    public class DisplayTriangleMesh : ModelDisplayObject<TriangleMesh>
    {
        /// <summary>
        /// Creates the display object for the entity.
        /// </summary>
        /// <param name="drawer">Drawer managing this display object.</param>
        /// <param name="displayedObject">Entity to draw.</param>
        public DisplayTriangleMesh(ModelDrawer drawer, TriangleMesh displayedObject)
            : base(drawer, displayedObject)
        {
        }

        public override int GetTriangleCountEstimate()
        {
            return DisplayedObject.Data.Indices.Length / 3;
        }

        public static void GetMeshData(TriangleMesh mesh, List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            var tempVertices = new VertexPositionNormalTexture[mesh.Data.Vertices.Length];
            for (int i = 0; i < mesh.Data.Vertices.Length; i++)
            {
                Vector3 v;
                mesh.Data.GetVertexPosition(i, out v);
                tempVertices[i] = new VertexPositionNormalTexture(v, Vector3.Zero, Vector2.Zero);
            }

            for (int i = 0; i < mesh.Data.Indices.Length; i++)
            {
                indices.Add((ushort)mesh.Data.Indices[i]);
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

        public override void GetMeshData(List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            GetMeshData(DisplayedObject, vertices, indices);
        }

        public override void Update()
        {
            WorldTransform = Matrix.Identity; //Transform baked into the vertices.
        }
    }
}