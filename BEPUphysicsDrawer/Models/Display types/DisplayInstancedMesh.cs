using System.Collections.Generic;
using BEPUphysics.Collidables;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using ConversionHelper;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Simple display object for triangles.
    /// </summary>
    public class DisplayInstancedMesh : ModelDisplayObject<InstancedMesh>
    {
        /// <summary>
        /// Creates the display object for the entity.
        /// </summary>
        /// <param name="drawer">Drawer managing this display object.</param>
        /// <param name="displayedObject">Entity to draw.</param>
        public DisplayInstancedMesh(ModelDrawer drawer, InstancedMesh displayedObject)
            : base(drawer, displayedObject)
        {
        }

        public override int GetTriangleCountEstimate()
        {
            return DisplayedObject.Shape.TriangleMesh.Data.Indices.Length / 3;
        }

        public override void GetMeshData(List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            var tempVertices = new VertexPositionNormalTexture[DisplayedObject.Shape.TriangleMesh.Data.Vertices.Length];
            for (int i = 0; i < DisplayedObject.Shape.TriangleMesh.Data.Vertices.Length; i++)
            {
                tempVertices[i] = new VertexPositionNormalTexture(
                    MathConverter.Convert(BEPUphysics.MathExtensions.AffineTransform.Transform(DisplayedObject.Shape.TriangleMesh.Data.Vertices[i], DisplayedObject.WorldTransform)),
                    Vector3.Zero, 
                    Vector2.Zero);
            }

            for (int i = 0; i < DisplayedObject.Shape.TriangleMesh.Data.Indices.Length; i++)
            {
                indices.Add((ushort)DisplayedObject.Shape.TriangleMesh.Data.Indices[i]);
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

        public override void Update()
        {
            WorldTransform = Matrix.Identity; //The graphical mesh bakes in the transform.
        }
    }
}