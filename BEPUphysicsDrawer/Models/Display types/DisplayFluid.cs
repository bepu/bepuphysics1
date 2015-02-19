using System.Collections.Generic;
using BEPUphysics.UpdateableSystems;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using ConversionHelper;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Simple display object for triangles.
    /// </summary>
    public class DisplayFluid : ModelDisplayObject<FluidVolume>
    {
        /// <summary>
        /// Creates the display object for the entity.
        /// </summary>
        /// <param name="drawer">Drawer managing this display object.</param>
        /// <param name="displayedObject">Entity to draw.</param>
        public DisplayFluid(ModelDrawer drawer, FluidVolume displayedObject)
            : base(drawer, displayedObject)
        {
        }

        public override int GetTriangleCountEstimate()
        {
            return 2 * DisplayedObject.SurfaceTriangles.Count;
        }

        public override void GetMeshData(List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            for (int i = 0; i < DisplayedObject.SurfaceTriangles.Count; i++)
            {
                Vector3 upVector = MathConverter.Convert(DisplayedObject.UpVector);
                vertices.Add(new VertexPositionNormalTexture(MathConverter.Convert(DisplayedObject.SurfaceTriangles[i][0]), upVector, new Vector2(0, 0)));
                vertices.Add(new VertexPositionNormalTexture(MathConverter.Convert(DisplayedObject.SurfaceTriangles[i][1]), upVector, new Vector2(0, 1)));
                vertices.Add(new VertexPositionNormalTexture(MathConverter.Convert(DisplayedObject.SurfaceTriangles[i][2]), upVector, new Vector2(1, 0)));

                vertices.Add(new VertexPositionNormalTexture(MathConverter.Convert(DisplayedObject.SurfaceTriangles[i][0]), -upVector, new Vector2(0, 0)));
                vertices.Add(new VertexPositionNormalTexture(MathConverter.Convert(DisplayedObject.SurfaceTriangles[i][1]), -upVector, new Vector2(0, 1)));
                vertices.Add(new VertexPositionNormalTexture(MathConverter.Convert(DisplayedObject.SurfaceTriangles[i][2]), -upVector, new Vector2(1, 0)));

                indices.Add((ushort) (i * 6));
                indices.Add((ushort) (i * 6 + 1));
                indices.Add((ushort) (i * 6 + 2));

                indices.Add((ushort) (i * 6 + 3));
                indices.Add((ushort) (i * 6 + 5));
                indices.Add((ushort) (i * 6 + 4));
            }
        }

        public override void Update()
        {
            WorldTransform = Matrix.Identity;
        }
    }
}