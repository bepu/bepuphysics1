using System.Collections.Generic;
using BEPUphysics.DataStructures;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.Collidables;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Simple display object for triangles.
    /// </summary>
    public class DisplayStaticMesh : ModelDisplayObject<StaticMesh>
    {
        /// <summary>
        /// Creates the display object for the object.
        /// </summary>
        /// <param name="drawer">Drawer managing this display object.</param>
        /// <param name="displayedObject">Object to draw.</param>
        public DisplayStaticMesh(ModelDrawer drawer, StaticMesh displayedObject)
            : base(drawer, displayedObject)
        {
        }

        public override int GetTriangleCountEstimate()
        {
            return DisplayedObject.Mesh.Data.Indices.Length / 3;
        }

        public override void GetMeshData(List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            DisplayTriangleMesh.GetMeshData(DisplayedObject.Mesh, vertices, indices);
        }

        public override void Update()
        {
            WorldTransform = Matrix.Identity; //Transform baked into the vertices.
        }
    }
}