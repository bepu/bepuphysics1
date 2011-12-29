using BEPUphysics.Entities;
using Microsoft.Xna.Framework;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework.Graphics;
using System.Collections.Generic;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Superclass of display objects that follow entities.
    /// </summary>
    public class DisplayEntity : ModelDisplayObject<Entity>
    {
        /// <summary>
        /// Constructs a new 
        /// </summary>
        /// <param name="drawer">Drawer to use.</param>
        /// <param name="entity">Entity to draw.</param>
        public DisplayEntity(ModelDrawer drawer, Entity entity)
            : base(drawer, entity)
        {
        }

        public override int GetTriangleCountEstimate()
        {
            return 100;
        }


        public override void GetMeshData(List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            ModelDrawer.ShapeMeshGetters[DisplayedObject.CollisionInformation.GetType()](DisplayedObject.CollisionInformation, vertices, indices);
        }


        public override void Update()
        {
            //TODO: This is awfully annoying.  Could use some built-in convenience methods to ease the usage.
            Vector3 translation = Matrix3X3.Transform(DisplayedObject.CollisionInformation.LocalPosition, DisplayedObject.BufferedStates.InterpolatedStates.OrientationMatrix);
            translation += DisplayedObject.BufferedStates.InterpolatedStates.Position;
            Matrix worldTransform = Matrix3X3.ToMatrix4X4(DisplayedObject.BufferedStates.InterpolatedStates.OrientationMatrix);
            worldTransform.Translation = translation;

            WorldTransform = worldTransform;

        }
    }
}