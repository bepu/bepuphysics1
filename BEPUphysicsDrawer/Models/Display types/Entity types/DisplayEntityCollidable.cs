using BEPUphysics.Entities;
using Microsoft.Xna.Framework;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework.Graphics;
using System.Collections.Generic;
using BEPUphysics.Collidables.MobileCollidables;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Superclass of display objects that follow entity collidables.
    /// </summary>
    public class DisplayEntityCollidable : ModelDisplayObject<EntityCollidable>
    {
        /// <summary>
        /// Constructs a new display entity.
        /// </summary>
        /// <param name="drawer">Drawer to use.</param>
        /// <param name="entityCollidable">EntityCollidable to draw.</param>
        public DisplayEntityCollidable(ModelDrawer drawer, EntityCollidable entityCollidable)
            : base(drawer, entityCollidable)
        {
        }

        public override int GetTriangleCountEstimate()
        {
            return 50;
        }


        public override void GetMeshData(List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            ModelDrawer.ShapeMeshGetters[DisplayedObject.GetType()](DisplayedObject, vertices, indices);
        }


        public override void Update()
        {
            if (DisplayedObject.Entity != null)
            {
                //The reason for this complexity is that we're drawing the shape's location directly and interpolated buffers might be active.
                //That means we can't rely solely on the collidable's world transform or the entity's world transform alone;
                //we must rebuild it from the entity's world transform and the collidable's local position.
                //TODO: This is awfully annoying.  Could use some built-in convenience methods to ease the usage.
                Vector3 translation = Matrix3X3.Transform(DisplayedObject.LocalPosition, DisplayedObject.Entity.BufferedStates.InterpolatedStates.OrientationMatrix);
                translation += DisplayedObject.Entity.BufferedStates.InterpolatedStates.Position;
                Matrix worldTransform = Matrix3X3.ToMatrix4X4(DisplayedObject.Entity.BufferedStates.InterpolatedStates.OrientationMatrix);
                worldTransform.Translation = translation;
                WorldTransform = worldTransform;
            }
            else
            {
                //Entityless EntityCollidables just go by what their current transform is.
                WorldTransform = DisplayedObject.WorldTransform.Matrix;
            }
        }
    }
}