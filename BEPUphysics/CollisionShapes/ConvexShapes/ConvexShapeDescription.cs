using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysics.CollisionShapes.ConvexShapes
{
    /// <summary>
    /// Stores information needed by all convex shapes.
    /// </summary>
    public struct ConvexShapeDescription
    {
        /// <summary>
        /// EntityShape information for this convex shape.
        /// </summary>
        public EntityShapeVolumeDescription EntityShapeVolume;

        /// <summary>
        /// Minimum radius of the convex shape.
        /// Must be contained fully by the shape when centered at the origin.
        /// </summary>
        public float MinimumRadius;

        /// <summary>
        /// Maximum radius of the convex shape.
        /// Must contain the shape fully when centered at the origin.
        /// </summary>
        public float MaximumRadius;

        /// <summary>
        /// Collision margin of the convex shape.
        /// </summary>
        public float CollisionMargin;
    }
}
