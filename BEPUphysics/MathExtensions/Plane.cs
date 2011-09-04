using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysics.MathExtensions
{
    /// <summary>
    /// Provides XNA-like plane functionality needed by the engine.
    /// </summary>
    public struct Plane
    {
        /// <summary>
        /// Normal of the plane.
        /// </summary>
        public Vector3 Normal;
        /// <summary>
        /// Negative distance to the plane from the origin along the normal.
        /// </summary>
        public float D;

        /// <summary>
        /// Constructs a new plane.
        /// </summary>
        /// <param name="normal">Normal of the plane.</param>
        /// <param name="d">Negative distance to the plane from the origin along the normal</param>
        public Plane(Vector3 normal, float d)
        {
            this.Normal = normal;
            this.D = d;
        }

        /// <summary>
        /// Gets the dot product of the position offset from the plane along the plane's normal.
        /// </summary>
        /// <param name="v">Position to compute the dot product of.</param>
        /// <param name="dot">Dot product.</param>
        public void DotCoordinate(ref Vector3 v, out float dot)
        {
            dot = Normal.X * v.X + Normal.Y * v.Y + Normal.Z * v.Z + D;
        }
    }
}
