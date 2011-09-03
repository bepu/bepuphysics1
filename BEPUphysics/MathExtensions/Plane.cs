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
        public Vector3 Normal;
        public float D;

        public Plane(Vector3 normal, float d)
        {
            this.Normal = normal;
            this.D = d;
        }

        public void DotCoordinate(ref Vector3 v, out float dot)
        {
            dot = Normal.X * v.X + Normal.Y * v.Y + Normal.Z * v.Z + D;
        }
    }
}
