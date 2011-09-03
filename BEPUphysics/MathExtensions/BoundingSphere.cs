using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysics.MathExtensions
{    
    /// <summary>
    /// Provides XNA-like bounding sphere functionality needed by the engine.
    /// </summary>
    public struct BoundingSphere
    {
        public float Radius;
        public Vector3 Center;

        public BoundingSphere(Vector3 center, float radius)
        {
            this.Center = center;
            this.Radius = radius;
        }
    }
}
