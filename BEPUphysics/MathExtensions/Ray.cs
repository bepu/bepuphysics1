using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysics.MathExtensions
{
    /// <summary>
    /// Provides XNA-like ray functionality needed by the engine.
    /// </summary>
    public struct Ray
    {
        /// <summary>
        /// Starting position of the ray.
        /// </summary>
        public Vector3 Position;
        /// <summary>
        /// Direction in which the ray points.
        /// </summary>
        public Vector3 Direction;


        /// <summary>
        /// Constructs a new ray.
        /// </summary>
        /// <param name="position">Starting position of the ray.</param>
        /// <param name="direction">Direction in which the ray points.</param>
        public Ray(Vector3 position, Vector3 direction)
        {
            this.Position = position;
            this.Direction = direction;
        }

        /// <summary>
        /// Determines if and when the ray intersects the bounding box.
        /// </summary>
        /// <param name="boundingBox">Bounding box to test against.</param>
        /// <returns>The length along the ray to the impact, or null if no impact is found.</returns>
        public float? Intersects(BoundingBox boundingBox)
        {
            float? toReturn;
            Intersects(ref boundingBox, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Determines if and when the ray intersects the bounding box.
        /// </summary>
        /// <param name="boundingBox">Bounding box to test against.</param>
        /// <param name="result">The length along the ray to the impact, or null if no impact is found.</param>
        public void Intersects(ref BoundingBox boundingBox, out float? result)
        {
            if (Math.Abs(Direction.X) < Toolbox.Epsilon && (Position.X < boundingBox.Min.X || Position.X > boundingBox.Max.X))
            {
                //If the ray isn't pointing along the axis at all, and is outside of the box's interval, then it
                //can't be intersecting.
                result = null;
                return;
            }
            float tmin = 0, tmax = float.MaxValue;
            float inverseDirection = 1 / Direction.X;
            float t1 = (boundingBox.Min.X - Position.X) * inverseDirection;
            float t2 = (boundingBox.Max.X - Position.X) * inverseDirection;
            if (t1 > t2)
            {
                float temp = t1;
                t1 = t2;
                t2 = temp;
            }
            tmin = Math.Max(tmin, t1);
            tmax = Math.Min(tmax, t2);
            if (tmin > tmax)
            {
                result = null;
                return;
            }
            if (Math.Abs(Direction.Y) < Toolbox.Epsilon && (Position.Y < boundingBox.Min.Y || Position.Y > boundingBox.Max.Y))
            {                
                //If the ray isn't pointing along the axis at all, and is outside of the box's interval, then it
                //can't be intersecting.
                result = null; 
                return;
            }
            inverseDirection = 1 / Direction.Y;
            t1 = (boundingBox.Min.Y - Position.Y) * inverseDirection;
            t2 = (boundingBox.Max.Y - Position.Y) * inverseDirection;
            if (t1 > t2)
            {
                float temp = t1;
                t1 = t2;
                t2 = temp;
            }
            tmin = Math.Max(tmin, t1);
            tmax = Math.Min(tmax, t2);
            if (tmin > tmax)
            {
                result = null; 
                return;
            }
            if (Math.Abs(Direction.Z) < Toolbox.Epsilon && (Position.Z < -boundingBox.Min.Z || Position.Z > boundingBox.Max.Z))
            {              
                //If the ray isn't pointing along the axis at all, and is outside of the box's interval, then it
                //can't be intersecting.
                result = null; 
                return;
            }
            inverseDirection = 1 / Direction.Z;
            t1 = (boundingBox.Min.Z - Position.Z) * inverseDirection;
            t2 = (boundingBox.Max.Z - Position.Z) * inverseDirection;
            if (t1 > t2)
            {
                float temp = t1;
                t1 = t2;
                t2 = temp;
            }
            tmin = Math.Max(tmin, t1);
            tmax = Math.Min(tmax, t2);
            if (tmin > tmax)
            {
                result = null; 
                return;
            }
            result = tmin;
        }
    }
}
