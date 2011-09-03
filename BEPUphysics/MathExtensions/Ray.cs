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
        public Vector3 Position;
        public Vector3 Direction;

        public Ray(Vector3 position, Vector3 direction)
        {
            this.Position = position;
            this.Direction = direction;
        }

        public float? Intersects(BoundingBox boundingBox)
        {
            float? toReturn;
            Intersects(ref boundingBox, out toReturn);
            return toReturn;
        }

        public void Intersects(ref BoundingBox boundingBox, out float? result)
        {
            float halfWidth = boundingBox.Max.X - boundingBox.Min.X;
            float halfHeight = boundingBox.Max.Y - boundingBox.Min.Y;
            float halfLength = boundingBox.Max.Z - boundingBox.Min.Z;
            if (Math.Abs(Direction.X) < Toolbox.Epsilon && (Position.X < -halfWidth || Position.X > halfWidth))
                result = null;
            float temp, tmin = 0, tmax = float.MaxValue;
            float inverseDirection = 1 / Direction.X;
            float t1 = (-halfWidth - Position.X) * inverseDirection;
            float t2 = (halfWidth - Position.X) * inverseDirection;
            var tempNormal = new Vector3(-1, 0, 0);
            if (t1 > t2)
            {
                temp = t1;
                t1 = t2;
                t2 = temp;
                tempNormal *= -1;
            }
            temp = tmin;
            tmin = Math.Max(tmin, t1);
            tmax = Math.Min(tmax, t2);
            if (tmin > tmax)
                result = null;
            if (Math.Abs(Direction.Y) < Toolbox.Epsilon && (Position.Y < -halfHeight || Position.Y > halfHeight))
                result = null;
            inverseDirection = 1 / Direction.Y;
            t1 = (-halfHeight - Position.Y) * inverseDirection;
            t2 = (halfHeight - Position.Y) * inverseDirection;
            tempNormal = new Vector3(0, -1, 0);
            if (t1 > t2)
            {
                temp = t1;
                t1 = t2;
                t2 = temp;
                tempNormal *= -1;
            }
            temp = tmin;
            tmin = Math.Max(tmin, t1);
            tmax = Math.Min(tmax, t2);
            if (tmin > tmax)
                result = null;
            if (Math.Abs(Direction.Z) < Toolbox.Epsilon && (Position.Z < -halfLength || Position.Z > halfLength))
                result = null; 
            inverseDirection = 1 / Direction.Z;
            t1 = (-halfLength - Position.Z) * inverseDirection;
            t2 = (halfLength - Position.Z) * inverseDirection;
            tempNormal = new Vector3(0, 0, -1);
            if (t1 > t2)
            {
                temp = t1;
                t1 = t2;
                t2 = temp;
                tempNormal *= -1;
            }
            temp = tmin;
            tmin = Math.Max(tmin, t1);
            tmax = Math.Min(tmax, t2);
            if (tmin > tmax)
                result = null;
            result = tmin;
        }
    }
}
