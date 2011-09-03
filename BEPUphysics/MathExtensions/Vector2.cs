using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysics.MathExtensions
{
    /// <summary>
    /// Provides XNA-like 2D vector math needed by the engine.
    /// </summary>
    public struct Vector2
    {
        public float X;
        public float Y;

        public Vector2(float x, float y)
        {
            this.X = x;
            this.Y = y;
        }

        public float LengthSquared()
        {
            return X * X + Y * Y;
        }

        public float Length()
        {
            return (float)Math.Sqrt(X * X + Y * Y);
        }


        public static void Add(ref Vector2 a, ref Vector2 b, out Vector2 sum)
        {
            sum.X = a.X + b.X;
            sum.Y = a.Y + b.Y;
        }

        public static void Subtract(ref Vector2 a, ref Vector2 b, out Vector2 difference)
        {
            difference.X = a.X - b.X;
            difference.Y = a.Y - b.Y;
        }

        public static void Multiply(ref Vector2 v, float scale, out Vector2 result)
        {
            result.X = v.X * scale;
            result.Y = v.Y * scale;
        }

        public static void Divide(ref Vector2 v, float divisor, out Vector2 result)
        {
            float inverse = 1 / divisor;
            result.X = v.X * inverse;
            result.Y = v.Y * inverse;
        }

        public static Vector2 Zero
        {
            get
            {
                return new Vector2();
            }
        }

        public static Vector2 Normalize(Vector2 v)
        {
            Vector2 toReturn;
            Vector2.Normalize(ref v, out toReturn);
            return toReturn;
        }

        public static void Normalize(ref Vector2 v, out Vector2 result)
        {
            float inverse = (float)(1 / Math.Sqrt(v.X * v.X + v.Y * v.Y));
            result.X = v.X * inverse;
            result.Y = v.Y * inverse;
        }

        public static void Negate(ref Vector2 v, out Vector2 negated)
        {
            negated.X = -v.X;
            negated.Y = -v.Y;
        }

        public void Normalize()
        {
            float inverse = (float)(1 / Math.Sqrt(X * X + Y * Y));
            X *= inverse;
            Y *= inverse;
        }
    }
}
