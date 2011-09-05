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
        /// <summary>
        /// X component of the vector.
        /// </summary>
        public float X;
        /// <summary>
        /// Y component of the vector.
        /// </summary>
        public float Y;

        /// <summary>
        /// Constructs a new two dimensional vector.
        /// </summary>
        /// <param name="x">X component of the vector.</param>
        /// <param name="y">Y component of the vector.</param>
        public Vector2(float x, float y)
        {
            this.X = x;
            this.Y = y;
        }

        /// <summary>
        /// Computes the squared length of the vector.
        /// </summary>
        /// <returns>Squared length of the vector.</returns>
        public float LengthSquared()
        {
            return X * X + Y * Y;
        }

        /// <summary>
        /// Computes the length of the vector.
        /// </summary>
        /// <returns>Length of the vector.</returns>
        public float Length()
        {
            return (float)Math.Sqrt(X * X + Y * Y);
        }

        /// <summary>
        /// Gets a string representation of the vector.
        /// </summary>
        /// <returns>String representing the vector.</returns>
        public override string ToString()
        {
            return "{" + X + ", " + Y + "}";
        }

        /// <summary>
        /// Adds two vectors together.
        /// </summary>
        /// <param name="a">First vector to add.</param>
        /// <param name="b">Second vector to add.</param>
        /// <param name="sum">Sum of the two vectors.</param>
        public static void Add(ref Vector2 a, ref Vector2 b, out Vector2 sum)
        {
            sum.X = a.X + b.X;
            sum.Y = a.Y + b.Y;
        }

        /// <summary>
        /// Subtracts two vectors.
        /// </summary>
        /// <param name="a">Vector to subtract from.</param>
        /// <param name="b">Vector to subtract from the first vector.</param>
        /// <param name="difference">Result of the subtraction.</param>
        public static void Subtract(ref Vector2 a, ref Vector2 b, out Vector2 difference)
        {
            difference.X = a.X - b.X;
            difference.Y = a.Y - b.Y;
        }

        /// <summary>
        /// Scales a vector.
        /// </summary>
        /// <param name="v">Vector to scale.</param>
        /// <param name="scale">Amount to scale.</param>
        /// <param name="result">Scaled vector.</param>
        public static void Multiply(ref Vector2 v, float scale, out Vector2 result)
        {
            result.X = v.X * scale;
            result.Y = v.Y * scale;
        }

        /// <summary>
        /// Divides a vector's components by some amount.
        /// </summary>
        /// <param name="v">Vector to divide.</param>
        /// <param name="divisor">Value to divide the vector's components.</param>
        /// <param name="result">Result of the division.</param>
        public static void Divide(ref Vector2 v, float divisor, out Vector2 result)
        {
            float inverse = 1 / divisor;
            result.X = v.X * inverse;
            result.Y = v.Y * inverse;
        }

        /// <summary>
        /// Gets the zero vector.
        /// </summary>
        public static Vector2 Zero
        {
            get
            {
                return new Vector2();
            }
        }


        /// <summary>
        /// Normalizes the vector.
        /// </summary>
        /// <param name="v">Vector to normalize.</param>
        /// <returns>Normalized copy of the vector.</returns>
        public static Vector2 Normalize(Vector2 v)
        {
            Vector2 toReturn;
            Vector2.Normalize(ref v, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Normalizes the vector.
        /// </summary>
        /// <param name="v">Vector to normalize.</param>
        /// <param name="result">Normalized vector.</param>
        public static void Normalize(ref Vector2 v, out Vector2 result)
        {
            float inverse = (float)(1 / Math.Sqrt(v.X * v.X + v.Y * v.Y));
            result.X = v.X * inverse;
            result.Y = v.Y * inverse;
        }

        /// <summary>
        /// Negates the vector.
        /// </summary>
        /// <param name="v">Vector to negate.</param>
        /// <param name="negated">Negated version of the vector.</param>
        public static void Negate(ref Vector2 v, out Vector2 negated)
        {
            negated.X = -v.X;
            negated.Y = -v.Y;
        }

        /// <summary>
        /// Normalizes the vector.
        /// </summary>
        public void Normalize()
        {
            float inverse = (float)(1 / Math.Sqrt(X * X + Y * Y));
            X *= inverse;
            Y *= inverse;
        }

        /// <summary>
        /// Scales a vector.
        /// </summary>
        /// <param name="v">Vector to scale.</param>
        /// <param name="f">Amount to scale.</param>
        /// <returns>Scaled vector.</returns>
        public static Vector2 operator *(Vector2 v, float f)
        {
            Vector2 toReturn;
            toReturn.X = v.X * f;
            toReturn.Y = v.Y * f;
            return toReturn;
        }
        /// <summary>
        /// Scales a vector.
        /// </summary>
        /// <param name="v">Vector to scale.</param>
        /// <param name="f">Amount to scale.</param>
        /// <returns>Scaled vector.</returns>
        public static Vector2 operator *(float f, Vector2 v)
        {
            Vector2 toReturn;
            toReturn.X = v.X * f;
            toReturn.Y = v.Y * f;
            return toReturn;
        }

        /// <summary>
        /// Divides a vector.
        /// </summary>
        /// <param name="v">Vector to divide.</param>
        /// <param name="f">Amount to divide.</param>
        /// <returns>Divided vector.</returns>
        public static Vector2 operator /(Vector2 v, float f)
        {
            Vector2 toReturn;
            f = 1 / f;
            toReturn.X = v.X * f;
            toReturn.Y = v.Y * f;
            return toReturn;
        }

        /// <summary>
        /// Subtracts two vectors.
        /// </summary>
        /// <param name="a">Vector to be subtracted from.</param>
        /// <param name="b">Vector to subtract from the first vector.</param>
        /// <returns>Resulting difference.</returns>
        public static Vector2 operator -(Vector2 a, Vector2 b)
        {
            Vector2 v;
            v.X = a.X - b.X;
            v.Y = a.Y - b.Y;
            return v;
        }

        /// <summary>
        /// Adds two vectors.
        /// </summary>
        /// <param name="a">First vector to add.</param>
        /// <param name="b">Second vector to add.</param>
        /// <returns>Sum of the addition.</returns>
        public static Vector2 operator +(Vector2 a, Vector2 b)
        {
            Vector2 v;
            v.X = a.X + b.X;
            v.Y = a.Y + b.Y;
            return v;
        }

        /// <summary>
        /// Negates the vector.
        /// </summary>
        /// <param name="v">Vector to negate.</param>
        /// <returns>Negated vector.</returns>
        public static Vector2 operator -(Vector2 v)
        {
            v.X = -v.X;
            v.Y = -v.Y;
            return v;
        }

        /// <summary>
        /// Tests two vectors for componentwise equivalence.
        /// </summary>
        /// <param name="a">First vector to test for equivalence.</param>
        /// <param name="b">Second vector to test for equivalence.</param>
        /// <returns>Whether the vectors were equivalent.</returns>
        public static bool operator ==(Vector2 a, Vector2 b)
        {
            return a.X == b.X && a.Y == b.Y;
        }
        /// <summary>
        /// Tests two vectors for componentwise inequivalence.
        /// </summary>
        /// <param name="a">First vector to test for inequivalence.</param>
        /// <param name="b">Second vector to test for inequivalence.</param>
        /// <returns>Whether the vectors were inequivalent.</returns>
        public static bool operator !=(Vector2 a, Vector2 b)
        {
            return a.X != b.X || a.Y != b.Y;
        }
    }
}
