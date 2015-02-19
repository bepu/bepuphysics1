using System;

namespace BEPUutilities
{
    /// <summary>
    /// Provides XNA-like 2D vector math.
    /// </summary>
    public struct Vector2 : IEquatable<Vector2>
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
            return (float)System.Math.Sqrt(X * X + Y * Y);
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
        /// Multiplies two vectors on a per-component basis.
        /// </summary>
        /// <param name="a">First vector to multiply.</param>
        /// <param name="b">Second vector to multiply.</param>
        /// <param name="result">Result of the componentwise multiplication.</param>
        public static void Multiply(ref Vector2 a, ref Vector2 b, out Vector2 result)
        {
            result.X = a.X * b.X;
            result.Y = a.Y * b.Y;
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
        /// Computes the dot product of the two vectors.
        /// </summary>
        /// <param name="a">First vector of the dot product.</param>
        /// <param name="b">Second vector of the dot product.</param>
        /// <param name="dot">Dot product of the two vectors.</param>
        public static void Dot(ref Vector2 a, ref Vector2 b, out float dot)
        {
            dot = a.X * b.X + a.Y * b.Y;
        }

        /// <summary>
        /// Computes the dot product of the two vectors.
        /// </summary>
        /// <param name="a">First vector of the dot product.</param>
        /// <param name="b">Second vector of the dot product.</param>
        /// <returns>Dot product of the two vectors.</returns>
        public static float Dot(Vector2 a, Vector2 b)
        {
            return a.X * b.X + a.Y * b.Y;
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
        /// Gets a vector pointing along the X axis.
        /// </summary>
        public static Vector2 UnitX
        {
            get { return new Vector2 { X = 1 }; }
        }

        /// <summary>
        /// Gets a vector pointing along the Y axis.
        /// </summary>
        public static Vector2 UnitY
        {
            get { return new Vector2 { Y = 1 }; }
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
            float inverse = (float)(1 / System.Math.Sqrt(v.X * v.X + v.Y * v.Y));
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
        /// Computes the absolute value of the input vector.
        /// </summary>
        /// <param name="v">Vector to take the absolute value of.</param>
        /// <param name="result">Vector with nonnegative elements.</param>
        public static void Abs(ref Vector2 v, out Vector2 result)
        {
            if (v.X < 0)
                result.X = -v.X;
            else
                result.X = v.X;
            if (v.Y < 0)
                result.Y = -v.Y;
            else
                result.Y = v.Y;
        }

        /// <summary>
        /// Computes the absolute value of the input vector.
        /// </summary>
        /// <param name="v">Vector to take the absolute value of.</param>
        /// <returns>Vector with nonnegative elements.</returns>
        public static Vector2 Abs(Vector2 v)
        {
            Vector2 result;
            Abs(ref v, out result);
            return result;
        }

        /// <summary>
        /// Creates a vector from the lesser values in each vector.
        /// </summary>
        /// <param name="a">First input vector to compare values from.</param>
        /// <param name="b">Second input vector to compare values from.</param>
        /// <param name="min">Vector containing the lesser values of each vector.</param>
        public static void Min(ref Vector2 a, ref Vector2 b, out Vector2 min)
        {
            min.X = a.X < b.X ? a.X : b.X;
            min.Y = a.Y < b.Y ? a.Y : b.Y;
        }

        /// <summary>
        /// Creates a vector from the lesser values in each vector.
        /// </summary>
        /// <param name="a">First input vector to compare values from.</param>
        /// <param name="b">Second input vector to compare values from.</param>
        /// <returns>Vector containing the lesser values of each vector.</returns>
        public static Vector2 Min(Vector2 a, Vector2 b)
        {
            Vector2 result;
            Min(ref a, ref b, out result);
            return result;
        }


        /// <summary>
        /// Creates a vector from the greater values in each vector.
        /// </summary>
        /// <param name="a">First input vector to compare values from.</param>
        /// <param name="b">Second input vector to compare values from.</param>
        /// <param name="max">Vector containing the greater values of each vector.</param>
        public static void Max(ref Vector2 a, ref Vector2 b, out Vector2 max)
        {
            max.X = a.X > b.X ? a.X : b.X;
            max.Y = a.Y > b.Y ? a.Y : b.Y;
        }

        /// <summary>
        /// Creates a vector from the greater values in each vector.
        /// </summary>
        /// <param name="a">First input vector to compare values from.</param>
        /// <param name="b">Second input vector to compare values from.</param>
        /// <returns>Vector containing the greater values of each vector.</returns>
        public static Vector2 Max(Vector2 a, Vector2 b)
        {
            Vector2 result;
            Max(ref a, ref b, out result);
            return result;
        }

        /// <summary>
        /// Normalizes the vector.
        /// </summary>
        public void Normalize()
        {
            float inverse = (float)(1 / System.Math.Sqrt(X * X + Y * Y));
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
        /// Multiplies two vectors on a per-component basis.
        /// </summary>
        /// <param name="a">First vector to multiply.</param>
        /// <param name="b">Second vector to multiply.</param>
        /// <returns>Result of the componentwise multiplication.</returns>
        public static Vector2 operator *(Vector2 a, Vector2 b)
        {
            Vector2 result;
            Multiply(ref a, ref b, out result);
            return result;
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

        /// <summary>
        /// Indicates whether the current object is equal to another object of the same type.
        /// </summary>
        /// <returns>
        /// true if the current object is equal to the <paramref name="other"/> parameter; otherwise, false.
        /// </returns>
        /// <param name="other">An object to compare with this object.</param>
        public bool Equals(Vector2 other)
        {
            return X == other.X && Y == other.Y;
        }

        /// <summary>
        /// Indicates whether this instance and a specified object are equal.
        /// </summary>
        /// <returns>
        /// true if <paramref name="obj"/> and this instance are the same type and represent the same value; otherwise, false.
        /// </returns>
        /// <param name="obj">Another object to compare to. </param><filterpriority>2</filterpriority>
        public override bool Equals(object obj)
        {
            if (obj is Vector2)
            {
                return Equals((Vector2)obj);
            }
            return false;
        }

        /// <summary>
        /// Returns the hash code for this instance.
        /// </summary>
        /// <returns>
        /// A 32-bit signed integer that is the hash code for this instance.
        /// </returns>
        /// <filterpriority>2</filterpriority>
        public override int GetHashCode()
        {
            return X.GetHashCode() + Y.GetHashCode();
        }


    }
}
