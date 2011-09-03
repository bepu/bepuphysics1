using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysics.MathExtensions
{
    /// <summary>
    /// Provides XNA-like 3D vector math needed by the engine.
    /// </summary>
    public struct Vector3
    {
        public float X;
        public float Y;
        public float Z;

        public Vector3(float x, float y, float z)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
        }

        public float LengthSquared()
        {
            return X * X + Y * Y + Z * Z;
        }

        public float Length()
        {
            return (float)Math.Sqrt(X * X + Y * Y + Z * Z);
        }

        public static float Dot(Vector3 a, Vector3 b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        public static void Dot(ref Vector3 a, ref Vector3 b, out float product)
        {
            product = a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        public static void Add(ref Vector3 a, ref Vector3 b, out Vector3 sum)
        {
            sum.X = a.X + b.X;
            sum.Y = a.Y + b.Y;
            sum.Z = a.Z + b.Z;
        }

        public static void Subtract(ref Vector3 a, ref Vector3 b, out Vector3 difference)
        {
            difference.X = a.X - b.X;
            difference.Y = a.Y - b.Y;
            difference.Z = a.Z - b.Z;
        }

        public static void Multiply(ref Vector3 v, float scale, out Vector3 result)
        {
            result.X = v.X * scale;
            result.Y = v.Y * scale;
            result.Z = v.Z * scale;
        }

        public static void Divide(ref Vector3 v, float divisor, out Vector3 result)
        {
            float inverse = 1 / divisor;
            result.X = v.X * inverse;
            result.Y = v.Y * inverse;
            result.Z = v.Z * inverse;
        }

        public static Vector3 operator *(Vector3 v, float f)
        {
            Vector3 toReturn;
            toReturn.X = v.X * f;
            toReturn.Y = v.Y * f;
            toReturn.Z = v.Z * f;
            return toReturn;
        }

        public static Vector3 operator *(float f, Vector3 v)
        {
            Vector3 toReturn;
            toReturn.X = v.X * f;
            toReturn.Y = v.Y * f;
            toReturn.Z = v.Z * f;
            return toReturn;
        }


        public static Vector3 operator /(Vector3 v, float f)
        {
            Vector3 toReturn;
            f = 1 / f;
            toReturn.X = v.X * f;
            toReturn.Y = v.Y * f;
            toReturn.Z = v.Z * f;
            return toReturn;
        }

        public static Vector3 operator -(Vector3 a, Vector3 b)
        {
            Vector3 v;
            v.X = a.X - b.X;
            v.Y = a.Y - b.Y;
            v.Z = a.Z - b.Z;
            return v;
        }

        public static Vector3 operator +(Vector3 a, Vector3 b)
        {
            Vector3 v;
            v.X = a.X + b.X;
            v.Y = a.Y + b.Y;
            v.Z = a.Z + b.Z;
            return v;
        }

        public static Vector3 operator -(Vector3 v)
        {
            v.X = -v.X;
            v.Y = -v.Y;
            v.Z = -v.Z;
            return v;
        }

        public static bool operator ==(Vector3 a, Vector3 b)
        {
            return a.X == b.X && a.Y == b.Y && a.Z == b.Z;
        }

        public static bool operator !=(Vector3 a, Vector3 b)
        {
            return a.X != b.X || a.Y != b.Y || a.Z != b.Z;
        }


        public static void DistanceSquared(ref Vector3 a, ref Vector3 b, out float distanceSquared)
        {
            float x = a.X - b.X;
            float y = a.Y - b.Y;
            float z = a.Z - b.Z;
            distanceSquared = x * x + y * y + z * z;
        }

        public static void Distance(ref Vector3 a, ref Vector3 b, out float distance)
        {
            float x = a.X - b.X;
            float y = a.Y - b.Y;
            float z = a.Z - b.Z;
            distance = (float)Math.Sqrt(x * x + y * y + z * z);
        }

        public static float Distance(Vector3 a, Vector3 b)
        {
            float toReturn;
            Distance(ref a, ref b, out toReturn);
            return toReturn;
        }


        public static Vector3 Zero
        {
            get
            {
                return new Vector3();
            }
        }

        public static Vector3 Up
        {
            get
            {
                return new Vector3()
                {
                    X = 0,
                    Y = 1,
                    Z = 0
                };
            }
        }

        public static Vector3 Down
        {
            get
            {
                return new Vector3()
                {
                    X = 0,
                    Y = -1,
                    Z = 0
                };
            }
        }

        public static Vector3 Right
        {
            get
            {
                return new Vector3()
                {
                    X = 1,
                    Y = 0,
                    Z = 0
                };
            }
        }

        public static Vector3 Left
        {
            get
            {
                return new Vector3()
                {
                    X = -1,
                    Y = 0,
                    Z = 0
                };
            }
        }

        public static Vector3 Forward
        {
            get
            {
                return new Vector3()
                {
                    X = 0,
                    Y = 0,
                    Z = -1
                };
            }
        }

        public static Vector3 Backward
        {
            get
            {
                return new Vector3()
                {
                    X = 0,
                    Y = 0,
                    Z = 1
                };
            }
        }

        public static Vector3 Cross(Vector3 a, Vector3 b)
        {
            Vector3 toReturn;
            Vector3.Cross(ref a, ref b, out toReturn);
            return toReturn;
        }

        public static void Cross(ref Vector3 a, ref Vector3 b, out Vector3 result)
        {
            result.X = a.Y * b.Z - a.Z * b.Y;
            result.Y = a.Z * b.X - a.X * b.Z;
            result.Z = a.X * b.Y - a.Y * b.X;
        }

        public static void TransformNormal(ref Vector3 v, ref Matrix matrix, out Vector3 result)
        {
            float vX = v.X;
            float vY = v.Y;
            float vZ = v.Z;
            result.X = vX * matrix.M11 + vY * matrix.M21 + vZ * matrix.M31;
            result.Y = vX * matrix.M12 + vY * matrix.M22 + vZ * matrix.M32;
            result.Z = vX * matrix.M13 + vY * matrix.M23 + vZ * matrix.M33;
        }

        public static void Transform(ref Vector3 v, ref Matrix matrix, out Vector3 result)
        {
            float vX = v.X;
            float vY = v.Y;
            float vZ = v.Z;
            result.X = vX * matrix.M11 + vY * matrix.M21 + vZ * matrix.M31 + matrix.M41;
            result.Y = vX * matrix.M12 + vY * matrix.M22 + vZ * matrix.M32 + matrix.M42;
            result.Z = vX * matrix.M13 + vY * matrix.M23 + vZ * matrix.M33 + matrix.M43;
        }

        public static Vector3 Transform(Vector3 v, Matrix matrix)
        {
            Vector3 toReturn;
            Transform(ref v, ref matrix, out toReturn);
            return toReturn;
        }

        public static void Transform(ref Vector3 v, ref Quaternion rotation, out Vector3 result)
        {
            //This operation is an optimized-down version of v' = q * v * q^-1.
            //The expanded form would be to treat v as an 'axis only' quaternion
            //and perform standard quaternion multiplication.  Assuming q is normalized,
            //q^-1 can be replaced by a conjugation.
            float x2 = rotation.X + rotation.X;
            float y2 = rotation.Y + rotation.Y;
            float z2 = rotation.Z + rotation.Z;
            float xx2 = rotation.X * x2;
            float xy2 = rotation.X * y2;
            float xz2 = rotation.X * z2;
            float yy2 = rotation.Y * y2;
            float yz2 = rotation.Y * z2;
            float zz2 = rotation.Z * z2;
            float wx2 = rotation.W * x2;
            float wy2 = rotation.W * y2;
            float wz2 = rotation.W * z2;
            //Defer the component setting since they're used in computation.
            float transformedX = v.X * ((1f - yy2) - zz2) + v.Y * (xy2 - wz2) + v.Z * (xz2 + wy2);
            float transformedY = v.X * (xy2 + wz2) + v.Y * ((1f - xx2) - zz2) + v.Z * (yz2 - wx2);
            float transformedZ = v.X * (xz2 - wy2) + v.Y * (yz2 + wx2) + v.Z * ((1f - xx2) - yy2);
            result.X = transformedX;
            result.Y = transformedY;
            result.Z = transformedZ;

        }

        public static Vector3 Transform(Vector3 v, Quaternion rotation)
        {
            Vector3 toReturn;
            Transform(ref v, ref rotation, out toReturn);
            return toReturn;
        }

        public static Vector3 Normalize(Vector3 v)
        {
            Vector3 toReturn;
            Vector3.Normalize(ref v, out toReturn);
            return toReturn;
        }

        public static void Normalize(ref Vector3 v, out Vector3 result)
        {
            float inverse = (float)(1 / Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z));
            result.X = v.X * inverse;
            result.Y = v.Y * inverse;
            result.Z = v.Z * inverse;
        }

        public static void Negate(ref Vector3 v, out Vector3 negated)
        {
            negated.X = -v.X;
            negated.Y = -v.Y;
            negated.Z = -v.Z;
        }

        public void Normalize()
        {
            float inverse = (float)(1 / Math.Sqrt(X * X + Y * Y + Z * Z));
            X *= inverse;
            Y *= inverse;
            Z *= inverse;
        }

        public static void Min(ref Vector3 a, ref Vector3 b, out Vector3 result)
        {
            result.X = a.X < b.X ? a.X : b.X;
            result.Y = a.Y < b.Y ? a.Y : b.Y;
            result.Z = a.Z < b.Z ? a.Z : b.Z;
        }

        public static void Max(ref Vector3 a, ref Vector3 b, out Vector3 result)
        {
            result.X = a.X > b.X ? a.X : b.X;
            result.Y = a.Y > b.Y ? a.Y : b.Y;
            result.Z = a.Z > b.Z ? a.Z : b.Z;
        }

        public static Vector3 Lerp(Vector3 start, Vector3 end, float interpolationAmount)
        {
            Vector3 toReturn;
            Lerp(ref start, ref end, interpolationAmount, out toReturn);
            return toReturn;
        }

        public static void Lerp(ref Vector3 start, ref Vector3 end, float interpolationAmount, out Vector3 result)
        {
            float startAmount = 1 - interpolationAmount;
            result.X = start.X * startAmount + end.X * interpolationAmount;
            result.Y = start.Y * startAmount + end.Y * interpolationAmount;
            result.Z = start.Z * startAmount + end.Z * interpolationAmount;
        }

        public static void Hermite(ref Vector3 value1, ref Vector3 tangent1, ref Vector3 value2, ref Vector3 tangent2, float interpolationAmount, out Vector3 result)
        {
            float weightSquared = interpolationAmount * interpolationAmount;
            float weightCubed = interpolationAmount * weightSquared;
            float value1Blend = 2 * weightCubed - 3 * weightSquared + 1;
            float tangent1Blend = weightCubed - 2 * weightSquared + interpolationAmount;
            float value2Blend = -2 * weightCubed + 3 * weightSquared;
            float tangent2Blend = weightCubed - weightSquared;
            result.X = value1.X * value1Blend + value2.X * value2Blend + tangent1.X * tangent1Blend + tangent2.X * tangent2Blend;
            result.Y = value1.Y * value1Blend + value2.Y * value2Blend + tangent1.Y * tangent1Blend + tangent2.Y * tangent2Blend;
            result.Z = value1.Z * value1Blend + value2.Z * value2Blend + tangent1.Z * tangent1Blend + tangent2.Z * tangent2Blend;
        }

        public static Vector3 Hermite(Vector3 value1, Vector3 tangent1, Vector3 value2, Vector3 tangent2, float interpolationAmount)
        {
            Vector3 toReturn;
            Hermite(ref value1, ref tangent1, ref value2, ref tangent2, interpolationAmount, out toReturn);
            return toReturn;
        }

    }
}
