using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysics.MathExtensions
{
    /// <summary>
    /// Provides XNA-like quaternion support needed by the engine.
    /// </summary>
    public struct Quaternion
    {
        /// <summary>
        /// X component of the quaternion.
        /// </summary>
        public float X;

        /// <summary>
        /// Y component of the quaternion.
        /// </summary>
        public float Y;

        /// <summary>
        /// Z component of the quaternion.
        /// </summary>
        public float Z;

        /// <summary>
        /// W component of the quaternion.
        /// </summary>
        public float W;

        /// <summary>
        /// Constructs a new Quaternion.
        /// </summary>
        /// <param name="x">X component of the quaternion.</param>
        /// <param name="y">Y component of the quaternion.</param>
        /// <param name="z">Z component of the quaternion.</param>
        /// <param name="w">W component of the quaternion.</param>
        public Quaternion(float x, float y, float z, float w)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
            this.W = w;
        }

        /// <summary>
        /// Adds two quaternions together.
        /// </summary>
        /// <param name="a">First quaternion to add.</param>
        /// <param name="b">Second quaternion to add.</param>
        /// <param name="result">Sum of the addition.</param>
        public static void Add(ref Quaternion a, ref Quaternion b, out Quaternion result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
            result.W = a.W + b.W;
        }

        /// <summary>
        /// Multiplies two quaternions.
        /// </summary>
        /// <param name="a">First quaternion to multiply.</param>
        /// <param name="b">Second quaternion to multiply.</param>
        /// <param name="result">Product of the multiplication.</param>
        public static void Multiply(ref Quaternion a, ref Quaternion b, out Quaternion result)
        {
            float x = a.X;
            float y = a.Y;
            float z = a.Z;
            float w = a.W;
            float bX = b.X;
            float bY = b.Y;
            float bZ = b.Z;
            float bW = b.W;
            result.X = x * bW + bX * w + y * bZ - z * bY;
            result.Y = y * bW + bY * w + z * bX - x * bZ;
            result.Z = z * bW + bZ * w + x * bY - y * bX;
            result.W = w * bW - x * bX - y * bY - z * bZ;
        }

        /// <summary>
        /// Scales a quaternion.
        /// </summary>
        /// <param name="q">Quaternion to multiply.</param>
        /// <param name="scale">Amount to multiply each component of the quaternion by.</param>
        /// <param name="result">Scaled quaternion.</param>
        public static void Multiply(ref Quaternion q, float scale, out Quaternion result)
        {
            result.X = q.X * scale;
            result.Y = q.Y * scale;
            result.Z = q.Z * scale;
            result.W = q.W * scale;
        }

        /// <summary>
        /// Multiplies two quaternions together in opposite order.
        /// </summary>
        /// <param name="a">First quaternion to multiply.</param>
        /// <param name="b">Second quaternion to multiply.</param>
        /// <param name="result">Product of the multiplication.</param>
        public static void Concatenate(ref Quaternion a, ref Quaternion b, out Quaternion result)
        {
            float aX = a.X;
            float aY = a.Y;
            float aZ = a.Z;
            float aW = a.W;
            float x = b.X;
            float y = b.Y;
            float z = b.Z;
            float w = b.W;
            result.X = x * aW + aX * w + y * aZ - z * aY;
            result.Y = y * aW + aY * w + z * aX - x * aZ;
            result.Z = z * aW + aZ * w + x * aY - y * aX;
            result.W = w * aW - x * aX - y * aY - z * aZ;
        }

        /// <summary>
        /// Quaternion representing the identity transform.
        /// </summary>
        public static Quaternion Identity
        {
            get
            {
                return new Quaternion(0, 0, 0, 1);
            }
        }


        /// <summary>
        /// Creates a quaternion from a rotation matrix.
        /// </summary>
        /// <param name="r">Rotation matrix used to create a new quaternion.</param>
        /// <returns>Quaternion representing the same rotation as the matrix.</returns>
        public static Quaternion CreateFromRotationMatrix(Matrix r)
        {
            Quaternion toReturn;
            CreateFromRotationMatrix(ref r, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Creates a quaternion from a rotation matrix.
        /// </summary>
        /// <param name="r">Rotation matrix used to create a new quaternion.</param>
        /// <param name="q">Quaternion representing the same rotation as the matrix.</param>
        public static void CreateFromRotationMatrix(ref Matrix r, out Quaternion q)
        {
            float tr = r.M11 + r.M22 + r.M33;
            if (tr > 0)
            {
                var S = (float)Math.Sqrt(tr + 1.0) * 2; // S=4*qw 
                var inverseS = 1 / S;
                q.W = 0.25f * S;
                q.X = (r.M32 - r.M23) * inverseS;
                q.Y = (r.M13 - r.M31) * inverseS;
                q.Z = (r.M21 - r.M12) * inverseS;
            }
            else if ((r.M11 > r.M22) & (r.M11 > r.M33))
            {
                var S = (float)Math.Sqrt(1.0 + r.M11 - r.M22 - r.M33) * 2; // S=4*qx 
                var inverseS = 1 / S;
                q.W = (r.M32 - r.M23) * inverseS;
                q.X = 0.25f * S;
                q.Y = (r.M12 + r.M21) * inverseS;
                q.Z = (r.M13 + r.M31) * inverseS;
            }
            else if (r.M22 > r.M33)
            {
                var S = (float)Math.Sqrt(1.0 + r.M22 - r.M11 - r.M33) * 2; // S=4*qy
                var inverseS = 1 / S;
                q.W = (r.M13 - r.M31) * inverseS;
                q.X = (r.M12 + r.M21) * inverseS;
                q.Y = 0.25f * S;
                q.Z = (r.M23 + r.M32) * inverseS;
            }
            else
            {
                var S = (float)Math.Sqrt(1.0 + r.M33 - r.M11 - r.M22) * 2; // S=4*qz
                var inverseS = 1 / S;
                q.W = (r.M21 - r.M12) * inverseS;
                q.X = (r.M13 + r.M31) * inverseS;
                q.Y = (r.M23 + r.M32) * inverseS;
                q.Z = 0.25f * S;
            }
        }

        /// <summary>
        /// Ensures the quaternion has unit length.
        /// </summary>
        /// <param name="quaternion">Quaternion to normalize.</param>
        /// <returns>Normalized quaternion.</returns>
        public static Quaternion Normalize(Quaternion quaternion)
        {
            Quaternion toReturn;
            Quaternion.Normalize(ref quaternion, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Ensures the quaternion has unit length.
        /// </summary>
        /// <param name="quaternion">Quaternion to normalize.</param>
        /// <param name="toReturn">Normalized quaternion.</param>
        public static void Normalize(ref Quaternion quaternion, out Quaternion toReturn)
        {
            float inverse = (float)(1 / Math.Sqrt(quaternion.X * quaternion.X + quaternion.Y * quaternion.Y + quaternion.Z * quaternion.Z + quaternion.W * quaternion.W));
            toReturn.X = quaternion.X * inverse;
            toReturn.Y = quaternion.Y * inverse;
            toReturn.Z = quaternion.Z * inverse;
            toReturn.W = quaternion.W * inverse;
        }

        /// <summary>
        /// Scales the quaternion such that it has unit length.
        /// </summary>
        public void Normalize()
        {
            float inverse = (float)(1 / Math.Sqrt(X * X + Y * Y + Z * Z + W * W));
            X *= inverse;
            Y *= inverse;
            Z *= inverse;
            W *= inverse;
        }

        /// <summary>
        /// Blends two quaternions together to get an intermediate state.
        /// </summary>
        /// <param name="start">Starting point of the interpolation.</param>
        /// <param name="end">Ending point of the interpolation.</param>
        /// <param name="interpolationAmount">Amount of the end point to use.</param>
        /// <returns>Interpolated intermediate quaternion.</returns>
        public static Quaternion Slerp(Quaternion start, Quaternion end, float interpolationAmount)
        {
            Quaternion toReturn;
            Slerp(ref start, ref end, interpolationAmount, out toReturn);
            return toReturn;
        }
        /// <summary>
        /// Blends two quaternions together to get an intermediate state.
        /// </summary>
        /// <param name="start">Starting point of the interpolation.</param>
        /// <param name="end">Ending point of the interpolation.</param>
        /// <param name="interpolationAmount">Amount of the end point to use.</param>
        /// <param name="result">Interpolated intermediate quaternion.</param>
        public static void Slerp(ref Quaternion start, ref Quaternion end, float interpolationAmount, out Quaternion result)
        {
            double cosHalfTheta = start.W * end.W + start.X * end.X + start.Y * end.Y + start.Z * end.Z;
            if (cosHalfTheta < 0)
            {
                //Negating a quaternion results in the same orientation, 
                //but we need cosHalfTheta to be positive to get the shortest path.
                end.X = -end.X;
                end.Y = -end.Y;
                end.Z = -end.Z;
                end.W = -end.W;
                cosHalfTheta = -cosHalfTheta;
            }
            // If the orientations are similar enough, then just pick one of the inputs.
            if (cosHalfTheta > .999999)
            {
                result.W = start.W;
                result.X = start.X;
                result.Y = start.Y;
                result.Z = start.Z;
                return;
            }
            // Calculate temporary values.
            double halfTheta = Math.Acos(cosHalfTheta);
            double sinHalfTheta = Math.Sqrt(1.0 - cosHalfTheta * cosHalfTheta);
            //Check to see if we're 180 degrees away from the target.
            if (Math.Abs(sinHalfTheta) < 0.00001)
            {
                //Woops! There are an infinite number of ways to get to the goal.
                //Pick one.
                result.X = (start.X + end.X) * .5f;
                result.Y = (start.Y + end.Y) * .5f;
                result.Z = (start.Z + end.Z) * .5f;
                result.W = (start.W + end.W) * .5f;
                return;
            }
            double aFraction = Math.Sin((1 - interpolationAmount) * halfTheta) / sinHalfTheta;
            double bFraction = Math.Sin(interpolationAmount * halfTheta) / sinHalfTheta;

            //Blend the two quaternions to get the result!
            result.X = (float)(start.X * aFraction + end.X * bFraction);
            result.Y = (float)(start.Y * aFraction + end.Y * bFraction);
            result.Z = (float)(start.Z * aFraction + end.Z * bFraction);
            result.W = (float)(start.W * aFraction + end.W * bFraction);




        }

        /// <summary>
        /// Computes the conjugate of the quaternion.
        /// </summary>
        /// <param name="quaternion">Quaternion to conjugate.</param>
        /// <returns>Conjugated quaternion.</returns>
        public static Quaternion Conjugate(Quaternion quaternion)
        {
            Quaternion toReturn;
            Conjugate(ref quaternion, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Computes the conjugate of the quaternion.
        /// </summary>
        /// <param name="quaternion">Quaternion to conjugate.</param>
        /// <param name="result">Conjugated quaternion.</param>
        public static void Conjugate(ref Quaternion quaternion, out Quaternion result)
        {
            result.X = -quaternion.X;
            result.Y = -quaternion.Y;
            result.Z = -quaternion.Z;
            result.W = quaternion.W;
        }

        /// <summary>
        /// Tests components for equality.
        /// </summary>
        /// <param name="a">First quaternion to test for equivalence.</param>
        /// <param name="b">Second quaternion to test for equivalence.</param>
        /// <returns>Whether or not the quaternions' components were equal.</returns>
        public static bool operator ==(Quaternion a, Quaternion b)
        {
            return a.X == b.X && a.Y == b.Y && a.Z == b.Z && a.W == b.W;
        }

        /// <summary>
        /// Tests components for inequality.
        /// </summary>
        /// <param name="a">First quaternion to test for equivalence.</param>
        /// <param name="b">Second quaternion to test for equivalence.</param>
        /// <returns>Whether the quaternions' components were not equal.</returns>
        public static bool operator !=(Quaternion a, Quaternion b)
        {
            return a.X != b.X || a.Y != b.Y || a.Z != b.Z || a.W != b.W;
        }

        /// <summary>
        /// Multiplies two quaternions.
        /// </summary>
        /// <param name="a">First quaternion to multiply.</param>
        /// <param name="b">Second quaternion to multiply.</param>
        /// <returns>Product of the multiplication.</returns>
        public static Quaternion operator *(Quaternion a, Quaternion b)
        {
            Quaternion toReturn;
            Multiply(ref a, ref b, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Creates a quaternion from an axis and angle.
        /// </summary>
        /// <param name="axis">Axis of rotation.</param>
        /// <param name="angle">Angle to rotate around the axis.</param>
        /// <returns>Quaternion representing the axis and angle rotation.</returns>
        public static Quaternion CreateFromAxisAngle(Vector3 axis, float angle)
        {
            float halfAngle = angle * .5f;
            float s = (float)Math.Sin(halfAngle);
            Quaternion q;
            q.X = axis.X * s;
            q.Y = axis.Y * s;
            q.Z = axis.Z * s;
            q.W = (float)Math.Cos(halfAngle);
            return q;
        }

        /// <summary>
        /// Creates a quaternion from an axis and angle.
        /// </summary>
        /// <param name="axis">Axis of rotation.</param>
        /// <param name="angle">Angle to rotate around the axis.</param>
        /// <param name="p">Quaternion representing the axis and angle rotation.</param>
        public static void CreateFromAxisAngle(ref Vector3 axis, float angle, out Quaternion q)
        {
            float halfAngle = angle * .5f;
            float s = (float)Math.Sin(halfAngle);
            q.X = axis.X * s;
            q.Y = axis.Y * s;
            q.Z = axis.Z * s;
            q.W = (float)Math.Cos(halfAngle);
        }

        /// <summary>
        /// Constructs a quaternion from yaw, pitch, and roll.
        /// </summary>
        /// <param name="yaw">Yaw of the rotation.</param>
        /// <param name="pitch">Pitch of the rotation.</param>
        /// <param name="roll">Roll of the rotation.</param>
        /// <returns>Quaternion representing the yaw, pitch, and roll.</returns>
        public static Quaternion CreateFromYawPitchRoll(float yaw, float pitch, float roll)
        {
            Quaternion toReturn;
            CreateFromYawPitchRoll(yaw, pitch, roll, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Constructs a quaternion from yaw, pitch, and roll.
        /// </summary>
        /// <param name="yaw">Yaw of the rotation.</param>
        /// <param name="pitch">Pitch of the rotation.</param>
        /// <param name="roll">Roll of the rotation.</param>
        /// <param name="q">Quaternion representing the yaw, pitch, and roll.</param>
        public static void CreateFromYawPitchRoll(float yaw, float pitch, float roll, out Quaternion q)
        {
            double cosYaw = Math.Cos(yaw * .5f);
            double cosPitch = Math.Cos(pitch * .5f);
            double cosRoll = Math.Cos(roll * .5f);

            double sinYaw = Math.Sin(yaw * .5f);
            double sinPitch = Math.Sin(pitch * .5f);
            double sinRoll = Math.Sin(roll * .5f);

            double cosYawCosPitch = cosYaw * cosPitch;
            double cosYawSinPitch = cosYaw * sinPitch;
            double sinYawCosPitch = sinYaw * cosPitch;
            double sinYawSinPitch = sinYaw * sinPitch;

            q.W = (float)(cosYawCosPitch * cosRoll + sinYawSinPitch * sinRoll);
            q.X = (float)(sinYawCosPitch * cosRoll - cosYawSinPitch * sinRoll);
            q.Y = (float)(cosYawSinPitch * cosRoll + sinYawCosPitch * sinRoll);
            q.Z = (float)(cosYawCosPitch * sinRoll - sinYawSinPitch * cosRoll);
        }
    }
}
