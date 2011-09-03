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
        public float X;
        public float Y;
        public float Z;
        public float W;

        public Quaternion(float x, float y, float z, float w)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
            this.W = w;
        }

        public static void Add(ref Quaternion a, ref Quaternion b, out Quaternion result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
            result.W = a.W + b.W;
        }

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

        public static void Multiply(ref Quaternion q, float scale, out Quaternion result)
        {
            result.X = q.X * scale;
            result.Y = q.Y * scale;
            result.Z = q.Z * scale;
            result.W = q.W * scale;
        }

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


        public static Quaternion Identity
        {
            get
            {
                return new Quaternion(0, 0, 0, 1);
            }
        }

        public static Quaternion CreateFromRotationMatrix(Matrix r)
        {
            Quaternion toReturn;
            CreateFromRotationMatrix(ref r, out toReturn);
            return toReturn;
        }

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

        public static Quaternion Normalize(Quaternion quaternion)
        {
            Quaternion toReturn;
            Quaternion.Normalize(ref quaternion, out toReturn);
            return toReturn;
        }

        public static void Normalize(ref Quaternion quaternion, out Quaternion toReturn)
        {
            float inverse = (float)(1 / Math.Sqrt(quaternion.X * quaternion.X + quaternion.Y * quaternion.Y + quaternion.Z * quaternion.Z + quaternion.W * quaternion.W));
            toReturn.X = quaternion.X * inverse;
            toReturn.Y = quaternion.Y * inverse;
            toReturn.Z = quaternion.Z * inverse;
            toReturn.W = quaternion.W * inverse;
        }

        public void Normalize()
        {
            float inverse = (float)(1 / Math.Sqrt(X * X + Y * Y + Z * Z + W * W));
            X *= inverse;
            Y *= inverse;
            Z *= inverse;
            W *= inverse;
        }

        public static Quaternion Slerp(Quaternion start, Quaternion end, float interpolationAmount)
        {
            Quaternion toReturn;
            Slerp(ref start, ref end, interpolationAmount, out toReturn);
            return toReturn;
        }
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

        public static Quaternion Conjugate(Quaternion quaternion)
        {
            Quaternion toReturn;
            Conjugate(ref quaternion, out toReturn);
            return toReturn;
        }

        public static void Conjugate(ref Quaternion quaternion, out Quaternion result)
        {
            result.X = -quaternion.X;
            result.Y = -quaternion.Y;
            result.Z = -quaternion.Z;
            result.W = quaternion.W;
        }

        public static bool operator ==(Quaternion a, Quaternion b)
        {
            return a.X == b.X && a.Y == b.Y && a.Z == b.Z && a.W == b.W;
        }

        public static bool operator !=(Quaternion a, Quaternion b)
        {
            return a.X != b.X || a.Y != b.Y || a.Z != b.Z || a.W != b.W;
        }

        public static Quaternion operator *(Quaternion a, Quaternion b)
        {
            Quaternion toReturn;
            Multiply(ref a, ref b, out toReturn);
            return toReturn;
        }

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

        public static void CreateFromAxisAngle(ref Vector3 axis, float angle, out Quaternion q)
        {
            float halfAngle = angle * .5f;
            float s = (float)Math.Sin(halfAngle);
            q.X = axis.X * s;
            q.Y = axis.Y * s;
            q.Z = axis.Z * s;
            q.W = (float)Math.Cos(halfAngle);
        }

        public static Quaternion CreateFromYawPitchRoll(float yaw, float pitch, float roll)
        {
            Quaternion toReturn;
            CreateFromYawPitchRoll(yaw, pitch, roll, out toReturn);
            return toReturn;
        }

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
