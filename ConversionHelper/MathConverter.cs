using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace ConversionHelper
{
    /// <summary>
    /// Helps convert between XNA math types and the BEPUphysics replacement math types.
    /// A version of this converter could be created for other platforms to ease the integration of the engine.
    /// </summary>
    public static class MathConverter
    {
        //Vector2
        public static Vector2 Convert(BEPUphysics.MathExtensions.Vector2 bepuVector)
        {
            Vector2 toReturn;
            toReturn.X = bepuVector.X;
            toReturn.Y = bepuVector.Y;
            return toReturn;
        }

        public static void Convert(ref BEPUphysics.MathExtensions.Vector2 bepuVector, out Vector2 xnaVector)
        {
            xnaVector.X = bepuVector.X;
            xnaVector.Y = bepuVector.Y;
        }

        public static BEPUphysics.MathExtensions.Vector2 Convert(Vector2 xnaVector)
        {
            BEPUphysics.MathExtensions.Vector2 toReturn;
            toReturn.X = xnaVector.X;
            toReturn.Y = xnaVector.Y;
            return toReturn;
        }

        public static void Convert(ref Vector2 xnaVector, out BEPUphysics.MathExtensions.Vector2 bepuVector)
        {
            bepuVector.X = xnaVector.X;
            bepuVector.Y = xnaVector.Y;
        }

        //Vector3
        public static Vector3 Convert(BEPUphysics.MathExtensions.Vector3 bepuVector)
        {
            Vector3 toReturn;
            toReturn.X = bepuVector.X;
            toReturn.Y = bepuVector.Y;
            toReturn.Z = bepuVector.Z;
            return toReturn;
        }

        public static void Convert(ref BEPUphysics.MathExtensions.Vector3 bepuVector, out Vector3 xnaVector)
        {
            xnaVector.X = bepuVector.X;
            xnaVector.Y = bepuVector.Y;
            xnaVector.Z = bepuVector.Z;
        }

        public static BEPUphysics.MathExtensions.Vector3 Convert(Vector3 xnaVector)
        {
            BEPUphysics.MathExtensions.Vector3 toReturn;
            toReturn.X = xnaVector.X;
            toReturn.Y = xnaVector.Y;
            toReturn.Z = xnaVector.Z;
            return toReturn;
        }

        public static void Convert(ref Vector3 xnaVector, out BEPUphysics.MathExtensions.Vector3 bepuVector)
        {
            bepuVector.X = xnaVector.X;
            bepuVector.Y = xnaVector.Y;
            bepuVector.Z = xnaVector.Z;
        }

        public static Vector3[] Convert(BEPUphysics.MathExtensions.Vector3[] bepuVectors)
        {
            Vector3[] xnaVectors = new Vector3[bepuVectors.Length];
            for (int i = 0; i < bepuVectors.Length; i++)
            {
                Convert(ref bepuVectors[i], out xnaVectors[i]);
            }
            return xnaVectors;

        }

        public static BEPUphysics.MathExtensions.Vector3[] Convert(Vector3[] xnaVectors)
        {
            var bepuVectors = new BEPUphysics.MathExtensions.Vector3[xnaVectors.Length];
            for (int i = 0; i < xnaVectors.Length; i++)
            {
                Convert(ref xnaVectors[i], out bepuVectors[i]);
            }
            return bepuVectors;

        }

        //Matrix
        public static Matrix Convert(BEPUphysics.MathExtensions.Matrix matrix)
        {
            Matrix toReturn;
            Convert(ref matrix, out toReturn);
            return toReturn;
        }

        public static BEPUphysics.MathExtensions.Matrix Convert(Matrix matrix)
        {
            BEPUphysics.MathExtensions.Matrix toReturn;
            Convert(ref matrix, out toReturn);
            return toReturn;
        }

        public static void Convert(ref BEPUphysics.MathExtensions.Matrix matrix, out Matrix xnaMatrix)
        {
            xnaMatrix.M11 = matrix.M11;
            xnaMatrix.M12 = matrix.M12;
            xnaMatrix.M13 = matrix.M13;
            xnaMatrix.M14 = matrix.M14;

            xnaMatrix.M21 = matrix.M21;
            xnaMatrix.M22 = matrix.M22;
            xnaMatrix.M23 = matrix.M23;
            xnaMatrix.M24 = matrix.M24;

            xnaMatrix.M31 = matrix.M31;
            xnaMatrix.M32 = matrix.M32;
            xnaMatrix.M33 = matrix.M33;
            xnaMatrix.M34 = matrix.M34;

            xnaMatrix.M41 = matrix.M41;
            xnaMatrix.M42 = matrix.M42;
            xnaMatrix.M43 = matrix.M43;
            xnaMatrix.M44 = matrix.M44;

        }

        public static void Convert(ref Matrix matrix, out BEPUphysics.MathExtensions.Matrix bepuMatrix)
        {
            bepuMatrix.M11 = matrix.M11;
            bepuMatrix.M12 = matrix.M12;
            bepuMatrix.M13 = matrix.M13;
            bepuMatrix.M14 = matrix.M14;

            bepuMatrix.M21 = matrix.M21;
            bepuMatrix.M22 = matrix.M22;
            bepuMatrix.M23 = matrix.M23;
            bepuMatrix.M24 = matrix.M24;

            bepuMatrix.M31 = matrix.M31;
            bepuMatrix.M32 = matrix.M32;
            bepuMatrix.M33 = matrix.M33;
            bepuMatrix.M34 = matrix.M34;

            bepuMatrix.M41 = matrix.M41;
            bepuMatrix.M42 = matrix.M42;
            bepuMatrix.M43 = matrix.M43;
            bepuMatrix.M44 = matrix.M44;

        }

        public static Matrix Convert(BEPUphysics.MathExtensions.Matrix3X3 matrix)
        {
            Matrix toReturn;
            Convert(ref matrix, out toReturn);
            return toReturn;
        }

        public static void Convert(ref BEPUphysics.MathExtensions.Matrix3X3 matrix, out Matrix xnaMatrix)
        {
            xnaMatrix.M11 = matrix.M11;
            xnaMatrix.M12 = matrix.M12;
            xnaMatrix.M13 = matrix.M13;
            xnaMatrix.M14 = 0;

            xnaMatrix.M21 = matrix.M21;
            xnaMatrix.M22 = matrix.M22;
            xnaMatrix.M23 = matrix.M23;
            xnaMatrix.M24 = 0;

            xnaMatrix.M31 = matrix.M31;
            xnaMatrix.M32 = matrix.M32;
            xnaMatrix.M33 = matrix.M33;
            xnaMatrix.M34 = 0;

            xnaMatrix.M41 = 0;
            xnaMatrix.M42 = 0;
            xnaMatrix.M43 = 0;
            xnaMatrix.M44 = 1;
        }

        public static void Convert(ref Matrix matrix, out BEPUphysics.MathExtensions.Matrix3X3 bepuMatrix)
        {
            bepuMatrix.M11 = matrix.M11;
            bepuMatrix.M12 = matrix.M12;
            bepuMatrix.M13 = matrix.M13;

            bepuMatrix.M21 = matrix.M21;
            bepuMatrix.M22 = matrix.M22;
            bepuMatrix.M23 = matrix.M23;

            bepuMatrix.M31 = matrix.M31;
            bepuMatrix.M32 = matrix.M32;
            bepuMatrix.M33 = matrix.M33;

        }

        //Quaternion
        public static Quaternion Convert(BEPUphysics.MathExtensions.Quaternion quaternion)
        {
            Quaternion toReturn;
            toReturn.X = quaternion.X;
            toReturn.Y = quaternion.Y;
            toReturn.Z = quaternion.Z;
            toReturn.W = quaternion.W;
            return toReturn;
        }

        public static BEPUphysics.MathExtensions.Quaternion Convert(Quaternion quaternion)
        {
            BEPUphysics.MathExtensions.Quaternion toReturn;
            toReturn.X = quaternion.X;
            toReturn.Y = quaternion.Y;
            toReturn.Z = quaternion.Z;
            toReturn.W = quaternion.W;
            return toReturn;
        }

        public static void Convert(ref BEPUphysics.MathExtensions.Quaternion bepuQuaternion, out Quaternion quaternion)
        {
            quaternion.X = bepuQuaternion.X;
            quaternion.Y = bepuQuaternion.Y;
            quaternion.Z = bepuQuaternion.Z;
            quaternion.W = bepuQuaternion.W;
        }

        public static void Convert(ref Quaternion quaternion, out  BEPUphysics.MathExtensions.Quaternion bepuQuaternion)
        {
            bepuQuaternion.X = quaternion.X;
            bepuQuaternion.Y = quaternion.Y;
            bepuQuaternion.Z = quaternion.Z;
            bepuQuaternion.W = quaternion.W;
        }

        //Ray
        public static BEPUphysics.MathExtensions.Ray Convert(Ray ray)
        {
            BEPUphysics.MathExtensions.Ray toReturn;
            Convert(ref ray.Position, out toReturn.Position);
            Convert(ref ray.Direction, out toReturn.Direction);
            return toReturn;
        }

        public static void Convert(ref Ray ray, out BEPUphysics.MathExtensions.Ray bepuRay)
        {
            Convert(ref ray.Position, out bepuRay.Position);
            Convert(ref ray.Direction, out bepuRay.Direction);
        }

        public static Ray Convert(BEPUphysics.MathExtensions.Ray ray)
        {
            Ray toReturn;
            Convert(ref ray.Position, out toReturn.Position);
            Convert(ref ray.Direction, out toReturn.Direction);
            return toReturn;
        }

        public static void Convert(ref BEPUphysics.MathExtensions.Ray ray, out Ray xnaRay)
        {
            Convert(ref ray.Position, out xnaRay.Position);
            Convert(ref ray.Direction, out xnaRay.Direction);
        }

        //BoundingBox
        public static BoundingBox Convert(BEPUphysics.MathExtensions.BoundingBox boundingBox)
        {
            BoundingBox toReturn;
            Convert(ref boundingBox.Min, out toReturn.Min);
            Convert(ref boundingBox.Max, out toReturn.Max);
            return toReturn;
        }

        public static BEPUphysics.MathExtensions.BoundingBox Convert(BoundingBox boundingBox)
        {
            BEPUphysics.MathExtensions.BoundingBox toReturn;
            Convert(ref boundingBox.Min, out toReturn.Min);
            Convert(ref boundingBox.Max, out toReturn.Max);
            return toReturn;
        }

        public static void Convert(ref BEPUphysics.MathExtensions.BoundingBox boundingBox, out BoundingBox xnaBoundingBox)
        {
            Convert(ref boundingBox.Min, out xnaBoundingBox.Min);
            Convert(ref boundingBox.Max, out xnaBoundingBox.Max);
        }

        public static void Convert(ref BoundingBox boundingBox, out BEPUphysics.MathExtensions.BoundingBox bepuBoundingBox)
        {
            Convert(ref boundingBox.Min, out bepuBoundingBox.Min);
            Convert(ref boundingBox.Max, out bepuBoundingBox.Max);
        }

        //BoundingSphere
        public static BoundingSphere Convert(BEPUphysics.MathExtensions.BoundingSphere boundingSphere)
        {
            BoundingSphere toReturn;
            Convert(ref boundingSphere.Center, out toReturn.Center);
            toReturn.Radius = boundingSphere.Radius;
            return toReturn;
        }

        public static BEPUphysics.MathExtensions.BoundingSphere Convert(BoundingSphere boundingSphere)
        {
            BEPUphysics.MathExtensions.BoundingSphere toReturn;
            Convert(ref boundingSphere.Center, out toReturn.Center);
            toReturn.Radius = boundingSphere.Radius;
            return toReturn;
        }

        public static void Convert(ref BEPUphysics.MathExtensions.BoundingSphere boundingSphere, out BoundingSphere xnaBoundingSphere)
        {
            Convert(ref boundingSphere.Center, out xnaBoundingSphere.Center);
            xnaBoundingSphere.Radius = boundingSphere.Radius;
        }

        public static void Convert(ref BoundingSphere boundingSphere, out BEPUphysics.MathExtensions.BoundingSphere bepuBoundingSphere)
        {
            Convert(ref boundingSphere.Center, out bepuBoundingSphere.Center);
            bepuBoundingSphere.Radius = boundingSphere.Radius;
        }

        //Plane
        public static Plane Convert(BEPUphysics.MathExtensions.Plane plane)
        {
            Plane toReturn;
            Convert(ref plane.Normal, out toReturn.Normal);
            toReturn.D = plane.D;
            return toReturn;
        }

        public static BEPUphysics.MathExtensions.Plane Convert(Plane plane)
        {
            BEPUphysics.MathExtensions.Plane toReturn;
            Convert(ref plane.Normal, out toReturn.Normal);
            toReturn.D = plane.D;
            return toReturn;
        }

        public static void Convert(ref BEPUphysics.MathExtensions.Plane plane, out Plane xnaPlane)
        {
            Convert(ref plane.Normal, out xnaPlane.Normal);
            xnaPlane.D = plane.D;
        }

        public static void Convert(ref Plane plane, out BEPUphysics.MathExtensions.Plane bepuPlane)
        {
            Convert(ref plane.Normal, out bepuPlane.Normal);
            bepuPlane.D = plane.D;
        }
    }
}
