using System;
using System.Collections.Generic;
using BEPUphysics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionShapes;
using BEPUphysics.Collidables.MobileCollidables;
using ConversionHelper;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Helper class that can create shape mesh data.
    /// </summary>
    public class DisplayWrappedBody
    {
        /// <summary>
        /// Number of sides of spherical sampling to take when creating graphical representations.
        /// </summary>
        public static int NumSamples = 16;


        public static void GetShapeMeshData(EntityCollidable collidable, List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            var wrappedShape = collidable.Shape as WrappedShape;
            if (wrappedShape == null)
                throw new ArgumentException("Wrong shape type");
            var points = new List<BEPUphysics.MathExtensions.Vector3>();
            BEPUphysics.MathExtensions.Vector3 max;
            var direction = new BEPUphysics.MathExtensions.Vector3();
            float angleChange = MathHelper.TwoPi / NumSamples;

            for (int i = 1; i < NumSamples / 2 - 1; i++)
            {
                float phi = MathHelper.PiOver2 - i * angleChange;
                var sinPhi = (float)Math.Sin(phi);
                var cosPhi = (float)Math.Cos(phi);
                for (int j = 0; j < NumSamples; j++)
                {
                    float theta = j * angleChange;
                    direction.X = (float)Math.Cos(theta) * cosPhi;
                    direction.Y = sinPhi;
                    direction.Z = (float)Math.Sin(theta) * cosPhi;

                    wrappedShape.GetLocalExtremePoint(direction, out max);
                    points.Add(max);
                }
            }

            wrappedShape.GetLocalExtremePoint(Toolbox.UpVector, out max);
            points.Add(max);
            wrappedShape.GetLocalExtremePoint(Toolbox.DownVector, out max);
            points.Add(max);

            var hullTriangleVertices = new List<BEPUphysics.MathExtensions.Vector3>();
            var hullTriangleIndices = new List<int>();
            Toolbox.GetConvexHull(points, hullTriangleIndices, hullTriangleVertices);
            //The hull triangle vertices are used as a dummy to get the unnecessary hull vertices, which are cleared afterwards.
            hullTriangleVertices.Clear();
            foreach (int i in hullTriangleIndices)
            {
                hullTriangleVertices.Add(points[i]);
            }

            for (ushort i = 0; i < hullTriangleVertices.Count; i += 3)
            {
                Vector3 normal = MathConverter.Convert(BEPUphysics.MathExtensions.Vector3.Normalize(BEPUphysics.MathExtensions.Vector3.Cross(hullTriangleVertices[i + 2] - hullTriangleVertices[i], hullTriangleVertices[i + 1] - hullTriangleVertices[i])));
                vertices.Add(new VertexPositionNormalTexture(MathConverter.Convert(hullTriangleVertices[i]), normal, new Vector2(0, 0)));
                vertices.Add(new VertexPositionNormalTexture(MathConverter.Convert(hullTriangleVertices[i + 1]), normal, new Vector2(1, 0)));
                vertices.Add(new VertexPositionNormalTexture(MathConverter.Convert(hullTriangleVertices[i + 2]), normal, new Vector2(0, 1)));
                indices.Add(i);
                indices.Add((ushort)(i + 1));
                indices.Add((ushort)(i + 2));
            }
        }
    }
}