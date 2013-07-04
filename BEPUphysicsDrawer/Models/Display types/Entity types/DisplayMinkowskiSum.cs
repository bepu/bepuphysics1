
using System;
using System.Collections.Generic;
using BEPUphysics;
using BEPUutilities;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionShapes;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using ConversionHelper;
using MathHelper = Microsoft.Xna.Framework.MathHelper;
using Vector2 = Microsoft.Xna.Framework.Vector2;
using Vector3 = Microsoft.Xna.Framework.Vector3;

namespace BEPUphysicsDrawer.Models
{
    /// <summary>
    /// Helper class that can create shape mesh data.
    /// </summary>
    public static class DisplayMinkowskiSum
    {
        /// <summary>
        /// Number of sides of spherical sampling to take when creating graphical representations.
        /// </summary>
        public static int NumSamples = 20;


        public static void GetShapeMeshData(EntityCollidable collidable, List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            var minkowskiSumShape = collidable.Shape as MinkowskiSumShape;
            if (minkowskiSumShape == null)
                throw new ArgumentException("Wrong shape type");
            var points = new List<BEPUutilities.Vector3>();
            BEPUutilities.Vector3 max;
            var direction = new BEPUutilities.Vector3();
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

                    minkowskiSumShape.GetLocalExtremePoint(direction, out max);
                    points.Add(max);
                }
            }

            minkowskiSumShape.GetLocalExtremePoint(Toolbox.UpVector, out max);
            points.Add(max);
            minkowskiSumShape.GetLocalExtremePoint(Toolbox.DownVector, out max);
            points.Add(max);

            var hullTriangleVertices = new List<BEPUutilities.Vector3>();
            var hullTriangleIndices = new List<int>();
            ConvexHullHelper.GetConvexHull(points, hullTriangleIndices, hullTriangleVertices);
            //The hull triangle vertices are used as a dummy to get the unnecessary hull vertices, which are cleared afterwards.
            hullTriangleVertices.Clear();
            foreach (int i in hullTriangleIndices)
            {
                hullTriangleVertices.Add(points[i]);
            }

            for (ushort i = 0; i < hullTriangleVertices.Count; i += 3)
            {
                Vector3 normal = MathConverter.Convert(BEPUutilities.Vector3.Normalize(BEPUutilities.Vector3.Cross(hullTriangleVertices[i + 2] - hullTriangleVertices[i], hullTriangleVertices[i + 1] - hullTriangleVertices[i])));
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