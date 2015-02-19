
using System;
using System.Collections.Generic;
using BEPUphysics;
using BEPUutilities;
using BEPUutilities.DataStructures;
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
    public static class DisplayConvex
    {
        private static BEPUutilities.Vector3[] SampleDirections;
        static DisplayConvex()
        {
            int[] sampleTriangleIndices;
            InertiaHelper.GenerateSphere(2, out SampleDirections, out sampleTriangleIndices);
        }

        public static void GetShapeMeshData(EntityCollidable collidable, List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            var shape = collidable.Shape as ConvexShape;
            if (shape == null)
                throw new ArgumentException("Wrong shape type for this helper.");
            var vertexPositions = new BEPUutilities.Vector3[SampleDirections.Length];

            for (int i = 0; i < SampleDirections.Length; ++i)
            {
                shape.GetLocalExtremePoint(SampleDirections[i], out vertexPositions[i]);
            }

            var hullIndices = new RawList<int>();
            ConvexHullHelper.GetConvexHull(vertexPositions, hullIndices);


            var hullTriangleVertices = new RawList<BEPUutilities.Vector3>();
            foreach (int i in hullIndices)
            {
                hullTriangleVertices.Add(vertexPositions[i]);
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