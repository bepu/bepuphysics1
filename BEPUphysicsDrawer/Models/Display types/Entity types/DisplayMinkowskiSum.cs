
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
        

        public static void GetShapeMeshData(EntityCollidable collidable, List<VertexPositionNormalTexture> vertices, List<ushort> indices)
        {
            InertiaHelper.GenerateSphere(0, out InertiaHelper.SampleDirections, out InertiaHelper.SampleTriangleIndices);
            var vertexPositions = new BEPUutilities.Vector3[InertiaHelper.SampleDirections.Length];
            InertiaHelper.ComputeSamples(collidable.Shape as ConvexShape, vertexPositions);
            //ConvexHullHelper.GetConvexHull(VertexPositionColor
            foreach (var index in InertiaHelper.SampleTriangleIndices)
            {
                indices.Add((ushort)index);
            }
            var normals = new BEPUutilities.Vector3[InertiaHelper.SampleDirections.Length];

            for (int triangleIndex = 0; triangleIndex < InertiaHelper.SampleTriangleIndices.Length; triangleIndex += 3)
            {
                var aIndex = InertiaHelper.SampleTriangleIndices[triangleIndex];
                var bIndex = InertiaHelper.SampleTriangleIndices[triangleIndex + 1];
                var cIndex = InertiaHelper.SampleTriangleIndices[triangleIndex + 2];
                var a = vertexPositions[aIndex];
                var b = vertexPositions[bIndex];
                var c = vertexPositions[cIndex];
                BEPUutilities.Vector3 ab, ac;
                BEPUutilities.Vector3.Subtract(ref b, ref a, out ab);
                BEPUutilities.Vector3.Subtract(ref c, ref a, out ac);
                BEPUutilities.Vector3 normal;
                BEPUutilities.Vector3.Cross(ref ac, ref ab, out normal);

                //The normal's magnitude is proportional to the area of the triangle.
                //So, the summed normals will be weighted by the adjacent triangle areas, which turns out to be a nice feature!

                BEPUutilities.Vector3.Add(ref normals[aIndex], ref normal, out normals[aIndex]);
                BEPUutilities.Vector3.Add(ref normals[bIndex], ref normal, out normals[bIndex]);
                BEPUutilities.Vector3.Add(ref normals[cIndex], ref normal, out normals[cIndex]);
            }
            for (int i = 0; i < normals.Length; ++i)
            {
                normals[i].Normalize();
            }

            for (int i = 0; i < vertexPositions.Length; ++i)
            {
                vertices.Add(new VertexPositionNormalTexture(MathConverter.Convert(vertexPositions[i]), MathConverter.Convert(normals[i]), new Vector2()));
            }


            //return;
            //var minkowskiSumShape = collidable.Shape as MinkowskiSumShape;
            //if (minkowskiSumShape == null)
            //    throw new ArgumentException("Wrong shape type");
            //var points = new List<BEPUutilities.Vector3>();
            //BEPUutilities.Vector3 max;
            //var direction = new BEPUutilities.Vector3();
            //float angleChange = MathHelper.TwoPi / NumSamples;

            //for (int i = 1; i < NumSamples / 2 - 1; i++)
            //{
            //    float phi = MathHelper.PiOver2 - i * angleChange;
            //    var sinPhi = (float)Math.Sin(phi);
            //    var cosPhi = (float)Math.Cos(phi);
            //    for (int j = 0; j < NumSamples; j++)
            //    {
            //        float theta = j * angleChange;
            //        direction.X = (float)Math.Cos(theta) * cosPhi;
            //        direction.Y = sinPhi;
            //        direction.Z = (float)Math.Sin(theta) * cosPhi;

            //        minkowskiSumShape.GetLocalExtremePoint(direction, out max);
            //        points.Add(max);
            //    }
            //}

            //minkowskiSumShape.GetLocalExtremePoint(Toolbox.UpVector, out max);
            //points.Add(max);
            //minkowskiSumShape.GetLocalExtremePoint(Toolbox.DownVector, out max);
            //points.Add(max);

            //var hullTriangleVertices = new List<BEPUutilities.Vector3>();
            //var hullTriangleIndices = new List<int>();
            //ConvexHullHelper.GetConvexHull(points, hullTriangleIndices, hullTriangleVertices);
            ////The hull triangle vertices are used as a dummy to get the unnecessary hull vertices, which are cleared afterwards.
            //hullTriangleVertices.Clear();
            //foreach (int i in hullTriangleIndices)
            //{
            //    hullTriangleVertices.Add(points[i]);
            //}

            //for (ushort i = 0; i < hullTriangleVertices.Count; i += 3)
            //{
            //    Vector3 normal = MathConverter.Convert(BEPUutilities.Vector3.Normalize(BEPUutilities.Vector3.Cross(hullTriangleVertices[i + 2] - hullTriangleVertices[i], hullTriangleVertices[i + 1] - hullTriangleVertices[i])));
            //    vertices.Add(new VertexPositionNormalTexture(MathConverter.Convert(hullTriangleVertices[i]), normal, new Vector2(0, 0)));
            //    vertices.Add(new VertexPositionNormalTexture(MathConverter.Convert(hullTriangleVertices[i + 1]), normal, new Vector2(1, 0)));
            //    vertices.Add(new VertexPositionNormalTexture(MathConverter.Convert(hullTriangleVertices[i + 2]), normal, new Vector2(0, 1)));
            //    indices.Add(i);
            //    indices.Add((ushort)(i + 1));
            //    indices.Add((ushort)(i + 2));
            //}
        }
    }
}