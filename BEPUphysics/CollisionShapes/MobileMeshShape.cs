using Microsoft.Xna.Framework;
using BEPUphysics.DataStructures;
using BEPUphysics.MathExtensions;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.ResourceManagement;
using System.Collections.Generic;
using System;

namespace BEPUphysics.CollisionShapes
{
    ///<summary>
    /// Local space data associated with an instanced mesh.
    /// This contains a hierarchy and all the other heavy data needed
    /// by an InstancedMesh.
    ///</summary>
    public class MobileMeshShape : EntityShape
    {
        TriangleMesh triangleMesh;
        ///<summary>
        /// Gets or sets the TriangleMesh data structure used by this shape.
        ///</summary>
        public TriangleMesh TriangleMesh
        {
            get
            {
                return triangleMesh;
            }
        }

        RawList<Vector3> surfaceVertices = new RawList<Vector3>();

        //TODO: Sidedness on a mobile mesh is a bit weird, since if solidity is enabled, then only one option is permissible.
        internal MobileMeshSolidity solidity = MobileMeshSolidity.DoubleSided;
        ///<summary>
        /// Gets the solidity of the mesh.
        ///</summary>
        public MobileMeshSolidity Solidity
        {
            get
            {
                return solidity;
            }
        }


        ///<summary>
        /// Constructs a new instanced mesh shape.
        ///</summary>
        ///<param name="vertices">Vertices of the mesh.</param>
        ///<param name="indices">Indices of the mesh.</param>
        ///<param name="localTransform">Local transform to apply to the shape.</param>
        ///<param name="solidity">Solidity state of the shape.</param>
        ///<param name="distributionInfo">Information computed about the shape during construction.</param>
        public MobileMeshShape(Vector3[] vertices, int[] indices, AffineTransform localTransform, MobileMeshSolidity solidity, out ShapeDistributionInformation distributionInfo)
        {
            this.solidity = solidity;
            var data = new TransformableMeshData(vertices, indices, localTransform);
            ComputeShapeInformation(data, out distributionInfo);
            data.worldTransform.Translation -= distributionInfo.Center;

            for (int i = 0; i < surfaceVertices.count; i++)
            {
                Vector3.Subtract(ref surfaceVertices.Elements[i], ref distributionInfo.Center, out surfaceVertices.Elements[i]);
            }
            triangleMesh = new TriangleMesh(data);

            ComputeSolidSidedness();
            //ComputeBoundingHull();
        }

        /// <summary>
        /// Sidedness required if the mesh is in solid mode.
        /// If the windings were reversed or double sided,
        /// the solidity would fight against shell contacts,
        /// leading to very bad jittering.
        /// </summary>
        internal TriangleSidedness solidSidedness;

        internal bool IsPointInsideMesh(ref Vector3 point, ref RigidTransform transform)
        {
            Ray ray = new Ray();
            //Pick a ray direction that goes to a random location on the mesh.  
            //A vertex would work, but targeting the middle of a triangle avoids some edge cases.
            Vector3 vA, vB, vC;
            triangleMesh.Data.GetTriangle(((triangleMesh.Data.indices.Length / 3) / 2) * 3, out vA, out vB, out vC);
            Vector3.Add(ref vA, ref vB, out ray.Direction);
            Vector3.Add(ref vC, ref ray.Direction, out ray.Direction);
            Vector3.Divide(ref ray.Direction, 3, out ray.Direction);

            var overlapList = Resources.GetIntList();
            if (triangleMesh.Tree.GetOverlaps(ray, overlapList))
            {
                var hits = Resources.GetRayHitList();
                for (int i = 0; i < overlapList.Count; i++)
                {
                    triangleMesh.Data.GetTriangle(overlapList[i], out vA, out vB, out vC);
                    RayHit hit;
                    if (Toolbox.FindRayTriangleIntersection(ref ray, float.MaxValue, TriangleSidedness.DoubleSided, ref vA, ref vB, ref vC, out hit))
                    {
                        //Adds if unique.
                        IsHitUnique(hits, ref hit);
                    }
                }
                int hitCount = hits.count;
                Resources.GiveBack(hits);
                Resources.GiveBack(overlapList);
                return hits.count % 2 != 0; //true -> outside, false -> inside
            }
            Resources.GiveBack(overlapList);
            return false;
        }

        internal bool IsHitUnique(RawList<RayHit> hits, ref RayHit hit)
        {
            for (int i = 0; i < hits.Count; i++)
            {
                if (Math.Abs(hits.Elements[i].T - hit.T) < Toolbox.Epsilon)
                    return false;
            }
            hits.Add(hit);
            return true;
        }

        void ComputeSolidSidedness()
        {
            //Raycast against the mesh.
            //If there's an even number of hits, then the ray start point is outside.
            //If there's an odd number of hits, then the ray start point is inside.

            //If the start is outside, then take the earliest toi hit and calibrate sidedness based on it.
            //If the start is inside, then take the latest toi hit and calibrate sidedness based on it.

            //This test assumes consistent winding across the entire mesh as well as a closed surface.
            //If those assumptions are not correct, then the raycast cannot determine inclusion or exclusion,
            //or there exists no calibration that will work across the entire surface.

            //Pick a ray direction that goes to a random location on the mesh.  
            //A vertex would work, but targeting the middle of a triangle avoids some edge cases.
            Ray ray = new Ray();
            Vector3 vA, vB, vC;
            triangleMesh.Data.GetTriangle(((triangleMesh.Data.indices.Length / 3) / 2) * 3, out vA, out vB, out vC);
            ray.Direction = (vA + vB + vC) / 3;

            var hitList = Resources.GetIntList();
            if (triangleMesh.Tree.GetOverlaps(ray, hitList))
            {
                var hits = Resources.GetRayHitList();
                //Identify the first and last hits.
                int minimum = 0;
                int maximum = 0;
                float minimumT = float.MaxValue;
                float maximumT = -1;
                for (int i = 0; i < hitList.Count; i++)
                {
                    triangleMesh.Data.GetTriangle(hitList[i], out vA, out vB, out vC);
                    RayHit hit;
                    if (Toolbox.FindRayTriangleIntersection(ref ray, float.MaxValue, TriangleSidedness.DoubleSided, ref vA, ref vB, ref vC, out hit) &&
                        IsHitUnique(hits, ref hit))
                    {
                        if (hit.T < minimumT)
                        {
                            minimumT = hit.T;
                            minimum = hits.count - 1;
                        }
                        if (hit.T > maximumT)
                        {
                            maximumT = hit.T;
                            maximum = hits.count - 1;
                        }
                    }
                }

                if (hits.count % 2 == 0)
                {
                    //Since we were outside, the first hit triangle should be calibrated
                    //such that it faces towards us.

                    triangleMesh.Data.GetTriangle(minimum, out vA, out vB, out vC);
                    var normal = Vector3.Cross(vA - vB, vA - vC);
                    if (Vector3.Dot(normal, ray.Direction) < 0)
                        solidSidedness = TriangleSidedness.Clockwise;
                    else
                        solidSidedness = TriangleSidedness.Counterclockwise;
                }
                else
                {
                    //Since we were inside, the last hit triangle should be calibrated
                    //such that it faces away from us.

                    triangleMesh.Data.GetTriangle(maximum, out vA, out vB, out vC);
                    var normal = Vector3.Cross(vA - vB, vA - vC);
                    if (Vector3.Dot(normal, ray.Direction) < 0)
                        solidSidedness = TriangleSidedness.Counterclockwise;
                    else
                        solidSidedness = TriangleSidedness.Clockwise;
                }

                Resources.GiveBack(hits);

            }
            Resources.GiveBack(hitList);
        }

        void ComputeShapeInformation(TransformableMeshData data, out ShapeDistributionInformation shapeInformation)
        {
            var indices = Resources.GetIntList();
            surfaceVertices.Clear();
            Toolbox.GetConvexHull(data.vertices, indices, surfaceVertices);
            for (int i = 0; i < surfaceVertices.count; i++)
            {
                AffineTransform.Transform(ref surfaceVertices.Elements[i], ref data.worldTransform, out surfaceVertices.Elements[i]);
            }
            shapeInformation.Center = new Vector3();

            if (solidity == MobileMeshSolidity.Solid)
            {

                //The following inertia tensor calculation assumes a closed mesh.

                //Source: Explicit Exact Formulas for the 3-D Tetrahedron Inertia Tensor in Terms of its Vertex Coordinates
                //http://www.scipub.org/fulltext/jms2/jms2118-11.pdf
                //x1, x2, x3, x4 are origin, triangle1, triangle2, triangle3
                //Looking to find inertia tensor matrix of the form
                // [  a  -b' -c' ]
                // [ -b'  b  -a' ]
                // [ -c' -a'  c  ]
                float a = 0, b = 0, c = 0, ao = 0, bo = 0, co = 0;



                shapeInformation.Volume = 0;
                for (int i = 0; i < data.indices.Length; i += 3)
                {
                    Vector3 v2, v3, v4;
                    data.GetTriangle(i, out v2, out v3, out v4);

                    //Determinant is 6 * volume.  It's signed, though; this is because the mesh isn't necessarily convex nor centered on the origin.
                    //float tetrahedronVolume = Vector3.Dot(v2, Vector3.Cross(v3, v4));
                    float tetrahedronVolume = v2.X * (v3.Y * v4.Z - v3.Z * v4.Y) -
                                              v3.X * (v2.Y * v4.Z - v2.Z * v4.Y) +
                                              v4.X * (v2.Y * v3.Z - v2.Z * v3.Y);

                    shapeInformation.Volume += tetrahedronVolume;
                    shapeInformation.Center += tetrahedronVolume * (v2 + v3 + v4);

                    a += tetrahedronVolume * (v2.Y * v2.Y + v2.Y * v3.Y + v3.Y * v3.Y + v2.Y * v4.Y + v3.Y * v4.Y + v4.Y * v4.Y +
                                              v2.Z * v2.Z + v2.Z * v3.Z + v3.Z * v3.Z + v2.Z * v4.Z + v3.Z * v4.Z + v4.Z * v4.Z);
                    b += tetrahedronVolume * (v2.X * v2.X + v2.X * v3.X + v3.X * v3.X + v2.X * v4.X + v3.X * v4.X + v4.X * v4.X +
                                              v2.Z * v2.Z + v2.Z * v3.Z + v3.Z * v3.Z + v2.Z * v4.Z + v3.Z * v4.Z + v4.Z * v4.Z);
                    c += tetrahedronVolume * (v2.X * v2.X + v2.X * v3.X + v3.X * v3.X + v2.X * v4.X + v3.X * v4.X + v4.X * v4.X +
                                              v2.Y * v2.Y + v2.Y * v3.Y + v3.Y * v3.Y + v2.Y * v4.Y + v3.Y * v4.Y + v4.Y * v4.Y);
                    ao += tetrahedronVolume * (2 * v2.Y * v2.Z + v3.Y * v2.Z + v4.Y * v2.Z + v2.Y * v3.Z + 2 * v3.Y * v3.Z + v4.Y * v3.Z + v2.Y * v4.Z + v3.Y * v4.Z + 2 * v4.Y * v4.Z);
                    bo += tetrahedronVolume * (2 * v2.X * v2.Z + v3.X * v2.Z + v4.X * v2.Z + v2.X * v3.Z + 2 * v3.X * v3.Z + v4.X * v3.Z + v2.X * v4.Z + v3.X * v4.Z + 2 * v4.X * v4.Z);
                    co += tetrahedronVolume * (2 * v2.X * v2.Y + v3.X * v2.Y + v4.X * v2.Y + v2.X * v3.Y + 2 * v3.X * v3.Y + v4.X * v3.Y + v2.X * v4.Y + v3.X * v4.Y + 2 * v4.X * v4.Y);
                }
                shapeInformation.Center /= shapeInformation.Volume * 4;
                shapeInformation.Volume /= 6;
                float density = 1 / shapeInformation.Volume;
                float diagonalFactor = density / 60;
                float offFactor = -density / 120;
                a *= diagonalFactor;
                b *= diagonalFactor;
                c *= diagonalFactor;
                ao *= offFactor;
                bo *= offFactor;
                co *= offFactor;
                shapeInformation.VolumeDistribution = new Matrix3X3(a, bo, co,
                                                                    bo, b, ao,
                                                                    co, ao, c);


                shapeInformation.Volume = Math.Abs(shapeInformation.Volume);
            }
            else
            {
                shapeInformation.Center = new Vector3();
                float totalWeight = 0;
                shapeInformation.VolumeDistribution = new Matrix3X3();
                for (int i = 0; i < data.indices.Length; i += 3)
                { //Configure the inertia tensor to be local.
                    Vector3 vA, vB, vC;
                    data.GetTriangle(i, out vA, out vB, out vC);
                    Vector3 vAvB;
                    Vector3 vAvC;
                    Vector3.Subtract(ref vB, ref vA, out vAvB);
                    Vector3.Subtract(ref vC, ref vA, out vAvC);
                    Vector3 cross;
                    Vector3.Cross(ref vAvB, ref vAvC, out cross);
                    float weight = cross.Length();
                    totalWeight += weight;

                    shapeInformation.Center += weight * (vA + vB + vC) / 3;

                    Matrix3X3 innerProduct;
                    Matrix3X3.CreateScale(vA.LengthSquared(), out innerProduct);
                    Matrix3X3 outerProduct;
                    Matrix3X3.CreateOuterProduct(ref vA, ref vA, out outerProduct);
                    Matrix3X3 contribution;
                    Matrix3X3.Subtract(ref innerProduct, ref outerProduct, out contribution);
                    Matrix3X3.Add(ref shapeInformation.VolumeDistribution, ref contribution, out shapeInformation.VolumeDistribution);

                    Matrix3X3.CreateScale(vB.LengthSquared(), out innerProduct);
                    Matrix3X3.CreateOuterProduct(ref vB, ref vB, out outerProduct);
                    Matrix3X3.Subtract(ref innerProduct, ref outerProduct, out contribution);
                    Matrix3X3.Add(ref shapeInformation.VolumeDistribution, ref contribution, out shapeInformation.VolumeDistribution);

                    Matrix3X3.CreateScale(vC.LengthSquared(), out innerProduct);
                    Matrix3X3.CreateOuterProduct(ref vC, ref vC, out outerProduct);
                    Matrix3X3.Subtract(ref innerProduct, ref outerProduct, out contribution);
                    Matrix3X3.Add(ref shapeInformation.VolumeDistribution, ref contribution, out shapeInformation.VolumeDistribution);
                }
                shapeInformation.Center /= totalWeight;
                shapeInformation.Volume = 0;
                Matrix3X3.Multiply(ref shapeInformation.VolumeDistribution, 1 / (6 * totalWeight), out shapeInformation.VolumeDistribution);
            }

            //Configure the inertia tensor to be local.
            Vector3 finalOffset = shapeInformation.Center;
            Matrix3X3 finalInnerProduct;
            Matrix3X3.CreateScale(finalOffset.LengthSquared(), out finalInnerProduct);
            Matrix3X3 finalOuterProduct;
            Matrix3X3.CreateOuterProduct(ref finalOffset, ref finalOffset, out finalOuterProduct);

            Matrix3X3 finalContribution;
            Matrix3X3.Subtract(ref finalInnerProduct, ref finalOuterProduct, out finalContribution);

            Matrix3X3.Subtract(ref shapeInformation.VolumeDistribution, ref finalContribution, out shapeInformation.VolumeDistribution);
        }

        ///// <summary>
        ///// Defines two planes that bound the mesh shape in local space.
        ///// </summary>
        //struct Extent
        //{
        //    internal Vector3 Direction;
        //    internal float Minimum;
        //    internal float Maximum;

        //    internal void Clamp(ref Vector3 v)
        //    {
        //        float dot;
        //        Vector3.Dot(ref v, ref Direction, out dot);
        //        float difference;
        //        if (dot < Minimum)
        //        {
        //            difference = dot - Minimum;
        //        }
        //        else if (dot > Maximum)
        //        {
        //            difference = dot - Maximum;
        //        }
        //        else return;

        //        //Subtract the component of v which is parallel to the normal.
        //        v.X -= difference * Direction.X;
        //        v.Y -= difference * Direction.Y;
        //        v.Z -= difference * Direction.Z;
        //    }

        //}

        //RawList<Extent> extents = new RawList<Extent>();

        //void ComputeBoundingHull()
        //{
        //    //TODO:
        //    //While we have computed a convex hull of the shape already, we don't really
        //    //need the full tightness of the convex hull.
        //    extents.Add(new Extent() { Direction = new Vector3(1, 0, 0) });
        //    extents.Add(new Extent() { Direction = new Vector3(0, 1, 0) });
        //    extents.Add(new Extent() { Direction = new Vector3(0, 0, 1) });
        //    //extents.Add(new Extent() { Direction = new Vector3(1, 1, 0) });
        //    //extents.Add(new Extent() { Direction = new Vector3(-1, 1, 0) });
        //    //extents.Add(new Extent() { Direction = new Vector3(0, 1, 1) });
        //    //extents.Add(new Extent() { Direction = new Vector3(0, 1, -1) });
        //    extents.Add(new Extent() { Direction = Vector3.Normalize(new Vector3(1, 0, 1)) });
        //    extents.Add(new Extent() { Direction = Vector3.Normalize(new Vector3(1, 0, -1)) });
        //    //Add more extents for a tighter volume

        //    //Initialize the max and mins.
        //    for (int i = 0; i < extents.count; i++)
        //    {
        //        extents.Elements[i].Minimum = float.MaxValue;
        //        extents.Elements[i].Maximum = -float.MaxValue;
        //    }

        //    for (int i = 0; i < triangleMesh.Data.vertices.Length; i++)
        //    {
        //        Vector3 v;
        //        triangleMesh.Data.GetVertexPosition(i, out v);
        //        for (int j = 0; j < extents.count; j++)
        //        {
        //            float dot;
        //            Vector3.Dot(ref v, ref extents.Elements[j].Direction, out dot);
        //            if (dot < extents.Elements[j].Minimum)
        //                extents.Elements[j].Minimum = dot;
        //            if (dot > extents.Elements[j].Maximum)
        //                extents.Elements[j].Maximum = dot;
        //        }
        //    }
        //}

        ///<summary>
        /// Computes the bounding box of the transformed mesh shape.
        ///</summary>
        ///<param name="shapeTransform">Transform to apply to the shape during the bounding box calculation.</param>
        ///<param name="boundingBox">Bounding box containing the transformed mesh shape.</param>
        public void GetBoundingBox(ref RigidTransform shapeTransform, out BoundingBox boundingBox)
        {
            ////TODO: Could use an approximate bounding volume.  Would be cheaper at runtime and use less memory, though the box would be bigger.
            //Matrix3X3 o;
            //Matrix3X3.CreateFromQuaternion(ref shapeTransform.Orientation, out o);
            ////Sample the local directions from the orientation matrix, implicitly transposed.
            //Vector3 right = new Vector3(o.M11 * 100000, o.M21 * 100000, o.M31 * 100000);
            //Vector3 up = new Vector3(o.M12 * 100000, o.M22 * 100000, o.M32 * 100000);
            //Vector3 backward = new Vector3(o.M13 * 100000, o.M23 * 100000, o.M33 * 100000);
            //Vector3 left, down, forward;
            //Vector3.Negate(ref right, out left);
            //Vector3.Negate(ref up, out down);
            //Vector3.Negate(ref backward, out forward);
            //for (int i = 0; i < extents.count; i++)
            //{
            //    extents.Elements[i].Clamp(ref right);
            //    extents.Elements[i].Clamp(ref left);
            //    extents.Elements[i].Clamp(ref up);
            //    extents.Elements[i].Clamp(ref down);
            //    extents.Elements[i].Clamp(ref backward);
            //    extents.Elements[i].Clamp(ref forward);
            //}

            //Matrix3X3.Transform(ref right, ref o, out right);
            //Matrix3X3.Transform(ref left, ref o, out left);
            //Matrix3X3.Transform(ref down, ref o, out down);
            //Matrix3X3.Transform(ref up, ref o, out up);
            //Matrix3X3.Transform(ref forward, ref o, out forward);
            //Matrix3X3.Transform(ref backward, ref o, out backward);


            //boundingBox.Max.X = shapeTransform.Position.X + right.X;
            //boundingBox.Max.Y = shapeTransform.Position.Y + up.Y;
            //boundingBox.Max.Z = shapeTransform.Position.Z + backward.Z;

            //boundingBox.Min.X = shapeTransform.Position.X + left.X;
            //boundingBox.Min.Y = shapeTransform.Position.Y + down.Y;
            //boundingBox.Min.Z = shapeTransform.Position.Z + forward.Z;


#if !WINDOWS
            boundingBox = new BoundingBox();
#endif
            Matrix3X3 o;
            Matrix3X3.CreateFromQuaternion(ref shapeTransform.Orientation, out o);
            //Sample the local directions from the orientation matrix, implicitly transposed.

            Vector3 rightDirection = new Vector3(o.M11, o.M21, o.M31);
            Vector3 upDirection = new Vector3(o.M12, o.M22, o.M32);
            Vector3 backDirection = new Vector3(o.M13, o.M23, o.M33);

            int right = 0, left = 0, up = 0, down = 0, backward = 0, forward = 0;
            float minX = float.MaxValue, maxX = -float.MaxValue, minY = float.MaxValue, maxY = -float.MaxValue, minZ = float.MaxValue, maxZ = -float.MaxValue;

            for (int i = 0; i < surfaceVertices.count; i++)
            {
                float dotX, dotY, dotZ;
                Vector3.Dot(ref rightDirection, ref surfaceVertices.Elements[i], out dotX);
                Vector3.Dot(ref upDirection, ref surfaceVertices.Elements[i], out dotY);
                Vector3.Dot(ref backDirection, ref surfaceVertices.Elements[i], out dotZ);
                if (dotX < minX)
                {
                    minX = dotX;
                    left = i;
                }
                if (dotX > maxX)
                {
                    maxX = dotX;
                    right = i;
                }

                if (dotY < minY)
                {
                    minY = dotY;
                    down = i;
                }
                if (dotY > maxY)
                {
                    maxY = dotY;
                    up = i;
                }

                if (dotZ < minZ)
                {
                    minZ = dotZ;
                    forward = i;
                }
                if (dotZ > maxZ)
                {
                    maxZ = dotZ;
                    backward = i;
                }

            }


            Vector3 vMinX, vMaxX, vMinY, vMaxY, vMinZ, vMaxZ;
            Matrix3X3.Transform(ref surfaceVertices.Elements[right], ref o, out vMaxX);
            Matrix3X3.Transform(ref surfaceVertices.Elements[left], ref o, out vMinX);
            Matrix3X3.Transform(ref surfaceVertices.Elements[up], ref o, out vMaxY);
            Matrix3X3.Transform(ref surfaceVertices.Elements[down], ref o, out vMinY);
            Matrix3X3.Transform(ref surfaceVertices.Elements[backward], ref o, out vMaxZ);
            Matrix3X3.Transform(ref surfaceVertices.Elements[forward], ref o, out vMinZ);


            boundingBox.Max.X = shapeTransform.Position.X + vMaxX.X;
            boundingBox.Max.Y = shapeTransform.Position.Y + vMaxY.Y;
            boundingBox.Max.Z = shapeTransform.Position.Z + vMaxZ.Z;

            boundingBox.Min.X = shapeTransform.Position.X + vMinX.X;
            boundingBox.Min.Y = shapeTransform.Position.Y + vMinY.Y;
            boundingBox.Min.Z = shapeTransform.Position.Z + vMinZ.Z;

        }

        public override float ComputeVolume()
        {
            throw new System.NotImplementedException();
        }

        public override Matrix3X3 ComputeVolumeDistribution(out float volume)
        {
            throw new System.NotImplementedException();
        }

        public override Matrix3X3 ComputeVolumeDistribution()
        {
            throw new System.NotImplementedException();
        }

        public override Vector3 ComputeCenter()
        {
            throw new System.NotImplementedException();
        }

        public override Vector3 ComputeCenter(out float volume)
        {
            throw new System.NotImplementedException();
        }

        public override void ComputeDistributionInformation(out ShapeDistributionInformation shapeInfo)
        {
            throw new System.NotImplementedException();
        }

        public override Collidables.MobileCollidables.EntityCollidable GetMobileInstance()
        {
            return new MobileMeshCollidable(this);
        }
    }

    ///<summary>
    /// Sidedness of a triangle or mesh.
    /// A triangle can be double sided, or allow one of its sides to let interacting objects through.
    ///</summary>
    public enum MobileMeshSolidity
    {
        /// <summary>
        /// The mesh will interact with objects coming from both directions.
        /// </summary>
        DoubleSided,
        /// <summary>
        /// The mesh will interact with objects from which the winding of the triangles appears to be clockwise.
        /// </summary>
        Clockwise,
        /// <summary>
        /// The mesh will interact with objects from which the winding of the triangles appears to be counterclockwise.
        /// </summary>
        Counterclockwise,
        /// <summary>
        /// The mesh will treat objects inside of its concave shell as if the mesh had volume.
        /// </summary>
        Solid
    }
}
