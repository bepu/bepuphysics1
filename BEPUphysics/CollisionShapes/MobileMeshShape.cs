
using System.Collections.Generic;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.DataStructures;
using BEPUutilities;
using System;
using BEPUphysics.Settings;
using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;

namespace BEPUphysics.CollisionShapes
{
    ///<summary>
    /// Local space data associated with a mobile mesh.
    /// This contains a hierarchy and all the other heavy data needed
    /// by an MobileMesh.
    ///</summary>
    public class MobileMeshShape : EntityShape
    {
        private float meshCollisionMargin = CollisionDetectionSettings.DefaultMargin;
        /// <summary>
        /// Gets or sets the margin of the mobile mesh to use when colliding with other meshes.
        /// When colliding with non-mesh shapes, the mobile mesh has no margin.
        /// </summary>
        public float MeshCollisionMargin
        {
            get
            {
                return meshCollisionMargin;
            }
            set
            {
                if (value < 0)
                    throw new ArgumentException("Mesh margin must be nonnegative.");
                meshCollisionMargin = value;
                OnShapeChanged();
            }
        }
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

        /// <summary>
        /// Gets the transform used by the local mesh shape.
        /// </summary>
        public AffineTransform Transform
        {
            get
            {
                return ((TransformableMeshData)triangleMesh.Data).worldTransform;
            }
        }

        RawList<Vector3> hullVertices = new RawList<Vector3>();

        /// <summary>
        /// Gets the list of vertices on the convex hull of the mesh used to compute the bounding box.
        /// </summary>
        public ReadOnlyList<Vector3> HullVertices
        {
            get { return new ReadOnlyList<Vector3>(hullVertices); }
        }

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

        /// <summary>
        /// Gets or sets the sidedness of the shape.  This is a convenience property based on the Solidity property.
        /// If the shape is solid, this returns whatever sidedness is computed to make the triangles of the shape face outward.
        /// If the shape is solid, setting this property will change the sidedness that is used while the shape is solid.
        /// </summary>
        public TriangleSidedness Sidedness
        {
            get
            {
                switch (solidity)
                {
                    case MobileMeshSolidity.Clockwise:
                        return TriangleSidedness.Clockwise;
                    case MobileMeshSolidity.Counterclockwise:
                        return TriangleSidedness.Counterclockwise;
                    case MobileMeshSolidity.DoubleSided:
                        return TriangleSidedness.DoubleSided;
                    case MobileMeshSolidity.Solid:
                        return SidednessWhenSolid;

                }
                return TriangleSidedness.DoubleSided;
            }
            set
            {
                if (solidity == MobileMeshSolidity.Solid)
                    SidednessWhenSolid = value;
                else
                {
                    switch (value)
                    {
                        case TriangleSidedness.Clockwise:
                            solidity = MobileMeshSolidity.Clockwise;
                            break;
                        case TriangleSidedness.Counterclockwise:
                            solidity = MobileMeshSolidity.Counterclockwise;
                            break;
                        case TriangleSidedness.DoubleSided:
                            solidity = MobileMeshSolidity.DoubleSided;
                            break;
                    }
                }
            }
        }

        ///<summary>
        /// Constructs a new mobile mesh shape.
        ///</summary>
        ///<param name="vertices">Vertices of the mesh.</param>
        ///<param name="indices">Indices of the mesh.</param>
        ///<param name="localTransform">Local transform to apply to the shape.</param>
        ///<param name="solidity">Solidity state of the shape.</param>
        public MobileMeshShape(Vector3[] vertices, int[] indices, AffineTransform localTransform, MobileMeshSolidity solidity)
        {
            this.solidity = solidity;
            var data = new TransformableMeshData(vertices, indices, localTransform);
            var shapeDistributionInformation = ComputeVolumeDistribution(data);
            data.worldTransform.Translation -= shapeDistributionInformation.Center;

            triangleMesh = new TriangleMesh(data);

            UpdateEntityShapeVolume(new EntityShapeVolumeDescription { Volume = shapeDistributionInformation.Volume, VolumeDistribution = shapeDistributionInformation.VolumeDistribution });

            ComputeSolidSidedness();

            UpdateSurfaceVertices();

        }

        ///<summary>
        /// Constructs a new mobile mesh shape.
        ///</summary>
        ///<param name="vertices">Vertices of the mesh.</param>
        ///<param name="indices">Indices of the mesh.</param>
        ///<param name="localTransform">Local transform to apply to the shape.</param>
        ///<param name="solidity">Solidity state of the shape.</param>
        /// <param name="center">Center of the shape.</param>
        public MobileMeshShape(Vector3[] vertices, int[] indices, AffineTransform localTransform, MobileMeshSolidity solidity, out Vector3 center)
        {
            this.solidity = solidity;
            var data = new TransformableMeshData(vertices, indices, localTransform);
            var shapeDistributionInformation = ComputeVolumeDistribution(data);
            data.worldTransform.Translation -= shapeDistributionInformation.Center;
            center = shapeDistributionInformation.Center;

            triangleMesh = new TriangleMesh(data);

            UpdateEntityShapeVolume(new EntityShapeVolumeDescription { Volume = shapeDistributionInformation.Volume, VolumeDistribution = shapeDistributionInformation.VolumeDistribution });

            ComputeSolidSidedness();

            UpdateSurfaceVertices();

        }

        ///<summary>
        /// Constructs a new mobile mesh shape from cached data.
        ///</summary>
        ///<param name="meshData">Mesh data reprsenting the shape. Should already be properly centered.</param>
        /// <param name="hullVertices">Outer hull vertices of the mobile mesh shape used to quickly compute the bounding box.</param>
        ///<param name="solidity">Solidity state of the shape.</param>
        /// <param name="sidednessWhenSolid">Triangle sidedness to use when the shape is solid.</param>
        /// <param name="collisionMargin">Collision margin used to expand the mesh triangles.</param>
        /// <param name="volumeDescription">Description of the volume and its distribution in the shape. Assumed to be correct; no processing or validation is performed.</param>
        public MobileMeshShape(TransformableMeshData meshData, IList<Vector3> hullVertices, MobileMeshSolidity solidity, TriangleSidedness sidednessWhenSolid, float collisionMargin, EntityShapeVolumeDescription volumeDescription)
        {
            triangleMesh = new TriangleMesh(meshData);
            this.hullVertices = new RawList<Vector3>(hullVertices);
            meshCollisionMargin = collisionMargin;
            this.solidity = solidity;
            SidednessWhenSolid = sidednessWhenSolid;

            UpdateEntityShapeVolume(volumeDescription);
        }


        /// <summary>
        /// Gets the triangle sidedness required if the mesh is in solid mode.
        /// If the windings were reversed or double sided,
        /// the solidity would fight against shell contacts,
        /// leading to very bad jittering.
        /// </summary>
        public TriangleSidedness SidednessWhenSolid { get; private set; }


        /// <summary>
        /// Tests to see if a ray's origin is contained within the mesh.
        /// If it is, the hit location is found.
        /// If it isn't, the hit location is still valid if a hit occurred.
        /// If the origin isn't inside and there was no hit, the hit has a T value of float.MaxValue.
        /// </summary>
        /// <param name="ray">Ray in the local space of the shape to test.</param>
        /// <param name="hit">The first hit against the mesh, if any.</param>
        /// <returns>Whether or not the ray origin was in the mesh.</returns>
        public bool IsLocalRayOriginInMesh(ref Ray ray, out RayHit hit)
        {
            var overlapList = CommonResources.GetIntList();
            hit = new RayHit();
            hit.T = float.MaxValue;
            if (triangleMesh.Tree.GetOverlaps(ray, overlapList))
            {
                bool minimumClockwise = false;
                for (int i = 0; i < overlapList.Count; i++)
                {
                    Vector3 vA, vB, vC;
                    triangleMesh.Data.GetTriangle(overlapList[i], out vA, out vB, out vC);
                    bool hitClockwise;
                    RayHit tempHit;
                    if (Toolbox.FindRayTriangleIntersection(ref ray, float.MaxValue, ref vA, ref vB, ref vC, out hitClockwise, out tempHit) &&
                        tempHit.T < hit.T)
                    {
                        hit = tempHit;
                        minimumClockwise = hitClockwise;
                    }
                }
                CommonResources.GiveBack(overlapList);

                //If the mesh is hit from behind by the ray on the first hit, then the ray is inside.
                return hit.T < float.MaxValue && ((SidednessWhenSolid == TriangleSidedness.Clockwise && !minimumClockwise) || (SidednessWhenSolid == TriangleSidedness.Counterclockwise && minimumClockwise));
            }
            CommonResources.GiveBack(overlapList);
            return false;

        }

        /// <summary>
        /// The difference in t parameters in a ray cast under which two hits are considered to be redundant.
        /// </summary>
        public static float MeshHitUniquenessThreshold = .0001f;

        internal bool IsHitUnique(RawList<RayHit> hits, ref RayHit hit)
        {
            for (int i = 0; i < hits.Count; i++)
            {
                if (Math.Abs(hits.Elements[i].T - hit.T) < MeshHitUniquenessThreshold)
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
            var ray = new Ray();
            Vector3 vA, vB, vC;
            triangleMesh.Data.GetTriangle(((triangleMesh.Data.indices.Length / 3) / 2) * 3, out vA, out vB, out vC);
            ray.Direction = (vA + vB + vC) / 3;
            ray.Direction.Normalize();

            SidednessWhenSolid = ComputeSolidSidednessHelper(ray);
            //ComputeSolidSidednessHelper is separated into another function just in case multiple queries were desired for validation.
            //If multiple rays returned different sidednesses, the shape would be inconsistent.

        }

        TriangleSidedness ComputeSolidSidednessHelper(Ray ray)
        {
            TriangleSidedness toReturn;
            var hitList = CommonResources.GetIntList();
            if (triangleMesh.Tree.GetOverlaps(ray, hitList))
            {
                Vector3 vA, vB, vC;
                var hits = CommonResources.GetRayHitList();
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
                            minimum = hitList[i];
                        }
                        if (hit.T > maximumT)
                        {
                            maximumT = hit.T;
                            maximum = hitList[i];
                        }
                    }
                }

                if (hits.Count % 2 == 0)
                {
                    //Since we were outside, the first hit triangle should be calibrated
                    //such that it faces towards us.

                    triangleMesh.Data.GetTriangle(minimum, out vA, out vB, out vC);
                    var normal = Vector3.Cross(vA - vB, vA - vC);
                    if (Vector3.Dot(normal, ray.Direction) < 0)
                        toReturn = TriangleSidedness.Clockwise;
                    else
                        toReturn = TriangleSidedness.Counterclockwise;
                }
                else
                {
                    //Since we were inside, the last hit triangle should be calibrated
                    //such that it faces away from us.

                    triangleMesh.Data.GetTriangle(maximum, out vA, out vB, out vC);
                    var normal = Vector3.Cross(vA - vB, vA - vC);
                    if (Vector3.Dot(normal, ray.Direction) < 0)
                        toReturn = TriangleSidedness.Counterclockwise;
                    else
                        toReturn = TriangleSidedness.Clockwise;
                }

                CommonResources.GiveBack(hits);

            }
            else
                toReturn = TriangleSidedness.DoubleSided; //This is a problem...
            CommonResources.GiveBack(hitList);
            return toReturn;
        }

        private void UpdateSurfaceVertices()
        {
            hullVertices.Clear();
            if (Volume > 0)
            {
                ConvexHullHelper.GetConvexHull(triangleMesh.Data.vertices, hullVertices);
                var transformableData = triangleMesh.Data as TransformableMeshData;
                if (transformableData != null)
                {
                    var transform = transformableData.worldTransform;
                    for (int i = 0; i < hullVertices.Count; i++)
                    {
                        AffineTransform.Transform(ref hullVertices.Elements[i], ref transform, out hullVertices.Elements[i]);
                    }
                }
            }
            else
            {
                hullVertices.Clear();
                //A mobile mesh is allowed to have zero volume, so long as it isn't solid.
                //In this case, compute the bounding box of all points.
                BoundingBox box = new BoundingBox();
                for (int i = 0; i < triangleMesh.Data.vertices.Length; i++)
                {
                    Vector3 v;
                    triangleMesh.Data.GetVertexPosition(i, out v);
                    if (v.X > box.Max.X)
                        box.Max.X = v.X;
                    if (v.X < box.Min.X)
                        box.Min.X = v.X;
                    if (v.Y > box.Max.Y)
                        box.Max.Y = v.Y;
                    if (v.Y < box.Min.Y)
                        box.Min.Y = v.Y;
                    if (v.Z > box.Max.Z)
                        box.Max.Z = v.Z;
                    if (v.Z < box.Min.Z)
                        box.Min.Z = v.Z;
                }
                //Add the corners.  This will overestimate the size of the surface a bit.
                hullVertices.Add(box.Min);
                hullVertices.Add(box.Max);
                hullVertices.Add(new Vector3(box.Min.X, box.Min.Y, box.Max.Z));
                hullVertices.Add(new Vector3(box.Min.X, box.Max.Y, box.Min.Z));
                hullVertices.Add(new Vector3(box.Max.X, box.Min.Y, box.Min.Z));
                hullVertices.Add(new Vector3(box.Min.X, box.Max.Y, box.Max.Z));
                hullVertices.Add(new Vector3(box.Max.X, box.Max.Y, box.Min.Z));
                hullVertices.Add(new Vector3(box.Max.X, box.Min.Y, box.Max.Z));
            }
        }

        /// <summary>
        /// Recenters the triangle data and computes the volume distribution.
        /// </summary>
        /// <param name="data">Mesh data to analyze.</param>
        /// <returns>Computed center, volume, and volume distribution.</returns>
        private ShapeDistributionInformation ComputeVolumeDistribution(TransformableMeshData data)
        {
            //Compute the surface vertices of the shape.
            ShapeDistributionInformation shapeInformation;
            if (solidity == MobileMeshSolidity.Solid)
            {

                //The following inertia tensor calculation assumes a closed mesh.
                var transformedVertices = CommonResources.GetVectorList();
                if (transformedVertices.Capacity < data.vertices.Length)
                    transformedVertices.Capacity = data.vertices.Length;
                transformedVertices.Count = data.vertices.Length;
                for (int i = 0; i < data.vertices.Length; ++i)
                {
                    data.GetVertexPosition(i, out transformedVertices.Elements[i]);
                }
                InertiaHelper.ComputeShapeDistribution(transformedVertices, data.indices, out shapeInformation.Center, out shapeInformation.Volume, out shapeInformation.VolumeDistribution);
                CommonResources.GiveBack(transformedVertices);
                if (shapeInformation.Volume > 0)
                    return shapeInformation;
                throw new ArgumentException("A solid mesh must have volume.");
            }
            shapeInformation.Center = new Vector3();
            shapeInformation.VolumeDistribution = new Matrix3x3();
            float totalWeight = 0;
            for (int i = 0; i < data.indices.Length; i += 3)
            {
                //Compute the center contribution.
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

                float perVertexWeight = weight * (1f / 3f);
                shapeInformation.Center += perVertexWeight * (vA + vB + vC);

                //Compute the inertia contribution of this triangle.
                //Approximate it using pointmasses positioned at the triangle vertices.
                //(There exists a direct solution, but this approximation will do plenty fine.)
                Matrix3x3 aContribution, bContribution, cContribution;
                InertiaHelper.GetPointContribution(perVertexWeight, ref Toolbox.ZeroVector, ref vA, out aContribution);
                InertiaHelper.GetPointContribution(perVertexWeight, ref Toolbox.ZeroVector, ref vB, out bContribution);
                InertiaHelper.GetPointContribution(perVertexWeight, ref Toolbox.ZeroVector, ref vC, out cContribution);
                Matrix3x3.Add(ref aContribution, ref shapeInformation.VolumeDistribution, out shapeInformation.VolumeDistribution);
                Matrix3x3.Add(ref bContribution, ref shapeInformation.VolumeDistribution, out shapeInformation.VolumeDistribution);
                Matrix3x3.Add(ref cContribution, ref shapeInformation.VolumeDistribution, out shapeInformation.VolumeDistribution);


            }
            shapeInformation.Center /= totalWeight;

            //The extra factor of 2 is used because the cross product length was twice the actual area.
            Matrix3x3.Multiply(ref shapeInformation.VolumeDistribution, 1 / (2 * totalWeight), out shapeInformation.VolumeDistribution);

            //Move the inertia tensor into position according to the center.
            Matrix3x3 additionalInertia;
            InertiaHelper.GetPointContribution(0.5f, ref Toolbox.ZeroVector, ref shapeInformation.Center, out additionalInertia);
            Matrix3x3.Subtract(ref shapeInformation.VolumeDistribution, ref additionalInertia, out shapeInformation.VolumeDistribution);

            shapeInformation.Volume = 0;


            return shapeInformation;
        }


        private void GetBoundingBox(ref Matrix3x3 o, out BoundingBox boundingBox)
        {
#if !WINDOWS
            boundingBox = new BoundingBox();
#endif
            //Sample the local directions from the matrix, implicitly transposed.
            var rightDirection = new Vector3(o.M11, o.M21, o.M31);
            var upDirection = new Vector3(o.M12, o.M22, o.M32);
            var backDirection = new Vector3(o.M13, o.M23, o.M33);

            int right = 0, left = 0, up = 0, down = 0, backward = 0, forward = 0;
            float minX = float.MaxValue, maxX = -float.MaxValue, minY = float.MaxValue, maxY = -float.MaxValue, minZ = float.MaxValue, maxZ = -float.MaxValue;

            for (int i = 0; i < hullVertices.Count; i++)
            {
                float dotX, dotY, dotZ;
                Vector3.Dot(ref rightDirection, ref hullVertices.Elements[i], out dotX);
                Vector3.Dot(ref upDirection, ref hullVertices.Elements[i], out dotY);
                Vector3.Dot(ref backDirection, ref hullVertices.Elements[i], out dotZ);
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

            //Incorporate the collision margin.
            Vector3.Multiply(ref rightDirection, meshCollisionMargin / (float)Math.Sqrt(rightDirection.Length()), out rightDirection);
            Vector3.Multiply(ref upDirection, meshCollisionMargin / (float)Math.Sqrt(upDirection.Length()), out upDirection);
            Vector3.Multiply(ref backDirection, meshCollisionMargin / (float)Math.Sqrt(backDirection.Length()), out backDirection);

            var rightElement = hullVertices.Elements[right];
            var leftElement = hullVertices.Elements[left];
            var upElement = hullVertices.Elements[up];
            var downElement = hullVertices.Elements[down];
            var backwardElement = hullVertices.Elements[backward];
            var forwardElement = hullVertices.Elements[forward];
            Vector3.Add(ref rightElement, ref rightDirection, out rightElement);
            Vector3.Subtract(ref leftElement, ref rightDirection, out leftElement);
            Vector3.Add(ref upElement, ref upDirection, out upElement);
            Vector3.Subtract(ref downElement, ref upDirection, out downElement);
            Vector3.Add(ref backwardElement, ref backDirection, out backwardElement);
            Vector3.Subtract(ref forwardElement, ref backDirection, out forwardElement);

            //Rather than transforming each axis independently (and doing three times as many operations as required), just get the 6 required values directly.
            TransformLocalExtremePoints(ref rightElement, ref upElement, ref backwardElement, ref o, out boundingBox.Max);
            TransformLocalExtremePoints(ref leftElement, ref downElement, ref forwardElement, ref o, out boundingBox.Min);
        }

        ///<summary>
        /// Computes the bounding box of the transformed mesh shape.
        ///</summary>
        ///<param name="shapeTransform">Transform to apply to the shape during the bounding box calculation.</param>
        ///<param name="boundingBox">Bounding box containing the transformed mesh shape.</param>
        public override void GetBoundingBox(ref RigidTransform shapeTransform, out BoundingBox boundingBox)
        {
            //TODO: Could use an approximate bounding volume.  Would be cheaper at runtime and use less memory, though the box would be bigger.
            Matrix3x3 o;
            Matrix3x3.CreateFromQuaternion(ref shapeTransform.Orientation, out o);
            GetBoundingBox(ref o, out boundingBox);

            Vector3.Add(ref boundingBox.Max, ref shapeTransform.Position, out boundingBox.Max);
            Vector3.Add(ref boundingBox.Min, ref shapeTransform.Position, out boundingBox.Min);

        }


        /// <summary>
        /// Gets the bounding box of the mesh transformed first into world space, and then into the local space of another affine transform.
        /// </summary>
        /// <param name="shapeTransform">Transform to use to put the shape into world space.</param>
        /// <param name="spaceTransform">Used as the frame of reference to compute the bounding box.
        /// In effect, the shape is transformed by the inverse of the space transform to compute its bounding box in local space.</param>
        /// <param name="boundingBox">Bounding box in the local space.</param>
        public void GetLocalBoundingBox(ref RigidTransform shapeTransform, ref AffineTransform spaceTransform, out BoundingBox boundingBox)
        {
#if !WINDOWS
            boundingBox = new BoundingBox();
#endif
            //TODO: This method peforms quite a few sqrts because the collision margin can get scaled, and so cannot be applied as a final step.
            //There should be a better way to do this.
            //Additionally, this bounding box is not consistent in all cases with the post-add version.  Adding the collision margin at the end can
            //slightly overestimate the size of a margin expanded shape at the corners, which is fine (and actually important for the box-box special case).

            //Move forward into convex's space, backwards into the new space's local space.
            AffineTransform transform;
            AffineTransform.Invert(ref spaceTransform, out transform);
            AffineTransform.Multiply(ref shapeTransform, ref transform, out transform);

            GetBoundingBox(ref transform.LinearTransform, out boundingBox);
            boundingBox.Max.X += transform.Translation.X;
            boundingBox.Max.Y += transform.Translation.Y;
            boundingBox.Max.Z += transform.Translation.Z;

            boundingBox.Min.X += transform.Translation.X;
            boundingBox.Min.Y += transform.Translation.Y;
            boundingBox.Min.Z += transform.Translation.Z;

        }

        /// <summary>
        /// Gets the bounding box of the mesh transformed first into world space, and then into the local space of another affine transform.
        /// </summary>
        /// <param name="shapeTransform">Transform to use to put the shape into world space.</param>
        /// <param name="spaceTransform">Used as the frame of reference to compute the bounding box.
        /// In effect, the shape is transformed by the inverse of the space transform to compute its bounding box in local space.</param>
        /// <param name="sweep">World space sweep direction to transform and add to the bounding box.</param>
        /// <param name="boundingBox">Bounding box in the local space.</param>
        public void GetSweptLocalBoundingBox(ref RigidTransform shapeTransform, ref AffineTransform spaceTransform, ref Vector3 sweep, out BoundingBox boundingBox)
        {
            GetLocalBoundingBox(ref shapeTransform, ref spaceTransform, out boundingBox);
            Vector3 expansion;
            Matrix3x3.TransformTranspose(ref sweep, ref spaceTransform.LinearTransform, out expansion);
            Toolbox.ExpandBoundingBox(ref boundingBox, ref expansion);
        }



        public override EntityCollidable GetCollidableInstance()
        {
            return new MobileMeshCollidable(this);
        }




    }

    ///<summary>
    /// Solidity of a triangle or mesh.
    /// A triangle can be double sided, or allow one of its sides to let interacting objects through.
    /// The entire mesh can be made solid, which means objects on the interior still generate contacts even if there aren't any triangles to hit.
    /// Solidity requires the mesh to be closed.
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
        /// The mesh will treat objects inside of its concave shell as if the mesh had volume.  Mesh must be closed for this to work properly.
        /// </summary>
        Solid
    }
}
