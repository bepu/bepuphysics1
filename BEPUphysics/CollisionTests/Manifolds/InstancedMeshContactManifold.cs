using System;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.DataStructures;
using BEPUutilities;
using BEPUutilities.DataStructures;

namespace BEPUphysics.CollisionTests.Manifolds
{
    ///<summary>
    /// Manages persistent contacts between a convex and an instanced mesh.
    ///</summary>
    public abstract class InstancedMeshContactManifold : TriangleMeshConvexContactManifold
    {
        protected InstancedMesh mesh;

        internal RawList<int> overlappedTriangles = new RawList<int>(8);

        ///<summary>
        /// Gets the mesh of the pair.
        ///</summary>
        public InstancedMesh Mesh
        {
            get
            {
                return mesh;
            }
        }

        protected internal override int FindOverlappingTriangles(float dt)
        {
            BoundingBox boundingBox;
            convex.Shape.GetLocalBoundingBox(ref convex.worldTransform, ref mesh.worldTransform, out boundingBox);
            if (convex.entity != null)
            {
                Vector3 transformedVelocity;
                Matrix3x3 inverse;
                Matrix3x3.Invert(ref mesh.worldTransform.LinearTransform, out inverse);
                Matrix3x3.Transform(ref convex.entity.linearVelocity, ref inverse, out transformedVelocity);
                Vector3.Multiply(ref transformedVelocity, dt, out transformedVelocity);

                if (transformedVelocity.X > 0)
                    boundingBox.Max.X += transformedVelocity.X;
                else
                    boundingBox.Min.X += transformedVelocity.X;

                if (transformedVelocity.Y > 0)
                    boundingBox.Max.Y += transformedVelocity.Y;
                else
                    boundingBox.Min.Y += transformedVelocity.Y;

                if (transformedVelocity.Z > 0)
                    boundingBox.Max.Z += transformedVelocity.Z;
                else
                    boundingBox.Min.Z += transformedVelocity.Z;
            }

            mesh.Shape.TriangleMesh.Tree.GetOverlaps(boundingBox, overlappedTriangles);
            return overlappedTriangles.Count;
        }

        /// <summary>
        /// Precomputes the transform to bring triangles from their native local space to the local space of the convex.
        /// </summary>
        /// <param name="convexInverseWorldTransform">Inverse of the world transform of the convex shape.</param>
        /// <param name="fromMeshLocalToConvexLocal">Transform to apply to native local triangles to bring them into the local space of the convex.</param>
        protected override void PrecomputeTriangleTransform(ref AffineTransform convexInverseWorldTransform, out AffineTransform fromMeshLocalToConvexLocal)
        {
            var data = ((TransformableMeshData)mesh.Shape.TriangleMesh.Data);
            //The mobile mesh has a shape-based transform followed by the instance transform.
            AffineTransform combinedMobileMeshWorldTransform;
            AffineTransform.Multiply(ref data.worldTransform, ref mesh.worldTransform, out combinedMobileMeshWorldTransform);
            AffineTransform.Multiply(ref combinedMobileMeshWorldTransform, ref convexInverseWorldTransform, out fromMeshLocalToConvexLocal);
        }

        protected override bool ConfigureLocalTriangle(int i, TriangleShape localTriangleShape, out TriangleIndices indices)
        {
            MeshBoundingBoxTreeData data = mesh.Shape.TriangleMesh.Data;
            int triangleIndex = overlappedTriangles.Elements[i];
            localTriangleShape.vA = data.vertices[data.indices[triangleIndex]];
            localTriangleShape.vB = data.vertices[data.indices[triangleIndex + 1]];
            localTriangleShape.vC = data.vertices[data.indices[triangleIndex + 2]];
            //In instanced meshes, the bounding box we found in local space could collect more triangles than strictly necessary.
            //By doing a second pass, we should be able to prune out quite a few of them.
            BoundingBox triangleAABB;
            Toolbox.GetTriangleBoundingBox(ref localTriangleShape.vA, ref localTriangleShape.vB, ref localTriangleShape.vC, out triangleAABB);
            bool toReturn;
            triangleAABB.Intersects(ref convex.boundingBox, out toReturn);
            if (!toReturn)
            {
                indices = new TriangleIndices();
                return false;
            }

            localTriangleShape.sidedness = mesh.sidedness;
            localTriangleShape.collisionMargin = 0;
            indices = new TriangleIndices
            {
                A = data.indices[triangleIndex],
                B = data.indices[triangleIndex + 1],
                C = data.indices[triangleIndex + 2]
            };

            return true;
        }

        protected internal override void CleanUpOverlappingTriangles()
        {
            overlappedTriangles.Clear();
        }

        protected override bool UseImprovedBoundaryHandling
        {
            get { return mesh.improveBoundaryBehavior; }
        }


        ///<summary>
        /// Cleans up the manifold.
        ///</summary>
        public override void CleanUp()
        {
            mesh = null;
            convex = null;
            base.CleanUp();
        }

        ///<summary>
        /// Initializes the manifold.
        ///</summary>
        ///<param name="newCollidableA">First collidable.</param>
        ///<param name="newCollidableB">Second collidable.</param>
        public override void Initialize(Collidable newCollidableA, Collidable newCollidableB)
        {
            convex = newCollidableA as ConvexCollidable;
            mesh = newCollidableB as InstancedMesh;


            if (convex == null || mesh == null)
            {
                convex = newCollidableB as ConvexCollidable;
                mesh = newCollidableA as InstancedMesh;
                if (convex == null || mesh == null)
                    throw new ArgumentException("Inappropriate types used to initialize contact manifold.");
            }

        }


    }
}
