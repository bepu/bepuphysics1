using System;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using Microsoft.Xna.Framework;
using BEPUphysics.DataStructures;
using BEPUphysics.MathExtensions;

namespace BEPUphysics.CollisionTests.Manifolds
{
    ///<summary>
    /// Manages persistent contacts between a convex and an instanced mesh.
    ///</summary>
    public class InstancedMeshConvexContactManifold : TriangleMeshConvexContactManifold
    {
        protected InstancedMesh mesh;

        internal RawList<int> overlappedTriangles = new RawList<int>(4);

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
            Vector3 transformedVelocity;
            Matrix3X3 inverse;
            Matrix3X3.Invert(ref mesh.worldTransform.LinearTransform, out inverse);
            Matrix3X3.Transform(ref convex.entity.linearVelocity, ref inverse, out transformedVelocity);
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

            mesh.Shape.TriangleMesh.Tree.GetOverlaps(boundingBox, overlappedTriangles);
            return overlappedTriangles.count;
        }

        protected override void ConfigureTriangle(int i, out TriangleIndices indices)
        {
            MeshBoundingBoxTreeData data = mesh.Shape.TriangleMesh.Data;
            int triangleIndex = overlappedTriangles.Elements[i];
            data.GetTriangle(triangleIndex, out localTriangleShape.vA, out localTriangleShape.vB, out localTriangleShape.vC);
            AffineTransform.Transform(ref localTriangleShape.vA, ref mesh.worldTransform, out localTriangleShape.vA);
            AffineTransform.Transform(ref localTriangleShape.vB, ref mesh.worldTransform, out localTriangleShape.vB);
            AffineTransform.Transform(ref localTriangleShape.vC, ref mesh.worldTransform, out localTriangleShape.vC);
            localTriangleShape.sidedness = mesh.sidedness;
            localTriangleShape.collisionMargin = 0;
            indices = new TriangleIndices();
            indices.A = data.indices[triangleIndex];
            indices.B = data.indices[triangleIndex + 1];
            indices.C = data.indices[triangleIndex + 2];
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
                    throw new Exception("Inappropriate types used to initialize contact manifold.");
            }

        }


    }
}
