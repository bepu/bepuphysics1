using System;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using Microsoft.Xna.Framework;
using BEPUphysics.DataStructures;
using BEPUphysics.MathExtensions;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionShapes;

namespace BEPUphysics.CollisionTests.Manifolds
{
    ///<summary>
    /// Manages persistent contacts between a convex and an instanced mesh.
    ///</summary>
    public class MobileMeshConvexContactManifold : TriangleMeshConvexContactManifold
    {
        protected MobileMeshCollidable mesh;

        internal RawList<int> overlappedTriangles = new RawList<int>(4);

        ///<summary>
        /// Gets the mesh of the pair.
        ///</summary>
        public MobileMeshCollidable Mesh
        {
            get
            {
                return mesh;
            }
        }

        protected override RigidTransform MeshTransform
        {
            get { return mesh.worldTransform; }
        }

        protected internal override int FindOverlappingTriangles(float dt)
        {
            BoundingBox boundingBox;
            AffineTransform transform = new AffineTransform(mesh.worldTransform.Orientation, mesh.worldTransform.Position);
            convex.Shape.GetLocalBoundingBox(ref convex.worldTransform, ref transform, out boundingBox); 
            Vector3 transformedVelocity;
            //Compute the relative velocity with respect to the mesh.  The mesh's bounding tree is NOT expanded with velocity,
            //so whatever motion there is between the two objects needs to be included in the convex's bounding box.
            Vector3.Subtract(ref convex.entity.linearVelocity, ref mesh.entity.linearVelocity, out transformedVelocity);
            //The linear transform is known to be orientation only, so using the transpose is allowed.
            Matrix3X3.TransformTranspose(ref transformedVelocity, ref transform.LinearTransform, out transformedVelocity);
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
            AffineTransform transform;
            AffineTransform.CreateFromRigidTransform(ref mesh.worldTransform, out transform);
            AffineTransform.Transform(ref localTriangleShape.vA, ref transform, out localTriangleShape.vA);
            AffineTransform.Transform(ref localTriangleShape.vB, ref transform, out localTriangleShape.vB);
            AffineTransform.Transform(ref localTriangleShape.vC, ref transform, out localTriangleShape.vC);
            TriangleSidedness sidedness;
            switch (mesh.Shape.solidity)
            {
                case MobileMeshSolidity.Clockwise:
                    sidedness = TriangleSidedness.Clockwise;
                    break;
                case MobileMeshSolidity.Counterclockwise:
                    sidedness = TriangleSidedness.Counterclockwise;
                    break;
                case MobileMeshSolidity.DoubleSided:
                    sidedness = TriangleSidedness.DoubleSided;
                    break;
                default:
                    sidedness = mesh.Shape.solidSidedness;
                    break;
            }
            localTriangleShape.sidedness = sidedness;
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
            mesh = newCollidableB as MobileMeshCollidable;


            if (convex == null || mesh == null)
            {
                convex = newCollidableB as ConvexCollidable;
                mesh = newCollidableA as MobileMeshCollidable;
                if (convex == null || mesh == null)
                    throw new Exception("Inappropriate types used to initialize contact manifold.");
            }

        }


    }
}
