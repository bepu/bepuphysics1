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
    /// Manages persistent contacts between a static mesh and a convex.
    ///</summary>
    public abstract class StaticMeshContactManifold : TriangleMeshConvexContactManifold
    {


        protected StaticMesh mesh;

        internal RawList<int> overlappedTriangles = new RawList<int>(4);

        ///<summary>
        /// Gets the static mesh associated with this pair.
        ///</summary>
        public StaticMesh Mesh
        {
            get
            {
                return mesh;
            }
        }

        protected internal override int FindOverlappingTriangles(float dt)
        {
            mesh.Mesh.Tree.GetOverlaps(convex.boundingBox, overlappedTriangles);
            return overlappedTriangles.Count;
        }

        /// <summary>
        /// Precomputes the transform to bring triangles from their native local space to the local space of the convex.
        /// </summary>
        /// <param name="convexInverseWorldTransform">Inverse of the world transform of the convex shape.</param>
        /// <param name="fromMeshLocalToConvexLocal">Transform to apply to native local triangles to bring them into the local space of the convex.</param>
        protected override void PrecomputeTriangleTransform(ref AffineTransform convexInverseWorldTransform, out AffineTransform fromMeshLocalToConvexLocal)
        {
            //StaticMeshes only have transformable mesh data.
            var data = ((TransformableMeshData) mesh.Mesh.Data);
            AffineTransform.Multiply(ref data.worldTransform, ref convexInverseWorldTransform, out fromMeshLocalToConvexLocal);
        }

        protected override bool ConfigureLocalTriangle(int i, TriangleShape localTriangleShape, out TriangleIndices indices)
        {
            int triangleIndex = overlappedTriangles.Elements[i];
            var data = mesh.Mesh.Data;
            localTriangleShape.vA = data.vertices[data.indices[triangleIndex]];
            localTriangleShape.vB = data.vertices[data.indices[triangleIndex + 1]];
            localTriangleShape.vC = data.vertices[data.indices[triangleIndex + 2]];
            //TODO: Note the IsQuery hack to avoid missing contacts. Avoid doing this in v2.
            localTriangleShape.sidedness = IsQuery ? TriangleSidedness.DoubleSided : mesh.sidedness;
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
            mesh = newCollidableB as StaticMesh;


            if (convex == null || mesh == null)
            {
                convex = newCollidableB as ConvexCollidable;
                mesh = newCollidableA as StaticMesh;
                if (convex == null || mesh == null)
                    throw new ArgumentException("Inappropriate types used to initialize contact manifold.");
            }

        }


    }
}
