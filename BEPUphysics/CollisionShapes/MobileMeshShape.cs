using Microsoft.Xna.Framework;
using BEPUphysics.DataStructures;
using BEPUphysics.MathExtensions;

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
            set
            {
                triangleMesh = value;
                OnShapeChanged();
            }
        }



        ///<summary>
        /// Constructs a new instanced mesh shape.
        ///</summary>
        ///<param name="vertices">Vertices of the mesh.</param>
        ///<param name="indices">Indices of the mesh.</param>
        public MobileMeshShape(Vector3[] vertices, int[] indices)
        {
            TriangleMesh = new TriangleMesh(new StaticMeshData(vertices, indices));
        }



        ///<summary>
        /// Computes the bounding box of the transformed mesh shape.
        ///</summary>
        ///<param name="transform">Transform to apply to the shape during the bounding box calculation.</param>
        ///<param name="boundingBox">Bounding box containing the transformed mesh shape.</param>
        public void GetBoundingBox(ref RigidTransform transform, out BoundingBox boundingBox)
        {
            //TODO: Use an approximate convex hull to create the AABB.
            boundingBox = new BoundingBox();


        }

        public override float ComputeMinimumRadius()
        {
            throw new System.NotImplementedException();
        }

        public override float ComputeMaximumRadius()
        {
            throw new System.NotImplementedException();
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
            throw new System.NotImplementedException();
        }
    }
}
