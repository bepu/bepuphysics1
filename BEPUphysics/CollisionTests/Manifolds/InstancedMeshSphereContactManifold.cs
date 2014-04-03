using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUutilities.ResourceManagement;

namespace BEPUphysics.CollisionTests.Manifolds
{
    ///<summary>
    /// Manages persistent contacts between a convex and an instanced mesh.
    ///</summary>
    public class InstancedMeshSphereContactManifold : InstancedMeshContactManifold
    {

        static LockingResourcePool<TriangleSpherePairTester> testerPool = new LockingResourcePool<TriangleSpherePairTester>();
        protected override void GiveBackTester(TrianglePairTester tester)
        {
            testerPool.GiveBack((TriangleSpherePairTester)tester);
        }

        protected override TrianglePairTester GetTester()
        {
            return testerPool.Take();
        }

    }
}
