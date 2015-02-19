using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUutilities.ResourceManagement;

namespace BEPUphysics.CollisionTests.Manifolds
{
    ///<summary>
    /// Manages persistent contacts between a static mesh and a convex.
    ///</summary>
    public class StaticMeshSphereContactManifold : StaticMeshContactManifold
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
