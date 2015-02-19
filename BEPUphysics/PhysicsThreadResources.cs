using System;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.CollisionTests.Manifolds;
using BEPUphysics.Entities;

using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.DeactivationManagement;
using BEPUutilities.DataStructures;
using BEPUutilities;
using BEPUutilities.ResourceManagement;

namespace BEPUphysics
{
    /// <summary>
    /// Handles allocation and management of commonly used resources.
    /// </summary>
    public static class PhysicsThreadResources
    {
        [ThreadStatic]
        static UnsafeResourcePool<RawList<RayCastResult>> SubPoolRayCastResultList;
        [ThreadStatic]
        static UnsafeResourcePool<RawList<BroadPhaseEntry>> SubPoolBroadPhaseEntryList;
        [ThreadStatic]
        static UnsafeResourcePool<RawList<Collidable>> SubPoolCollidableList;
        [ThreadStatic]
        static UnsafeResourcePool<RawList<Entity>> SubPoolEntityRawList;
        [ThreadStatic]
        static UnsafeResourcePool<TriangleShape> SubPoolTriangleShape;
        [ThreadStatic]
        static UnsafeResourcePool<RawList<CompoundChild>> SubPoolCompoundChildList;
        [ThreadStatic]
        static UnsafeResourcePool<TriangleCollidable> SubPoolTriangleCollidables;
        [ThreadStatic]
        static UnsafeResourcePool<SimulationIslandConnection> SimulationIslandConnections;
        //#endif
        /// <summary>
        /// Retrieves a ray cast result list from the resource pool.
        /// </summary>
        /// <returns>Empty ray cast result list.</returns>
        public static RawList<RayCastResult> GetRayCastResultList()
        {
            if (SubPoolRayCastResultList == null)
                SubPoolRayCastResultList = new UnsafeResourcePool<RawList<RayCastResult>>();
            return SubPoolRayCastResultList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(RawList<RayCastResult> list)
        {
            if (SubPoolRayCastResultList == null)
                SubPoolRayCastResultList = new UnsafeResourcePool<RawList<RayCastResult>>();
            list.Clear();
            SubPoolRayCastResultList.GiveBack(list);
        }

        /// <summary>
        /// Retrieves an BroadPhaseEntry list from the resource pool.
        /// </summary>
        /// <returns>Empty BroadPhaseEntry list.</returns>
        public static RawList<BroadPhaseEntry> GetBroadPhaseEntryList()
        {
            if (SubPoolBroadPhaseEntryList == null)
                SubPoolBroadPhaseEntryList = new UnsafeResourcePool<RawList<BroadPhaseEntry>>();
            return SubPoolBroadPhaseEntryList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(RawList<BroadPhaseEntry> list)
        {
            if (SubPoolBroadPhaseEntryList == null)
                SubPoolBroadPhaseEntryList = new UnsafeResourcePool<RawList<BroadPhaseEntry>>();
            list.Clear();
            SubPoolBroadPhaseEntryList.GiveBack(list);
        }

        /// <summary>
        /// Retrieves a Collidable list from the resource pool.
        /// </summary>
        /// <returns>Empty Collidable list.</returns>
        public static RawList<Collidable> GetCollidableList()
        {
            if (SubPoolCollidableList == null)
                SubPoolCollidableList = new UnsafeResourcePool<RawList<Collidable>>();
            return SubPoolCollidableList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(RawList<Collidable> list)
        {
            if (SubPoolCollidableList == null)
                SubPoolCollidableList = new UnsafeResourcePool<RawList<Collidable>>();
            list.Clear();
            SubPoolCollidableList.GiveBack(list);
        }

        /// <summary>
        /// Retrieves an CompoundChild list from the resource pool.
        /// </summary>
        /// <returns>Empty information list.</returns>
        public static RawList<CompoundChild> GetCompoundChildList()
        {
            if (SubPoolCompoundChildList == null)
                SubPoolCompoundChildList = new UnsafeResourcePool<RawList<CompoundChild>>();
            return SubPoolCompoundChildList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(RawList<CompoundChild> list)
        {
            if (SubPoolCompoundChildList == null)
                SubPoolCompoundChildList = new UnsafeResourcePool<RawList<CompoundChild>>();
            list.Clear();
            SubPoolCompoundChildList.GiveBack(list);
        }



        /// <summary>
        /// Retrieves an Entity RawList from the resource pool.
        /// </summary>
        /// <returns>Empty Entity raw list.</returns>
        public static RawList<Entity> GetEntityRawList()
        {
            if (SubPoolEntityRawList == null)
                SubPoolEntityRawList = new UnsafeResourcePool<RawList<Entity>>();
            return SubPoolEntityRawList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(RawList<Entity> list)
        {
            if (SubPoolEntityRawList == null)
                SubPoolEntityRawList = new UnsafeResourcePool<RawList<Entity>>();
            list.Clear();
            SubPoolEntityRawList.GiveBack(list);
        }

        /// <summary>
        /// Retrieves a Triangle shape from the resource pool.
        /// </summary>
        /// <returns>Initialized TriangleShape.</returns>
        public static TriangleShape GetTriangle()
        {
            if (SubPoolTriangleShape == null)
                SubPoolTriangleShape = new UnsafeResourcePool<TriangleShape>();
            return SubPoolTriangleShape.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="triangle">Triangle to return.</param>
        public static void GiveBack(TriangleShape triangle)
        {
            if (SubPoolTriangleShape == null)
                SubPoolTriangleShape = new UnsafeResourcePool<TriangleShape>();
            triangle.collisionMargin = 0;
            triangle.sidedness = TriangleSidedness.DoubleSided;
            SubPoolTriangleShape.GiveBack(triangle);
        }


        /// <summary>
        /// Retrieves a TriangleCollidable from the resource pool.
        /// </summary>
        /// <param name="a">First vertex in the triangle.</param>
        /// <param name="b">Second vertex in the triangle.</param>
        /// <param name="c">Third vertex in the triangle.</param>
        /// <returns>Initialized TriangleCollidable.</returns>
        public static TriangleCollidable GetTriangleCollidable(ref Vector3 a, ref Vector3 b, ref Vector3 c)
        {
            if (SubPoolTriangleCollidables == null)
                SubPoolTriangleCollidables = new UnsafeResourcePool<TriangleCollidable>();
            var tri = SubPoolTriangleCollidables.Take();
            var shape = tri.Shape;
            shape.vA = a;
            shape.vB = b;
            shape.vC = c;
            var identity = RigidTransform.Identity;
            tri.UpdateBoundingBoxForTransform(ref identity);
            return tri;

        }

        /// <summary>
        /// Retrieves a TriangleCollidable from the resource pool.
        /// </summary>
        /// <returns>Initialized TriangleCollidable.</returns>
        public static TriangleCollidable GetTriangleCollidable()
        {
            if (SubPoolTriangleCollidables == null)
                SubPoolTriangleCollidables = new UnsafeResourcePool<TriangleCollidable>();
            return SubPoolTriangleCollidables.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="triangle">Triangle collidable to return.</param>
        public static void GiveBack(TriangleCollidable triangle)
        {
            if (SubPoolTriangleCollidables == null)
                SubPoolTriangleCollidables = new UnsafeResourcePool<TriangleCollidable>();
            triangle.CleanUp();
            SubPoolTriangleCollidables.GiveBack(triangle);
        }


        /// <summary>
        /// Retrieves a simulation island connection from the resource pool.
        /// </summary>
        /// <returns>Uninitialized simulation island connection.</returns>
        public static SimulationIslandConnection GetSimulationIslandConnection()
        {
            if (SimulationIslandConnections == null)
                SimulationIslandConnections = new UnsafeResourcePool<SimulationIslandConnection>();
            return SimulationIslandConnections.Take();

        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="connection">Connection to return.</param>
        public static void GiveBack(SimulationIslandConnection connection)
        {
            if (SimulationIslandConnections == null)
                SimulationIslandConnections = new UnsafeResourcePool<SimulationIslandConnection>();
            connection.CleanUp();
            SimulationIslandConnections.GiveBack(connection);

        }
    }
}