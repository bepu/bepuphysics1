using System;
using System.Collections.Generic;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.Entities;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.DataStructures;

namespace BEPUphysics.ResourceManagement
{
    /// <summary>
    /// Handles allocation and management of commonly used resources.
    /// </summary>
    public static class Resources
    {
        private static readonly Action<List<Entity>> InitializeEntityListDelegate = delegate(List<Entity> list) { list.Capacity = 16; };
#if WINDOWS
        [ThreadStatic]
        private static ResourcePool<List<bool>> subPoolBoolList;

        private static ResourcePool<List<bool>> SubPoolBoolList
        {
            get { return subPoolBoolList ?? (subPoolBoolList = new UnsafeResourcePool<List<bool>>()); }
        }

        [ThreadStatic]
        private static ResourcePool<RawList<RayHit>> subPoolRayHitList;

        private static ResourcePool<RawList<RayHit>> SubPoolRayHitList
        {
            get { return subPoolRayHitList ?? (subPoolRayHitList = new UnsafeResourcePool<RawList<RayHit>>()); }
        }

        [ThreadStatic]
        private static ResourcePool<RawList<RayCastResult>> subPoolRayCastResultList;

        private static ResourcePool<RawList<RayCastResult>> SubPoolRayCastResultList
        {
            get { return subPoolRayCastResultList ?? (subPoolRayCastResultList = new UnsafeResourcePool<RawList<RayCastResult>>()); }
        }

        [ThreadStatic]
        private static ResourcePool<RawList<BroadPhaseEntry>> subPoolCollisionEntryList;

        private static ResourcePool<RawList<BroadPhaseEntry>> SubPoolCollisionEntryList
        {
            get { return subPoolCollisionEntryList ?? (subPoolCollisionEntryList = new UnsafeResourcePool<RawList<BroadPhaseEntry>>()); }
        }

        [ThreadStatic]
        private static ResourcePool<RawList<CompoundChild>> subPoolCompoundChildList;

        private static ResourcePool<RawList<CompoundChild>> SubPoolCompoundChildList
        {
            get { return subPoolCompoundChildList ?? (subPoolCompoundChildList = new UnsafeResourcePool<RawList<CompoundChild>>()); }
        }

        [ThreadStatic]
        private static ResourcePool<List<int>> subPoolIntList;

        private static ResourcePool<List<int>> SubPoolIntList
        {
            get { return subPoolIntList ?? (subPoolIntList = new UnsafeResourcePool<List<int>>()); }
        }

        [ThreadStatic]
        private static ResourcePool<Queue<int>> subPoolIntQueue;

        private static ResourcePool<Queue<int>> SubPoolIntQueue
        {
            get { return subPoolIntQueue ?? (subPoolIntQueue = new UnsafeResourcePool<Queue<int>>()); }
        }

        [ThreadStatic]
        private static ResourcePool<List<float>> subPoolFloatList;

        private static ResourcePool<List<float>> SubPoolFloatList
        {
            get { return subPoolFloatList ?? (subPoolFloatList = new UnsafeResourcePool<List<float>>()); }
        }

        [ThreadStatic]
        private static ResourcePool<List<Vector3>> subPoolVectorList;

        private static ResourcePool<List<Vector3>> SubPoolVectorList
        {
            get { return subPoolVectorList ?? (subPoolVectorList = new UnsafeResourcePool<List<Vector3>>()); }
        }

        private static readonly ResourcePool<List<Entity>> SubPoolEntityList = new LockingResourcePool<List<Entity>>(16, InitializeEntityListDelegate);

        [ThreadStatic]
        private static ResourcePool<RawList<Entity>> subPoolEntityRawList;
        private static ResourcePool<RawList<Entity>> SubPoolEntityRawList
        {
            get { return subPoolEntityRawList ?? (subPoolEntityRawList = new UnsafeResourcePool<RawList<Entity>>(16)); }
        }


        [ThreadStatic]
        private static ResourcePool<Queue<Entity>> subPoolEntityQueue;

        private static ResourcePool<Queue<Entity>> SubPoolEntityQueue
        {
            get { return subPoolEntityQueue ?? (subPoolEntityQueue = new UnsafeResourcePool<Queue<Entity>>()); }
        }



        [ThreadStatic]
        private static ResourcePool<TriangleShape> subPoolTriangleShape;

        private static ResourcePool<TriangleShape> SubPoolTriangleShape
        {
            get { return subPoolTriangleShape ?? (subPoolTriangleShape = new UnsafeResourcePool<TriangleShape>()); }
        }

#else
        static ResourcePool<List<bool>> SubPoolBoolList = new LockingResourcePool<List<bool>>();
        static ResourcePool<RawList<RayHit>> SubPoolRayHitList = new LockingResourcePool<RawList<RayHit>>();
        static ResourcePool<RawList<RayCastResult>> SubPoolRayCastResultList = new LockingResourcePool<RawList<RayCastResult>>();
        static ResourcePool<RawList<BroadPhaseEntry>> SubPoolCollisionEntryList = new LockingResourcePool<RawList<BroadPhaseEntry>>();
        static ResourcePool<List<int>> SubPoolIntList = new LockingResourcePool<List<int>>();
        static ResourcePool<Queue<int>> SubPoolIntQueue = new LockingResourcePool<Queue<int>>();
        static ResourcePool<List<float>> SubPoolFloatList = new LockingResourcePool<List<float>>();
        static ResourcePool<List<Vector3>> SubPoolVectorList = new LockingResourcePool<List<Vector3>>();
        static ResourcePool<List<Entity>> SubPoolEntityList = new LockingResourcePool<List<Entity>>(32, InitializeEntityListDelegate);
        static ResourcePool<RawList<Entity>> SubPoolEntityRawList = new LockingResourcePool<RawList<Entity>>(16);
        static ResourcePool<Queue<Entity>> SubPoolEntityQueue = new LockingResourcePool<Queue<Entity>>();
        static ResourcePool<TriangleShape> SubPoolTriangleShape = new LockingResourcePool<TriangleShape>();
        static ResourcePool<RawList<CompoundChild>> SubPoolCompoundChildList = new LockingResourcePool<RawList<CompoundChild>>();
#endif
        /// <summary>
        /// Retrieves a ray cast result list from the resource pool.
        /// </summary>
        /// <returns>Empty ray cast result list.</returns>
        public static RawList<RayCastResult> GetRayCastResultList()
        {
            return SubPoolRayCastResultList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(RawList<RayCastResult> list)
        {
            list.Clear();
            SubPoolRayCastResultList.GiveBack(list);
        }

        /// <summary>
        /// Retrieves a ray hit list from the resource pool.
        /// </summary>
        /// <returns>Empty ray hit list.</returns>
        public static RawList<RayHit> GetRayHitList()
        {
            return SubPoolRayHitList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(RawList<RayHit> list)
        {
            list.Clear();
            SubPoolRayHitList.GiveBack(list);
        }

        /// <summary>
        /// Retrieves an CollisionEntry list from the resource pool.
        /// </summary>
        /// <returns>Empty CollisionEntry list.</returns>
        public static RawList<BroadPhaseEntry> GetCollisionEntryList()
        {
            return SubPoolCollisionEntryList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(RawList<BroadPhaseEntry> list)
        {
            list.Clear();
            SubPoolCollisionEntryList.GiveBack(list);
        }

        /// <summary>
        /// Retrieves an CompoundChild list from the resource pool.
        /// </summary>
        /// <returns>Empty information list.</returns>
        public static RawList<CompoundChild> GetCompoundChildList()
        {
            return SubPoolCompoundChildList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(RawList<CompoundChild> list)
        {
            list.Clear();
            SubPoolCompoundChildList.GiveBack(list);
        }

        /// <summary>
        /// Retrieves a bool list from the resource pool.
        /// </summary>
        /// <returns>Empty bool list.</returns>
        public static List<bool> GetBoolList()
        {
            return SubPoolBoolList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(List<bool> list)
        {
            list.Clear();
            SubPoolBoolList.GiveBack(list);
        }

        /// <summary>
        /// Retrieves a int list from the resource pool.
        /// </summary>
        /// <returns>Empty int list.</returns>
        public static List<int> GetIntList()
        {
            return SubPoolIntList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(List<int> list)
        {
            list.Clear();
            SubPoolIntList.GiveBack(list);
        }

        /// <summary>
        /// Retrieves a int queue from the resource pool.
        /// </summary>
        /// <returns>Empty int queue.</returns>
        public static Queue<int> GetIntQueue()
        {
            return SubPoolIntQueue.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="queue">Queue to return.</param>
        public static void GiveBack(Queue<int> queue)
        {
            queue.Clear();
            SubPoolIntQueue.GiveBack(queue);
        }

        /// <summary>
        /// Retrieves a float list from the resource pool.
        /// </summary>
        /// <returns>Empty float list.</returns>
        public static List<float> GetFloatList()
        {
            return SubPoolFloatList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(List<float> list)
        {
            list.Clear();
            SubPoolFloatList.GiveBack(list);
        }

        /// <summary>
        /// Retrieves a Vector3 list from the resource pool.
        /// </summary>
        /// <returns>Empty Vector3 list.</returns>
        public static List<Vector3> GetVectorList()
        {
            return SubPoolVectorList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(List<Vector3> list)
        {
            list.Clear();
            SubPoolVectorList.GiveBack(list);
        }

        /// <summary>
        /// Retrieves an Entity list from the resource pool.
        /// </summary>
        /// <returns>Empty Entity list.</returns>
        public static List<Entity> GetEntityList()
        {
            return SubPoolEntityList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(List<Entity> list)
        {
            list.Clear();
            SubPoolEntityList.GiveBack(list);
        }

        /// <summary>
        /// Retrieves an Entity RawList from the resource pool.
        /// </summary>
        /// <returns>Empty Entity raw list.</returns>
        public static RawList<Entity> GetEntityRawList()
        {
            return SubPoolEntityRawList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(RawList<Entity> list)
        {
            list.Clear();
            SubPoolEntityRawList.GiveBack(list);
        }

        /// <summary>
        /// Retrieves a Entity queue from the resource pool.
        /// </summary>
        /// <returns>Empty Entity queue.</returns>
        public static Queue<Entity> GetEntityQueue()
        {
            return SubPoolEntityQueue.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="queue">Queue to return.</param>
        public static void GiveBack(Queue<Entity> queue)
        {
            queue.Clear();
            SubPoolEntityQueue.GiveBack(queue);
        }

        ///// <summary>
        ///// Retrieves an Int3 list from the resource pool.
        ///// </summary>
        ///// <returns>Empty Int3 list.</returns>
        //public static List<Int3> GetInt3List()
        //{
        //    return SubPoolInt3List.Take();
        //}

        ///// <summary>
        ///// Returns a resource to the pool.
        ///// </summary>
        ///// <param name="list">List to return.</param>
        //public static void GiveBack(List<Int3> list)
        //{
        //    list.Clear();
        //    SubPoolInt3List.GiveBack(list);
        //}


        ///// <summary>
        ///// Retrieves a Triangle list from the resource pool.
        ///// </summary>
        ///// <returns>Empty Triangle list.</returns>
        //public static List<Triangle> GetStaticTriangleList()
        //{
        //    return SubPoolTriangleList.Take();
        //}

        ///// <summary>
        ///// Returns a resource to the pool.
        ///// </summary>
        ///// <param name="list">List to return.</param>
        //public static void GiveBack(List<Triangle> list)
        //{
        //    list.Clear();
        //    SubPoolTriangleList.GiveBack(list);
        //}

        ///// <summary>
        ///// Retrieves a Contact list from the resource pool.
        ///// </summary>
        ///// <returns>Empty Contact list.</returns>
        //public static List<Contact> GetContactList()
        //{
        //    return SubPoolContactList.Take();
        //}

        ///// <summary>
        ///// Returns a resource to the pool.
        ///// </summary>
        ///// <param name="list">List to return.</param>
        //public static void GiveBack(List<Contact> list)
        //{
        //    list.Clear();
        //    SubPoolContactList.GiveBack(list);
        //}


        ///// <summary>
        ///// Retrieves a collision pair from the resource pool.
        ///// </summary>
        ///// <param name="a">First entity in the collision pair.</param>
        ///// <param name="b">Second entity in the collision pair.</param>
        ///// <param name="s">Space that contains the collision pair.</param>
        ///// <returns>New collision pair.</returns>
        //public static CollisionPair GetCollisionPair(Entity a, Entity b, Space s)
        //{
        //    CollisionPair toReturn = SubPoolCollisionPair.Take();
        //    toReturn.Setup(a, b, s);
        //    return toReturn;
        //}

        ///// <summary>
        ///// Returns a resource to the pool.
        ///// </summary>
        ///// <param name="collisionPair">Collision pair to return.</param>
        //public static void GiveBack(CollisionPair collisionPair)
        //{
        //    collisionPair.ColliderA = null;
        //    collisionPair.ColliderB = null;
        //    collisionPair.ParentA = null;
        //    collisionPair.ParentB = null;
        //    SubPoolCollisionPair.GiveBack(collisionPair);
        //}

        ///// <summary>
        ///// Retrieves a collision pair list from the resource pool.
        ///// </summary>
        ///// <returns>Empty collision pair list.</returns>
        //public static List<CollisionPair> GetCollisionPairList()
        //{
        //    return SubPoolCollisionPairList.Take();
        //}

        ///// <summary>
        ///// Returns a resource to the pool.
        ///// </summary>
        ///// <param name="list">List to return.</param>
        //public static void GiveBack(List<CollisionPair> list)
        //{
        //    list.Clear();
        //    SubPoolCollisionPairList.GiveBack(list);
        //}

        ///// <summary>
        ///// Retrieves a Constraint list from the resource pool.
        ///// </summary>
        ///// <returns>Empty Constraint list.</returns>
        //public static List<Constraint> GetConstraintList()
        //{
        //    return SubPoolConstraintList.Take();
        //}

        ///// <summary>
        ///// Returns a resource to the pool.
        ///// </summary>
        ///// <param name="list">List to return.</param>
        //public static void GiveBack(List<Constraint> list)
        //{
        //    list.Clear();
        //    SubPoolConstraintList.GiveBack(list);
        //}

        /// <summary>
        /// Retrieves a Triangle shape from the resource pool.
        /// </summary>
        /// <param name="v1">Position of the first vertex.</param>
        /// <param name="v2">Position of the second vertex.</param>
        /// <param name="v3">Position of the third vertex.</param>
        /// <returns>Initialized TriangleShape.</returns>
        public static TriangleShape GetTriangle(ref Vector3 v1, ref Vector3 v2, ref Vector3 v3)
        {
            TriangleShape toReturn = SubPoolTriangleShape.Take();
            toReturn.vA = v1;
            toReturn.vB = v2;
            toReturn.vC = v3;
            return toReturn;
        }

        /// <summary>
        /// Retrieves a Triangle shape from the resource pool.
        /// </summary>
        /// <returns>Initialized TriangleShape.</returns>
        public static TriangleShape GetTriangle()
        {
            return SubPoolTriangleShape.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="triangle">Triangle to return.</param>
        public static void GiveBack(TriangleShape triangle)
        {
            SubPoolTriangleShape.GiveBack(triangle);
        }


        ///// <summary>
        ///// Retrieves a SimulationIsland from the resource pool.
        ///// </summary>
        ///// <param name="s">Space in which the SimulationIsland exists.</param>
        ///// <returns>New SimulationIsland.</returns>
        //internal static SimulationIsland GetSimulationIsland(Space s)
        //{
        //    SimulationIsland toReturn = SubPoolSimulationIsland.Take();
        //    toReturn.Setup(s);
        //    return toReturn;
        //}

        ///// <summary>
        ///// Returns a resource to the pool.
        ///// </summary>
        ///// <param name="simulationIsland">Simulation island to return.</param>
        //internal static void GiveBack(SimulationIsland simulationIsland)
        //{
        //    simulationIsland.isDeactivationCandidate = false;
        //    simulationIsland.isActive = true;
        //    simulationIsland.numDeactivationCandidatesContained = 0;
        //    simulationIsland.space = null;
        //    simulationIsland.Entities.Clear();
        //    SubPoolSimulationIsland.GiveBack(simulationIsland);
        //}

        ///// <summary>
        ///// Retrieves a List of SimulationIslands from the resource pool.
        ///// </summary>
        ///// <returns>New SimulationIsland List.</returns>
        //internal static List<SimulationIsland> GetSimulationIslandList()
        //{
        //    List<SimulationIsland> toReturn = SubPoolSimulationIslandLists.Take();
        //    return toReturn;
        //}

        ///// <summary>
        ///// Returns a resource to the pool.
        ///// </summary>
        ///// <param name="simulationIslandList">Simulation island list to return.</param>
        //internal static void GiveBack(List<SimulationIsland> simulationIslandList)
        //{
        //    foreach (SimulationIsland island in simulationIslandList)
        //    {
        //        GiveBack(island);
        //    }

        //    SubPoolSimulationIslandLists.GiveBack(simulationIslandList);
        //}

        ///// <summary>
        ///// Retrieves a Queue of SimulationIslands from the resource pool.
        ///// </summary>
        ///// <returns>New SimulationIsland queue.</returns>
        //internal static Queue<SimulationIsland> GetSimulationIslandQueue()
        //{
        //    Queue<SimulationIsland> toReturn = SubPoolSimulationIslandQueues.Take();
        //    return toReturn;
        //}

        ///// <summary>
        ///// Returns a resource to the pool.
        ///// </summary>
        ///// <param name="simulationIslandQueue">Simulation island queue to return.</param>
        //internal static void GiveBack(Queue<SimulationIsland> simulationIslandQueue)
        //{
        //    while (simulationIslandQueue.Count > 0)
        //    {
        //        GiveBack(simulationIslandQueue.Dequeue());
        //    }

        //    SubPoolSimulationIslandQueues.GiveBack(simulationIslandQueue);
        //}



    }
}