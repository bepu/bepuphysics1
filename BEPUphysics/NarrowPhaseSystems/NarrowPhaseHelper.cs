using System.Collections.Generic;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.NarrowPhaseSystems.Factories;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.CollisionShapes.ConvexShapes;

namespace BEPUphysics.NarrowPhaseSystems
{

    ///<summary>
    /// Contains the collision managers dictionary and other helper methods for creating pairs.
    ///</summary>
    public static class NarrowPhaseHelper
    {

        ///<summary>
        /// The fallback factory used for convex pairs.
        ///</summary>
        public static NarrowPhasePairFactory GeneralConvexPairFactory
        {
            get;
            set;
        }

        static NarrowPhaseHelper()
        {
            collisionManagers = new Dictionary<TypePair, NarrowPhasePairFactory>();
            GeneralConvexPairFactory = new GeneralConvexFactory();
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<BoxShape>), typeof(ConvexCollidable<BoxShape>)), new BoxPairFactory());
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<BoxShape>), typeof(ConvexCollidable<SphereShape>)), new BoxSpherePairFactory());
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<SphereShape>), typeof(ConvexCollidable<SphereShape>)), new SpherePairFactory());

            var triangleConvexFactory = new TriangleConvexPairFactory();
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<BoxShape>), typeof(ConvexCollidable<TriangleShape>)), triangleConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<SphereShape>), typeof(ConvexCollidable<TriangleShape>)), triangleConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<CapsuleShape>), typeof(ConvexCollidable<TriangleShape>)), triangleConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<TriangleShape>), typeof(ConvexCollidable<TriangleShape>)), triangleConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<CylinderShape>), typeof(ConvexCollidable<TriangleShape>)), triangleConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<ConeShape>), typeof(ConvexCollidable<TriangleShape>)), triangleConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<TransformableShape>), typeof(ConvexCollidable<TriangleShape>)), triangleConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<MinkowskiSumShape>), typeof(ConvexCollidable<TriangleShape>)), triangleConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<WrappedShape>), typeof(ConvexCollidable<TriangleShape>)), triangleConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<ConvexHullShape>), typeof(ConvexCollidable<TriangleShape>)), triangleConvexFactory);

            var staticTriangleMeshConvexFactory = new StaticMeshConvexPairFactory();
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<BoxShape>), typeof(StaticMesh)), staticTriangleMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<SphereShape>), typeof(StaticMesh)), staticTriangleMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<CapsuleShape>), typeof(StaticMesh)), staticTriangleMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<TriangleShape>), typeof(StaticMesh)), staticTriangleMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<CylinderShape>), typeof(StaticMesh)), staticTriangleMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<ConeShape>), typeof(StaticMesh)), staticTriangleMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<TransformableShape>), typeof(StaticMesh)), staticTriangleMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<MinkowskiSumShape>), typeof(StaticMesh)), staticTriangleMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<WrappedShape>), typeof(StaticMesh)), staticTriangleMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<ConvexHullShape>), typeof(StaticMesh)), staticTriangleMeshConvexFactory);

            var terrainConvexFactory = new TerrainConvexPairFactory();
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<BoxShape>), typeof(Terrain)), terrainConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<SphereShape>), typeof(Terrain)), terrainConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<CapsuleShape>), typeof(Terrain)), terrainConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<TriangleShape>), typeof(Terrain)), terrainConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<CylinderShape>), typeof(Terrain)), terrainConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<ConeShape>), typeof(Terrain)), terrainConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<TransformableShape>), typeof(Terrain)), terrainConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<MinkowskiSumShape>), typeof(Terrain)), terrainConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<WrappedShape>), typeof(Terrain)), terrainConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<ConvexHullShape>), typeof(Terrain)), terrainConvexFactory);

            var instancedMeshConvexFactory = new InstancedMeshConvexPairFactory();
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<BoxShape>), typeof(InstancedMesh)), instancedMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<SphereShape>), typeof(InstancedMesh)), instancedMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<CapsuleShape>), typeof(InstancedMesh)), instancedMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<TriangleShape>), typeof(InstancedMesh)), instancedMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<CylinderShape>), typeof(InstancedMesh)), instancedMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<ConeShape>), typeof(InstancedMesh)), instancedMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<TransformableShape>), typeof(InstancedMesh)), instancedMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<MinkowskiSumShape>), typeof(InstancedMesh)), instancedMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<WrappedShape>), typeof(InstancedMesh)), instancedMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<ConvexHullShape>), typeof(InstancedMesh)), instancedMeshConvexFactory);

            var compoundConvexFactory = new CompoundConvexPairFactory();
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<BoxShape>), typeof(CompoundCollidable)), compoundConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<SphereShape>), typeof(CompoundCollidable)), compoundConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<CapsuleShape>), typeof(CompoundCollidable)), compoundConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<TriangleShape>), typeof(CompoundCollidable)), compoundConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<CylinderShape>), typeof(CompoundCollidable)), compoundConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<ConeShape>), typeof(CompoundCollidable)), compoundConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<TransformableShape>), typeof(CompoundCollidable)), compoundConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<MinkowskiSumShape>), typeof(CompoundCollidable)), compoundConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<WrappedShape>), typeof(CompoundCollidable)), compoundConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<ConvexHullShape>), typeof(CompoundCollidable)), compoundConvexFactory);

            collisionManagers.Add(new TypePair(typeof(CompoundCollidable), typeof(CompoundCollidable)), new CompoundPairFactory());
            collisionManagers.Add(new TypePair(typeof(CompoundCollidable), typeof(StaticMesh)), new CompoundStaticMeshPairFactory());
            collisionManagers.Add(new TypePair(typeof(CompoundCollidable), typeof(Terrain)), new CompoundTerrainPairFactory());
            collisionManagers.Add(new TypePair(typeof(CompoundCollidable), typeof(InstancedMesh)), new CompoundInstancedMeshPairFactory());

            var mobileMeshConvexFactory = new MobileMeshConvexPairFactory();
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<BoxShape>), typeof(MobileMeshCollidable)), mobileMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<SphereShape>), typeof(MobileMeshCollidable)), mobileMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<CapsuleShape>), typeof(MobileMeshCollidable)), mobileMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<TriangleShape>), typeof(MobileMeshCollidable)), mobileMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<CylinderShape>), typeof(MobileMeshCollidable)), mobileMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<ConeShape>), typeof(MobileMeshCollidable)), mobileMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<TransformableShape>), typeof(MobileMeshCollidable)), mobileMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<MinkowskiSumShape>), typeof(MobileMeshCollidable)), mobileMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<WrappedShape>), typeof(MobileMeshCollidable)), mobileMeshConvexFactory);
            collisionManagers.Add(new TypePair(typeof(ConvexCollidable<ConvexHullShape>), typeof(MobileMeshCollidable)), mobileMeshConvexFactory);

            collisionManagers.Add(new TypePair(typeof(CompoundCollidable), typeof(MobileMeshCollidable)), new CompoundMobileMeshPairFactory());
            collisionManagers.Add(new TypePair(typeof(MobileMeshCollidable), typeof(StaticMesh)), new MobileMeshStaticMeshPairFactory());


        }

        internal static Dictionary<TypePair, NarrowPhasePairFactory> collisionManagers;
        ///<summary>
        /// Gets or sets the dictionary that defines the factory to use for various type pairs.
        ///</summary>
        public static Dictionary<TypePair, NarrowPhasePairFactory> CollisionManagers
        {
            get
            {
                return collisionManagers;
            }
            set
            {
                collisionManagers = value;
            }
        }

        ///<summary>
        /// Gets a narrow phase pair for a given broad phase overlap.
        ///</summary>
        ///<param name="pair">Overlap to use to create the pair.</param>
        ///<returns>A INarrowPhasePair for the overlap.</returns>
        public static INarrowPhasePair GetPair(ref BroadPhaseOverlap pair)
        {
            NarrowPhasePairFactory factory;
            if (collisionManagers.TryGetValue(new TypePair(pair.entryA.GetType(), pair.entryB.GetType()), out factory))
            {
                INarrowPhasePair toReturn = factory.GetNarrowPhasePair(pair);
                toReturn.BroadPhaseOverlap = pair;
                toReturn.Factory = factory;
                return toReturn;
            }
            //Convex-convex collisions are a pretty significant chunk of all tests, so rather than defining them all, just have a fallback.
            var a = pair.entryA as ConvexCollidable;
            var b = pair.entryB as ConvexCollidable;
            if (a != null && b != null)
            {
                INarrowPhasePair toReturn = GeneralConvexPairFactory.GetNarrowPhasePair(pair);
                toReturn.BroadPhaseOverlap = pair;
                toReturn.Factory = GeneralConvexPairFactory;
                return toReturn;
            }
            return null;
        }

        ///<summary>
        /// Gets a narrow phase pair for a given pair of entries.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        /// <param name="entryB">Second entry in the pair.</param>
        /// <param name="rule">Collision rule governing the pair.</param>
        ///<returns>A INarrowPhasePair for the overlap.</returns>
        public static INarrowPhasePair GetPair(BroadPhaseEntry entryA, BroadPhaseEntry entryB, CollisionRule rule)
        {
            BroadPhaseOverlap overlap = new BroadPhaseOverlap(entryA, entryB, rule);
            return GetPair(ref overlap);
        }

        ///<summary>
        /// Gets a narrow phase pair for a given pair of entries.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        /// <param name="entryB">Second entry in the pair.</param>
        ///<returns>A INarrowPhasePair for the overlap.</returns>
        public static INarrowPhasePair GetPair(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {
            BroadPhaseOverlap overlap = new BroadPhaseOverlap(entryA, entryB);
            return GetPair(ref overlap);
        }

        /// <summary>
        /// Gets a collidable pair handler for a pair of collidables.
        /// </summary>
        /// <param name="pair">Pair of collidables to use to create the pair handler.</param>
        /// <param name="rule">Collision rule governing the pair.</param>
        /// <returns>CollidablePairHandler for the pair.</returns>
        public static CollidablePairHandler GetPairHandler(ref CollidablePair pair, CollisionRule rule)
        {
            BroadPhaseOverlap overlap = new BroadPhaseOverlap(pair.collidableA, pair.collidableB, rule);
            return GetPair(ref overlap) as CollidablePairHandler;
        }
        /// <summary>
        /// Gets a collidable pair handler for a pair of collidables.
        /// </summary>
        /// <param name="pair">Pair of collidables to use to create the pair handler.</param>
        /// <returns>CollidablePairHandler for the pair.</returns>
        public static CollidablePairHandler GetPairHandler(ref CollidablePair pair)
        {
            BroadPhaseOverlap overlap = new BroadPhaseOverlap(pair.collidableA, pair.collidableB);
            return GetPair(ref overlap) as CollidablePairHandler;
        }

        //Note that this does not check collision rules.
        /// <summary>
        /// Tests the pair of collidables for intersection without regard for collision rules.
        /// </summary>
        /// <param name="pair">Pair to test.</param>
        /// <returns>Whether or not the pair is intersecting.</returns>
        public static bool Intersecting(ref CollidablePair pair)
        {
            var pairHandler = GetPairHandler(ref pair);
            pairHandler.SuppressEvents = true;
            pairHandler.UpdateCollision(0);
            bool toReturn = pairHandler.ContactCount > 0;
            pairHandler.SuppressEvents = false;
            pairHandler.CleanUp();
            (pairHandler as INarrowPhasePair).Factory.GiveBack(pairHandler);
            return toReturn;
        }
    }
}
