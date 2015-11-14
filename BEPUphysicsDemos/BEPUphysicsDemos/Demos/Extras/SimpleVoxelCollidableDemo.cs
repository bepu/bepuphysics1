using System;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.BroadPhaseEntries.Events;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionTests;
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using BEPUphysics.CollisionTests.Manifolds;
using BEPUphysics.Constraints.Collision;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.NarrowPhaseSystems;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.OtherSpaceStages;
using BEPUphysics.PositionUpdating;
using BEPUphysics.Settings;
using BEPUphysicsDrawer.Models;
using BEPUutilities;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.BroadPhaseSystems;
using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System.Diagnostics;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDemos.Demos.Extras
{
    public struct Int3 : IEquatable<Int3>
    {
        public int X;
        public int Y;
        public int Z;

        public override int GetHashCode()
        {
            return (X * 533000401) ^ (Y * 920419813) ^ (Z * 694847539);
        }
        public bool Equals(Int3 other)
        {
            return other.X == X && other.Y == Y && other.Z == Z;
        }
    }

    /// <summary>
    /// Extremely simple and unoptimized voxel grid shape.
    /// Shapes can be shared between multiple collidables.
    /// </summary>
    public class VoxelGridShape : CollisionShape
    {
        /// <summary>
        /// Three dimensional grid of cells. A true value means the cell is filled, and false means it's empty.
        /// </summary>
        public bool[, ,] Cells { get; private set; }
        //Note: This representation is very inefficient. Each bool occupies a full byte, and there is no space skipping.
        //Large blocks of empty space take just as much space as high frequency data.
        //If memory is a concern, it would be a good idea to optimize this. It would be pretty easy to get an order of magnitude (or three) improvement.

        /// <summary>
        /// Width of a single voxel cell.
        /// </summary>
        public float CellWidth { get; private set; }

        public void GetBoundingBox(ref Vector3 position, out BoundingBox boundingBox)
        {
            var size = new Vector3(CellWidth * Cells.GetLength(0), CellWidth * Cells.GetLength(1), CellWidth * Cells.GetLength(2));
            boundingBox.Min = position;
            Vector3.Add(ref size, ref position, out boundingBox.Max);
        }

        public VoxelGridShape(bool[, ,] cells, float cellWidth)
        {
            Cells = cells;
            CellWidth = cellWidth;
        }

        public void GetOverlaps(Vector3 gridPosition, BoundingBox boundingBox, ref QuickList<Int3> overlaps)
        {
            Vector3.Subtract(ref boundingBox.Min, ref gridPosition, out boundingBox.Min);
            Vector3.Subtract(ref boundingBox.Max, ref gridPosition, out boundingBox.Max);
            var inverseWidth = 1f / CellWidth;
            var min = new Int3
            {
                X = Math.Max(0, (int)(boundingBox.Min.X * inverseWidth)),
                Y = Math.Max(0, (int)(boundingBox.Min.Y * inverseWidth)),
                Z = Math.Max(0, (int)(boundingBox.Min.Z * inverseWidth))
            };
            var max = new Int3
            {
                X = Math.Min(Cells.GetLength(0) - 1, (int)(boundingBox.Max.X * inverseWidth)),
                Y = Math.Min(Cells.GetLength(1) - 1, (int)(boundingBox.Max.Y * inverseWidth)),
                Z = Math.Min(Cells.GetLength(2) - 1, (int)(boundingBox.Max.Z * inverseWidth))
            };

            for (int i = min.X; i <= max.X; ++i)
            {
                for (int j = min.Y; j <= max.Y; ++j)
                {
                    for (int k = min.Z; k <= max.Z; ++k)
                    {
                        if (Cells[i, j, k])
                        {
                            overlaps.Add(new Int3 { X = i, Y = j, Z = k });
                        }
                    }
                }
            }
        }
    }

    /// <summary>
    /// Simple voxel grid collidable. Uses the VoxelGridShape as a data source and provides the 
    /// </summary>
    public class VoxelGrid : StaticCollidable
    {
        public new VoxelGridShape Shape
        {
            get { return (VoxelGridShape)base.Shape; }
        }

        /// <summary>
        /// Position of the minimum corner of the voxel grid.
        /// </summary>
        public Vector3 Position;

        public VoxelGrid(VoxelGridShape shape, Vector3 position)
        {
            Position = position;
            base.Shape = shape;
            events = new ContactEventManager<VoxelGrid>(this);
        }

        public override bool RayCast(Ray ray, float maximumLength, out RayHit rayHit)
        {
            //This example is primarily to show custom collidable pair management with a minimum of other complexity, and this isn't vital.
            //Note: the character controller makes significant use of ray casts. While its basic features work without ray casts, 
            //implementing them will unlock more features and improve behavior.
            rayHit = new RayHit();
            return false;
        }

        public override bool ConvexCast(ConvexShape castShape, ref RigidTransform startingTransform, ref Vector3 sweep, out RayHit hit)
        {
            //This example is primarily to show custom collidable pair management with a minimum of other complexity, and this isn't vital.
            hit = new RayHit();
            return false;
        }

        public override void UpdateBoundingBox()
        {
            Shape.GetBoundingBox(ref Position, out boundingBox);
        }

        //For simplicity, the event manager is read only. The other collidables like StaticMesh and InstancedMesh have a setter, but it complicates things
        //and doesn't add a lot. For example implementations of setters, check those classes out.
        protected internal ContactEventManager<VoxelGrid> events;
        ///<summary>
        /// Gets the event manager of the mesh.
        ///</summary>
        public ContactEventManager<VoxelGrid> Events
        {
            get { return events; }
        }
        protected override IContactEventTriggerer EventTriggerer
        {
            get { return events; }
        }

        protected override IDeferredEventCreator EventCreator
        {
            get { return events; }
        }
    }

    public class ReusableBoxCollidable : ConvexCollidable<BoxShape>
    {
        public ReusableBoxCollidable()
            : base(new BoxShape(1, 1, 1))
        {
        }

        protected override void UpdateBoundingBoxInternal(float dt)
        {
            Shape.GetBoundingBox(ref worldTransform, out boundingBox);
        }

    }

    /// <summary>
    /// Manages the contacts associated with a convex-voxelgrid collision.
    /// This is a slightly different kind of manifold- instead of directly managing collision, it manages
    /// a set of testers for each box near the opposing convex.
    /// </summary>
    public class VoxelGridConvexContactManifold : ContactManifold
    {


        static LockingResourcePool<ReusableBoxCollidable> boxCollidablePool = new LockingResourcePool<ReusableBoxCollidable>();
        static LockingResourcePool<GeneralConvexPairTester> testerPool = new LockingResourcePool<GeneralConvexPairTester>();

        private VoxelGrid voxelGrid;
        private ConvexCollidable convex;


        public QuickDictionary<Int3, GeneralConvexPairTester> ActivePairs;
        private QuickDictionary<Int3, GeneralConvexPairTester> activePairsBackBuffer;
        protected RawValueList<ContactSupplementData> supplementData = new RawValueList<ContactSupplementData>(4);

        public VoxelGridConvexContactManifold()
        {
            contacts = new RawList<Contact>(4);
            unusedContacts = new UnsafeResourcePool<Contact>(4);
            contactIndicesToRemove = new RawList<int>(4);
        }


        private GeneralConvexPairTester GetPair(ref Int3 position)
        {
            var pair = testerPool.Take();
            var boxCollidable = boxCollidablePool.Take();
            boxCollidable.Shape.Width = voxelGrid.Shape.CellWidth;
            boxCollidable.Shape.Height = voxelGrid.Shape.CellWidth;
            boxCollidable.Shape.Length = voxelGrid.Shape.CellWidth;
            pair.Initialize(convex, boxCollidable);
            boxCollidable.WorldTransform = new RigidTransform(new Vector3(
                voxelGrid.Position.X + (position.X + 0.5f) * voxelGrid.Shape.CellWidth,
                voxelGrid.Position.Y + (position.Y + 0.5f) * voxelGrid.Shape.CellWidth,
                voxelGrid.Position.Z + (position.Z + 0.5f) * voxelGrid.Shape.CellWidth));
            return pair;
        }


        private void ReturnPair(GeneralConvexPairTester pair)
        {
            boxCollidablePool.GiveBack((ReusableBoxCollidable)pair.CollidableB);
            pair.CleanUp();
            testerPool.GiveBack(pair);
        }



        public override void Initialize(Collidable newCollidableA, Collidable newCollidableB)
        {
            convex = newCollidableA as ConvexCollidable;
            voxelGrid = newCollidableB as VoxelGrid;


            if (convex == null || voxelGrid == null)
            {
                convex = newCollidableB as ConvexCollidable;
                voxelGrid = newCollidableA as VoxelGrid;
                if (convex == null || voxelGrid == null)
                    throw new ArgumentException("Inappropriate types used to initialize contact manifold.");
            }
            ActivePairs = new QuickDictionary<Int3, GeneralConvexPairTester>(BufferPools<Int3>.Locking, BufferPools<GeneralConvexPairTester>.Locking, BufferPools<int>.Locking, 3);
            activePairsBackBuffer = new QuickDictionary<Int3, GeneralConvexPairTester>(BufferPools<Int3>.Locking, BufferPools<GeneralConvexPairTester>.Locking, BufferPools<int>.Locking, 3);

        }

        public override void CleanUp()
        {
            convex = null;

            for (int i = ActivePairs.Count - 1; i >= 0; --i)
            {
                ReturnPair(ActivePairs.Values[i]);
                ActivePairs.Values[i].CleanUp();
            }
            //Clear->dispose is technically unnecessary now, but it may avoid some pain later on when this behavior changes in v2.
            //This will be a very sneaky breaking change...
            ActivePairs.Clear();
            ActivePairs.Dispose();
            Debug.Assert(activePairsBackBuffer.Count == 0);
            activePairsBackBuffer.Dispose();
            base.CleanUp();
        }


        public override void Update(float dt)
        {
            //Refresh the contact manifold for this frame.
            var transform = new RigidTransform(voxelGrid.Position);
            var convexTransform = convex.WorldTransform;
            ContactRefresher.ContactRefresh(contacts, supplementData, ref convexTransform, ref transform, contactIndicesToRemove);
            RemoveQueuedContacts();

            //Collect the set of overlapped cell indices.
            //Not the fastest way to do this, but it's relatively simple and easy.
            var overlaps = new QuickList<Int3>(BufferPools<Int3>.Thread);
            voxelGrid.Shape.GetOverlaps(voxelGrid.Position, convex.BoundingBox, ref overlaps);

            var candidatesToAdd = new QuickList<ContactData>(BufferPools<ContactData>.Thread, BufferPool<int>.GetPoolIndex(overlaps.Count));
            for (int i = 0; i < overlaps.Count; ++i)
            {
                GeneralConvexPairTester manifold;
                if (!ActivePairs.TryGetValue(overlaps.Elements[i], out manifold))
                {
                    //This manifold did not previously exist.
                    manifold = GetPair(ref overlaps.Elements[i]);
                }
                else
                {
                    //It did previously exist.
                    ActivePairs.FastRemove(overlaps.Elements[i]);
                }
                activePairsBackBuffer.Add(overlaps.Elements[i], manifold);
                ContactData contactCandidate;
                if (manifold.GenerateContactCandidate(out contactCandidate))
                {

                    candidatesToAdd.Add(ref contactCandidate);
                }
            }
            overlaps.Dispose();
            //Any pairs remaining in the activePairs set no longer exist. Clean them up.
            for (int i = ActivePairs.Count - 1; i >= 0; --i)
            {
                ReturnPair(ActivePairs.Values[i]);
                ActivePairs.FastRemove(ActivePairs.Keys[i]);
            }
            //Swap the pair sets.
            var temp = ActivePairs;
            ActivePairs = activePairsBackBuffer;
            activePairsBackBuffer = temp;

            //Check if adding the new contacts would overflow the manifold.
            if (contacts.Count + candidatesToAdd.Count > 4)
            {
                //Adding all the contacts would overflow the manifold.  Reduce to the best subset.
                var reducedCandidates = new QuickList<ContactData>(BufferPools<ContactData>.Thread, 3);
                ContactReducer.ReduceContacts(contacts, ref candidatesToAdd, contactIndicesToRemove, ref reducedCandidates);
                RemoveQueuedContacts();
                for (int i = reducedCandidates.Count - 1; i >= 0; i--)
                {
                    Add(ref reducedCandidates.Elements[i]);
                    reducedCandidates.RemoveAt(i);
                }
                reducedCandidates.Dispose();
            }
            else if (candidatesToAdd.Count > 0)
            {
                //Won't overflow the manifold, so just toss it in.
                for (int i = 0; i < candidatesToAdd.Count; i++)
                {
                    Add(ref candidatesToAdd.Elements[i]);
                }
            }


            candidatesToAdd.Dispose();

        }


        protected override void Add(ref ContactData contactCandidate)
        {
            ContactSupplementData supplement;
            supplement.BasePenetrationDepth = contactCandidate.PenetrationDepth;
            var convexTransform = convex.WorldTransform;
            var gridTransform = new RigidTransform(voxelGrid.Position);
            RigidTransform.TransformByInverse(ref contactCandidate.Position, ref convexTransform, out supplement.LocalOffsetA);
            RigidTransform.TransformByInverse(ref contactCandidate.Position, ref gridTransform, out supplement.LocalOffsetB);
            supplementData.Add(ref supplement);
            base.Add(ref contactCandidate);
        }
        protected override void Remove(int contactIndex)
        {
            supplementData.RemoveAt(contactIndex);
            base.Remove(contactIndex);
        }


    }

    public class VoxelGridConvexPairHandler : StandardPairHandler
    {
        public static void EnsurePairsAreRegistered()
        {
            //Assume if one has been added, all have.
            if (!NarrowPhaseHelper.CollisionManagers.ContainsKey(new TypePair(typeof(ConvexCollidable<BoxShape>), typeof(VoxelGrid))))
            {
                var factory = new NarrowPhasePairFactory<VoxelGridConvexPairHandler>();
                NarrowPhaseHelper.CollisionManagers.Add(new TypePair(typeof(ConvexCollidable<BoxShape>), typeof(VoxelGrid)), factory);
                NarrowPhaseHelper.CollisionManagers.Add(new TypePair(typeof(ConvexCollidable<SphereShape>), typeof(VoxelGrid)), factory);
                NarrowPhaseHelper.CollisionManagers.Add(new TypePair(typeof(ConvexCollidable<CapsuleShape>), typeof(VoxelGrid)), factory);
                NarrowPhaseHelper.CollisionManagers.Add(new TypePair(typeof(ConvexCollidable<TriangleShape>), typeof(VoxelGrid)), factory);
                NarrowPhaseHelper.CollisionManagers.Add(new TypePair(typeof(ConvexCollidable<CylinderShape>), typeof(VoxelGrid)), factory);
                NarrowPhaseHelper.CollisionManagers.Add(new TypePair(typeof(ConvexCollidable<ConeShape>), typeof(VoxelGrid)), factory);
                NarrowPhaseHelper.CollisionManagers.Add(new TypePair(typeof(ConvexCollidable<TransformableShape>), typeof(VoxelGrid)), factory);
                NarrowPhaseHelper.CollisionManagers.Add(new TypePair(typeof(ConvexCollidable<MinkowskiSumShape>), typeof(VoxelGrid)), factory);
                NarrowPhaseHelper.CollisionManagers.Add(new TypePair(typeof(ConvexCollidable<WrappedShape>), typeof(VoxelGrid)), factory);
                NarrowPhaseHelper.CollisionManagers.Add(new TypePair(typeof(ConvexCollidable<ConvexHullShape>), typeof(VoxelGrid)), factory);
            }
        }

        private VoxelGrid voxelGrid;
        private ConvexCollidable convex;
        public override Collidable CollidableA
        {
            get { return convex; }
        }

        public override Collidable CollidableB
        {
            get { return voxelGrid; }
        }

        public override Entity EntityA
        {
            get { return convex.Entity; }
        }

        public override Entity EntityB
        {
            get { return null; }
        }

        public override void UpdateTimeOfImpact(Collidable requester, float dt)
        {
            //Complicated and not vital. Leaving it out for simplicity. Check out InstancedMeshPairHandler for an example implementation.
            //Notice that we don't test for convex entity null explicitly.  The convex.IsActive property does that for us.
            if (convex.IsActive && convex.Entity.PositionUpdateMode == PositionUpdateMode.Continuous)
            {
                //Only perform the test if the minimum radii are small enough relative to the size of the velocity.
                Vector3 velocity = convex.Entity.LinearVelocity * dt;
                float velocitySquared = velocity.LengthSquared();

                var minimumRadius = convex.Shape.MinimumRadius * MotionSettings.CoreShapeScaling;
                timeOfImpact = 1;
                if (minimumRadius * minimumRadius < velocitySquared)
                {
                    for (int i = 0; i < contactManifold.ActivePairs.Count; ++i)
                    {
                        var pair = contactManifold.ActivePairs.Values[i];
                        //In the contact manifold, the box collidable is always put into the second slot.
                        var boxCollidable = (ReusableBoxCollidable)pair.CollidableB;
                        RayHit rayHit;
                        var worldTransform = boxCollidable.WorldTransform;
                        if (GJKToolbox.CCDSphereCast(new Ray(convex.WorldTransform.Position, velocity), minimumRadius, boxCollidable.Shape, ref worldTransform, timeOfImpact, out rayHit) &&
                            rayHit.T > Toolbox.BigEpsilon)
                        {
                            timeOfImpact = rayHit.T;
                        }
                    }
                }
            }
        }

        protected override void GetContactInformation(int index, out ContactInformation info)
        {
            info.Contact = contactManifold.Contacts[index];
            //Find the contact's normal and friction forces.
            info.FrictionImpulse = 0;
            info.NormalImpulse = 0;

            for (int i = 0; i < constraint.ContactFrictionConstraints.Count; i++)
            {
                if (constraint.ContactFrictionConstraints[i].PenetrationConstraint.Contact == info.Contact)
                {
                    info.FrictionImpulse = constraint.ContactFrictionConstraints[i].TotalImpulse;
                    info.NormalImpulse = constraint.ContactFrictionConstraints[i].PenetrationConstraint.NormalImpulse;
                    break;
                }
            }

            //Compute relative velocity
            if (convex.Entity != null)
            {
                info.RelativeVelocity = Toolbox.GetVelocityOfPoint(info.Contact.Position, convex.Entity.Position, convex.Entity.LinearVelocity, convex.Entity.AngularVelocity);
            }
            else
                info.RelativeVelocity = new Vector3();


            info.Pair = this;
        }

        public VoxelGridConvexPairHandler()
        {
            constraint = new NonConvexContactManifoldConstraint(this);
        }

        private VoxelGridConvexContactManifold contactManifold = new VoxelGridConvexContactManifold();
        private NonConvexContactManifoldConstraint constraint;
        public override ContactManifold ContactManifold
        {
            get { return contactManifold; }
        }

        public override ContactManifoldConstraint ContactConstraint
        {
            get { return constraint; }
        }


        public override void Initialize(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {

            voxelGrid = entryA as VoxelGrid;
            convex = entryB as ConvexCollidable;

            if (voxelGrid == null || convex == null)
            {
                voxelGrid = entryB as VoxelGrid;
                convex = entryA as ConvexCollidable;

                if (voxelGrid == null || convex == null)
                    throw new ArgumentException("Inappropriate types used to initialize pair.");
            }

            //Contact normal goes from A to B.
            broadPhaseOverlap = new BroadPhaseOverlap(convex, voxelGrid, broadPhaseOverlap.CollisionRule);

            UpdateMaterialProperties(convex.Entity != null ? convex.Entity.Material : null, voxelGrid.Material);


            base.Initialize(entryA, entryB);





        }


        ///<summary>
        /// Cleans up the pair handler.
        ///</summary>
        public override void CleanUp()
        {
            base.CleanUp();
            voxelGrid = null;
            convex = null;

        }
    }

    /// <summary>
    /// Demo showing the voxel grid in action.
    /// </summary>
    public class SimpleVoxelCollidableDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public SimpleVoxelCollidableDemo(DemosGame game)
            : base(game)
        {
            VoxelGridConvexPairHandler.EnsurePairsAreRegistered();
            int cellCountX = 16;
            int cellCountY = 16;
            int cellCountZ = 16;
            var cells = new bool[cellCountX, cellCountY, cellCountZ];
            for (int i = 0; i < cellCountX; ++i)
            {
                for (int j = 0; j < cellCountY; ++j)
                {
                    for (int k = 0; k < cellCountZ; ++k)
                    {
                        cells[i, j, k] = (Math.Sin(i * 0.55f + 6f + j * -0.325f) + Math.Sin(j * 0.35f - 0.5f + MathHelper.PiOver2) + Math.Sin(k * 0.5f + MathHelper.Pi + 6 + j * 0.25f)) > 0;
                    }
                }
            }
            var cellWidth = 1f;
            var shape = new VoxelGridShape(cells, cellWidth);
            var grid = new VoxelGrid(shape, new Vector3(-cellCountX * cellWidth * 0.5f, -cellCountY * cellWidth, -cellCountZ * cellWidth * 0.5f));
            Space.Add(grid);

            int width = 10;
            int height = 10;
            float blockWidth = 2f;
            float blockHeight = 1f;
            float blockLength = 1f;

            for (int i = 0; i < width; i++)
            {
                for (int j = 0; j < height; j++)
                {
                    var toAdd =
                        new Box(
                            new Vector3(
                                i * blockWidth + .5f * blockWidth * (j % 2) - width * blockWidth * .5f,
                                blockHeight * .5f + j * (blockHeight),
                                0),
                            blockWidth, blockHeight, blockLength, 10);
                    Space.Add(toAdd);
                }
            }

            game.Camera.Position = new Vector3(0, 0, 25);

            for (int i = 0; i < cellCountX; ++i)
            {
                for (int j = 0; j < cellCountY; ++j)
                {
                    for (int k = 0; k < cellCountZ; ++k)
                    {
                        if (shape.Cells[i, j, k])
                        {
                            //This is a turbo-inefficient way to render things, but good enough for now. If you want to visualize a larger amount... you'll probably have to write your own.
                            game.ModelDrawer.Add(new DisplayModel(game.Content.Load<Model>("cube"), game.ModelDrawer)
                                {
                                    WorldTransform = Matrix.CreateWorldRH(grid.Position + new Vector3((i + 0.5f) * cellWidth, (j + 0.5f) * cellWidth, (k + 0.5f) * cellWidth), Vector3.Forward, Vector3.Up)
                                });
                        }
                    }
                }
            }

        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Voxel Grid Demo"; }
        }
    }
}