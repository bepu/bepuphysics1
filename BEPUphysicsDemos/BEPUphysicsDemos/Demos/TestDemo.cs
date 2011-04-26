using BEPUphysics.Collidables;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.PositionUpdating;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;
using System.Diagnostics;
using System.Collections.Generic;
using System;
using BEPUphysics.MathExtensions;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.DataStructures;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.CollisionShapes;
using BEPUphysics.Settings;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Constraints;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using BEPUphysics.Constraints.SolverGroups;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class TestDemo : StandardDemo
    {
        void ConstructStuff()
        {
            CompoundBody body = new CompoundBody(
            new List<CompoundShapeEntry>
            {
                new CompoundShapeEntry(new BoxShape(1,1,1), new Vector3(0,.5f,0)),
                new CompoundShapeEntry(new BoxShape(2,1,2), new Vector3(0,-.5f,0))
            }, 10);
            Cylinder wheel = new Cylinder(new Vector3(0, .5f, 0), .1f, 1, 1);
            CollisionRules.AddRule(body, wheel, CollisionRule.NoBroadPhase);
            var joint = new RevoluteJoint(body, wheel, new Vector3(0, 1, 0), Vector3.Up);
            joint.Motor.IsActive = true;
            joint.Motor.Settings.VelocityMotor.GoalVelocity = 1;
            Space.Add(body);
            Space.Add(wheel);
            Space.Add(joint);
        }

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public TestDemo(DemosGame game)
            : base(game)
        {
            Vector3[] vertices;
            int[] indices;


            TriangleMesh.GetVerticesAndIndicesFromModel(game.Content.Load<Model>("cube"), out vertices, out indices);
            AffineTransform transform = new AffineTransform(new Vector3(100, 1, 100), Quaternion.Identity, new Vector3(0, 0, 0));
            StaticMesh staticMesh = new StaticMesh(vertices, indices, transform);
            Space.Add(staticMesh);
            game.ModelDrawer.Add(staticMesh.Mesh);

            TriangleMesh.GetVerticesAndIndicesFromModel(game.Content.Load<Model>("hollowsphere"), out vertices, out indices);
            MotionSettings.DefaultPositionUpdateMode = PositionUpdateMode.Continuous;
            ShapeDistributionInformation info;
            //AffineTransform transform = new AffineTransform(new Vector3(2, 1, 2), Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathHelper.Pi), new Vector3(0, 0, 0));
            transform = new AffineTransform(new Vector3(.03f, .03f, .03f), Quaternion.Identity, new Vector3(0, 0, 0));
            var shape = new MobileMeshShape(vertices, indices, transform, MobileMeshSolidity.Solid, out info);
            //CollisionResponseSettings.PenetrationRecoveryStiffness = 1f;
            //CollisionResponseSettings.MaximumPositionCorrectionSpeed = float.MaxValue;


            Space.Remove(kapow);
            kapow = new Sphere(new Vector3(10000, 0, 0), .5f, 1);
            Space.Add(kapow);
            Matrix3X3 inertia;
            float mass = 100;
            //inertia = new Matrix3X3();
            //inertia.M11 = mass;
            //inertia.M22 = mass;
            //inertia.M33 = mass;
            Matrix3X3.Multiply(ref info.VolumeDistribution, mass * InertiaHelper.InertiaTensorScale, out inertia);
            for (int i = 0; i < 1; i++)
            {
                var entityMesh = new Entity<MobileMeshCollidable>(new MobileMeshCollidable(shape), mass, inertia, info.Volume);
                entityMesh.CollisionInformation.ImproveBoundaryBehavior = true;
                entityMesh.Position = new Vector3(0, 40, 0);
                //entityMesh.AngularVelocity = new Vector3(1, 1, 1);
                //entityMesh.LinearVelocity = new Vector3(1, 0, 0);
                //entityMesh.Material.KineticFriction = 0;
                //entityMesh.Material.StaticFriction = 0;
                //entityMesh.LocalInertiaTensorInverse = new Matrix3X3();
                Space.Add(entityMesh);
                //entityMesh.IsAlwaysActive = true;
                meshCollidable = entityMesh.CollisionInformation;
            }

            //for (int j = 0; j < 1; j++)
            //{
            //    var toAdd = new Box(new Vector3(0, 25 + j * 0, 0), 2, 2, 2, 1);
            //    //toAdd.Material.KineticFriction = 0;
            //    //toAdd.Material.StaticFriction = 0;
            //    Space.Add(toAdd);
            //}

            Space.Add(new Box(new Vector3(0, -10, 0), 100, 1, 100));
            game.Camera.Position = new Vector3(0, 20, 45);
            game.Camera.Yaw = 0;
            game.Camera.Pitch = 0;

            //Space.NarrowPhase.AllowMultithreading = false;
            //Space.Solver.AllowMultithreading = false;

            //Space.ForceUpdater.Gravity = new Vector3();

            ////MeshBoundingBoxTree incrementalTree = new MeshBoundingBoxTree(data);
            ////var topDownTree = new TopDownMeshBoundingBoxTree(data);


            ////List<int> depths;
            ////int minDepth, maxDepth, nodeCount;
            ////incrementalTree.Analyze(out depths, out minDepth, out maxDepth, out nodeCount);
            ////MeshBoundingBoxTree topDownTree = new MeshBoundingBoxTree(data);

            ////topDownTree.Analyze(out depths, out minDepth, out maxDepth, out nodeCount);




            ////var offset = incrementalTree.BoundingBox.Max - incrementalTree.BoundingBox.Min;
            ////var origin = incrementalTree.BoundingBox.Min;



            //List<int> overlaps = new List<int>(10000);
            //List<int> overlaps2 = new List<int>(10000);
            //int numRuns = 10;
            //Vector3 boundingBoxOffset = new Vector3(.01f, .01f, .01f);

            //Random random = new Random(1);
            //double start = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            //for (int i = 0; i < numRuns; i++)
            //{

            //    //overlaps.Clear();
            //    //Vector3 center = origin + new Vector3((float)random.NextDouble() * offset.X, (float)random.NextDouble() * offset.Y, (float)random.NextDouble() * offset.Z);
            //    //Vector3 direction = new Vector3(-.5f + (float)random.NextDouble(), -.5f + (float)random.NextDouble(), -.5f + (float)random.NextDouble());
            //    //BoundingBox overlap = new BoundingBox(center - boundingBoxOffset, center + boundingBoxOffset);
            //    //Ray ray = new Ray(center, direction);

            //    //for (int j = 0; j < 100; j++)
            //    //{
            //    //    topDownTree.GetOverlaps(overlap, overlaps);
            //    //}

            //    var topDownTree = new TopDownMeshBoundingBoxTree(data);

            //}

            //double end = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;


            //time1 = (end - start) / numRuns;

            //random = new Random(1);
            //start = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            //for (int i = 0; i < numRuns; i++)
            //{
            //    //overlaps2.Clear();
            //    //Vector3 center = origin + new Vector3((float)random.NextDouble() * offset.X, (float)random.NextDouble() * offset.Y, (float)random.NextDouble() * offset.Z);
            //    //Vector3 direction = new Vector3(-.5f + (float)random.NextDouble(), -.5f + (float)random.NextDouble(), -.5f + (float)random.NextDouble());
            //    //BoundingBox overlap = new BoundingBox(center - boundingBoxOffset, center + boundingBoxOffset);
            //    //Ray ray = new Ray(center, direction);

            //    //for (int j = 0; j < 100; j++)
            //    //{
            //    //    incrementalTree.GetOverlaps(overlap, overlaps2);
            //    //}

            //    MeshBoundingBoxTree incrementalTree = new MeshBoundingBoxTree(data);
            //}

            //end = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;

            //time2 = (end - start) / numRuns;

            //Console.WriteLine("timeper: " + time1);


        }

        CompoundBody body;
        Box boxA, boxB;

        double time1;
        double time2;
        bool contained = true;
        Random random = new Random();

        MobileMeshCollidable meshCollidable;

        List<Entity> containedEntities = new List<Entity>();

        public override void Update(float dt)
        {
            KeyboardState state = Keyboard.GetState();

            //if (state.IsKeyDown(Microsoft.Xna.Framework.Input.Keys.P))
            //{
            //    Debug.WriteLine("Break.");
            //    var overlaps = new List<BroadPhaseEntry>();
            //    Space.BroadPhase.QueryAccelerator.GetEntries(new BoundingBox(new Vector3(-10000000, -10000000, -10000000), new Vector3(10000000, -50, 10000000)), overlaps);
            //    Debug.WriteLine("Count: " + overlaps.Count);
            //}

            base.Update(dt);
        }

        public override void DrawUI()
        {
            Game.DataTextDrawer.Draw(time1 * 1000000000, new Vector2(50, 100));
            Game.DataTextDrawer.Draw(time2 * 1000000000, new Vector2(50, 130));
            base.DrawUI();
        }


        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Test"; }
        }

        public override void CleanUp()
        {
            //CollisionRules.CollisionGroupRules.Remove(new CollisionGroupPair(CollisionRules.DefaultDynamicCollisionGroup, CollisionRules.DefaultDynamicCollisionGroup));
            base.CleanUp();
        }
        //float minX = -1;
        //float minY = -10;
        //float minZ = -1;
        //float maxX = 1;
        //float maxY = 10;
        //float maxZ = 1;

        //List<Vector3> oldSimplex = new List<Vector3>();
        //List<int> subsimplex = new List<int>();
        //List<float> baryCoords = new List<float>();

        //Random rand = new Random();

        //void TestSimplexes()
        //{
        //    for (int i = 0; i < 100000000; i++)
        //    {
        //        TestSimplex();
        //    }
        //}

        //Vector3 GetRandomVector()
        //{
        //    return new Vector3((float)rand.NextDouble() * (maxX - minX) + minX, (float)rand.NextDouble() * (maxY - minY) + minY, (float)rand.NextDouble() * (maxZ - minZ) + minZ);
        //}

        //void TestSimplex()
        //{
        //    int simplexSize = rand.Next(1, 5);
        //    oldSimplex.Clear();
        //    PairSimplex simplex = new PairSimplex();
        //    switch (simplexSize)
        //    {
        //        case 1:
        //            Vector3 a = GetRandomVector();
        //            simplex.A = a;
        //            simplex.State = SimplexState.Point;
        //            oldSimplex.Add(a);
        //            break;
        //        case 2:
        //            a = GetRandomVector();
        //            Vector3 b = GetRandomVector();

        //            simplex.State = SimplexState.Segment;
        //            simplex.A = a;
        //            simplex.B = b;
        //            oldSimplex.Add(a);
        //            oldSimplex.Add(b);

        //            break;
        //        case 3:
        //            a = GetRandomVector();
        //            b = GetRandomVector();
        //            Vector3 c = GetRandomVector();

        //            simplex.State = SimplexState.Triangle;
        //            simplex.A = a;
        //            simplex.B = b;
        //            simplex.C = c;
        //            oldSimplex.Add(a);
        //            oldSimplex.Add(b);
        //            oldSimplex.Add(c);
        //            break;
        //        case 4:
        //            a = GetRandomVector();
        //            b = GetRandomVector();
        //            c = GetRandomVector();
        //            Vector3 d = GetRandomVector();

        //            simplex.State = SimplexState.Tetrahedron;
        //            simplex.A = a;
        //            simplex.B = b;
        //            simplex.C = c;
        //            simplex.D = d;
        //            oldSimplex.Add(a);
        //            oldSimplex.Add(b);
        //            oldSimplex.Add(c);
        //            oldSimplex.Add(d);
        //            break;
        //    }

        //    VerifySimplex(simplex, oldSimplex);
        //}

        //void VerifySimplex(PairSimplex simplex, List<Vector3> oldSimplex)
        //{
        //    PairSimplex simplexCopy = simplex; //easier to debug
        //    Vector3 closestPoint;
        //    simplexCopy.GetPointClosestToOrigin(out closestPoint);

        //    Vector3 closestPointOld;
        //    OldGJKVerifier.FindPointOfMinimumNorm(oldSimplex, subsimplex, baryCoords, out closestPointOld);

        //    if (closestPoint.Length() - closestPointOld.Length() > .00001f ||
        //        (closestPointOld - closestPoint).Length() > .01f)
        //        Debug.WriteLine("Break.");


        //    List<SimplexEntry> entries = new List<SimplexEntry>();

        //    List<SimplexEntry> oldEntries = new List<SimplexEntry>();
        //    switch (simplexCopy.State)
        //    {
        //        case SimplexState.Point:
        //            if (subsimplex.Count == 1)
        //            {
        //                entries.Add(new SimplexEntry(simplexCopy.A, simplexCopy.U));
        //                oldEntries.Add(new SimplexEntry(oldSimplex[subsimplex[0]], baryCoords[0]));
        //            }
        //            break;
        //        case SimplexState.Segment:
        //            if (subsimplex.Count == 2)
        //            {
        //                entries.Add(new SimplexEntry(simplexCopy.A, simplexCopy.U));
        //                entries.Add(new SimplexEntry(simplexCopy.B, simplexCopy.V));
        //                oldEntries.Add(new SimplexEntry(oldSimplex[subsimplex[0]], baryCoords[0]));
        //                oldEntries.Add(new SimplexEntry(oldSimplex[subsimplex[1]], baryCoords[1]));
        //            }
        //            break;
        //        case SimplexState.Triangle:

        //            if (subsimplex.Count == 3)
        //            {
        //                entries.Add(new SimplexEntry(simplexCopy.A, simplexCopy.U));
        //                entries.Add(new SimplexEntry(simplexCopy.B, simplexCopy.V));
        //                entries.Add(new SimplexEntry(simplexCopy.C, simplexCopy.W));
        //                oldEntries.Add(new SimplexEntry(oldSimplex[subsimplex[0]], baryCoords[0]));
        //                oldEntries.Add(new SimplexEntry(oldSimplex[subsimplex[1]], baryCoords[1]));
        //                oldEntries.Add(new SimplexEntry(oldSimplex[subsimplex[2]], baryCoords[2]));
        //            }
        //            break;
        //    }

        //    for (int i = 0; i < entries.Count; i++)
        //    {
        //        int index;
        //        if ((index = oldEntries.IndexOf(entries[i])) != -1)
        //        {
        //            oldEntries.RemoveAt(index);
        //        }
        //        else
        //            Debug.WriteLine("break.");
        //    }
        //}

        //struct SimplexEntry : IEquatable<SimplexEntry>
        //{
        //    public SimplexEntry(Vector3 position, float weight)
        //    {
        //        this.position = position;
        //        this.weight = weight;
        //    }
        //    internal Vector3 position;
        //    internal float weight;



        //    public bool Equals(SimplexEntry other)
        //    {
        //        return Vector3.Distance(position, other.position) < .0001f && Math.Abs(weight - other.weight) < .0001f;
        //    }
        //}
    }
}