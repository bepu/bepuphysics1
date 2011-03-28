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

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class TestDemo : StandardDemo
    {

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public TestDemo(DemosGame game)
            : base(game)
        {

            Random random = new Random();
            MotionSettings.DefaultPositionUpdateMode = PositionUpdateMode.Continuous;
            MotionSettings.UseExtraExpansionForContinuousBoundingBoxes = true;
            //CollisionRules.CollisionGroupRules.Add(new CollisionGroupPair(CollisionRules.DefaultDynamicCollisionGroup, CollisionRules.DefaultDynamicCollisionGroup), CollisionRule.NoBroadPhase);
#if XBOX360
            for (int i = 0; i < 300; i++)
#else
            for (int i = 0; i < 1000; i++)
#endif
            {
                Space.Add(new Sphere(new Vector3((i % 2) * .5f, 10 + i * 4, (i % 3) * .33f), .5f, 1) { Material = new BEPUphysics.Materials.Material(1, 1, .2f) });
                //Space.Add(new Sphere(new Vector3(-20 + (float)random.NextDouble() * 40, 10 + i * 4, -20 + (float)random.NextDouble() * 40), .5f, 1));
            }

            Space.ForceUpdater.Gravity = new Vector3(0, -18, 0);
            Space.Add(new Box(new Vector3(0, 0, 0), 10, 0, 10));
            //Space.Solver.IterationLimit = 10;
            //SolverSettings.DefaultMinimumIterations = 5;
            //Space.PositionUpdater.AllowMultithreading = false;
            //Space.NarrowPhase.AllowMultithreading = false;


            Space.Add(new Box(new Vector3(5000, -20, 0), 10000 - 10, 40, 10000));
            Space.Add(new Box(new Vector3(-5000, -20, 0), 10000 - 10, 40, 10000));
            Space.Add(new Box(new Vector3(0, -20, 5000), 10, 40, 10000 - 10));
            Space.Add(new Box(new Vector3(0, -20, -5000), 10, 40, 10000 - 10));
            

        }

        CompoundBody body;
        Box boxA, boxB;

        double time;
        bool contained = true;
        Random random = new Random();

        List<Entity> containedEntities = new List<Entity>();

        public override void Update(float dt)
        {
            KeyboardState state = Keyboard.GetState();

            if (state.IsKeyDown(Keys.P))
            {
                Debug.WriteLine("Break.");
                var overlaps = new List<BroadPhaseEntry>();
                Space.BroadPhase.QueryAccelerator.GetEntries(new BoundingBox(new Vector3(-10000000, -10000000, -10000000), new Vector3(10000000, -50, 10000000)), overlaps);
                Debug.WriteLine("Count: " + overlaps.Count);
            }

            base.Update(dt);
        }

        public override void DrawUI()
        {
            Game.DataTextDrawer.Draw(time, new Vector2(0, 100));
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