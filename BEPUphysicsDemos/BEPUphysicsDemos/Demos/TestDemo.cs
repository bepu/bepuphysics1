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
using System.Windows.Forms;

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

            Vector3[] vertices;
            int[] indices;

            TriangleMesh.GetVerticesAndIndicesFromModel(game.Content.Load<Model>("barrelandplatform"), out vertices, out indices);
            StaticMeshData data = new StaticMeshData(vertices, indices);


            float radius = .5f;
            var scaleTransform = Matrix.CreateScale(20 * radius);

            List<CompoundShapeEntry> triangles = new List<CompoundShapeEntry>(indices.Length / 3);

            for (int i = 0; i < indices.Length; i += 3)
            {
                // [Recentered Triangle]
                Vector3 centre;
                TriangleShape tri = new TriangleShape(
                    Vector3.Transform(vertices[indices[i]], scaleTransform),
                    Vector3.Transform(vertices[indices[i + 1]], scaleTransform),
                    Vector3.Transform(vertices[indices[i + 2]], scaleTransform),
                    out centre
                    );

                //Depending on the winding of the model, using clockwise or counterclockwise will permit
                //objects to move freely from the inside to the outside, or from the outside inside.
                //Doublesided stops both ways.  (It's the default too, so setting this isn't really necessary)
                tri.Sidedness = TriangleSidedness.DoubleSided;
                //By now, if we checked the triangle shape's vertices, they would be in the triangle's local space.
                //If we added centre to the vertices, they would be in the original world space locations.
                //Knowing that, we can pass the centre to the DynamicCompoundEntry's position and they will be in the correct location.
                triangles.Add(
                    new CompoundShapeEntry(
                        tri,
                        centre,
                        1f
                    )
                );
            }

            var body = new CompoundBody(triangles, 100);

            Space.Add(body);

            Space.Add(new Box(new Vector3(0, -25, 0), 100, 1, 100));


            //MeshBoundingBoxTree incrementalTree = new MeshBoundingBoxTree(data);


            ////List<int> depths;
            ////int minDepth, maxDepth, nodeCount;
            ////incrementalTree.Analyze(out depths, out minDepth, out maxDepth, out nodeCount);
            ////MeshBoundingBoxTree topDownTree = new MeshBoundingBoxTree(data);

            ////topDownTree.Analyze(out depths, out minDepth, out maxDepth, out nodeCount);




            //var offset = incrementalTree.BoundingBox.Max - incrementalTree.BoundingBox.Min;
            //var origin = incrementalTree.BoundingBox.Min;



            //List<int> overlaps = new List<int>(10000);
            //List<int> overlaps2 = new List<int>(10000);
            //int numRuns = 1000000;
            //Vector3 boundingBoxOffset = new Vector3(1f, 1f, 1f);

            //Random random = new Random(1);
            //double start = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            ////for (int i = 0; i < numRuns / 100; i++)
            ////{

            ////    overlaps.Clear();
            ////    Vector3 center = origin + new Vector3((float)random.NextDouble() * offset.X, (float)random.NextDouble() * offset.Y, (float)random.NextDouble() * offset.Z);
            ////    Vector3 direction = new Vector3(-.5f + (float)random.NextDouble(), -.5f + (float)random.NextDouble(), -.5f + (float)random.NextDouble());
            ////    //BoundingBox overlap = new BoundingBox(center - boundingBoxOffset, center + boundingBoxOffset);
            ////    Ray ray = new Ray(center, direction);

            ////    for (int j = 0; j < 100; j++)
            ////    {
            ////        topDownTree.RayCast(ray, overlaps);
            ////    }


            ////}

            //double end = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;


            //time1 = (end - start) / numRuns;

            //random = new Random(1);
            //start = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            //for (int i = 0; i < numRuns / 100; i++)
            //{
            //    overlaps2.Clear();
            //    Vector3 center = origin + new Vector3((float)random.NextDouble() * offset.X, (float)random.NextDouble() * offset.Y, (float)random.NextDouble() * offset.Z);
            //    Vector3 direction = new Vector3(-.5f + (float)random.NextDouble(), -.5f + (float)random.NextDouble(), -.5f + (float)random.NextDouble());
            //    //BoundingBox overlap = new BoundingBox(center - boundingBoxOffset, center + boundingBoxOffset);
            //    Ray ray = new Ray(center, direction);

            //    for (int j = 0; j < 100; j++)
            //    {
            //        incrementalTree.GetOverlaps(ray, overlaps2);
            //    }

            //}

            //end = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;

            //time2 = (end - start) / numRuns;

            ////Console.WriteLine("timeper: " + time1);
            //Clipboard.SetData(DataFormats.StringFormat, "Top down: " + time1 + ", Incremental: " + time2);


        }

        CompoundBody body;
        Box boxA, boxB;

        double time1;
        double time2;
        bool contained = true;
        Random random = new Random();

        List<Entity> containedEntities = new List<Entity>();

        public override void Update(float dt)
        {
            KeyboardState state = Keyboard.GetState();

            if (state.IsKeyDown(Microsoft.Xna.Framework.Input.Keys.P))
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
            Game.DataTextDrawer.Draw(time1 * 1000000000, new Vector2(0, 100));
            Game.DataTextDrawer.Draw(time2 * 1000000000, new Vector2(0, 130));
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