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

            float radius = 1;
            var scaleTransform = Matrix.CreateScale(10 * radius);

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
                        1
                    )
                );
            }


            body = new CompoundBody(triangles, 400);

            Space.Add(body);

            Space.Add(new Box(new Vector3(0, -50, 0), 100, 1, 100));


            game.Camera.Position = new Vector3(0, 0, 10);
            
            //Vector3[] vertices;
            //int[] indices;
            //TriangleMesh.GetVerticesAndIndicesFromModel(game.Content.Load<Model>("playground"), out vertices, out indices);
            ////InstancedMesh mesh = new InstancedMesh(new InstancedMeshShape(vertices, indices), new AffineTransform(new Vector3(0, -15, 0)));
            //var mesh = new StaticMesh(vertices, indices, new AffineTransform(Matrix3X3.CreateFromAxisAngle(Vector3.Up, MathHelper.Pi), new Vector3(0, -10, 0)));
            //mesh.Sidedness = TriangleSidedness.DoubleSided;
            //Space.Add(mesh);
            //game.ModelDrawer.Add(mesh.Mesh);

            //Triangle t = new Triangle(new Vector3(0, -10, 0), new Vector3(10, -10, 0), new Vector3(0, -10, 10));
            //t.Sidedness = TriangleSidedness.Clockwise;
            //Space.Add(t);
            //t.Position -= new Vector3(10, 0, 0);
            ////Space.ForceUpdater.Gravity = new Vector3();

            //game.Camera.Position = new Vector3(0, 0, 0);

            //for (int i = 0; i < 100; i++)
            //{
            //    Entity toAdd = new Box(new Vector3((float)random.NextDouble() * 10, (float)random.NextDouble() * 10, (float)random.NextDouble() * 10), 1, 1, 1, 1);
            //    containedEntities.Add(toAdd);
            //    Space.Add(toAdd);
            //}

            //boxA = new Box(new Vector3(0, 50, 0), 1, 1, 1, 1);
            //boxA.PositionUpdateMode = PositionUpdateMode.Continuous;
            //Space.Add(boxA);

            Space.NarrowPhase.AllowMultithreading = false;
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

            if (state.IsKeyDown(Keys.O))
            {
                foreach (CompoundConvexPairHandler pair in body.CollisionInformation.Pairs)
                {
                    foreach (ContactInformation contactInfo in pair.Contacts)
                    {
                        if (contactInfo.Contact.PenetrationDepth > 1)
                            Debug.WriteLine("Breka.");
                    }
                }
            }

            if (state.IsKeyDown(Keys.P))
            {
                Debug.WriteLine("Break.");
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