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
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUphysics.CollisionTests;
using BEPUphysics;
//using BEPUphysics.CollisionTests.CollisionAlgorithms.Testing;

namespace BEPUphysicsDemos.Demos.Tests
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class MPRTestDemo : StandardDemo
    {


        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public MPRTestDemo(DemosGame game)
            : base(game)
        {
            var shapeA = new BoxShape(1, 1, 1);
            shapeA.CollisionMargin = 0;
            var shapeB = new BoxShape(1, 1, 1);
            shapeB.CollisionMargin = 0;

            var transformA = new RigidTransform(new Vector3(0, 0, 0));
            var transformB = new RigidTransform(new Vector3(.5f, .5f, 0));
            Vector3 overlap;
            bool overlapped = MPRToolbox.GetLocalOverlapPosition(shapeA, shapeB, ref transformB, out overlap);
            Vector3 normal;
            float depth;
            Vector3 direction = new Vector3(0, -1, 0);
            MPRToolbox.LocalSurfaceCast(shapeA, shapeB, ref transformB, ref direction, out depth, out normal);

            ContactData contactData;
            //bool overlappedOld = MPRToolboxOld.AreObjectsColliding(shapeA, shapeB, ref transformA, ref transformB, out contactData);

            //Random rand = new Random(0);
            //for (int i = 0; i < 10000000; i++)
            //{
            //    transformA = new RigidTransform(new Vector3((float)rand.NextDouble() * 10 - 5, (float)rand.NextDouble() * 10 - 5, (float)rand.NextDouble() * 10 - 5),
            //        Quaternion.CreateFromYawPitchRoll((float)rand.NextDouble() * 1000, (float)rand.NextDouble() * 1000, (float)rand.NextDouble() * 1000));
            //    transformB = new RigidTransform(new Vector3((float)rand.NextDouble() * 10 - 5, (float)rand.NextDouble() * 10 - 5, (float)rand.NextDouble() * 10 - 5),
            //        Quaternion.CreateFromYawPitchRoll((float)rand.NextDouble() * 1000, (float)rand.NextDouble() * 1000, (float)rand.NextDouble() * 1000));

            //    overlapped = MPRTesting.GetOverlapPosition(shapeA, shapeB, ref transformA, ref transformB, out overlap);

            //    overlappedOld = MPRToolbox.AreObjectsColliding(shapeA, shapeB, ref transformA, ref transformB, out contactData);

            //    if (overlapped && !overlappedOld &&
            //        (!MPRToolbox.IsPointInsideShape(ref overlap, shapeA, ref transformA) ||
            //        !MPRToolbox.IsPointInsideShape(ref overlap, shapeB, ref transformB)))
            //        Debug.WriteLine("Break.");
            //    if (overlappedOld && !overlapped &&
            //        (!MPRToolbox.IsPointInsideShape(ref contactData.Position, shapeA, ref transformA) ||
            //        !MPRToolbox.IsPointInsideShape(ref contactData.Position, shapeB, ref transformB)))
            //        Debug.WriteLine("Break.");
            //    if (overlapped && overlappedOld &&
            //        (!MPRToolbox.IsPointInsideShape(ref overlap, shapeA, ref transformA) ||
            //        !MPRToolbox.IsPointInsideShape(ref overlap, shapeB, ref transformB) ||
            //        !MPRToolbox.IsPointInsideShape(ref contactData.Position, shapeA, ref transformA) ||
            //        !MPRToolbox.IsPointInsideShape(ref contactData.Position, shapeB, ref transformB)))
            //        Debug.WriteLine("Break.");
            //}
            
            //Do these tests with rotationally immobile objects.
            CollisionDetectionSettings.DefaultMargin = 0;
            groundWidth = 10;
            groundHeight = .1f;
            groundLength = 10;
            //a = new Box(new Vector3(0, -5, 0), groundWidth, groundHeight, groundLength, 1);
            //a = new TransformableEntity(new Vector3(0,0,0), new TriangleShape(new Vector3(-5, -5, -5), new Vector3(5, -5, -5), new Vector3(-5, -5, 5)), Matrix3X3.Identity);         
            a = new Triangle(new Vector3(0, -5, 0), new Vector3(5, -5, 0), new Vector3(5, -5, 5), 1);
            Space.Add(a);
            
            Space.ForceUpdater.Gravity = new Vector3();
            boxWidth = .25f;
            boxHeight = .05f;
            boxLength = 1f;
            b = new TransformableEntity(new Vector3(0, 2, 0), new BoxShape(boxWidth, boxHeight, boxLength), Matrix3X3.Identity, 1);
            //b = new Cone(new Vector3(0, 2, 0), .2f, .1f, 1);
            //b = new Capsule(new Vector3(0, 2, 0), 1, .5f, 1);
            //b = new Capsule(new Vector3(0, 2, 0), 1, .5f, 1);
            b.LocalInertiaTensorInverse = new Matrix3X3();
            CollisionRules.AddRule(b, a, CollisionRule.NoSolver);
            b.ActivityInformation.IsAlwaysActive = true;
            Space.Add(b);
            //Space.Add(new TransformableEntity(new Vector3(0, 4, 0), new BoxShape(1, 1, 1), Matrix3X3.Identity, 1));
            //Space.Add( new TransformableEntity(new Vector3(0, 6, 0), new BoxShape(1, 1, 1), Matrix3X3.Identity, 1));

            //Vector3[] vertices = new Vector3[] { new Vector3(0, -5, 0), new Vector3(5, -5, 0), new Vector3(5, -5, 5), new Vector3(0, -60, 5) };
            //int[] indices = new int[] { 0, 1, 2 , 0, 2, 3 };
            //StaticMesh mesh = new StaticMesh(vertices, indices);
            //Space.Add(mesh);
            //mesh.ImproveBoundaryBehavior = true;
            //mesh.Sidedness = TriangleSidedness.Counterclockwise;
            //game.ModelDrawer.Add(mesh);
            //mesh.CollisionRules.Personal = CollisionRule.NoSolver;
        }

        Entity a;
        Entity b;
        float groundWidth, groundHeight, groundLength;
        float boxWidth, boxHeight, boxLength;


        Vector3 rayCastDirection = Vector3.Up;

        public override void Update(float dt)
        {

            if (Game.KeyboardInput.IsKeyDown(Keys.Left))
                rayCastDirection = Vector3.Transform(rayCastDirection, Matrix.CreateFromAxisAngle(Vector3.Forward, .01f));
            if (Game.KeyboardInput.IsKeyDown(Keys.Right))
                rayCastDirection = Vector3.Transform(rayCastDirection, Matrix.CreateFromAxisAngle(Vector3.Forward, -.01f));
            if (Game.KeyboardInput.IsKeyDown(Keys.Down))
                rayCastDirection = Vector3.Transform(rayCastDirection, Matrix.CreateFromAxisAngle(Vector3.Right, .01f));
            if (Game.KeyboardInput.IsKeyDown(Keys.Up))
                rayCastDirection = Vector3.Transform(rayCastDirection, Matrix.CreateFromAxisAngle(Vector3.Right, -.01f));


            if (Game.KeyboardInput.IsKeyDown(Keys.P))
                Debug.WriteLine("Break.");

            base.Update(dt);

            RigidTransform localTransformB;
            RigidTransform aTransform = a.CollisionInformation.WorldTransform, bTransform = b.CollisionInformation.WorldTransform;
            MinkowskiToolbox.GetLocalTransform(ref aTransform, ref bTransform, out localTransformB);

            Vector3 position;
            if (MPRToolbox.GetLocalOverlapPosition((a.CollisionInformation.Shape as ConvexShape), (b.CollisionInformation.Shape as ConvexShape), ref localTransformB, out position))
            {
                //Vector3 rayCastDirection = new Vector3(1,0,0);// (Vector3.Normalize(localDirection) + Vector3.Normalize(collidableB.worldTransform.Position - collidableA.worldTransform.Position)) / 2;
                float previousT;
                Vector3 previousNormal;
                float t;
                Vector3 normal;

                rayCastDirection = localTransformB.Position;
                MPRToolbox.LocalSurfaceCast((a.CollisionInformation.Shape as ConvexShape), (b.CollisionInformation.Shape as ConvexShape), ref localTransformB, ref rayCastDirection, out previousT, out previousNormal);
                //Vector3 secondDirection = Vector3.Cross(rayCastDirection, Vector3.Up);
                //MPRTesting.LocalSurfaceCast((a.CollisionInformation.Shape as ConvexShape), (b.CollisionInformation.Shape as ConvexShape), ref localTransformB, ref secondDirection, out t, out normal);
                //if (t < previousT)
                //{
                //    previousNormal = normal;
                //    previousT = t;
                //}
                //Vector3 thirdDirection = Vector3.Cross(secondDirection, rayCastDirection);
                //MPRTesting.LocalSurfaceCast((a.CollisionInformation.Shape as ConvexShape), (b.CollisionInformation.Shape as ConvexShape), ref localTransformB, ref thirdDirection, out t, out normal);
                //if (t < previousT)
                //{
                //    previousNormal = normal;
                //    previousT = t;
                //}
                //Vector3 fourthDirection = -secondDirection;
                //MPRTesting.LocalSurfaceCast((a.CollisionInformation.Shape as ConvexShape), (b.CollisionInformation.Shape as ConvexShape), ref localTransformB, ref fourthDirection, out t, out normal);
                //if (t < previousT)
                //{
                //    previousNormal = normal;
                //    previousT = t;
                //} 
                //Vector3 fifthDirection = -thirdDirection;
                //MPRTesting.LocalSurfaceCast((a.CollisionInformation.Shape as ConvexShape), (b.CollisionInformation.Shape as ConvexShape), ref localTransformB, ref fifthDirection, out t, out normal);
                //if (t < previousT)
                //{
                //    previousNormal = normal;
                //    previousT = t;
                //}

                //Correct the penetration depth.

                MPRToolbox.LocalSurfaceCast((a.CollisionInformation.Shape as ConvexShape), (b.CollisionInformation.Shape as ConvexShape), ref localTransformB, ref previousNormal, out t, out normal);
                contactDepth = t;
                contactNormal = previousNormal;

                ////Converge to local minimum.
                //while (true)
                //{
                //    MPRTesting.LocalSurfaceCast((a.CollisionInformation.Shape as ConvexShape), (b.CollisionInformation.Shape as ConvexShape), ref localTransformB, ref previousNormal, out t, out normal);
                //    if (previousT - t <= Toolbox.BigEpsilon)
                //        break;

                //    previousT = t;
                //    previousNormal = normal;
                //}
            }

            #region Box Box minkowski sum
            ////Construct explicit minkowski sum.
            //Vector3[] aLines = new Vector3[8];
            //aLines[0] = new Vector3(-boxWidth / 2, -boxHeight / 2, -boxLength / 2);
            //aLines[1] = new Vector3(-boxWidth / 2, -boxHeight / 2, boxLength / 2);
            //aLines[2] = new Vector3(-boxWidth / 2, boxHeight / 2, -boxLength / 2);
            //aLines[3] = new Vector3(-boxWidth / 2, boxHeight / 2, boxLength / 2);
            //aLines[4] = new Vector3(boxWidth / 2, -boxHeight / 2, -boxLength / 2);
            //aLines[5] = new Vector3(boxWidth / 2, -boxHeight / 2, boxLength / 2);
            //aLines[6] = new Vector3(boxWidth / 2, boxHeight / 2, -boxLength / 2);
            //aLines[7] = new Vector3(boxWidth / 2, boxHeight / 2, boxLength / 2);

            //Vector3[] bLines = new Vector3[8];
            //bLines[0] = new Vector3(-groundWidth / 2, -groundHeight / 2, -groundLength / 2);
            //bLines[1] = new Vector3(-groundWidth / 2, -groundHeight / 2, groundLength / 2);
            //bLines[2] = new Vector3(-groundWidth / 2, groundHeight / 2, -groundLength / 2);
            //bLines[3] = new Vector3(-groundWidth / 2, groundHeight / 2, groundLength / 2);
            //bLines[4] = new Vector3(groundWidth / 2, -groundHeight / 2, -groundLength / 2);
            //bLines[5] = new Vector3(groundWidth / 2, -groundHeight / 2, groundLength / 2);
            //bLines[6] = new Vector3(groundWidth / 2, groundHeight / 2, -groundLength / 2);
            //bLines[7] = new Vector3(groundWidth / 2, groundHeight / 2, groundLength / 2);

            //for (int i = 0; i < 8; i++)
            //    aLines[i] = Vector3.Transform(aLines[i], localTransformB.Matrix);

            //List<Vector3> vertices = new List<Vector3>();
            //for (int i = 0; i < 8; i++)
            //{
            //    for (int j = 0; j < 8; j++)
            //    {

            //        if (b.CollisionInformation.Pairs.Count > 0)
            //        {
            //            if (b.CollisionInformation.Pairs[0].BroadPhaseOverlap.EntryA == b.CollisionInformation)
            //                vertices.Add(aLines[i] - bLines[j]);
            //            else
            //                vertices.Add(bLines[i] - aLines[j]);
            //        }
            //        else
            //        {
            //            vertices.Add(bLines[i] - aLines[j]);
            //        }
            //    }
            //}

            //var indices = new List<int>();
            //Toolbox.GetConvexHull(vertices, indices);
            #endregion

            #region Arbitrary minkowski sum
            var vertices = new List<Vector3>();
            Vector3 max;
            var direction = new Vector3();
            int NumSamples = 16;
            float angleChange = MathHelper.TwoPi / NumSamples;

            for (int i = 1; i < NumSamples / 2 - 1; i++)
            {
                float phi = MathHelper.PiOver2 - i * angleChange;
                var sinPhi = (float)Math.Sin(phi);
                var cosPhi = (float)Math.Cos(phi);
                for (int j = 0; j < NumSamples; j++)
                {
                    float theta = j * angleChange;
                    direction.X = (float)Math.Cos(theta) * cosPhi;
                    direction.Y = sinPhi;
                    direction.Z = (float)Math.Sin(theta) * cosPhi;


                    MinkowskiToolbox.GetLocalMinkowskiExtremePoint(a.CollisionInformation.Shape as ConvexShape, b.CollisionInformation.Shape as ConvexShape, ref direction, ref localTransformB, out max);

                    vertices.Add(max);
                }
            }


            MinkowskiToolbox.GetLocalMinkowskiExtremePoint(a.CollisionInformation.Shape as ConvexShape, b.CollisionInformation.Shape as ConvexShape, ref Toolbox.UpVector, ref localTransformB, out max);
            vertices.Add(max);
            MinkowskiToolbox.GetLocalMinkowskiExtremePoint(a.CollisionInformation.Shape as ConvexShape, b.CollisionInformation.Shape as ConvexShape, ref Toolbox.DownVector, ref localTransformB, out max);
            vertices.Add(max);



            var indices = new List<int>();
            Toolbox.GetConvexHull(vertices, indices);
            #endregion

            minkowskiLines.Clear();
            for (int i = 0; i < indices.Count; i += 3)
            {
                minkowskiLines.Add(new VertexPositionColor(vertices[indices[i]], Color.Blue));
                minkowskiLines.Add(new VertexPositionColor(vertices[indices[i + 1]], Color.Blue));

                minkowskiLines.Add(new VertexPositionColor(vertices[indices[i + 1]], Color.Blue));
                minkowskiLines.Add(new VertexPositionColor(vertices[indices[i + 2]], Color.Blue));

                minkowskiLines.Add(new VertexPositionColor(vertices[indices[i + 2]], Color.Blue));
                minkowskiLines.Add(new VertexPositionColor(vertices[indices[i]], Color.Blue));
            }

        }



        public override void DrawUI()
        {
            //Vector3 screenContactPosition = Game.GraphicsDevice.Viewport.Project(MPRTesting.DEBUGlastPosition, Game.Camera.ProjectionMatrix, Game.Camera.ViewMatrix, Matrix.Identity);
            //Vector3 screenContactPosition = Game.GraphicsDevice.Viewport.Project(Vector3.Zero, Game.Camera.ProjectionMatrix, Game.Camera.ViewMatrix, Matrix.Identity);
            //Game.UIDrawer.Draw(whitePixel, new Rectangle((int)screenContactPosition.X - 2, (int)screenContactPosition.Y - 2, 4, 4), Color.White);

            //Vector3 v1 = Game.GraphicsDevice.Viewport.Project(MPRTesting.DEBUGlastV1, Game.Camera.ProjectionMatrix, Game.Camera.ViewMatrix, Matrix.Identity);
            //Game.UIDrawer.Draw(whitePixel, new Rectangle((int)v1.X - 3, (int)v1.Y - 3, 6, 6), Color.Red);

            //Vector3 v2 = Game.GraphicsDevice.Viewport.Project(MPRTesting.DEBUGlastV2, Game.Camera.ProjectionMatrix, Game.Camera.ViewMatrix, Matrix.Identity);
            //Game.UIDrawer.Draw(whitePixel, new Rectangle((int)v2.X - 3, (int)v2.Y - 3, 6, 6), Color.Red);

            //Vector3 v3 = Game.GraphicsDevice.Viewport.Project(MPRTesting.DEBUGlastV3, Game.Camera.ProjectionMatrix, Game.Camera.ViewMatrix, Matrix.Identity);
            //Game.UIDrawer.Draw(whitePixel, new Rectangle((int)v3.X - 3, (int)v3.Y - 3, 6, 6), Color.Red);

            //Vector3 rayHit = Game.GraphicsDevice.Viewport.Project(MPRTesting.DEBUGlastRayDirection * MPRTesting.DEBUGlastRayT, Game.Camera.ProjectionMatrix, Game.Camera.ViewMatrix, Matrix.Identity);
            //Game.UIDrawer.Draw(whitePixel, new Rectangle((int)rayHit.X - 3, (int)rayHit.Y - 3, 6, 6), Color.Purple);

            base.DrawUI();
        }

        List<VertexPositionColor> minkowskiLines = new List<VertexPositionColor>();

        Vector3 contactNormal;
        float contactDepth;

        VertexPositionColor[] lines = new VertexPositionColor[12];
        public override void Draw()
        {
            //Add the raycast.  Starts at contact position, goes to contact position + rayDirection * T;
            //Vector3 basePosition = MPRTesting.DEBUGlastPosition;
            //Vector3 basePosition = Vector3.Zero;
            //lines[0] = new VertexPositionColor(basePosition, Color.Red);
            //lines[1] = new VertexPositionColor(basePosition + MPRTesting.DEBUGlastRayDirection * MPRTesting.DEBUGlastRayT, Color.Red);
            ////Add the normal.  It goes from the surface hit plus the normal.
            //lines[2] = new VertexPositionColor(basePosition + MPRTesting.DEBUGlastRayDirection * MPRTesting.DEBUGlastRayT, Color.Purple);
            //lines[3] = new VertexPositionColor(basePosition + MPRTesting.DEBUGlastRayDirection * MPRTesting.DEBUGlastRayT + contactNormal, Color.Purple);
            ////Add another line from the contact position along the normal with the depth.
            //lines[4] = new VertexPositionColor(basePosition, Color.White);
            //lines[5] = new VertexPositionColor(basePosition + contactNormal * contactDepth, Color.White);

            ////Add the v1v2v3 triangle.
            //lines[6] = new VertexPositionColor(MPRTesting.DEBUGlastV1, Color.Red);
            //lines[7] = new VertexPositionColor(MPRTesting.DEBUGlastV2, Color.Red);
            //lines[8] = new VertexPositionColor(MPRTesting.DEBUGlastV2, Color.Red);
            //lines[9] = new VertexPositionColor(MPRTesting.DEBUGlastV3, Color.Red);
            //lines[10] = new VertexPositionColor(MPRTesting.DEBUGlastV3, Color.Red);
            //lines[11] = new VertexPositionColor(MPRTesting.DEBUGlastV1, Color.Red);

            //foreach (EffectPass pass in Game.LineDrawer.CurrentTechnique.Passes)
            //{
            //    pass.Apply();

            //    Game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, lines, 0, lines.Length / 2);
            //}

            //foreach (EffectPass pass in Game.LineDrawer.CurrentTechnique.Passes)
            //{
            //    pass.Apply();

            //    Game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, minkowskiLines.ToArray(), 0, minkowskiLines.Count / 2);
            //}
            base.Draw();
        }


        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Test2"; }
        }

    }
}