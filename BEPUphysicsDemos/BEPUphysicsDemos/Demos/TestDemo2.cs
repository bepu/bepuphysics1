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

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class TestDemo2 : StandardDemo
    {


        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public TestDemo2(DemosGame game)
            : base(game)
        {
            var shapeA = new BoxShape(1, 1, 1);
            shapeA.CollisionMargin = 0;
            var shapeB = new BoxShape(1, 1, 1);
            shapeB.CollisionMargin = 0;

            var transformA = new RigidTransform(new Vector3(0, 0, 0));
            var transformB = new RigidTransform(new Vector3(.5f, .5f, 0));
            Vector3 overlap;
            bool overlapped = MPRTesting.GetLocalOverlapPosition(shapeA, shapeB, ref transformB, out overlap);
            Vector3 normal;
            float depth;
            Vector3 direction = new Vector3(0, -1, 0);
            MPRTesting.LocalSurfaceCast(shapeA, shapeB, ref transformB, ref direction, out depth, out normal);

            ContactData contactData;
            bool overlappedOld = MPRToolbox.AreObjectsColliding(shapeA, shapeB, ref transformA, ref transformB, out contactData);

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
            groundHeight = 0f;
            groundLength = 10;
            a = new Box(new Vector3(0, -5, 0), groundWidth, groundHeight, groundLength);
            //a = new TransformableEntity(new Vector3(0,0,0), new TriangleShape(new Vector3(-5, -5, -5), new Vector3(5, -5, -5), new Vector3(-5, -5, 5)), Matrix3X3.Identity);
            Space.Add(a);

            Space.ForceUpdater.Gravity = new Vector3();
            boxWidth = 1;
            boxHeight = .05f;
            boxLength = .25f;
            b = new TransformableEntity(new Vector3(0, 2, 0), new BoxShape(boxWidth, boxHeight, boxLength), Matrix3X3.Identity, 1);
            //b = new Triangle(new Vector3(0, 2, 0), new Vector3(1, 2, 0), new Vector3(1, 2, 0), 1);
            //box.LocalInertiaTensorInverse = new Matrix3X3();
            CollisionRules.AddRule(b, a, CollisionRule.NoSolver);
            b.IsAlwaysActive = true;
            Space.Add(b);
            //Space.Add(new TransformableEntity(new Vector3(0, 4, 0), new BoxShape(1, 1, 1), Matrix3X3.Identity, 1));
            //Space.Add( new TransformableEntity(new Vector3(0, 6, 0), new BoxShape(1, 1, 1), Matrix3X3.Identity, 1));



        }

        Entity a;
        Entity b;
        float groundWidth, groundHeight, groundLength;
        float boxWidth, boxHeight, boxLength;


        public override void Update(float dt)
        {


            base.Update(dt);

            //Construct explicit minkowski sum.
            Vector3[] aLines = new Vector3[8];
            aLines[0] = new Vector3(-boxWidth / 2, -boxHeight / 2, -boxLength / 2);
            aLines[1] = new Vector3(-boxWidth / 2, -boxHeight / 2, boxLength / 2);
            aLines[2] = new Vector3(-boxWidth / 2, boxHeight / 2, -boxLength / 2);
            aLines[3] = new Vector3(-boxWidth / 2, boxHeight / 2, boxLength / 2);
            aLines[4] = new Vector3(boxWidth / 2, -boxHeight / 2, -boxLength / 2);
            aLines[5] = new Vector3(boxWidth / 2, -boxHeight / 2, boxLength / 2);
            aLines[6] = new Vector3(boxWidth / 2, boxHeight / 2, -boxLength / 2);
            aLines[7] = new Vector3(boxWidth / 2, boxHeight / 2, boxLength / 2);

            for (int i = 0; i < 8; i++)
                aLines[i] = Vector3.Transform(aLines[i], b.WorldTransform);

            Vector3[] bLines = new Vector3[8];
            bLines[0] = new Vector3(-groundWidth / 2, -groundHeight / 2, -groundLength / 2);
            bLines[1] = new Vector3(-groundWidth / 2, -groundHeight / 2, groundLength / 2);
            bLines[2] = new Vector3(-groundWidth / 2, groundHeight / 2, -groundLength / 2);
            bLines[3] = new Vector3(-groundWidth / 2, groundHeight / 2, groundLength / 2);
            bLines[4] = new Vector3(groundWidth / 2, -groundHeight / 2, -groundLength / 2);
            bLines[5] = new Vector3(groundWidth / 2, -groundHeight / 2, groundLength / 2);
            bLines[6] = new Vector3(groundWidth / 2, groundHeight / 2, -groundLength / 2);
            bLines[7] = new Vector3(groundWidth / 2, groundHeight / 2, groundLength / 2);

            for (int i = 0; i < 8; i++)
                bLines[i] = Vector3.Transform(bLines[i], a.WorldTransform);

            List<Vector3> vertices = new List<Vector3>();
            for (int i = 0; i < 8; i++)
            {
                for (int j = 0; j < 8; j++)
                {

                    if (b.CollisionInformation.Pairs.Count > 0)
                    {
                        if (b.CollisionInformation.Pairs[0].BroadPhaseOverlap.EntryA == b.CollisionInformation)
                            vertices.Add(aLines[i] - bLines[j]);
                        else
                            vertices.Add(bLines[i] - aLines[j]);
                    }
                    else
                    {
                        vertices.Add(bLines[i] - aLines[j]);
                    }
                }
            }

            var indices = new List<int>();
            Toolbox.GetConvexHull(vertices, indices);

            //var vertices = new List<Vector3>();
            //Vector3 max;
            //var direction = new Vector3();
            //int NumSamples = 24;
            //float angleChange = MathHelper.TwoPi / NumSamples;

            //RigidTransform groundTransform = a.CollisionInformation.WorldTransform;
            //RigidTransform boxTransform = b.CollisionInformation.WorldTransform;
            //RigidTransform localTransformB;
            //if (b.CollisionInformation.Pairs.Count == 0 || b.CollisionInformation.Pairs[0].BroadPhaseOverlap.EntryA == a.CollisionInformation)
            //    MinkowskiToolbox.GetLocalTransform(ref groundTransform, ref boxTransform, out localTransformB);
            //else
            //    MinkowskiToolbox.GetLocalTransform(ref boxTransform, ref groundTransform, out localTransformB);

            //for (int i = 1; i < NumSamples / 2 - 1; i++)
            //{
            //    float phi = MathHelper.PiOver2 - i * angleChange;
            //    var sinPhi = (float)Math.Sin(phi);
            //    var cosPhi = (float)Math.Cos(phi);
            //    for (int j = 0; j < NumSamples; j++)
            //    {
            //        float theta = j * angleChange;
            //        direction.X = (float)Math.Cos(theta) * cosPhi;
            //        direction.Y = sinPhi;
            //        direction.Z = (float)Math.Sin(theta) * cosPhi;

            //        if (b.CollisionInformation.Pairs.Count == 0 || b.CollisionInformation.Pairs[0].BroadPhaseOverlap.EntryA == a.CollisionInformation)
            //        {
            //            MinkowskiToolbox.GetLocalMinkowskiExtremePoint(a.CollisionInformation.Shape as ConvexShape, b.CollisionInformation.Shape as ConvexShape, ref direction, ref localTransformB, out max);
            //            //RigidTransform.Transform(ref max, ref groundTransform, out max);
            //        }
            //        else
            //        {
            //            MinkowskiToolbox.GetLocalMinkowskiExtremePoint(b.CollisionInformation.Shape as ConvexShape, a.CollisionInformation.Shape as ConvexShape, ref direction, ref localTransformB, out max);
            //            //RigidTransform.Transform(ref max, ref boxTransform, out max);
            //        }
            //        vertices.Add(max);
            //    }
            //}

            //if (b.CollisionInformation.Pairs.Count == 0 || b.CollisionInformation.Pairs[0].BroadPhaseOverlap.EntryA == a.CollisionInformation)
            //{
            //    MinkowskiToolbox.GetLocalMinkowskiExtremePoint(a.CollisionInformation.Shape as ConvexShape, b.CollisionInformation.Shape as ConvexShape, ref Toolbox.UpVector, ref localTransformB, out max);
            //    //RigidTransform.Transform(ref max, ref groundTransform, out max);
            //    vertices.Add(max);
            //    MinkowskiToolbox.GetLocalMinkowskiExtremePoint(a.CollisionInformation.Shape as ConvexShape, b.CollisionInformation.Shape as ConvexShape, ref Toolbox.DownVector, ref localTransformB, out max);
            //    //RigidTransform.Transform(ref max, ref groundTransform, out max);
            //    vertices.Add(max);
            //}
            //else
            //{
            //    MinkowskiToolbox.GetLocalMinkowskiExtremePoint(b.CollisionInformation.Shape as ConvexShape, a.CollisionInformation.Shape as ConvexShape, ref Toolbox.UpVector, ref localTransformB, out max);
            //    //RigidTransform.Transform(ref max, ref boxTransform, out max);
            //    vertices.Add(max);
            //    MinkowskiToolbox.GetLocalMinkowskiExtremePoint(b.CollisionInformation.Shape as ConvexShape, a.CollisionInformation.Shape as ConvexShape, ref Toolbox.DownVector, ref localTransformB, out max);
            //    //RigidTransform.Transform(ref max, ref boxTransform, out max);
            //    vertices.Add(max);
            //}

            //var indices = new List<int>();
            //Toolbox.GetConvexHull(vertices, indices);

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
            Vector3 screenContactPosition = Game.GraphicsDevice.Viewport.Project(Vector3.Zero, Game.Camera.ProjectionMatrix, Game.Camera.ViewMatrix, Matrix.Identity);
            Game.UIDrawer.Draw(whitePixel, new Rectangle((int)screenContactPosition.X - 2, (int)screenContactPosition.Y - 2, 4, 4), Color.White);

            Vector3 v1 = Game.GraphicsDevice.Viewport.Project(MPRTesting.DEBUGlastV1, Game.Camera.ProjectionMatrix, Game.Camera.ViewMatrix, Matrix.Identity);
            Game.UIDrawer.Draw(whitePixel, new Rectangle((int)v1.X - 3, (int)v1.Y - 3, 6, 6), Color.Red);

            Vector3 v2 = Game.GraphicsDevice.Viewport.Project(MPRTesting.DEBUGlastV2, Game.Camera.ProjectionMatrix, Game.Camera.ViewMatrix, Matrix.Identity);
            Game.UIDrawer.Draw(whitePixel, new Rectangle((int)v2.X - 3, (int)v2.Y - 3, 6, 6), Color.Red);

            Vector3 v3 = Game.GraphicsDevice.Viewport.Project(MPRTesting.DEBUGlastV3, Game.Camera.ProjectionMatrix, Game.Camera.ViewMatrix, Matrix.Identity);
            Game.UIDrawer.Draw(whitePixel, new Rectangle((int)v3.X - 3, (int)v3.Y - 3, 6, 6), Color.Red);

            Vector3 rayHit = Game.GraphicsDevice.Viewport.Project(MPRTesting.DEBUGlastRayDirection * MPRTesting.DEBUGlastRayT, Game.Camera.ProjectionMatrix, Game.Camera.ViewMatrix, Matrix.Identity);
            Game.UIDrawer.Draw(whitePixel, new Rectangle((int)rayHit.X - 3, (int)rayHit.Y - 3, 6, 6), Color.Purple);

            base.DrawUI();
        }

        List<VertexPositionColor> minkowskiLines = new List<VertexPositionColor>();


        VertexPositionColor[] lines = new VertexPositionColor[12];
        public override void Draw()
        {
            //Add the raycast.  Starts at contact position, goes to contact position + rayDirection * T;
            //Vector3 basePosition = MPRTesting.DEBUGlastPosition;
            Vector3 basePosition = Vector3.Zero;
            lines[0] = new VertexPositionColor(basePosition, Color.Red);
            lines[1] = new VertexPositionColor(basePosition + MPRTesting.DEBUGlastRayDirection * MPRTesting.DEBUGlastRayT, Color.Red);
            //Add the normal.  It goes from the surface hit plus the normal.
            lines[2] = new VertexPositionColor(basePosition + MPRTesting.DEBUGlastRayDirection * MPRTesting.DEBUGlastRayT, Color.Purple);
            lines[3] = new VertexPositionColor(basePosition + MPRTesting.DEBUGlastRayDirection * MPRTesting.DEBUGlastRayT + MPRTesting.DEBUGlastNormal, Color.Purple);
            //Add another line from the contact position along the normal with the depth.
            lines[4] = new VertexPositionColor(basePosition, Color.White);
            lines[5] = new VertexPositionColor(basePosition + MPRTesting.DEBUGlastNormal * MPRTesting.DEBUGlastDepth, Color.White);

            //Add the v1v2v3 triangle.
            lines[6] = new VertexPositionColor(MPRTesting.DEBUGlastV1, Color.Red);
            lines[7] = new VertexPositionColor(MPRTesting.DEBUGlastV2, Color.Red);
            lines[8] = new VertexPositionColor(MPRTesting.DEBUGlastV2, Color.Red);
            lines[9] = new VertexPositionColor(MPRTesting.DEBUGlastV3, Color.Red);
            lines[10] = new VertexPositionColor(MPRTesting.DEBUGlastV3, Color.Red);
            lines[11] = new VertexPositionColor(MPRTesting.DEBUGlastV1, Color.Red);

            foreach (EffectPass pass in Game.LineDrawer.CurrentTechnique.Passes)
            {
                pass.Apply();

                Game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, lines, 0, lines.Length / 2);
            }

            foreach (EffectPass pass in Game.LineDrawer.CurrentTechnique.Passes)
            {
                pass.Apply();

                Game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, minkowskiLines.ToArray(), 0, minkowskiLines.Count / 2);
            }
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