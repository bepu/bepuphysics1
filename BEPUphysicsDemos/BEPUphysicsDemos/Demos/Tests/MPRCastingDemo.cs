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
using BEPUphysics.EntityStateManagement;
using BEPUphysics.ResourceManagement;
using BEPUphysics.NarrowPhaseSystems;

namespace BEPUphysicsDemos.Demos.Tests
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class MPRCastingDemo : StandardDemo
    {
        Entity a, b;
        ConvexShape aShape, bShape;
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public MPRCastingDemo(DemosGame game)
            : base(game)
        {
            bShape = new BoxShape(1, 0, 1);
            //bShape.CollisionMargin = 0;
            aShape = new ConeShape(1, .4f);
            //aShape.CollisionMargin = 0;
            a = new Entity(aShape);
            b = new Entity(bShape);
            CollisionRules.AddRule(a, b, CollisionRule.NoSolver);
            NarrowPhaseHelper.CollisionManagers.Remove(new TypePair(typeof(ConvexCollidable<BoxShape>), typeof(ConvexCollidable<BoxShape>)));
            Space.Add(a);
            Space.Add(b);
            a.Orientation = Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathHelper.PiOver4);
            b.Orientation = Quaternion.Identity;
            aTransform = new RigidTransform(new Vector3(0, 0, 0), a.Orientation);
            bTransform = new RigidTransform(new Vector3(0, 10, 0), b.Orientation);

            game.Camera.Position = new Vector3(0, 5, 17);
        }


        RigidTransform aTransform, bTransform;
        bool hit;
        RayHit hitData;

        public override void Update(float dt)
        {
            if (Game.KeyboardInput.IsKeyDown(Keys.NumPad6))
                aTransform.Position += Vector3.Right * dt;
            if (Game.KeyboardInput.IsKeyDown(Keys.NumPad4))
                aTransform.Position += Vector3.Left * dt;
            if (Game.KeyboardInput.IsKeyDown(Keys.NumPad1))
                aTransform.Position += Vector3.Up * dt;
            if (Game.KeyboardInput.IsKeyDown(Keys.NumPad0))
                aTransform.Position += Vector3.Down * dt;
            if (Game.KeyboardInput.IsKeyDown(Keys.NumPad8))
                aTransform.Position += Vector3.Forward * dt;
            if (Game.KeyboardInput.IsKeyDown(Keys.NumPad5))
                aTransform.Position += Vector3.Backward * dt;

            Vector3 sweepA = new Vector3(0, 10, 0);
            Vector3 sweepB = new Vector3(0, -10, 0);

            if (Game.KeyboardInput.IsKeyDown(Keys.P))
                Debug.WriteLine("Breka.");

            if (hit = MPRToolbox.Sweep(aShape, bShape, ref sweepA, ref sweepB, ref aTransform, ref bTransform, out hitData))
            //if (hit = OldGJKVerifier.ConvexCast(a.CollisionInformation.Shape, b.CollisionInformation.Shape, ref sweepA, ref sweepB, ref aTransform, ref bTransform, out hitData))
            {
                a.Position = aTransform.Position + sweepA * hitData.T;
                b.Position = bTransform.Position + sweepB * hitData.T;
            }
            else
            {
                a.Position = aTransform.Position;
                b.Position = bTransform.Position;
            }
            base.Update(dt);
        }

        public override void DrawUI()
        {
            if (hit)
            {
                Game.TinyTextDrawer.Draw("Time: ", hitData.T, 10, new Vector2(50, 50));
            }
            else
            {
                Game.TinyTextDrawer.Draw("No hit.", new Vector2(50, 50));
            }
            base.DrawUI();
        }

        VertexPositionColor[] lines = new VertexPositionColor[] { new VertexPositionColor(new Vector3(), Color.Red), new VertexPositionColor(new Vector3(), Color.White) };
        public override void Draw()
        {
            if (hit)
            {
                lines[0].Position = hitData.Location;
                lines[1].Position = hitData.Location + hitData.Normal;
                foreach (EffectPass pass in Game.LineDrawer.CurrentTechnique.Passes)
                {
                    pass.Apply();

                    Game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, lines, 0, lines.Length / 2);
                }
            }
            base.Draw();
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Test4"; }
        }

    }
}