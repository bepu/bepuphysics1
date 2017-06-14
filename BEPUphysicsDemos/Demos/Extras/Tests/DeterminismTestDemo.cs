using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using System;
using BEPUutilities;
using BEPUphysics.CollisionShapes.ConvexShapes;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics;
using BEPUphysicsDrawer.Models;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class DeterminismTestDemo : StandardDemo
    {
        Space testSpace0;
        Space testSpace1;

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public DeterminismTestDemo(DemosGame game)
            : base(game)
        {

            //while (Space.ThreadManager.ThreadCount > 0)
            //    Space.ThreadManager.RemoveThread();
            //Resources.ResetPools();

            //Space.BroadPhase.AllowMultithreading = false;
            //Space.Solver.AllowMultithreading = false;
            //Space.NarrowPhase.AllowMultithreading = false;

            //Space.BoundingBoxUpdater.AllowMultithreading = false;
            //Space.DeactivationManager.AllowMultithreading = false;
            //Space.ForceUpdater.AllowMultithreading = false;
            //Space.PositionUpdater.AllowMultithreading = false;

            //Space.BeforeNarrowPhaseUpdateables.Enabled = false;
            //Space.DuringForcesUpdateables.Enabled = false;
            //Space.EndOfFrameUpdateables.Enabled = false;
            //Space.EndOfTimeStepUpdateables.Enabled = false;

            //Space.BroadPhase = new BruteForce();
            //Space.NarrowPhase.BroadPhaseOverlaps = Space.BroadPhase.Overlaps;

            //SolverSettings.DefaultMinimumImpulse = 0;
            //SolverSettings.DefaultMinimumIterations = 10;

            //Working against the demo paradigm a little bit here. Don't want to show any of the default simulation.
            for (int i = Space.Entities.Count - 1; i >= 0; --i)
            {
                Space.Remove(Space.Entities[i]);
            }

            testSpace0 = new Space();
            testSpace1 = new Space();

            FillSpace(testSpace0, game.ModelDrawer);
            FillSpace(testSpace1);


        }

        void FillSpace(Space space, ModelDrawer modelDrawer = null)
        {
            Entity ground = new MorphableEntity(new BoxShape(50, 1, 50));
            space.Add(ground);
            space.ForceUpdater.Gravity = new Vector3(0, -10, 0);

            ModelDataExtractor.GetVerticesAndIndicesFromModel(Game.Content.Load<Model>("playground"), out Vector3[] vertices, out int[] indices);
            var mesh = new StaticMesh(vertices, indices, new AffineTransform(new Vector3(50, -20, 0)));
            space.Add(mesh);
            modelDrawer?.Add(mesh);

            for (int i = 0; i < 100; i++)
            {
                Entity e = new Box(new Vector3(.1f * i, 1 * i + 1, 0), 1, 1, 1, 1);
                //Entity e = new Capsule(new Vector3(.1f * i, 1 * i + 1, 0), .5f, .5f, 1);
                //Entity e = new Sphere(new Vector3(.1f * i, 1 * i + 1, 0), .5f, 1);
                e.ActivityInformation.IsAlwaysActive = true;
                e.CollisionInformation.Tag = i;
                space.Add(e);
            }
            for (int i = 0; i < 200; i++)
            {
                //Entity e = new Box(new Vector3(.1f * i, 1 * i + 1, 2), 1, 1, 1, 1);
                Entity e = new Capsule(new Vector3(.1f * i, 1 * i + 1, 2), .5f, .5f, 1);
                //Entity e = new Sphere(new Vector3(.1f * i, 1 * i + 1, 2), .5f, 1);
                e.ActivityInformation.IsAlwaysActive = true;
                e.CollisionInformation.Tag = i;
                space.Add(e);
            }
            for (int i = 0; i < 300; i++)
            {
                //Entity e = new Box(new Vector3(.1f * i, 1 * i + 1, 4), 1, 1, 1, 1);
                //Entity e = new Capsule(new Vector3(.1f * i, 1 * i + 1, 4), .5f, .5f, 1);
                Entity e = new Sphere(new Vector3(.1f * i, 1 * i + 1, 4), .5f, 1);
                e.ActivityInformation.IsAlwaysActive = true;
                e.CollisionInformation.Tag = i;
                space.Add(e);
            }
            if(modelDrawer!= null)
            {
                for (int i =0; i < space.Entities.Count; ++i)
                {
                    modelDrawer.Add(space.Entities[i]);
                }
            }
        }

        int frameIndex;
        public override void Update(float dt)
        {
            base.Update(dt);
            testSpace0.Update();
            testSpace1.Update();

            for (int i = 0; i < testSpace0.Entities.Count; ++i)
            {
                if(!testSpace0.Entities[i].MotionState.Equals(testSpace1.Entities[i].MotionState))
                {
                    Console.WriteLine($"Determinism failed on frame {frameIndex}, entity {i}: {testSpace0.Entities[i].MotionState} vs {testSpace1.Entities[i].MotionState}");
                }
            }
            frameIndex++;
        }
        
        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Determinism Test"; }
        }

    }
}