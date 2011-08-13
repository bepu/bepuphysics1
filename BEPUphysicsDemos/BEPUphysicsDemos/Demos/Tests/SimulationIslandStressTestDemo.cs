using System;
using BEPUphysics.Collidables;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;
using BEPUphysics.Settings;
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.CollisionTests.Manifolds;
using System.Diagnostics;
using BEPUphysics.BroadPhaseSystems.SortAndSweep;
using BEPUphysics.NarrowPhaseSystems;

namespace BEPUphysicsDemos.Demos.Tests
{
    /// <summary>
    /// Spheres fall onto a large terrain.  Try driving around on it!
    /// </summary>
    public class SimulationIslandStressTestDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public SimulationIslandStressTestDemo(DemosGame game)
            : base(game)
        {

            Space.Add(new Box(new Vector3(0, 0, 0), 500, 10, 500));
            //MotionSettings.DefaultPositionUpdateMode = BEPUphysics.PositionUpdating.PositionUpdateMode.Continuous;

            NarrowPhaseHelper.Factories.SphereSphere.EnsureCount(10000);
            NarrowPhaseHelper.Factories.BoxSphere.EnsureCount(3000);
            //Space.BroadPhase = new Grid2DSortAndSweep(Space.ThreadManager);

            //ConfigurationHelper.ApplySuperSpeedySettings(Space);
            //Space.ForceUpdater.Gravity = new Vector3();
            Space.Solver.IterationLimit = 1;
            //for (int i = 0; i < 20; i++)
            //{
            //    for (int j = 0; j < 10; j++)
            //    {
            //        for (int k = 0; k < 20; k++)
            //        {
            //            Space.Add(new Sphere(new Vector3(0 + i * 3, 20 + j * 3, 0 + k * 3), 0.5f, 1)
            //            {
            //                //LocalInertiaTensorInverse = new Matrix3X3()
            //                //Tag = "noDisplayObject",
            //                IsAlwaysActive = true
            //            });
            //        }
            //    }
            //}


            Random rand = new Random();


            //float width = 30;
            //float height = 200;
            //float length = 30;
            //for (int i = 0; i < 3000; i++)
            //{
            //    Vector3 position =
            //        new Vector3((float)rand.NextDouble() * width - width * .5f,
            //            (float)rand.NextDouble() * height + 20,
            //            (float)rand.NextDouble() * length - length * .5f);
            //    var sphere = new Sphere(position, 1, 1) { Tag = "noDisplayObject" };
            //    sphere.ActivityInformation.IsAlwaysActive = true;
            //    Space.Add(sphere);
            //}

            float width = 10;
            float height = 30;
            float length = 10;
            for (int i = 0; i < width; i++)
            {
                for (int j = 0; j < height; j++)
                {
                    for (int k = 0; k < length; k++)
                    {
                        Vector3 position =
                            new Vector3(i * 3 + j * .2f,
                                20 + j * 3f,
                                k * 3 + j * .2f);
                        var sphere = new Sphere(position, 1, 1) { Tag = "noDisplayObject" };
                        sphere.ActivityInformation.IsAlwaysActive = true;
                        Space.Add(sphere);
                    }
                }
            }





            game.Camera.Position = new Vector3(0, 30, 20);

            //Pre-simulate.
            for (int i = 0; i < 30; i++)
            {
                Space.Update();
            }

            int numRuns = 500;

            double startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            for (int i = 0; i < numRuns; i++)
            {
                Space.Update();
            }

            double endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            time = endTime - startTime;
        }

        double time;

        public override void DrawUI()
        {
            Game.DataTextDrawer.Draw("Simulation time: ", time, 5, new Vector2(50, 50)); 
            base.DrawUI();
        }


        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Simulation Island Stress Test"; }
        }
    }
}