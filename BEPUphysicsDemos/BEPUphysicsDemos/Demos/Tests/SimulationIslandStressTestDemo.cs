using System;
using BEPUphysics.Collidables;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.MathExtensions;
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

            Space.Add(new Box(new Vector3(0, 0, 0), 1000, 10, 1000));
            //MotionSettings.DefaultPositionUpdateMode = BEPUphysics.PositionUpdating.PositionUpdateMode.Continuous;

            NarrowPhaseHelper.Factories.SphereSphere.EnsureCount(15000);
            NarrowPhaseHelper.Factories.BoxSphere.EnsureCount(3000);
            NarrowPhaseHelper.Factories.BoxBox.EnsureCount(35000);
            //Space.BroadPhase = new Grid2DSortAndSweep(Space.ThreadManager);

            ConfigurationHelper.ApplySuperSpeedySettings(Space);
            //Space.ForceUpdater.Gravity = new Vector3();
            Space.Solver.IterationLimit = 1;



            Random rand = new Random();


            //float width = 200;
            //float height = 200;
            //float length = 200;
            //for (int i = 0; i < 5000; i++)
            //{
            //    Vector3 position =
            //        new Vector3((float)rand.NextDouble() * width - width * .5f,
            //            (float)rand.NextDouble() * height + 20,
            //            (float)rand.NextDouble() * length - length * .5f);
            //    var sphere = new Sphere(position, 1, 1);// { Tag = "noDisplayObject" };
            //    sphere.ActivityInformation.IsAlwaysActive = true;
            //    Space.Add(sphere);
            //}

            float width = 10;
            float height = 50;
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
                        var e = new Sphere(position, 1, 1) { Tag = "noDisplayObject" };
                        //var e = new Box(position, 1, 1, 1, 1) { Tag = "noDisplayObject" };
                        e.ActivityInformation.IsAlwaysActive = true;
                        Space.Add(e);
                    }
                }
            }





            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(0, 30, 20);

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
            Game.DataTextDrawer.Draw("Simulation time: ", time, 5, new Microsoft.Xna.Framework.Vector2(50, 50));
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