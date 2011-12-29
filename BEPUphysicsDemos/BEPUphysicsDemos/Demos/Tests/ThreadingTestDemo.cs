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
using BEPUphysics.Materials;
using System.Threading;
using BEPUphysics.Threading;

namespace BEPUphysicsDemos.Demos.Tests
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class ThreadingTestDemo : StandardDemo
    {
        public void InitializeSimulation(bool threaded)
        {
            //            Space.Dispose();
            //            SolverSettings.DefaultMinimumIterations = 10;
            //            Space = new BEPUphysics.Space();

            //            if (threaded)
            //            {
            //#if XBOX360
            //            //Note that not all four available hardware threads are used.
            //            //Currently, BEPUphysics will allocate an equal amount of work to each thread on the xbox360.
            //            //If two threads are put on one core, it will bottleneck the engine and run significantly slower than using 3 hardware threads.
            //            Space.ThreadManager.AddThread(delegate { Thread.CurrentThread.SetProcessorAffinity(new[] { 1 }); }, null);
            //            Space.ThreadManager.AddThread(delegate { Thread.CurrentThread.SetProcessorAffinity(new[] { 3 }); }, null);
            //            Space.ThreadManager.AddThread(delegate { Thread.CurrentThread.SetProcessorAffinity(new[] { 5 }); }, null);

            //#else
            //                if (Environment.ProcessorCount > 1)
            //                {
            //                    for (int i = 0; i < Environment.ProcessorCount; i++)
            //                    {
            //                        Space.ThreadManager.AddThread();
            //                    }
            //                }
            //#endif
            //            }

            for (int i = Space.Entities.Count - 1; i >= 0; i--)
            {
                Space.Remove(Space.Entities[i]);
            }
            SolverSettings.DefaultMinimumIterations = 100;

            int width = 15;
            int height = 15;
            float blockWidth = 2f;
            float blockHeight = 1f;
            float blockLength = 1f;



            for (int i = 0; i < width; i++)
            {
                for (int j = 0; j < height; j++)
                {
                    var toAdd =
                        new Box(
                            new Vector3(
                                i * blockWidth + .5f * blockWidth * (j % 2) - width * blockWidth * .5f,
                                blockHeight * .5f + j * (blockHeight),
                                0),
                            blockWidth, blockHeight, blockLength, 10);
                    toAdd.ActivityInformation.IsAlwaysActive = true;
                    Space.Add(toAdd);
                }
            }
            Box ground = new Box(new Vector3(0, -.5f, 0), 50, 1, 50);
            Space.Add(ground);

            GC.Collect();
        }

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public ThreadingTestDemo(DemosGame game)
            : base(game)
        {

            int numRuns = 10;
            int numFrames = 100;


            //SINGLE THREADED
            //singleThreadedTime = 0;
            //for (int i = 0; i < numRuns; i++)
            //{
            //    InitializeSimulation(0);
            //    double startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            //    for (int j = 0; j < numFrames; j++)
            //    {
            //        Space.Update();
            //    }
            //    double endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            //    singleThreadedTime += endTime - startTime;
            //}
            //singleThreadedTime /= (numRuns * numFrames);

            //MULTI THREADED
            multiThreadedTime = 0;
            for (int i = 0; i < numRuns; i++)
            {
                InitializeSimulation(true);
                double startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int j = 0; j < numFrames; j++)
                {
                    Space.Update();
                }
                double endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                multiThreadedTime += endTime - startTime;
            }
            multiThreadedTime /= (numRuns * numFrames);


        }


        double singleThreadedTime;
        double multiThreadedTime;


        public override void DrawUI()
        {
            base.DrawUI();
            Game.DataTextDrawer.Draw("Time per SingleThreaded:    ", singleThreadedTime * 1e6, new Vector2(50, 50));
            Game.DataTextDrawer.Draw("Time per Multithreaded:    ", multiThreadedTime * 1e6, new Vector2(50, 80));

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