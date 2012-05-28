using BEPUphysics;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.NarrowPhaseSystems;
using Microsoft.Xna.Framework;
using System.Diagnostics;
using System;
using BEPUphysics.Threading;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Benchmarks and verifies various broad phases against each other.
    /// </summary>
    public class MultithreadedScalingTestDemo : StandardDemo
    {
        Func<Space, int>[] simulationBuilders;
        double[,] testResults;

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public MultithreadedScalingTestDemo(DemosGame game)
            : base(game)
        {
            simulationBuilders = new Func<Space, int>[]
                                     {
                                         BuildPileSimulation,
                                         BuildWallSimulation
                                     };

            int coreCountMax = Environment.ProcessorCount;
            testResults = new double[coreCountMax, simulationBuilders.Length];


            int reruns = 10;
            for (int i = 0; i < reruns; i++)
            {
                GC.Collect();
                var threadManager = new SpecializedThreadManager();



                //Try different thread counts.
                for (int j = 0; j < coreCountMax; j++)
                {
                    threadManager.AddThread();
                    for (int k = 0; k < simulationBuilders.Length; k++)
                        testResults[j, k] = RunTest(threadManager, simulationBuilders[k]);
                    GC.Collect();

                }
            }



        }

        int BuildPileSimulation(Space space)
        {
            Random rand = new Random(0);

            NarrowPhaseHelper.Factories.BoxBox.EnsureCount(30000);

            BoundingBox box = new BoundingBox(new Vector3(-5, 10, -5), new Vector3(5, 300, 5));
            for (int k = 0; k < 5000; k++)
            {
                Vector3 position = new Vector3((float)(rand.NextDouble() * (box.Max.X - box.Min.X) + box.Min.X),
                                               (float)(rand.NextDouble() * (box.Max.Y - box.Min.Y) + box.Min.Y),
                                               (float)(rand.NextDouble() * (box.Max.Z - box.Min.Z) + box.Min.Z));
                var toAdd = new Box(position, 1, 1, 1, 1);
                toAdd.ActivityInformation.IsAlwaysActive = true;

                space.Add(toAdd);


            }

            Box ground = new Box(new Vector3(0, 0, 0), 300, 10, 300);
            space.Add(ground);

            return 700;
        }

        int BuildWallSimulation(Space space)
        {
            int width = 100;
            int height = 40;
            float blockWidth = 2f;
            float blockHeight = 1f;
            float blockLength = 3f;

            NarrowPhaseHelper.Factories.BoxBox.EnsureCount(20000);

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
                    space.Add(toAdd);

                }
            }

            Box ground = new Box(new Vector3(0, -5f, 0), 500, 10, 500);
            space.Add(ground);

            return 800;
        }

        double RunTest(IThreadManager threadManager, Func<Space, int> simulationBuilder)
        {

            Space space = new Space(threadManager);

            var timeStepCount = simulationBuilder(space);


            //Perform one starter frame to warm things up.
            space.Update();

            var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            for (int i = 0; i < timeStepCount; i++)
            {
                space.Update();
            }
            var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            return endTime - startTime;
        }

        private int timeStepsElapsed;

        public override void Update(float dt)
        {
            timeStepsElapsed++;
            base.Update(dt);
        }



        public override void DrawUI()
        {
            base.DrawUI();
            //Game.DataTextDrawer.Draw("Time steps elapsed: ", timeStepsElapsed, new Vector2(600, 600));
            //return;
            Vector2 origin = new Vector2(100, 50);
            Vector2 spacing = new Vector2(120, 50);
            //Draw the horizontal core counts.
            for (int i = 0; i < testResults.GetLength(0); i++)
            {
                Game.DataTextDrawer.Draw(i + 1, origin + new Vector2(spacing.X * i, -30));
            }
            for (int i = 0; i < testResults.GetLength(1); i++)
            {
                Game.DataTextDrawer.Draw(i, origin + new Vector2(-30, spacing.Y * i));
            }

            for (int i = 0; i < testResults.GetLength(0); i++)
            {
                int lowestTime = 0;
                for (int j = 0; j < testResults.GetLength(1); j++)
                {
                    Game.DataTextDrawer.Draw(testResults[i, j] * 1e3, 0, origin + new Vector2(spacing.X * i, spacing.Y * j));
                    if (testResults[i, j] < testResults[i, lowestTime])
                        lowestTime = j;
                }
            }



        }



        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Multithreaded Scaling"; }
        }

    }
}