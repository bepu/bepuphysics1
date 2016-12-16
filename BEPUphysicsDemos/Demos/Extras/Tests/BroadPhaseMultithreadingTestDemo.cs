using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;
using BEPUutilities.DataStructures;
using System.Diagnostics;
using System;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.BroadPhaseSystems.Hierarchies;
using BEPUutilities.Threading;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Benchmarks dynamic hierarchy performance under different multithreading conditions.
    /// </summary>
    public class BroadPhaseMultithreadingTestDemo : StandardDemo
    {
        enum Test
        {
            Update,
            Refit,
            Overlap,
            RayCast,
            BoundingBoxQuery
        }

        double[,] testResults;

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public BroadPhaseMultithreadingTestDemo(DemosGame game)
            : base(game)
        {
            Space.Solver.IterationLimit = 0;


#if WINDOWS
            int coreCountMax = Environment.ProcessorCount;
            int splitOffsetMax = 6;
            testResults = new double[coreCountMax, splitOffsetMax + 1];

            int reruns = 10;
            for (int i = 0; i < reruns; i++)
            {
                GC.Collect();
                var looper = new ParallelLooper();



                //Try different thread counts.
                for (int j = 0; j < coreCountMax; j++)
                {
                    looper.AddThread();
                    //Try different split levels.);
                    for (int k = 0; k <= splitOffsetMax; k++)
                    {
                        testResults[j, k] += RunTest(k, looper);
                        GC.Collect();
                    }
                }
            }

            for (int i = 0; i < testResults.GetLength(0); i++)
            {
                for (int j = 0; j < testResults.GetLength(1); j++)
                {
                    testResults[i, j] /= reruns;
                }
            }
#else
            int splitOffsetMax = 6;

            testResults = new double[4, splitOffsetMax + 1];

            int reruns = 10;
            for (int i = 0; i < reruns; i++)
            {
                var threadManager = new SpecializedThreadManager();
                GC.Collect();

                threadManager.AddThread(delegate { Thread.CurrentThread.SetProcessorAffinity(new[] { 1 }); }, null);
                for (int j = 0; j <= splitOffsetMax; j++)
                {
                    testResults[0, j] += RunTest(j, threadManager);
                    GC.Collect();
                }
                threadManager.AddThread(delegate { Thread.CurrentThread.SetProcessorAffinity(new[] { 3 }); }, null);
                for (int j = 0; j <= splitOffsetMax; j++)
                {
                    testResults[1, j] += RunTest(j, threadManager);
                    GC.Collect();
                }
                threadManager.AddThread(delegate { Thread.CurrentThread.SetProcessorAffinity(new[] { 5 }); }, null);
                for (int j = 0; j <= splitOffsetMax; j++)
                {
                    testResults[2, j] += RunTest(j, threadManager);
                    GC.Collect();
                }
                threadManager.AddThread(delegate { Thread.CurrentThread.SetProcessorAffinity(new[] { 4 }); }, null);
                for (int j = 0; j <= splitOffsetMax; j++)
                {
                    testResults[3, j] += RunTest(j, threadManager);
                    GC.Collect();
                }
            }

            for (int i = 0; i < testResults.GetLength(0); i++)
            {
                for (int j = 0; j < testResults.GetLength(1); j++)
                {
                    testResults[i,j] /= reruns;
                }
            }
#endif


        }

        double RunTest(int splitOffset, IParallelLooper parallelLooper)
        {
            Entity toAdd;
            //BoundingBox box = new BoundingBox(new Vector3(-5, 1, 1), new Vector3(5, 7, 7));
            BoundingBox box = new BoundingBox(new Vector3(-500, -500, -500), new Vector3(500, 500, 500));

            int splitDepth = splitOffset + (int)Math.Ceiling(Math.Log(parallelLooper.ThreadCount, 2));

            DynamicHierarchy dh = new DynamicHierarchy(parallelLooper);

            Random rand = new Random(0);

            RawList<Entity> entities = new RawList<Entity>();
            for (int k = 0; k < 10000; k++)
            {
                Vector3 position = new Vector3((float)(rand.NextDouble() * (box.Max.X - box.Min.X) + box.Min.X),
                                               (float)(rand.NextDouble() * (box.Max.Y - box.Min.Y) + box.Min.Y),
                                               (float)(rand.NextDouble() * (box.Max.Z - box.Min.Z) + box.Min.Z));
                toAdd = new Box(position, 1, 1, 1, 1);
                toAdd.CollisionInformation.CollisionRules.Personal = CollisionRule.NoNarrowPhasePair;
                toAdd.CollisionInformation.UpdateBoundingBox(0);


                dh.Add(toAdd.CollisionInformation);
                entities.Add(toAdd);

            }


            Space.ForceUpdater.Gravity = new Vector3();

            int numRuns = 3000;
            //Prime the system.
            dh.Update();
            var testType = Test.Update;

            BroadPhaseOverlap[] overlapBasis = new BroadPhaseOverlap[dh.Overlaps.Count];
            dh.Overlaps.CopyTo(overlapBasis, 0);


            double time = 0;
            double startTime, endTime;


            switch (testType)
            {
                #region Update Timing
                case Test.Update:
                    for (int i = 0; i < numRuns; i++)
                    {
                        //DH4
                        startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                        //dh.Update();
                        //lock (dh.Locker)
                        //{
                        //    dh.Overlaps.Clear();
                        //    if (dh.ROOTEXISTS)
                        //    {
                        //        dh.MultithreadedRefitPhase(splitDepth);

                        //        dh.MultithreadedOverlapPhase(splitDepth);
                        //    }
                        //}

                        //dh.Update();

                        //lock (dh.Locker)
                        //{
                        //    dh.Overlaps.Clear();
                        //    if (dh.ROOTEXISTS)
                        //    {
                        //        dh.SingleThreadedRefitPhase();
                        //        dh.SingleThreadedOverlapPhase();
                        //    }
                        //}

                        endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                        time += endTime - startTime;

                        //if (dh.Overlaps.Count != overlapBasis.Length)
                        //    Debug.WriteLine("Failed Update.");
                        //for (int j = 0; j < overlapBasis.Length; j++)
                        //{
                        //    if (!dh.Overlaps.Contains(overlapBasis[j]))
                        //        Debug.WriteLine("Failed Update.");
                        //}


                        //MoveEntities(entities);
                    }
                    break;
                #endregion
                #region Refit Timing
                case Test.Refit:
                    for (int i = 0; i < numRuns; i++)
                    {

                        dh.Overlaps.Clear();

                        //DH4
                        startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                        //dh.MultithreadedRefitPhase(splitDepth);
                        //dh.SingleThreadedRefitPhase();
                        endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                        time += endTime - startTime;

                        //dh.SingleThreadedOverlapPhase();

                        //if (dh.Overlaps.Count != overlapBasis.Length)
                        //    Debug.WriteLine("Failed Refit.");
                        //for (int j = 0; j < overlapBasis.Length; j++)
                        //{
                        //    if (!dh.Overlaps.Contains(overlapBasis[j]))
                        //        Debug.WriteLine("Failed Refit.");
                        //}

                        //MoveEntities(entities);
                    }
                    break;
                #endregion
                #region Overlap Timing
                case Test.Overlap:
                    for (int i = 0; i < numRuns; i++)
                    {
                        dh.Overlaps.Clear();
                        //dh.MultithreadedRefitPhase(splitDepth);
                        //DH4
                        startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                        //dh.MultithreadedOverlapPhase(splitDepth);
                        //dh.SingleThreadedOverlapPhase();
                        endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                        time += endTime - startTime;


                        //if (dh.Overlaps.Count != overlapBasis.Length)
                        //    Debug.WriteLine("Failed Overlap.");
                        //for (int j = 0; j < overlapBasis.Length; j++)
                        //{
                        //    if (!dh.Overlaps.Contains(overlapBasis[j]))
                        //        Debug.WriteLine("Failed Overlap.");
                        //}

                        //MoveEntities(entities);
                    }
                    break;
                #endregion
                #region Ray cast timing
                case Test.RayCast:
                    float rayLength = 100;
                    RawList<Ray> rays = new RawList<Ray>();
                    for (int i = 0; i < numRuns; i++)
                    {
                        rays.Add(new Ray()
                        {
                            Position = new Vector3((float)(rand.NextDouble() * (box.Max.X - box.Min.X) + box.Min.X),
                                               (float)(rand.NextDouble() * (box.Max.Y - box.Min.Y) + box.Min.Y),
                                               (float)(rand.NextDouble() * (box.Max.Z - box.Min.Z) + box.Min.Z)),
                            Direction = Vector3.Normalize(new Vector3((float)(rand.NextDouble() - .5), (float)(rand.NextDouble() - .5), (float)(rand.NextDouble() - .5)))
                        });
                    }
                    RawList<BroadPhaseEntry> outputIntersections = new RawList<BroadPhaseEntry>();



                    //DH4
                    startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    for (int i = 0; i < numRuns; i++)
                    {
                        dh.QueryAccelerator.RayCast(rays.Elements[i], rayLength, outputIntersections);
                        outputIntersections.Clear();
                    }

                    endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    time = endTime - startTime;


                    break;
                #endregion
                #region Bounding box query timing
                case Test.BoundingBoxQuery:
                    float boundingBoxSize = 10;
                    var boundingBoxes = new RawList<BoundingBox>();
                    Vector3 offset = new Vector3(boundingBoxSize / 2, boundingBoxSize / 2, boundingBoxSize / 2);
                    for (int i = 0; i < numRuns; i++)
                    {
                        Vector3 center = new Vector3((float)(rand.NextDouble() * (box.Max.X - box.Min.X) + box.Min.X),
                                                     (float)(rand.NextDouble() * (box.Max.Y - box.Min.Y) + box.Min.Y),
                                                     (float)(rand.NextDouble() * (box.Max.Z - box.Min.Z) + box.Min.Z));
                        boundingBoxes.Add(new BoundingBox()
                        {
                            Min = center - offset,
                            Max = center + offset
                        });
                    }

                    outputIntersections = new RawList<BroadPhaseEntry>();


                    //DH4
                    startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    for (int i = 0; i < numRuns; i++)
                    {
                        dh.QueryAccelerator.GetEntries(boundingBoxes.Elements[i], outputIntersections);
                        outputIntersections.Clear();
                    }

                    endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    time = endTime - startTime;


                    break;
                #endregion
            }


            return time / numRuns;
        }

        void MoveEntities(RawList<Entity> entities)
        {
            for (int i = 0; i < entities.Count; i++)
            {
                float speed = .1f;
                //entities[i].Position += new Vector3((float)rand.NextDouble() * speed - speed * .5f, (float)rand.NextDouble() * speed - speed * .5f, (float)rand.NextDouble() * speed - speed * .5f);
                entities[i].Position += new Vector3(0, speed, 0);
                entities[i].CollisionInformation.UpdateBoundingBox(0);
            }
        }


        double DHtime;

        public override void DrawUI()
        {
            base.DrawUI();
            //Game.DataTextDrawer.Draw("Time per DH:    ", DHtime * 1e6f, new Vector2(50, 80));

            var origin = new Microsoft.Xna.Framework.Vector2(100, 50);
            var spacing = new Microsoft.Xna.Framework.Vector2(80, 50);
            //Draw the horizontal core counts.
            for (int i = 0; i < testResults.GetLength(0); i++)
            {
                Game.DataTextDrawer.Draw(i + 1, origin + new Microsoft.Xna.Framework.Vector2(spacing.X * i, -30));
            }
            for (int i = 0; i < testResults.GetLength(1); i++)
            {
                Game.DataTextDrawer.Draw(i, origin + new Microsoft.Xna.Framework.Vector2(-30, spacing.Y * i));
            }

            for (int i = 0; i < testResults.GetLength(0); i++)
            {
                int lowestTime = 0;
                for (int j = 0; j < testResults.GetLength(1); j++)
                {
                    Game.DataTextDrawer.Draw(testResults[i, j] * 1e6, 0, origin + new Microsoft.Xna.Framework.Vector2(spacing.X * i, spacing.Y * j));
                    if (testResults[i, j] < testResults[i, lowestTime])
                        lowestTime = j;
                }

                Game.DataTextDrawer.Draw(testResults[i, lowestTime] * 1e6, 0, origin + new Microsoft.Xna.Framework.Vector2(spacing.X * i, spacing.Y * (testResults.GetLength(1))));
                Game.DataTextDrawer.Draw(lowestTime, 0, origin + new Microsoft.Xna.Framework.Vector2(spacing.X * i, spacing.Y * (testResults.GetLength(1) + 1)));
            }


        }



        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Broad Phase Multithreading Testing"; }
        }

    }
}