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
using BEPUphysics.BroadPhaseSystems.Hierarchies;
using BEPUphysics.BroadPhaseSystems.SortAndSweep;
//using BEPUphysics.BroadPhaseSystems.Hierarchies.Testing.Old;

namespace BEPUphysicsDemos.Demos.Tests
{
    /// <summary>
    /// Benchmarks and verifies various broad phases against each other.
    /// </summary>
    public class BroadPhasesTestDemo : StandardDemo
    {
        enum Test
        {
            Update,
            RayCast,
            BoundingBoxQuery
        }
        Random rand = new Random(0);

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public BroadPhasesTestDemo(DemosGame game)
            : base(game)
        {
            Space.Solver.IterationLimit = 0;
            Entity toAdd;
            //BoundingBox box = new BoundingBox(new Vector3(-5, 1, 1), new Vector3(5, 7, 7));
            BoundingBox box = new BoundingBox(new Vector3(-50, -50, -50), new Vector3(50, 50, 50));

            //DynamicHierarchyOld dhOld = new DynamicHierarchyOld(Space.ThreadManager);
            DynamicHierarchy dh = new DynamicHierarchy(Space.ThreadManager);
            SortAndSweep1D sas1d = new SortAndSweep1D(Space.ThreadManager);
            Grid2DSortAndSweep grid2DSAS = new Grid2DSortAndSweep(Space.ThreadManager);
            //DynamicHierarchy dh = new DynamicHierarchy();
            //DynamicHierarchy4 dh4 = new DynamicHierarchy4();
            //SortAndSweep1D sas1d = new SortAndSweep1D();
            //Grid2DSortAndSweep grid2DSAS = new Grid2DSortAndSweep();

            //DynamicHierarchy2 dh2 = new DynamicHierarchy2();
            //DynamicHierarchy3 dh3 = new DynamicHierarchy3();
            //SortAndSweep3D sap3d = new SortAndSweep3D();

            RawList<Entity> entities = new RawList<Entity>();
            for (int k = 0; k < 100; k++)
            {
                Vector3 position = new Vector3((float)(rand.NextDouble() * (box.Max.X - box.Min.X) + box.Min.X),
                                               (float)(rand.NextDouble() * (box.Max.Y - box.Min.Y) + box.Min.Y),
                                               (float)(rand.NextDouble() * (box.Max.Z - box.Min.Z) + box.Min.Z));
                toAdd = new Box(position, 1, 1, 1, 1);
                toAdd.CollisionInformation.CollisionRules.Personal = CollisionRule.NoNarrowPhasePair;
                toAdd.CollisionInformation.UpdateBoundingBox(0);
                //Space.Add(toAdd);
                //dhOld.Add(toAdd.CollisionInformation);
                dh.Add(toAdd.CollisionInformation);
                sas1d.Add(toAdd.CollisionInformation);
                grid2DSAS.Add(toAdd.CollisionInformation);
                entities.Add(toAdd);

            }


            Space.ForceUpdater.Gravity = new Vector3();

            int numRuns = 10000;
            //Prime the system.
            grid2DSAS.Update();
            sas1d.Update();
            //dhOld.Update();
            dh.Update();
            var testType = Test.Update;

            double startTime, endTime;

            switch (testType)
            {
                #region Update Timing
                case Test.Update:
                    for (int i = 0; i < numRuns; i++)
                    {
                        ////DH
                        //startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                        //dhOld.Update();
                        //endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                        //DHOldTime += endTime - startTime;

                        //DH4
                        startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                        dh.Update();
                        endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                        DHtime += endTime - startTime;

                        //SAP1D
                        startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                        sas1d.Update();
                        endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                        SAS1Dtime += endTime - startTime;

                        //Grid2D SOS
                        startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                        grid2DSAS.Update();
                        endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                        grid2DSAStime += endTime - startTime;

                        //if (sap1d.Overlaps.Count != dh.Overlaps.Count)
                        //    Debug.WriteLine("SAP1D Failure");
                        //if (grid2DSOS.Overlaps.Count != dh.Overlaps.Count)
                        //    Debug.WriteLine("grid2DSOS Failure");

                        //for (int j = 0; j < dh2.Overlaps.Count; j++)
                        //{
                        //    if (!grid2DSOS.Overlaps.Contains(dh2.Overlaps[j]))
                        //        Debug.WriteLine("Break.");
                        //}
                        //for (int j = 0; j < grid2DSOS.Overlaps.Count; j++)
                        //{
                        //    if (!dh2.Overlaps.Contains(grid2DSOS.Overlaps[j]))
                        //        break;
                        //}

                        //for (int j = 0; j < grid2DSOS.Overlaps.Count; j++)
                        //{
                        //    if (!dh4.Overlaps.Contains(grid2DSOS.Overlaps[j]))
                        //        break;
                        //}

                        //for (int j = 0; j < dh.Overlaps.Count; j++)
                        //{
                        //    if (!dh.Overlaps[j].EntryA.BoundingBox.Intersects(dh.Overlaps[j].EntryB.BoundingBox))
                        //        Debug.WriteLine("Break.");
                        //}

                        //for (int j = 0; j < sap1d.Overlaps.Count; j++)
                        //{
                        //    if (!sap1d.Overlaps[j].EntryA.BoundingBox.Intersects(sap1d.Overlaps[j].EntryB.BoundingBox))
                        //        Debug.WriteLine("Break.");
                        //}

                        //for (int j = 0; j < grid2DSOS.Overlaps.Count; j++)
                        //{
                        //    if (!grid2DSOS.Overlaps[j].EntryA.BoundingBox.Intersects(grid2DSOS.Overlaps[j].EntryB.BoundingBox))
                        //        Debug.WriteLine("Break.");
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


                    ////DH
                    //startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    //for (int i = 0; i < numRuns; i++)
                    //{
                    //    dhOld.QueryAccelerator.RayCast(rays.Elements[i], rayLength, outputIntersections);
                    //    outputIntersections.Clear();

                    //}
                    //endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    //DHOldTime = endTime - startTime;

                    //DH4
                    startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    for (int i = 0; i < numRuns; i++)
                    {
                        dh.QueryAccelerator.RayCast(rays.Elements[i], rayLength, outputIntersections);
                        outputIntersections.Clear();
                    }

                    endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    DHtime = endTime - startTime;

                    //Grid2DSAS
                    startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    for (int i = 0; i < numRuns; i++)
                    {
                        grid2DSAS.QueryAccelerator.RayCast(rays.Elements[i], rayLength, outputIntersections);
                        outputIntersections.Clear();
                    }
                    endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    grid2DSAStime = endTime - startTime;
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

                    ////DH
                    //startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    //for (int i = 0; i < numRuns; i++)
                    //{
                    //    dhOld.QueryAccelerator.GetEntries(boundingBoxes.Elements[i], outputIntersections);
                    //    outputIntersections.Clear();

                    //}
                    //endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    //DHOldTime = endTime - startTime;

                    //DH4
                    startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    for (int i = 0; i < numRuns; i++)
                    {
                        dh.QueryAccelerator.GetEntries(boundingBoxes.Elements[i], outputIntersections);
                        outputIntersections.Clear();
                    }

                    endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    DHtime = endTime - startTime;

                    //Grid2DSAS
                    startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    for (int i = 0; i < numRuns; i++)
                    {
                        grid2DSAS.QueryAccelerator.GetEntries(boundingBoxes.Elements[i], outputIntersections);
                        outputIntersections.Clear();
                    }
                    endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    grid2DSAStime = endTime - startTime;
                    break;
                #endregion
            }


            DHOldTime /= numRuns;
            DH2time /= numRuns;
            DH3time /= numRuns;
            DHtime /= numRuns;
            SAS1Dtime /= numRuns;
            grid2DSAStime /= numRuns;


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

        double DHOldTime;
        double DH2time;
        double DH3time;
        double DHtime;
        double SAS1Dtime;
        double SAP3Dtime;
        double grid2DSAStime;

        public override void DrawUI()
        {
            base.DrawUI();
            Game.DataTextDrawer.Draw("Time per DHold:    ", DHOldTime * 1e6f, new Vector2(50, 50));
            Game.DataTextDrawer.Draw("Time per DH:    ", DHtime * 1e6f, new Vector2(50, 80));
            Game.DataTextDrawer.Draw("Time per SAP1D: ", SAS1Dtime * 1e6f, new Vector2(50, 110));
            Game.DataTextDrawer.Draw("Time per Grid2DSortAndSweep: ", grid2DSAStime * 1e6f, new Vector2(50, 140));
        }



        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Broad Phase Testing"; }
        }

    }
}