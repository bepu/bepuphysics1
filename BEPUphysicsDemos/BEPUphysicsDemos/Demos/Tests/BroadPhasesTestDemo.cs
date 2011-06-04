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
using BEPUphysics.BroadPhaseSystems.Hierarchies.Insertion;
using BEPUphysics.BroadPhaseSystems.Hierarchies.TopDown;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Benchmarks and verifies various broad phases against each other.
    /// </summary>
    public class BroadPhasesTestDemo : StandardDemo
    {

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

            DynamicHierarchy dh = new DynamicHierarchy(Space.ThreadManager);
            DynamicHierarchy4 dh4 = new DynamicHierarchy4(Space.ThreadManager);
            SortAndSweep1D sap1d = new SortAndSweep1D(Space.ThreadManager);
            Grid2DSortAndSweep grid2DSOS = new Grid2DSortAndSweep(Space.ThreadManager);
            //DynamicHierarchy dh = new DynamicHierarchy();
            //DynamicHierarchy4 dh4 = new DynamicHierarchy4();
            //SortAndSweep1D sap1d = new SortAndSweep1D();
            //Grid2DSortAndSweep grid2DSOS = new Grid2DSortAndSweep();

            //DynamicHierarchy2 dh2 = new DynamicHierarchy2();
            //DynamicHierarchy3 dh3 = new DynamicHierarchy3();
            //SortAndSweep3D sap3d = new SortAndSweep3D();

            RawList<Entity> entities = new RawList<Entity>();
            for (int k = 0; k < 1000; k++)
            {
                Vector3 position = new Vector3((float)(rand.NextDouble() * (box.Max.X - box.Min.X) + box.Min.X),
                                               (float)(rand.NextDouble() * (box.Max.Y - box.Min.Y) + box.Min.Y),
                                               (float)(rand.NextDouble() * (box.Max.Z - box.Min.Z) + box.Min.Z));
                toAdd = new Box(position, 1, 1, 1, 1);
                toAdd.CollisionInformation.CollisionRules.Personal = CollisionRule.NoNarrowPhasePair;
                toAdd.CollisionInformation.UpdateBoundingBox(0);
                //Space.Add(toAdd);
                dh.Add(toAdd.CollisionInformation);
                dh4.Add(toAdd.CollisionInformation);
                sap1d.Add(toAdd.CollisionInformation);
                grid2DSOS.Add(toAdd.CollisionInformation);
                entities.Add(toAdd);

                //dh2.Add(toAdd.CollisionInformation);
                //dh3.Add(toAdd.CollisionInformation);
                //sap3d.Add(toAdd.CollisionInformation);
            }

            //List<int> depths = new List<int>();
            //int nodeCount;
            //dh2.Analyze(depths, out nodeCount);

            //depths.Clear();
            //dh2.ForceRevalidation();
            //dh2.Analyze(depths, out nodeCount);

            Space.ForceUpdater.Gravity = new Vector3();

            int numRuns = 1000;
            //Prime the system.
            grid2DSOS.Update();
            sap1d.Update();
            dh.Update();
            //dh2.Update();
            //dh3.Update();
            dh4.Update();
            //DynamicHierarchy.DEBUGAllowRefit = false;
            //DynamicHierarchy2.DEBUGAllowRefit = false;
            //DynamicHierarchy4.DEBUGAllowRefit = false;

            double startTime, endTime;

            for (int i = 0; i < numRuns; i++)
            {
                //DH
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                //dh.Update();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                DHtime += endTime - startTime;

                ////DH2
                //startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                ////dh2.Update();
                //endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                //DH2time += endTime - startTime;

                ////DH3
                //startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                ////dh3.Update();
                //endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                //DH3time += endTime - startTime;

                //DH4
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                dh4.Update();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                DH4time += endTime - startTime;



                //SAP1D
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                //sap1d.Update();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                SAP1Dtime += endTime - startTime;


                //Grid2D SOS
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                //grid2DSOS.Update();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                grid2DSOStime += endTime - startTime;

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

            DHtime /= numRuns;
            DH2time /= numRuns;
            DH3time /= numRuns;
            DH4time /= numRuns;
            SAP1Dtime /= numRuns;
            grid2DSOStime /= numRuns;


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
        double DH2time;
        double DH3time;
        double DH4time;
        double SAP1Dtime;
        double SAP3Dtime;
        double grid2DSOStime;

        public override void DrawUI()
        {
            base.DrawUI();
            Game.DataTextDrawer.Draw("Time per DH:    ", DHtime * 1000000, new Vector2(50, 50));
            Game.DataTextDrawer.Draw("Time per DH4:    ", DH4time * 1000000, new Vector2(50, 80));
            Game.DataTextDrawer.Draw("Time per SAP1D: ", SAP1Dtime * 1000000, new Vector2(50, 110));
            Game.DataTextDrawer.Draw("Time per Grid2DSortAndSweep: ", grid2DSOStime * 1000000, new Vector2(50, 140));
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