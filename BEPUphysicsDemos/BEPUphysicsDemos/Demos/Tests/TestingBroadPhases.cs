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

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class TestingBroadPhases : StandardDemo
    {

        Random rand = new Random(0);

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public TestingBroadPhases(DemosGame game)
            : base(game)
        {
            Space.Solver.IterationLimit = 0;
            Entity toAdd;
            //BoundingBox box = new BoundingBox(new Vector3(-5, 1, 1), new Vector3(5, 7, 7));
            BoundingBox box = new BoundingBox(new Vector3(-50, -50, -50), new Vector3(50, 50, 50));

            DynamicHierarchy dh = new DynamicHierarchy(Space.ThreadManager);
            SortAndSweep1D sap1d = new SortAndSweep1D(Space.ThreadManager);
            //SortAndSweep3D sap3d = new SortAndSweep3D();
            Grid2DSortAndSweep grid2DSOS = new Grid2DSortAndSweep(Space.ThreadManager);

            RawList<Entity> entities = new RawList<Entity>();
            for (int k = 0; k < 10000; k++)
            {
                Vector3 position = new Vector3((float)(rand.NextDouble() * (box.Max.X - box.Min.X) + box.Min.X),
                                               (float)(rand.NextDouble() * (box.Max.Y - box.Min.Y) + box.Min.Y),
                                               (float)(rand.NextDouble() * (box.Max.Z - box.Min.Z) + box.Min.Z));
                toAdd = new Box(position, 1, 1, 1, 1);
                toAdd.CollisionInformation.CollisionRules.Personal = CollisionRule.NoNarrowPhasePair;
                toAdd.CollisionInformation.UpdateBoundingBox(0);
                //Space.Add(toAdd);
                dh.Add(toAdd.CollisionInformation);
                sap1d.Add(toAdd.CollisionInformation);
                //sap3d.Add(toAdd.CollisionInformation);
                grid2DSOS.Add(toAdd.CollisionInformation);
                entities.Add(toAdd);
            }


            Space.ForceUpdater.Gravity = new Vector3();


            int numRuns = 1;
            //Prime the system.
            grid2DSOS.Update();
            sap1d.Update();
            dh.Update();

            double startTime, endTime;

            for (int i = 0; i < numRuns; i++)
            {
                //DH
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                dh.Update();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                DHtime += endTime - startTime;


                //SAP1D
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                sap1d.Update();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                SAP1Dtime += endTime - startTime;


                //Grid2D SOS
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                grid2DSOS.Update();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                grid2DSOStime += endTime - startTime;

                //if (sap1d.Overlaps.Count != dh.Overlaps.Count)
                //    Debug.WriteLine("SAP1D Failure");
                //if (grid2DSOS.Overlaps.Count != dh.Overlaps.Count)
                //    Debug.WriteLine("grid2DSOS Failure");

                //for (int j = 0; j < dh.Overlaps.Count; j++)
                //{
                //    if (!grid2DSOS.Overlaps.Contains(dh.Overlaps[j]))
                //        Debug.WriteLine("Break.");
                //}

                ////for (int j = 0; j < grid2DSOS.Overlaps.Count; j++)
                ////{
                ////    if (!dh.Overlaps.Contains(grid2DSOS.Overlaps[j]))
                ////        Debug.WriteLine("Break.");
                ////}

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
            SAP1Dtime /= numRuns;
            grid2DSOStime /= numRuns;


        }

        void MoveEntities(RawList<Entity> entities)
        {
            for (int i = 0; i < entities.Count; i++)
            {
                float speed = 10;
                //entities[i].Position += new Vector3((float)rand.NextDouble() * speed - speed * .5f, (float)rand.NextDouble() * speed - speed * .5f, (float)rand.NextDouble() * speed - speed * .5f);
                entities[i].Position += new Vector3(speed, 0, 0);
                entities[i].CollisionInformation.UpdateBoundingBox(0);
            }
        }

        double DHtime;
        double SAP1Dtime;
        double SAP3Dtime;
        double grid2DSOStime;

        public override void DrawUI()
        {
            base.DrawUI();
            Game.DataTextDrawer.Draw("Time per DH:    ", DHtime * 1000000, new Vector2(50, 50));
            Game.DataTextDrawer.Draw("Time per SAP1D: ", SAP1Dtime * 1000000, new Vector2(50, 80));
            Game.DataTextDrawer.Draw("Time per Grid2DSortAndSweep: ", grid2DSOStime * 1000000, new Vector2(50, 110));
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