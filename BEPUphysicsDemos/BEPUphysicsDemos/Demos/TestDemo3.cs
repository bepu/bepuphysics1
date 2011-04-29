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

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class TestDemo3 : StandardDemo
    {
        List<Entity> entities = new List<Entity>();
        int numFrames;

        static int numRuns = 0;
        static List<MotionState> motionState = new List<MotionState>();

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public TestDemo3(DemosGame game)
            : base(game)
        {

            //while (Space.ThreadManager.ThreadCount > 0)
            //    Space.ThreadManager.RemoveThread();
            //Resources.ResetPools();

            //Space.BroadPhase.AllowMultithreading = false;
            //Space.Solver.AllowMultithreading = false;
            ////Space.NarrowPhase.AllowMultithreading = false;

            ////Space.BoundingBoxUpdater.AllowMultithreading = false;
            ////Space.DeactivationManager.AllowMultithreading = false;
            ////Space.ForceUpdater.AllowMultithreading = false;
            ////Space.PositionUpdater.AllowMultithreading = false;

            //Space.BeforeNarrowPhaseUpdateables.Enabled = false;
            //Space.DuringForcesUpdateables.Enabled = false;
            //Space.EndOfFrameUpdateables.Enabled = false;
            //Space.EndOfTimeStepUpdateables.Enabled = false;

            Space.BroadPhase = new BruteForce();
            Space.NarrowPhase.BroadPhaseOverlaps = Space.BroadPhase.Overlaps;


            Entity ground = new MorphableEntity(new BoxShape(50, 1, 50));
            Space.Add(ground);

            for (int i = 0; i < 100; i++)
            {
                Entity box = new Box(new Vector3(.1f * i, 1 * i + 1, 0), 1, 1, 1, 1);
                box.IsAlwaysActive = true;
                entities.Add(box);
                box.CollisionInformation.Tag = i;
                Space.Add(box);
            }

            for (int i = 0; i < 100; i++)
                game.ModelDrawer.Add(entities[i]);
            for (int i = 0; i < 100; i++)
                game.ModelDrawer.Remove(entities[i]);
        }

        bool allowUpdates = true;

        public override void Update(float dt)
        {
            if (allowUpdates)
                base.Update(dt);
            numFrames++;

            if (numFrames > 300)
            {
                numRuns++;
                if (numRuns > 1)
                {
                    for (int i = 0; i < entities.Count; i++)
                    {
                        if (!motionState[i].Equals(entities[i].MotionState))
                        {
                            //Determinism failed.
                            Debug.WriteLine("Break");
                        }
                    }

                    //Determinism success! 
                }
                motionState.Clear();
                foreach (var e in entities)
                    motionState.Add(e.MotionState);
                allowUpdates = false;



            }
            else
            {

                if (numRuns == 0)
                {
                    if (numFrames > frameOverlaps.Count)
                    {
                        frameOverlaps.Add(new List<BroadPhaseOverlap>(Space.BroadPhase.Overlaps));
                    }
                    else
                    {
                        frameOverlaps[numFrames - 1] = new List<BroadPhaseOverlap>(Space.BroadPhase.Overlaps);
                    }
                }
                else
                {
                    //Test determinism of broadphase.
                    if (frameOverlaps[numFrames - 1].Count != Space.BroadPhase.Overlaps.Count)
                        Debug.WriteLine("Determinism failed.");
                    for (int i = 0; i < frameOverlaps[numFrames - 1].Count; i++)
                    {
                        if (frameOverlaps[numFrames - 1][i].EntryA.Tag != null && frameOverlaps[numFrames - 1][i].EntryB.Tag != null)
                        {
                            if (!(((int)frameOverlaps[numFrames - 1][i].EntryA.Tag == (int)Space.BroadPhase.Overlaps[i].EntryA.Tag &&
                               (int)frameOverlaps[numFrames - 1][i].EntryB.Tag == (int)Space.BroadPhase.Overlaps[i].EntryB.Tag) ||
                               ((int)frameOverlaps[numFrames - 1][i].EntryB.Tag == (int)Space.BroadPhase.Overlaps[i].EntryA.Tag &&
                               (int)frameOverlaps[numFrames - 1][i].EntryA.Tag == (int)Space.BroadPhase.Overlaps[i].EntryB.Tag)))
                            {
                                Debug.WriteLine("Determinism failed.");
                            }
                        }
                    }
                }
            }
        }

        static List<List<BroadPhaseOverlap>> frameOverlaps = new List<List<BroadPhaseOverlap>>();






        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Test3"; }
        }

    }
}