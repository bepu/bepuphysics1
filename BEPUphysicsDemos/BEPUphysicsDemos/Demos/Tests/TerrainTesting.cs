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
    /// Boxes fall onto a large terrain.  Try driving around on it!
    /// </summary>
    public class TerrainTesting : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public TerrainTesting(DemosGame game)
            : base(game)
        {
            //x and y, in terms of heightmaps, refer to their local x and y coordinates.  In world space, they correspond to x and z.
            //Setup the heights of the terrain.
            int xLength = 256;
            int zLength = 256;

            float xSpacing = 8f;
            float zSpacing = 8f;
            var heights = new float[xLength, zLength];
            for (int i = 0; i < xLength; i++)
            {
                for (int j = 0; j < zLength; j++)
                {
                    float x = i - xLength / 2;
                    float z = j - zLength / 2;
                    //heights[i,j] = (float)(x * y / 1000f);
                    heights[i, j] = (float)(2 * (Math.Sin(x / 8) + Math.Sin(z / 8)));
                    //heights[i,j] = 3 * (float)Math.Sin(x * y / 100f);
                    //heights[i,j] = (x * x * x * y - y * y * y * x) / 1000f;
                }
            }
            //Create the terrain.
            var terrain = new Terrain(heights, new AffineTransform(
                    new Vector3(xSpacing, 1, zSpacing),
                    Quaternion.Identity,
                    new Vector3(-xLength * xSpacing / 2, 0, -zLength * zSpacing / 2)));
            terrain.Thickness = 0;

            //MotionSettings.DefaultPositionUpdateMode = BEPUphysics.PositionUpdating.PositionUpdateMode.Continuous;

            //NarrowPhaseHelper.Factories.SphereSphere.EnsureCount(5000);
            //NarrowPhaseHelper.Factories.TerrainSphere.EnsureCount(5000);
            //Space.BroadPhase = new Grid2DSortAndSweep(Space.ThreadManager);

            //ConfigurationHelper.ApplySuperSpeedySettings(Space);
            //Space.ForceUpdater.Gravity = new Vector3();
            Space.Solver.IterationLimit = 1;
            Space.Add(terrain);
            for (int i = 0; i < 20; i++)
            {
                for (int j = 0; j < 10; j++)
                {
                    for (int k = 0; k < 20; k++)
                    {
                        Space.Add(new Sphere(new Vector3(0 + i * 3, 20 + j * 3, 0 + k * 3), 0.5f, 1)
                        {
                            //LocalInertiaTensorInverse = new Matrix3X3()
                            //Tag = "noDisplayObject"
                        });
                        //Space.Add(new Box(
                        //    new Vector3(0 + i * 4, 1000 + -j * 10, 0 + k * 4),
                        //    2 + i * j * k,
                        //    2 + i * j * k,
                        //    2 + i * j * k,
                        //    4 + 20 * i * j * k));
                    }
                }
            }


            //Random rand = new Random();


            //for (int i = 0; i < 7; i++)
            //{
            //    for (int j = 0; j < 7; j++)
            //    {
            //        for (int k = 0; k < 8; k++)
            //        {
            //            Vector3 position =
            //                new Vector3((float)rand.NextDouble() * 10 + i * 128,
            //                    400 + -j * 2,
            //                    (float)rand.NextDouble() * 10 + k * 128);
            //            float radius = (float)rand.NextDouble() + 1;
            //            Space.Add(new Sphere(position,
            //                radius,
            //                radius * radius * radius));
            //        }
            //    }
            //}



            game.ModelDrawer.Add(terrain);

            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(0, 30, 20);

            ////Pre-simulate.
            //for (int i = 0; i < 150; i++)
            //{
            //    Space.Update();
            //}

            //int numRuns = 10;
            ////Space.BeforeNarrowPhaseUpdateables.Enabled = false;
            ////Space.DuringForcesUpdateables.Enabled = false;
            ////Space.EndOfTimeStepUpdateables.Enabled = false;
            ////Space.EndOfFrameUpdateables.Enabled = false;

            ////Space.EntityStateWriteBuffer.Enabled = false;
            ////Space.BufferedStates.Enabled = false;
            ////Space.DeactivationManager.Enabled = false;
            ////Space.SpaceObjectBuffer.Enabled = false;
            ////Space.DeferredEventDispatcher.Enabled = false;
            ////Space.BoundingBoxUpdater.Enabled = false;

            ////Space.BroadPhase.Enabled = false;
            ////Space.NarrowPhase.Enabled = false;
            ////Space.ForceUpdater.Enabled = false;
            //////Space.Solver.Enabled = false;
            ////Space.PositionUpdater.Enabled = false;
            //for (int i = 0; i < numRuns; i++)
            //{
            //    Space.Update();
            //}

        }




        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Terrain"; }
        }
    }
}