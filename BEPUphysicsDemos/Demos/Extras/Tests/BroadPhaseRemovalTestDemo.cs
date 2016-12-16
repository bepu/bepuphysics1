using BEPUphysics.BroadPhaseSystems.Hierarchies;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities.DataStructures;
using ConversionHelper;
using Microsoft.Xna.Framework;
using System.Diagnostics;
using System;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Tests the speed of broad phase removals.
    /// </summary>
    public class BroadPhaseRemovalTestDemo : StandardDemo
    {
        double[] testResults;

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public BroadPhaseRemovalTestDemo(DemosGame game)
            : base(game)
        {
            Entity toAdd;
            //BoundingBox box = new BoundingBox(new Vector3(-5, 1, 1), new Vector3(5, 7, 7));
            BoundingBox box = new BoundingBox(new Vector3(-500, -500, -500), new Vector3(500, 500, 500));

            DynamicHierarchy dh = new DynamicHierarchy();

            Random rand = new Random(0);

            RawList<Entity> entities = new RawList<Entity>();
            for (int k = 0; k < 1000; k++)
            {
                Vector3 position = new Vector3((float)(rand.NextDouble() * (box.Max.X - box.Min.X) + box.Min.X),
                                               (float)(rand.NextDouble() * (box.Max.Y - box.Min.Y) + box.Min.Y),
                                               (float)(rand.NextDouble() * (box.Max.Z - box.Min.Z) + box.Min.Z));
                toAdd = new Box(MathConverter.Convert(position), 1, 1, 1, 1);

                entities.Add(toAdd);

            }

            testResults = new double[2];
            int runCount = 10;
            for (int k = 0; k < runCount; k++)
            {

                for (int i = 0; i < entities.Count; i++)
                {
                    dh.Add(entities[i].CollisionInformation);
                }

                long start = Stopwatch.GetTimestamp();
                for (int i = 0; i < entities.Count; i++)
                {
                    //dh.RemoveFast(entities[i].CollisionInformation);
                }
                long end = Stopwatch.GetTimestamp();
                testResults[0] += (end - start) / (double)Stopwatch.Frequency;

                

                for (int i = 0; i < entities.Count; i++)
                {
                    dh.Add(entities[i].CollisionInformation);
                }

                start = Stopwatch.GetTimestamp();
                for (int i = 0; i < entities.Count; i++)
                {
                    //dh.RemoveBrute(entities[i].CollisionInformation);
                }
                end = Stopwatch.GetTimestamp();
                testResults[1] += (end - start) / (double)Stopwatch.Frequency;

            }
            testResults[0] /= runCount;
            testResults[1] /= runCount;


        }


        public override void DrawUI()
        {
            base.DrawUI();
            //Game.DataTextDrawer.Draw("Time steps elapsed: ", timeStepsElapsed, new Vector2(600, 600));
            //return;
            Vector2 origin = new Vector2(100, 50);
            Vector2 spacing = new Vector2(80, 50);
            //Draw the horizontal core counts.
            for (int i = 0; i < testResults.GetLength(0); i++)
            {
                Game.DataTextDrawer.Draw(i + 1, origin + new Vector2(spacing.X * i, -30));
            }

            for (int i = 0; i < testResults.GetLength(0); i++)
            {
                Game.DataTextDrawer.Draw(testResults[i] *1e3, 1, origin + new Vector2(spacing.X * i, 0));
            }



        }



        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Hierarchy Removal"; }
        }

    }
}