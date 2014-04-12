
using System;
using System.Diagnostics;
using BEPUphysics.Constraints;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.Materials;
using BEPUphysics.NarrowPhaseSystems;
using BEPUphysics.Settings;
using BEPUutilities;
using Microsoft.Xna.Framework.Input;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Tests out the performance characteristics of the persistent manifold's contact management.
    /// </summary>
    public class PersistentManifoldStressTestDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public PersistentManifoldStressTestDemo(DemosGame game)
            : base(game)
        {
            var ground = new Box(new Vector3(0, -.5f, 0), 200, 1, 200);
            Space.Add(ground);



            var spawnVolume = new BoundingBox
                {
                    Min = new Vector3(-25, 2, -25),
                    Max = new Vector3(25, 102, 25)
                };

            var span = spawnVolume.Max - spawnVolume.Min;

            NarrowPhaseHelper.Factories.ConvexConvex.EnsureCount(30000);

            var random = new Random(5);
            for (int i = 0; i < 5000; ++i)
            {
                Vector3 position;
                position.X = spawnVolume.Min.X + (float)random.NextDouble() * span.X;
                position.Y = spawnVolume.Min.Y + (float)random.NextDouble() * span.Y;
                position.Z = spawnVolume.Min.Z + (float)random.NextDouble() * span.Z;

                var entity = new Capsule(position, 2, 0.8f, 10);
                Space.Add(entity);
            }

            for (int i = 0; i < 60; ++i)
            {
                Space.Update();
            }

            Process.GetCurrentProcess().PriorityClass = ProcessPriorityClass.RealTime;
            
            var start = Stopwatch.GetTimestamp();
            for (int i = 0; i < 1000; ++i)
            {
                Space.Update();
            }
            var end = Stopwatch.GetTimestamp();
            var time = (end - start) / (double)Stopwatch.Frequency;

            Console.WriteLine("Time: {0}", time);
 
            game.Camera.Position = new Vector3(-10, 10, 10);
            game.Camera.Yaw((float)Math.PI / -4f);
            game.Camera.Pitch((float)Math.PI / 9f);
        }



        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Persistent Manifold Stress"; }
        }
    }
}