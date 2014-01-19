using System;
using System.Collections.Generic;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;
using BEPUutilities.DataStructures;
using BEPUphysics.CollisionShapes;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Demo testing convex hulls.
    /// </summary>
    public class ConvexHullTestDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public ConvexHullTestDemo(DemosGame game)
            : base(game)
        {

            var random = new Random(5);

            for (int i = 0; i < 500000; ++i)
            {
                List<Vector3> points = new List<Vector3>();
                for (int k = 0; k < random.Next(8, 60); k++)
                {
                    points.Add(new Vector3(-100 + 30 * (float)random.NextDouble(), 100 + 500 * (float)random.NextDouble(), 100 + 30 * (float)random.NextDouble()));
                }
                var convexHull = new ConvexHull(new Vector3(0, 7, 0), points, 10);
                Console.WriteLine(convexHull.CollisionInformation.Shape.Vertices.Count);
            }
            
            var vertices = new[] 
            { 
                new Vector3(0, -1.750886E-9f, -1.5f),
                new Vector3(1, 1, 0.5f), 
                new Vector3(1, -1, 0.5f),
                new Vector3(-1, 1, 0.5f), 
                new Vector3(-1, -1, 0.5f), 
            };

            var hullVertices = new RawList<Vector3>();
            ConvexHullHelper.GetConvexHull(vertices, hullVertices);

            ConvexHull hull = new ConvexHull(vertices, 5);
            Space.Add(hull);

            Box ground = new Box(new Vector3(0, -.5f, 0), 50, 1, 50);
            Space.Add(ground);
            game.Camera.Position = new Vector3(0, 6, 15);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Convex Hull Testing"; }
        }
    }
}