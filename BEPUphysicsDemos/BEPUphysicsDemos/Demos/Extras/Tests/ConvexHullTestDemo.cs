using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using BEPUphysics;
using BEPUphysics.DataStructures;
using BEPUphysics.CollisionShapes;
using BEPUphysics.Materials;

namespace BEPUphysicsDemos.Demos.Tests
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
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
            var vertices = new[] 
            { 
                new Vector3(0, -1.750886E-9f, -1.5f),
                new Vector3(1, 1, 0.5f), 
                new Vector3(1, -1, 0.5f),
                new Vector3(-1, 1, 0.5f), 
                new Vector3(-1, -1, 0.5f), 
            };
            RawList<Vector3> hullVertices = new RawList<Vector3>();
            ConvexHullHelper.GetConvexHull(vertices, hullVertices);

            ConvexHull hull = new ConvexHull(vertices, 5);
            ShapeDistributionInformation shapeInfo;
            hull.CollisionInformation.Shape.ComputeDistributionInformation(out shapeInfo);
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