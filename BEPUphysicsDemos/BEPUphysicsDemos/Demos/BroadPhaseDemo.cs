using System;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// A large bunch of cubes suspended in space.
    /// </summary>
    public class BroadPhaseDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public BroadPhaseDemo(DemosGame game)
            : base(game)
        {
            //Make a fatter kapow sphere.
            Space.Remove(kapow);
            kapow = new Sphere(new Vector3(11000, 0, 0), 1.5f, 1000);
            Space.Add(kapow);
            Space.Solver.IterationLimit = 1; //Essentially no sustained contacts, so don't need to worry about accuracy.
            Space.ForceUpdater.Gravity = Vector3.Zero;

            int numColumns = 15;
            int numRows = 15;
            int numHigh = 15;
            float separation = 3;

            Entity toAdd;

            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                    for (int k = 0; k < numHigh; k++)
                    {
                        toAdd = new Box(new Vector3(separation * i, k * separation, separation * j), 1, 1, 1, 1);
                        toAdd.Material.Bounciness = 1; //Superbouncy boxes help propagate shock waves.
                        toAdd.LinearDamping = 0f;
                        toAdd.AngularDamping = 0f;
                        Space.Add(toAdd);
                    }

            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(0, 3, -10);
            game.Camera.Yaw = -(float)Math.PI;
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Broad phase Stress Test"; }
        }
    }
}