using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.UpdateableSystems.ForceFields;
using BEPUphysicsDemos.SampleCode;
using Microsoft.Xna.Framework;
using System.Collections.Generic;
using BEPUphysics.NarrowPhaseSystems;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Boxes fall on a planetoid.
    /// </summary>
    public class PlanetDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public PlanetDemo(DemosGame game)
            : base(game)
        {
            Space.ForceUpdater.Gravity = Vector3.Zero;

            //By pre-allocating a bunch of box-box pair handlers, the simulation will avoid having to allocate new ones at runtime.
            NarrowPhaseHelper.Factories.BoxBox.EnsureCount(1000);

            var planet = new Sphere(new Vector3(0, 0, 0), 30);
            Space.Add(planet);

            var field = new GravitationalField(new InfiniteForceFieldShape(), planet.Position, 66730 / 2f, 100, Space.BroadPhase.QueryAccelerator);
            Space.Add(field);

            //Drop the "meteorites" on the planet.
            Entity toAdd;
            int numColumns = 10;
            int numRows = 10;
            int numHigh = 10;
            float separation = 5;
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                    for (int k = 0; k < numHigh; k++)
                    {
                        toAdd = new Box(new Vector3(separation * i - numRows * separation / 2, 40 + k * separation, separation * j - numColumns * separation / 2), 1f, 1f, 1f, 5);
                        toAdd.LinearVelocity = new Vector3(30, 0, 0);
                        toAdd.LinearDamping = 0;
                        toAdd.AngularDamping = 0;
                        Space.Add(toAdd);
                    }
            game.Camera.Position = new Vector3(0, 0, 150);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Planet"; }
        }
    }
}