using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Demo showing a tower of blocks being smashed by a sphere.
    /// </summary>
    public class IncomingDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public IncomingDemo(DemosGame game)
            : base(game)
        {
            Entity toAdd;
            //Build the stack...
            for (int k = 1; k <= 12; k++)
            {
                if (k % 2 == 1)
                {
                    toAdd = new Box(new Vector3(-3, k, 0), 1, 1, 7, 10);
                    Space.Add(toAdd);
                    toAdd = new Box(new Vector3(3, k, 0), 1, 1, 7, 10);
                    Space.Add(toAdd);
                }
                else
                {
                    toAdd = new Box(new Vector3(0, k, -3), 7, 1, 1, 10);
                    Space.Add(toAdd);
                    toAdd = new Box(new Vector3(0, k, 3), 7, 1, 1, 10);
                    Space.Add(toAdd);
                }
            }
            //And then smash it!
            toAdd = new Sphere(new Vector3(0, 150, 0), 3, 100);

            Space.Add(toAdd);
            Space.Add(new Box(new Vector3(0, 0, 0), 10, 1f, 10));
            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(0, 6, 30);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Incoming!"; }
        }
    }
}