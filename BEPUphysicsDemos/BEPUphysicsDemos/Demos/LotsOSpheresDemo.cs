using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// A cube of stacked spheres sits and waits to be knocked over.
    /// </summary>
    public class LotsOSpheresDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public LotsOSpheresDemo(DemosGame game)
            : base(game)
        {
            game.Camera.Position = new Vector3(0, 8, 25);
            Space.Add(new Box(new Vector3(0, 0, 0), 120, 1, 120));

            int numColumns = 5;
            int numRows = 5;
            int numHigh = 5;
            float xSpacing = 2.09f;
            float ySpacing = 2.08f;
            float zSpacing = 2.09f;
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                    for (int k = 0; k < numHigh; k++)
                    {
                        Space.Add(new Sphere(new Vector3(
                                                 xSpacing * i - (numRows - 1) * xSpacing / 2f,
                                                 1.58f + k * (ySpacing),
                                                 2 + zSpacing * j - (numColumns - 1) * zSpacing / 2f),
                                             1, 1));
                    }
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Lots o' Spheres"; }
        }
    }
}