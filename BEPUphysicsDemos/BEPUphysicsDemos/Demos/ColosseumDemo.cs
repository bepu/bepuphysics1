using System;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Ring-shaped structure made of blocks.
    /// </summary>
    public class ColosseumDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public ColosseumDemo(DemosGame game)
            : base(game)
        {
            double angle;
            int numBoxesPerRing = 12;
            float blockWidth = 2;
            float blockHeight = 2;
            float blockLength = 6f;
            float radius = 15;
            Entity toAdd;
            Space.Add(new Box(new Vector3(0, -blockHeight / 2 - 1, 0), 100, 2f, 100));
            double increment = MathHelper.TwoPi / numBoxesPerRing;
            for (int i = 0; i < 8; i++)
            {
                for (int k = 0; k < numBoxesPerRing; k++)
                {
                    if (i % 2 == 0)
                    {
                        angle = k * increment;
                        toAdd = new Box(new Vector3(-(float) Math.Cos(angle) * radius, i * blockHeight, (float) Math.Sin(angle) * radius), blockWidth, blockHeight, blockLength, 20);
                        toAdd.Orientation = Quaternion.CreateFromAxisAngle(Vector3.Up, (float) angle);
                        Space.Add(toAdd);
                    }
                    else
                    {
                        angle = (k + .5f) * increment;
                        toAdd = new Box(new Vector3(-(float)Math.Cos(angle) * radius, i * blockHeight, (float)Math.Sin(angle) * radius), blockWidth, blockHeight, blockLength, 20);
                        toAdd.Orientation = Quaternion.CreateFromAxisAngle(Vector3.Up, (float) angle);
                        Space.Add(toAdd);
                    }
                }
            }
            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(0, 2, 2);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Colosseum"; }
        }
    }
}