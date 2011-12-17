using System;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// A large number of blocks fall from the sky into stacks and deactivate.
    /// </summary>
    public class SleepModeDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public SleepModeDemo(DemosGame game)
            : base(game)
        {
            //Create a bunch of blocks.
            int zOffset = 5;
            int numRows = 1;

            int numColumns = 40;

            Entity toAdd;
            for (int i = 0; i < numRows; i++)
            {
                for (int j = 0; j < numColumns; j++)
                {
                    for (int k = 1; k <= 7; k++)
                    {
                        if (k % 2 == 1)
                        {
                            toAdd = new Box(new Vector3(j * 10 + -3, j * 10 + 3 * k, i * 10 + zOffset), 1, 1, 7, 20);
                            toAdd.LinearDamping = .9f;
                            toAdd.AngularDamping = .9f;
                            Space.Add(toAdd);
                            toAdd = new Box(new Vector3(j * 10 + 3, j * 10 + 3 * k, i * 10 + zOffset), 1, 1, 7, 20);
                            toAdd.LinearDamping = .9f;
                            toAdd.AngularDamping = .9f;
                            Space.Add(toAdd);
                        }
                        else
                        {
                            toAdd = new Box(new Vector3(j * 10 + 0, j * 10 + 3 * k, i * 10 + zOffset - 3), 7, 1, 1, 20);
                            toAdd.LinearDamping = .9f;
                            toAdd.AngularDamping = .9f;
                            Space.Add(toAdd);
                            toAdd = new Box(new Vector3(j * 10 + 0, j * 10 + 3 * k, i * 10 + zOffset + 3), 7, 1, 1, 20);
                            toAdd.LinearDamping = .9f;
                            toAdd.AngularDamping = .9f;
                            Space.Add(toAdd);
                        }
                    }
                    Space.Add(new Box(new Vector3(10 * j, -.5f, i * 10 + zOffset), 10, 1f, 10));
                }
            }

            game.Camera.Position = new Vector3(-30, 5, 25);
            game.Camera.Yaw = (float) Math.PI / -3;
            game.Camera.Pitch = -(float) Math.PI / -12;
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Sleep Mode Stress Test"; }
        }
    }
}