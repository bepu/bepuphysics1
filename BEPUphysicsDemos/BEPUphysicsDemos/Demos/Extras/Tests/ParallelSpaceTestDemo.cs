using System;
using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using BEPUphysics;
using System.Collections.Generic;
using BEPUphysics.Entities;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Verifies the functionality of multiple spaces running simultaneously.
    /// </summary>
    public class ParallelSpaceTestDemo : StandardDemo
    {
        List<Space> spaces = new List<Space>();
        List<Entity> entities = new List<Entity>();
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public ParallelSpaceTestDemo(DemosGame game)
            : base(game)
        {
            for (int i = 0; i < 32; i++)
            {
                var space = new Space();
                space.ForceUpdater.Gravity = new Vector3(0, -9.81f, 0);
                var box = new Box(new Vector3(20 * i, 0, 0), 100, 1, 100);
                space.Add(box);
                //game.ModelDrawer.Add(box);
                for (int j = 0; j < 30; j++)
                {
                    for (int k = 0; k < 10; k++)
                    {
                        box = new Box(new Vector3(20 * i, 2 + j * 1.1f, 0), 1, 1, 1, 1);
                        entities.Add(box);
                        space.Add(box);
                        //game.ModelDrawer.Add(box);
                    }
                }
                spaces.Add(space);
            }
            game.Camera.Position = new Vector3(20, 10, 70);

        }

        Random random = new Random();
        public override void Update(float dt)
        {
            base.Update(dt);
            for (int i = 0; i < entities.Count; i++)
            {
                var impulse = dt * new Vector3((float)(random.NextDouble() * 30 - 15), (float)(random.NextDouble() * 30 - 15), (float)(random.NextDouble() * 30 - 15));
                entities[i].ApplyLinearImpulse(ref impulse);
                impulse = dt * new Vector3((float)(random.NextDouble() * 10 - 5), (float)(random.NextDouble() * 10 - 5), (float)(random.NextDouble() * 10 - 5));
                entities[i].ApplyAngularImpulse(ref impulse);
            }

            //for (int i = 0; i < numberOfSpaces; i++)
            //{
            //    spaces[i].Update();
            //}
            //The "real" space has a convenient thread pool we can use.
            Space.ThreadManager.ForLoop(0, spaces.Count, i =>
            {
                spaces[i].Update();
            });

            timeSinceLastReset += dt;
            if (timeSinceLastReset > 10)
            {
                Console.WriteLine("Resetting.  Number of resets performed: " + (++resets));
                Game.SwitchSimulation(1);
            }   
        }
        static int resets;
        float timeSinceLastReset;

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Terrain"; }
        }
    }
}