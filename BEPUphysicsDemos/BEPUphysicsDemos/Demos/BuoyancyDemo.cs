using System;
using System.Collections.Generic;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.UpdateableSystems;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Blocks floating in a viscous fluid.
    /// </summary>
    public class BuoyancyDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public BuoyancyDemo(DemosGame game)
            : base(game)
        {
            var tris = new List<Vector3[]>();
            float basinWidth = 100;
            float basinLength = 100;
            float basinHeight = 16;
            float waterHeight = 15;

            //Remember, the triangles composing the surface need to be coplanar with the surface.  In this case, this means they have the same height.
            tris.Add(new[]
                         {
                             new Vector3(-basinWidth / 2, waterHeight, -basinLength / 2), new Vector3(basinWidth / 2, waterHeight, -basinLength / 2),
                             new Vector3(-basinWidth / 2, waterHeight, basinLength / 2)
                         });
            tris.Add(new[]
                         {
                             new Vector3(-basinWidth / 2, waterHeight, basinLength / 2), new Vector3(basinWidth / 2, waterHeight, -basinLength / 2),
                             new Vector3(basinWidth / 2, waterHeight, basinLength / 2)
                         });
            var fluid = new FluidVolume(Vector3.Up, -9.81f, tris, waterHeight, .8f, .8f, .7f, Space.BroadPhase.QueryAccelerator, Space.ThreadManager);


            //fluid.flowDirection = Vector3.Right;
            //fluid.flowForce = 80;
            //fluid.maxFlowSpeed = 50;
            Space.Add(fluid);
            game.ModelDrawer.Add(fluid);
            //Create the container.
            Space.Add(new Box(new Vector3(0, 0, 0), basinWidth, 1, basinLength));
            Space.Add(new Box(new Vector3(-basinWidth / 2 - .5f, basinHeight / 2 - .5f, 0), 1, basinHeight, basinLength));
            Space.Add(new Box(new Vector3(basinWidth / 2 + .5f, basinHeight / 2 - .5f, 0), 1, basinHeight, basinLength));
            Space.Add(new Box(new Vector3(0, basinHeight / 2 - .5f, -basinLength / 2 - .5f), basinWidth + 2, basinHeight, 1));
            Space.Add(new Box(new Vector3(0, basinHeight / 2 - .5f, basinLength / 2 + .5f), basinWidth + 2, basinHeight, 1));
            var random = new Random();

            //Create a bunch of random blocks.
            /*for (int k = 0; k < 1; k++)
            {
                toAddBox = new Box(new Vector3(random.Next((int)basinWidth) - basinWidth / 2f, 30 + (.1f) * k, random.Next((int)basinLength) - basinLength / 2f), 2, 4, 2, 12);
                toAddBox.CollisionMargin = .2f;
                toAddBox.AllowedPenetration = .1f;
                toAddBox.Orientation = Quaternion.Normalize(new Quaternion((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()));
                space.Add(toAddBox);
            }*/

            //Create a tiled floor.
            Entity toAdd;
            float boxWidth = 10;
            int numBoxesWide = 8;
            for (int i = 0; i < numBoxesWide; i++)
            {
                for (int k = 0; k < numBoxesWide; k++)
                {
                    toAdd = new Box(new Vector3(
                        -boxWidth * numBoxesWide / 2f + (boxWidth + .1f) * i,
                        15,
                        -boxWidth * numBoxesWide / 2f + (boxWidth + .1f)* k),
                        boxWidth, 5, boxWidth, 300);
                    
                    Space.Add(toAdd);
                }
            }


            //Create a bunch o' spheres and dump them into the water.
            /*for (int k = 0; k < 80; k++)
            {
                toAddSphere = new Sphere(new Vector3(-48 + k * 1f, 12 + 4 * k, (float)random.Next(-15, 15)), 2, 27);
                space.Add(toAddSphere);
            }*/


            game.Camera.Position = new Vector3(0, waterHeight + 5, 35);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Buoyancy"; }
        }
    }
}