using System;
using System.Collections.Generic;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.UpdateableSystems;
using BEPUutilities;

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
            var fluid = new FluidVolume(Vector3.Up, -9.81f, tris, waterHeight, .8f, .8f, .7f);
            Space.ForceUpdater.Gravity = new Vector3(0, -9.81f, 0);


            //fluid.FlowDirection = Vector3.Right;
            //fluid.FlowForce = 80;
            //fluid.MaxFlowSpeed = 50;
            Space.Add(fluid);
            game.ModelDrawer.Add(fluid);
            //Create the container.
            Space.Add(new Box(new Vector3(0, 0, 0), basinWidth, 1, basinLength));
            Space.Add(new Box(new Vector3(-basinWidth / 2 - .5f, basinHeight / 2 - .5f, 0), 1, basinHeight, basinLength));
            Space.Add(new Box(new Vector3(basinWidth / 2 + .5f, basinHeight / 2 - .5f, 0), 1, basinHeight, basinLength));
            Space.Add(new Box(new Vector3(0, basinHeight / 2 - .5f, -basinLength / 2 - .5f), basinWidth + 2, basinHeight, 1));
            Space.Add(new Box(new Vector3(0, basinHeight / 2 - .5f, basinLength / 2 + .5f), basinWidth + 2, basinHeight, 1));


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
                        -boxWidth * numBoxesWide / 2f + (boxWidth + .1f) * k),
                        boxWidth, 5, boxWidth, 300);

                    Space.Add(toAdd);
                }
            }


            //Create a bunch o' spheres and dump them into the water.
            //Random random = new Random();
            //for (int k = 0; k < 80; k++)
            //{
            //    var toAddSphere = new Sphere(new Vector3(-48 + k * 1f, 12 + 4 * k, random.Next(-15, 15)), 2, 27);
            //    Space.Add(toAddSphere);
            //}


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