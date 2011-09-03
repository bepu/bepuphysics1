using System;
using BEPUphysics.Collidables;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.MathExtensions;
using BEPUphysics.UpdateableSystems.ForceFields;
using BEPUphysicsDemos.SampleCode;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Unsuspecting blocks get ambushed by a whirlwind.
    /// </summary>
    public class TornadoDemo : StandardDemo
    {
        private readonly BoundingBoxForceFieldShape shape;
        private readonly Tornado tornado;

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public TornadoDemo(DemosGame game)
            : base(game)
        {
            shape = new BoundingBoxForceFieldShape(new BoundingBox(new Vector3(-100, -20, -40), new Vector3(-20, 120, 40)));
            tornado = new Tornado(shape, (shape.BoundingBox.Min + shape.BoundingBox.Max) / 2, new Vector3(0, 1, 0),
                                  150, false, 50, 10, 200, 200, 80, 2000, 40, 10, Space.BroadPhase.QueryAccelerator);
            tornado.ForceWakeUp = true; //The tornado will be moving, so it should wake up things that it comes into contact with.
            Space.Add(tornado);

            //Create the unfortunate box-like citizens about to be hit by the tornado.
            int numColumns = 10;
            int numRows = 10;
            int numHigh = 1;
            float separation = 1.5f;
            Entity toAdd;
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                    for (int k = 0; k < numHigh; k++)
                    {
                        toAdd = new Box(new Vector3(
                                            separation * i - numRows * separation / 2,
                                            5 + k * separation,
                                            separation * j - numColumns * separation / 2),
                                        1, 1, 1, 10);
                        Space.Add(toAdd);
                    }

            //x and y, in terms of heightmaps, refer to their local x and y coordinates.  In world space, they correspond to x and z.
            //Setup the heights of the terrain.
            int xLength = 256;
            int zLength = 256;

            float xSpacing = 8f;
            float zSpacing = 8f;
            var heights = new float[xLength,zLength];
            for (int i = 0; i < xLength; i++)
            {
                for (int j = 0; j < zLength; j++)
                {
                    float x = i - xLength / 2;
                    float z = j - zLength / 2;
                    //heights[i,j] = (float)Math.Pow(1.2 * Math.Sqrt(x * x + y * y), 2);
                    //heights[i,j] = -1f / (x * x + y * y);
                    //heights[i,j] = (float)(x * y / 100f);
                    heights[i,j] = (float)(5 * (Math.Sin(x / 8f) + Math.Sin(z / 8f)));
                    //heights[i,j] = 3 * (float)Math.Sin(x * y / 100f);
                    //heights[i,j] = (x * x * x * y - y * y * y * x) / 1000f;
                }
            }

            //Create the terrain.
            var terrain = new Terrain(heights, new AffineTransform(
                new Vector3(xSpacing, 1, zSpacing), 
                Quaternion.Identity, 
                new Vector3(-xLength * xSpacing / 2, 0, -zLength * zSpacing / 2)));
            Space.Add(terrain);
            game.ModelDrawer.Add(terrain);
            game.Camera.Position = new Vector3(0, 5, 60);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Tornado"; }
        }

        public override void Update(float dt)
        {
            //Move the origin of the force of the tornado,
            Vector3 increment = new Vector3(10, 0, 0) * dt;
            //Move the detection shape as well.
            shape.BoundingBox = new BoundingBox(shape.BoundingBox.Min + increment, shape.BoundingBox.Max + increment);
            tornado.Position += increment;
            base.Update(dt);
        }
    }
}