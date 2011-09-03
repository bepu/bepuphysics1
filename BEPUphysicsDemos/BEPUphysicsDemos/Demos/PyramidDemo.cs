
using System;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Bunch of blocks arranged in a pyramid, waiting to be blown up.
    /// </summary>
    public class PyramidDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public PyramidDemo(DemosGame game)
            : base(game)
        {
            float boxSize = 2f;
            int boxCount = 20;
            float platformLength = Math.Min(50, boxCount * boxSize + 10);
            Space.Add(new Box(new Vector3(0, -.5f, 0), boxCount * boxSize + 20, 1,
                              platformLength));

            for (int i = 0; i < boxCount; i++)
            {
                for (int j = 0; j < boxCount - i; j++)
                {
                    Space.Add(new Box(
                                  new Vector3(
                                      -boxCount * boxSize / 2 + boxSize / 2 * i + j * (boxSize),
                                      (boxSize / 2) + i * boxSize,
                                      0),
                                  boxSize, boxSize, boxSize, 20));
                }
            }
            //Down here are the 'destructors' used to blow up the pyramid.  One set is physically simulated, the other kinematic.

            //Sphere pow = new Sphere(new Vector3(-25, 5, 70), 2, 40);
            //pow.LinearVelocity = new Vector3(0, 10, -100);
            //Space.Add(pow);
            //pow = new Sphere(new Vector3(-15, 10, 70), 2, 40);
            //pow.LinearVelocity = new Vector3(0, 10, -100);
            //Space.Add(pow);
            //pow = new Sphere(new Vector3(-5, 15, 70), 2, 40);
            //pow.LinearVelocity = new Vector3(0, 10, -100);
            //Space.Add(pow);
            //pow = new Sphere(new Vector3(5, 15, 70), 2, 40);
            //pow.LinearVelocity = new Vector3(0, 10, -100);
            //Space.Add(pow);
            //pow = new Sphere(new Vector3(15, 10, 70), 2, 40);
            //pow.LinearVelocity = new Vector3(0, 10, -100);
            //Space.Add(pow);
            //pow = new Sphere(new Vector3(25, 5, 70), 2, 40);
            //pow.LinearVelocity = new Vector3(0, 10, -100);
            //Space.Add(pow);


            //var staticSphere = new Sphere(new Vector3(-40, 40, 120), 4);
            //staticSphere.LinearVelocity = new Vector3(0, 0, -70);
            //Space.Add(staticSphere);
            //staticSphere = new Sphere(new Vector3(-20, 40, 120), 9);
            //staticSphere.LinearVelocity = new Vector3(0, 0, -85);
            //Space.Add(staticSphere);
            //staticSphere = new Sphere(new Vector3(0, 40, 120), 14);
            //staticSphere.LinearVelocity = new Vector3(0, 0, -100);
            //Space.Add(staticSphere);
            //staticSphere = new Sphere(new Vector3(20, 40, 120), 9);
            //staticSphere.LinearVelocity = new Vector3(0, 0, -85);
            //Space.Add(staticSphere);
            //staticSphere = new Sphere(new Vector3(40, 40, 120), 4);
            //staticSphere.LinearVelocity = new Vector3(0, 0, -70);
            //Space.Add(staticSphere);


            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(-boxCount * boxSize, 2, boxCount * boxSize);
            game.Camera.Yaw = (float)Math.PI / -4f;
            game.Camera.Pitch = (float)Math.PI / 9f;
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Pyramid"; }
        }
    }
}