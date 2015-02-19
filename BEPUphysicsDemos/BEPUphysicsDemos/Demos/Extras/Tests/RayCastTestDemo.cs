
using System;
using System.Collections.Generic;
using BEPUphysics;
using BEPUphysics.Entities.Prefabs;
using ConversionHelper;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Matrix = BEPUutilities.Matrix;
using Ray = BEPUutilities.Ray;
using Vector3 = BEPUutilities.Vector3;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Casts a ray against an object and visualizes the result.
    /// </summary>
    public class RayCastTestDemo : StandardDemo
    {
        private Vector3 origin;
        private Vector3 direction;
        private RayCastResult result;
        private bool hitAnything;

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public RayCastTestDemo(DemosGame game)
            : base(game)
        {
            Space.Add(new Box(new Vector3(0, -0.5f, 0), 50, 1, 50));

            //Put whatever you'd like to ray cast here.
            var capsule = new Capsule(new Vector3(0, 1.2f, 0), 1, 0.6f);
            capsule.AngularVelocity = new Vector3(1, 1, 1);
            Space.Add(capsule);

            var cylinder = new Cylinder(new Vector3(0, 5, 0), 2, .5f);
            cylinder.AngularVelocity = new Vector3(1, -1, 1);
            Space.Add(cylinder);

            var points = new List<Vector3>();

            var random = new Random(0);
            for (int k = 0; k < 40; k++)
            {
                points.Add(new Vector3(1 * (float)random.NextDouble(), 3 * (float)random.NextDouble(), 2 * (float)random.NextDouble()));
            }
            var convexHull = new ConvexHull(new Vector3(0, 10, 0), points);
            convexHull.AngularVelocity = new Vector3(-1, 1, 1);
            Space.Add(convexHull);


            game.Camera.Position = new Vector3(-10, 5, 10);
            game.Camera.Yaw((float)Math.PI / -4f);
            game.Camera.Pitch(-(float)Math.PI / 9f);

            //Starter ray.
            origin = new Vector3(10, 5, 0);
            direction = new Vector3(-3, -1, 0);

        }



        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Ray Cast Test"; }
        }

        public override void Update(float dt)
        {
            base.Update(dt);

#if WINDOWS
            if (Game.MouseInput.RightButton == ButtonState.Pressed)
            {
                origin = Game.Camera.Position;
                direction = Game.Camera.WorldMatrix.Forward;
            }
#endif
            hitAnything = Space.RayCast(new Ray(origin, direction * 3), 10000, out result);

        }

        public override void Draw()
        {

            base.Draw();

            if (!hitAnything)
            {
                //If we didn't hit anything, just point out into something approximating infinity.
                result.HitData.Location = origin + direction * 10000;
            }
            Game.LineDrawer.LightingEnabled = false;
            Game.LineDrawer.VertexColorEnabled = true;
            Game.LineDrawer.World = Microsoft.Xna.Framework.Matrix.Identity;
            Game.LineDrawer.View = MathConverter.Convert(Game.Camera.ViewMatrix);
            Game.LineDrawer.Projection = MathConverter.Convert(Game.Camera.ProjectionMatrix);
            Game.GraphicsDevice.BlendState = BlendState.Opaque;
            Game.GraphicsDevice.DepthStencilState = DepthStencilState.Default;

            foreach (var pass in Game.LineDrawer.CurrentTechnique.Passes)
            {
                pass.Apply();
                Game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList,
                                                       new[]
                                                               {
                                                                   new VertexPositionColor(MathConverter.Convert(origin), Color.Blue),
                                                                   new VertexPositionColor(MathConverter.Convert(result.HitData.Location), Color.Blue),
                                                                   new VertexPositionColor(MathConverter.Convert(result.HitData.Location), Color.Blue),
                                                                   new VertexPositionColor(MathConverter.Convert(result.HitData.Normal + result.HitData.Location), Color.Blue)
                                                               },
                                                       0, 2);
            }

        }



    }
}
