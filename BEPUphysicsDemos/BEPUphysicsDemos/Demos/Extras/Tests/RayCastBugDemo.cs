
using System;
using BEPUphysics;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using BEPUphysics.Collidables.MobileCollidables;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Bunch of blocks arranged in a pyramid, waiting to be blown up.
    /// </summary>
    public class RayCastBugDemo : StandardDemo
    {
        private Vector3 origin;
        private Vector3 direction;
        private RayHit hitSlow;
        private RayHit hitFast;
        private RayHit hitTEST;
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public RayCastBugDemo(DemosGame game)
            : base(game)
        {
            Space.Add(new Box(new Vector3(0, -0.5f, 0), 50, 1, 50));

            Capsule capsule = new Capsule(new Vector3(0, 1.2f, 0), 1, 0.6f);
            capsule.Tag = "Character";
            Space.Add(capsule);


            game.Camera.Position = new Vector3(-10, 5, 10);
            game.Camera.Yaw = (float)Math.PI / -4f;
            game.Camera.Pitch = -(float)Math.PI / 9f;

            Vector3 position = new Vector3(5.1f, 3.1f, 0.1f);
            //Vector3 position = new Vector3(1.95f, 2.47f, 0.163f);
            Vector3 velocity = new Vector3(-1, -0.2f, 0.02f) * 3;
            //position = position + velocity * 79.486f / 60f;
            //position = position + velocity * 79.487f / 60f;
            position = position + velocity * -90000 / 60f;

            RayCastResult resultTEST;
            if (Space.RayCast(new Ray(position, velocity), 10000, out resultTEST))
            {
                Console.WriteLine("POS:{0}, HIT:{1}, NRM:{2}, T:{3}", position, resultTEST.HitData.Location, resultTEST.HitData.Normal, resultTEST.HitData.T);
                hitTEST = resultTEST.HitData;
            }

            this.origin = position;
            this.direction = velocity;


            position = new Vector3(5.1f, 3.1f, 0.1f);

            bool targetHit = false;


            for (int i = 0; i < 100; i++)
            {
                if (targetHit)
                {
                    break;
                }

                Vector3 dir = velocity;
                dir.Normalize();
                Ray ray = new Ray(position, dir);


                BEPUphysics.RayCastResult result;
                if (Space.RayCast(ray, 10000, out result))
                {
                    Console.WriteLine("POS:{0}, HIT:{1}, NRM:{2}, T:{3}", position, result.HitData.Location, result.HitData.Normal, result.HitData.T);
                    hitFast = result.HitData;
                }
                var dt = 1 / 60f;
                if (i == 92)
                    Console.WriteLine("Break");
                if (Space.RayCast(ray, velocity.Length() * dt, out result))
                {
                    targetHit = true;
                    string name = (string)(result.HitObject as ConvexCollidable).Entity.Tag;
                    Console.WriteLine("Target hit: {0}", name);
                    Console.WriteLine(" Location {0}", result.HitData.Location);
                    Console.WriteLine(" Normal   {0}", result.HitData.Normal);
                    Console.WriteLine(" T        {0}", result.HitData.T);
                    hitSlow = result.HitData;

                }
                else
                {
                    position += velocity * dt;
                }


            }
        }



        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "RayCast() bug demo"; }
        }

        public override void Draw()
        {

            base.Draw();

            Game.LineDrawer.LightingEnabled = false;
            Game.LineDrawer.VertexColorEnabled = true;
            Game.LineDrawer.World = Matrix.Identity;
            Game.LineDrawer.View = Game.Camera.ViewMatrix;
            Game.LineDrawer.Projection = Game.Camera.ProjectionMatrix;
            Game.GraphicsDevice.BlendState = BlendState.Opaque;
            Game.GraphicsDevice.DepthStencilState = DepthStencilState.Default;

            foreach (var pass in Game.LineDrawer.CurrentTechnique.Passes)
            {
                pass.Apply();
                Game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList,
                                                       new[]
                                                           {
                                                               new VertexPositionColor(origin, Color.Blue),
                                                               new VertexPositionColor(hitTEST.Location, Color.Blue),
                                                               new VertexPositionColor(hitTEST.Location, Color.Blue),
                                                               new VertexPositionColor(hitTEST.Normal + hitTEST.Location, Color.Blue)
                                                           },
                                                       0, 2);
            }
        }



    }
}
