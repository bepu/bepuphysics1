using System;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;
using BEPUphysics;
using System.Threading;
using System.Diagnostics;
using BEPUphysicsDrawer.Models;
using Vector3 = BEPUutilities.Vector3;
using BoundingBox = BEPUutilities.BoundingBox;

namespace AsynchronousUpdateDemo
{
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    public class AsynchronousUpdateGame : Game
    {
        GraphicsDeviceManager graphics;
        Thread physicsThread;
        ModelDrawer modelDrawer;
        Camera camera;

        public Space Space;
        
        public KeyboardState KeyboardState;
        public MouseState MouseState;



        public AsynchronousUpdateGame()
        {
            graphics = new GraphicsDeviceManager(this);
            graphics.PreferredBackBufferWidth = 800;
            graphics.PreferredBackBufferHeight = 600;

            Content.RootDirectory = "Content";
        }

        /// <summary>
        /// Allows the game to perform any initialization it needs to before starting to run.
        /// This is where it can query for any required services and load any non-graphic
        /// related content.  Calling base.Initialize will enumerate through any components
        /// and initialize them as well.
        /// </summary>
        protected override void Initialize()
        {
            camera = new Camera(this, new Vector3(0, 10, 10), 5);



            base.Initialize();
        }

        /// <summary>
        /// LoadContent will be called once per game and is the place to load
        /// all of your content.
        /// </summary>
        protected override void LoadContent()
        {
            //Set up drawers
            modelDrawer = new InstancedModelDrawer(this);


            //Create the space and tell it that it should keep track of buffered states.  This will let the 
            //positions/orientations of entities be interpolated, producing a cleaner appearance.
            Space = new Space();
            Space.BufferedStates.Enabled = true;


            Space.ForceUpdater.Gravity = new Vector3(0, -9.81f, 0);
            Space.Add(new Box(Vector3.Zero, 30, 1, 30));  //Make the ground

            //Create a bunch of boxes randomly.

            Random random = new Random();
            int boxCount = 50;
            var spawnVolume = new BoundingBox(new Vector3(-5, 15, -5), new Vector3(5, 25, 5));
            for (int i = 0; i < boxCount; i++)
            {
                var position = new Vector3((float)(random.NextDouble() - 0.5f) * (spawnVolume.Max.X - spawnVolume.Min.X),
                                               (float)(random.NextDouble() - 0.5f) * (spawnVolume.Max.Y - spawnVolume.Min.Y),
                                               (float)(random.NextDouble() - 0.5f) * (spawnVolume.Max.Z - spawnVolume.Min.Z)) +
                                   (spawnVolume.Min + spawnVolume.Max) / 2;
                Space.Add(new Box(position, 1, 1, 1, 1));

            }


            //Start up the physics loop.
            physicsThread = new Thread(PhysicsLoop);
            physicsThread.IsBackground = true;
            physicsThread.Start();

            #region DisplayObject creation
            foreach (Entity e in Space.Entities)
            {
                if ((string)e.Tag != "noDisplayObject")
                {
                    modelDrawer.Add(e);
                }
                else//Remove the now unnecessary tag.
                    e.Tag = null;
            }
            #endregion

        }

        /// <summary>
        /// UnloadContent will be called once per game and is the place to unload
        /// all content.
        /// </summary>
        protected override void UnloadContent()
        {
            // TODO: Unload any non ContentManager content here
        }

        /// <summary>
        /// Allows the game to run logic such as updating the world,
        /// checking for collisions, gathering input, and playing audio.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Update(GameTime gameTime)
        {
            KeyboardState = Keyboard.GetState();
            MouseState = Mouse.GetState();
            // Allows the game to exit
            if (KeyboardState.IsKeyDown(Keys.Escape))
            {
                this.Exit();
                return;
            }
            camera.Update((float)gameTime.ElapsedGameTime.TotalSeconds);


            //Calling Space.Add while it is possibly running is a no-no; it will interfere
            //with the update process and probably crash!  Instead, enqueue everything
            //to a thread-safe buffer which will be flushed in the physics loop.

            if (MouseState.LeftButton == ButtonState.Pressed)
            {
                Box toAdd = new Box(camera.Position, 1, 1, 1, 1);
                toAdd.LinearVelocity = camera.WorldMatrix.Forward * 10;
                Space.SpaceObjectBuffer.Add(toAdd);

                modelDrawer.Add(toAdd);
            }

            //Prevent the engine from flipping read buffers while we're reading data out of it.
            //It's a good idea to hold the LockerMotionStateBuffers as briefly as possible;
            //This will block if the engine tries to flip its internal buffers.
            //Technically, Space.BufferedStates.InterpolatedStates.GetStates() would 
            //hold the lock more briefly, but this lock lets us use the BEPUphysicsDrawer
            //system without significant difficulty or modifications.
            lock (Space.BufferedStates.InterpolatedStates.FlipLocker)
            {
                modelDrawer.Update();
            }

            base.Update(gameTime);

        }

        void PhysicsLoop()
        {
            double dt;
            double time;
            double previousTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency; //Give the engine a reasonable starting point.


            while (true)
            {
                time = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency; //compute the current time
                dt = time - previousTime; //find the time passed since the previous frame
                previousTime = time;

                Space.Update((float)dt);

                Thread.Sleep(0); //Explicitly give other threads (if any) a chance to execute
            }
        }

        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.CornflowerBlue);

            modelDrawer.Draw(camera.ViewMatrix, camera.ProjectionMatrix);

            base.Draw(gameTime);
        }
    }
}
