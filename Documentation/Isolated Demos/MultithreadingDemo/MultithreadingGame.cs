
using System;
using BEPUphysics;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysicsDrawer.Models;
using BEPUutilities.Threading;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;
using Vector3 = BEPUutilities.Vector3;
using BoundingBox = BEPUutilities.BoundingBox;

namespace MultithreadingDemo
{
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    public class MultithreadingGame : Game
    {
        GraphicsDeviceManager graphics;

        public Camera Camera;
        public Space Space;
        public ModelDrawer ModelDrawer;
#if XBOX360
        public GamePadState GamePadState;
#else
        public KeyboardState KeyboardState;
        public MouseState MouseState;
#endif


        public MultithreadingGame()
        {
            graphics = new GraphicsDeviceManager(this);
            graphics.PreferredBackBufferWidth = 800;
            graphics.PreferredBackBufferHeight = 600;
            Content.RootDirectory = "Content";
            IsFixedTimeStep = false;
        }

        /// <summary>
        /// Allows the game to perform any initialization it needs to before starting to run.
        /// This is where it can query for any required services and load any non-graphic
        /// related content.  Calling base.Initialize will enumerate through any components
        /// and initialize them as well.
        /// </summary>
        protected override void Initialize()
        {
            Camera = new Camera(this, new Vector3(0, 10, 10), 5);

            base.Initialize();
        }

        /// <summary>
        /// LoadContent will be called once per game and is the place to load
        /// all of your content.
        /// </summary>
        protected override void LoadContent()
        {
            //Set up drawers
            ModelDrawer = new InstancedModelDrawer(this);

            //Create the space itself
            Space = new Space();

            var parallelLooper = new ParallelLooper();
            //This section lets the engine know that it can make use of multithreaded systems
            //by adding threads to its thread pool.
            if (Environment.ProcessorCount > 1)
            {
                for (int i = 0; i < Environment.ProcessorCount; i++)
                {
                    parallelLooper.AddThread();
                }
            }

            Space = new Space(parallelLooper);

            Space.ForceUpdater.Gravity = new Vector3(0, -9.81f, 0);
            Space.Add(new Box(Vector3.Zero, 30, 1, 30));

            //Create a bunch of boxes randomly.

            Random random = new Random();
            int boxCount = 100;
            BoundingBox spawnVolume = new BoundingBox(new Vector3(-10, 10, -10), new Vector3(10, 30, 10));
            for (int i = 0; i < boxCount; i++)
            {
                Vector3 position = new Vector3((float)(random.NextDouble() - 0.5f) * (spawnVolume.Max.X - spawnVolume.Min.X),
                                               (float)(random.NextDouble() - 0.5f) * (spawnVolume.Max.Y - spawnVolume.Min.Y),
                                               (float)(random.NextDouble() - 0.5f) * (spawnVolume.Max.Z - spawnVolume.Min.Z)) +
                                   (spawnVolume.Min + spawnVolume.Max) / 2;
                Space.Add(new Box(position, 1, 1, 1, 1));

            }


            #region DisplayObject creation
            foreach (Entity e in Space.Entities)
            {
                if ((string)e.Tag != "noDisplayObject")
                {
                    ModelDrawer.Add(e);
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

            float dt = (float)gameTime.ElapsedGameTime.TotalSeconds;

            KeyboardState = Keyboard.GetState();
            MouseState = Mouse.GetState();
            // Allows the game to exit
            if (KeyboardState.IsKeyDown(Keys.Escape))
            {
                this.Exit();
                return;
            }

            Camera.Update(dt);
            

            if (MouseState.LeftButton == ButtonState.Pressed)
            {
                Box toAdd = new Box(Camera.Position, 1, 1, 1, 1);
                toAdd.LinearVelocity = Camera.WorldMatrix.Forward * 10;
                Space.Add(toAdd);

                ModelDrawer.Add(toAdd);
            }

            Space.Update();
            ModelDrawer.Update();

            base.Update(gameTime);
        }

        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.CornflowerBlue);

            ModelDrawer.Draw(Camera.ViewMatrix, Camera.ProjectionMatrix);

            base.Draw(gameTime);
        }
    }
}
