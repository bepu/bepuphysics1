#region Using Statements

using System;
using System.Collections.Generic;
using BEPUphysics.DeactivationManagement;
using BEPUphysics.Entities;
using BEPUphysicsDemos.Demos;
using BEPUphysicsDemos.SampleCode;
using BEPUphysicsDrawer.Font;
using BEPUphysicsDrawer.Lines;
using BEPUphysicsDrawer.Models;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using BEPUphysics.NarrowPhaseSystems.Pairs;

#endregion

namespace BEPUphysicsDemos
{
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    public class DemosGame : Game
    {
        public Camera Camera;

        private int currentSimulationIndex;
        private Demo currentSimulation;

        //Rendering Variables
        public GraphicsDeviceManager Graphics;
        private readonly List<VertexPositionColor> collisionNormalList = new List<VertexPositionColor>();
        private readonly List<VertexPositionColor> boundingBoxLines = new List<VertexPositionColor>();

        //Rendering tools
        public ModelDrawer ModelDrawer;
        public LineDrawer ConstraintDrawer;
        public BasicEffect LineDrawer;
        public SpriteBatch UIDrawer;
        public TextDrawer DataTextDrawer;
        public TextDrawer TinyTextDrawer;

        //Content
        private SpriteFont dataFont;
        private SpriteFont tinyFont;
        private Texture2D controlsMenu;

        //FPS calculation variables
        private double FPSlastTime;
        private double FPStotalSinceLast;
        private double FPStoDisplay;
        private double averagePhysicsTime;
        private int FPStotalFramesSinceLast;

        //Input
        public KeyboardState KeyboardInput;
        public KeyboardState PreviousKeyboardInput;
        public GamePadState GamePadInput;
        public GamePadState PreviousGamePadInput;
#if WINDOWS
        public MouseState MouseInput;
#endif

        //Display Booleans        
        private bool displayEntities = true;
        private bool displayUI = true;
        private bool displayConstraints = true;
        private bool displayMenu;
        private bool displayContacts;
        private bool displayBoundingBoxes;
        private bool displaySimulationIslands;

        private readonly Type[] demoTypes = new[]
                                                {
                                                    typeof (WallDemo),
                                                    typeof (PyramidDemo),
                                                    typeof (ColosseumDemo),
                                                    typeof (LotsOSpheresDemo),
                                                    typeof (JengaDemo),
                                                    typeof (IncomingDemo),
                                                    typeof (FancyShapesDemo),
                                                    typeof (CompoundBodiesDemo),
                                                    typeof (TerrainDemo),
                                                    typeof (StaticMeshDemo),
                                                    typeof (InstancedMeshDemo),
                                                    typeof (EntityConstructionDemo),
                                                    typeof (MultipendulumDemo),
                                                    typeof (ClothDemo),
                                                    typeof (EarthquakeDemo),
                                                    typeof (BridgeDemo),
                                                    typeof (ActionFigureDemo),
                                                    typeof (UnfortunateGuyDemo),
                                                    typeof (SawContraptionDemo),
                                                    typeof (DogbotDemo),
                                                    typeof (RobotArmDemo),
                                                    typeof (DiscreteVsContinuousDemo),
                                                    typeof (CoefficientsDemo),
                                                    typeof (CollisionFilteringDemo),
                                                    typeof (SpaceshipDemo),
                                                    typeof (SleepModeDemo),
                                                    typeof (BroadPhaseDemo),
                                                    typeof (BuoyancyDemo),
                                                    typeof (TornadoDemo),
                                                    typeof (PlanetDemo),
                                                    typeof (PathFollowingDemo),
                                                    typeof (FishInABarrelDemo),
                                                    typeof (DetectorVolumeDemo),
                                                    typeof (TestDemo),
                                                    typeof (TestDemo2),
                                                    typeof (TestDemo3),
                                                    typeof (TestingBroadPhases),
                                                    typeof (TestDemo4),
                                                    typeof (TerrainTesting)
                                                };


        public DemosGame()
        {
            Graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";

            Graphics.PreferredBackBufferWidth = 800;
            Graphics.PreferredBackBufferHeight = 600;
            Camera = new Camera(Vector3.Zero, 10, 0, 0, Matrix.CreatePerspectiveFieldOfView(MathHelper.PiOver4, 800 / 600f, .1f, 10000));


            Exiting += DemosGameExiting;
        }

        private void DemosGameExiting(object sender, EventArgs e)
        {
            currentSimulation.CleanUp();
        }


        protected override void Initialize()
        {
            if (GraphicsDevice.GraphicsProfile == GraphicsProfile.HiDef)
                ModelDrawer = new InstancedModelDrawer(this);
            else
                ModelDrawer = new BruteModelDrawer(this);

            ConstraintDrawer = new LineDrawer(this);
            ConstraintDrawer.DisplayTypes.Add(typeof(GrabSpring), typeof(DisplayGrabSpring));
            ConstraintDrawer.DisplayTypes.Add(typeof(MotorizedGrabSpring), typeof(DisplayMotorizedGrabSpring));

            base.Initialize();
        }


        /// <summary>
        /// Manages the switch to a new physics engine simulation.
        /// </summary>
        /// <param name="sim">Index of the simulation to switch to.</param>
        private void SwitchSimulation(int sim)
        {
            currentSimulationIndex = sim;

            //Clear out any old rendering stuff.
            ModelDrawer.Clear();
            ConstraintDrawer.Clear();

            //Tell the previous simulation it's done.
            if (currentSimulation != null)
            {
                currentSimulation.CleanUp();
            }
            //Create the new demo.
            Type demoType = demoTypes[currentSimulationIndex - 1];

#if !WINDOWS

            currentSimulation = (Demo)demoType.GetConstructor(new[] { typeof(DemosGame) })
                                                    .Invoke(new object[] { this });

#else
            currentSimulation = (Demo)Activator.CreateInstance(demoType, new object[] { this });
#endif
            
            #region DisplayObject creation

            foreach (Entity e in currentSimulation.Space.Entities)
            {
                if ((string)e.Tag != "noDisplayObject")
                {
                    ModelDrawer.Add(e);
                }
                else //Remove the now unnecessary tag.
                    e.Tag = null;
            }
            for (int i = 0; i < currentSimulation.Space.Solver.SolverUpdateables.Count; i++)
            {
                //Add the solver updateable and match up the activity setting.
                LineDisplayObjectBase objectAdded = ConstraintDrawer.Add(currentSimulation.Space.Solver.SolverUpdateables[i]);
                if (objectAdded != null)
                    objectAdded.IsDrawing = currentSimulation.Space.Solver.SolverUpdateables[i].IsActive;
            }

            #endregion

            GC.Collect();
        }

        protected override void LoadContent()
        {
            dataFont = Content.Load<SpriteFont>("DataFont");
            tinyFont = Content.Load<SpriteFont>("TinyFont");

            controlsMenu = Content.Load<Texture2D>("bepuphysicscontrols");

            IsFixedTimeStep = false;

            LineDrawer = new BasicEffect(GraphicsDevice);

            UIDrawer = new SpriteBatch(GraphicsDevice);

            DataTextDrawer = new TextDrawer(UIDrawer, dataFont, Color.White);
            TinyTextDrawer = new TextDrawer(UIDrawer, tinyFont, Color.White);

#if WINDOWS
            Mouse.SetPosition(200, 200); //This helps the camera stay on track even if the mouse is offset during startup.
#endif

            SwitchSimulation(1);
        }


        /// <summary>
        /// Determines whether or not the key was pressed this frame.
        /// </summary>
        /// <param name="key">Key to check.</param>
        /// <returns>Whether or not the key was pressed.</returns>
        public bool WasKeyPressed(Keys key)
        {
            return KeyboardInput.IsKeyDown(key) && PreviousKeyboardInput.IsKeyUp(key);
        }

        /// <summary>
        /// Determines whether or not the button was pressed this frame.
        /// </summary>
        /// <param name="button">Button to check.</param>
        /// <returns>Whether or not the button was pressed.</returns>
        public bool WasButtonPressed(Buttons button)
        {
            return GamePadInput.IsButtonDown(button) && PreviousGamePadInput.IsButtonUp(button);
        }

        /// <summary>
        /// Allows the game to run logic such as updating the world,
        /// checking for collisions, gathering input and playing audio.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Update(GameTime gameTime)
        {
            PreviousKeyboardInput = KeyboardInput;
            KeyboardInput = Keyboard.GetState();
            var dt = (float)gameTime.ElapsedGameTime.TotalSeconds;
#if WINDOWS
            MouseInput = Mouse.GetState();

            //Keep the mouse within the screen
            Mouse.SetPosition(200, 200);
#endif
            PreviousGamePadInput = GamePadInput;
            for (int i = 0; i < 4; i++)
            {
                GamePadInput = GamePad.GetState((PlayerIndex)i);
                if (GamePadInput.IsConnected)
                    break;
            }

            // Allows the default game to exit on Xbox 360 and Windows
            if (KeyboardInput.IsKeyDown(Keys.Escape) || GamePadInput.Buttons.Back == ButtonState.Pressed)
                Exit();

            #region Camera

            //Update the camera
#if !WINDOWS

            Camera.Update(dt, KeyboardInput, GamePadInput);
#else
            Camera.Update(dt, KeyboardInput, MouseInput, GamePadInput);
#endif

            #endregion

            #region UI Toggles

#if !WINDOWS
            if (WasButtonPressed(Buttons.Start))
            {
                displayMenu = !displayMenu;
            }
#else
            if (WasKeyPressed(Keys.F1))
            {
                displayMenu = !displayMenu;
            }
#endif
            if (WasKeyPressed(Keys.I))
            {
                displayUI = !displayUI;
            }
            if (WasKeyPressed(Keys.J))
            {
                displayConstraints = !displayConstraints;
            }
            if (WasKeyPressed(Keys.K))
            {
                displayContacts = !displayContacts;
            }
            if (WasKeyPressed(Keys.U))
            {
                displayBoundingBoxes = !displayBoundingBoxes;
            }
            if (WasKeyPressed(Keys.Y))
            {
                displayEntities = !displayEntities;
            }
            if (WasKeyPressed(Keys.H))
            {
                displaySimulationIslands = !displaySimulationIslands;
            }
            if (WasKeyPressed(Keys.G))
            {
                ModelDrawer.IsWireframe = !ModelDrawer.IsWireframe;
            }

            #endregion

            #region Simulation Switcharoo

#if !WINDOWS

            int switchTo = -2;
            if (WasButtonPressed(Buttons.DPadLeft))
                switchTo = currentSimulationIndex - 1;
            else if (WasButtonPressed(Buttons.DPadRight))
                switchTo = currentSimulationIndex + 1;
            if (switchTo != -2)
            {
                if (switchTo < 1)
                    switchTo += demoTypes.Length;
                else if (switchTo > demoTypes.Length)
                    switchTo = 1;
                SwitchSimulation(switchTo);
            }
#else

            foreach (Keys key in KeyboardInput.GetPressedKeys())
            {
                int code = key.GetHashCode();

                if (code >= 48 && code <= 57)
                {
                    int simNumber;
                    if (code == 48)
                        simNumber = 10;
                    else
                        simNumber = code - 48;
                    if (KeyboardInput.IsKeyDown(Keys.LeftShift))
                        simNumber += 10;
                    if (KeyboardInput.IsKeyDown(Keys.LeftControl))
                        simNumber += 20;

                    if (simNumber <= demoTypes.Length)
                    {
                        currentSimulation.Space.ThreadManager.Dispose(); //While it would clean up by itself, this gets it done a little quicker.
                        SwitchSimulation(simNumber);
                    }
                }
            }


#endif

            #endregion

            currentSimulation.Update(dt);

            ConstraintDrawer.Update();

            ModelDrawer.Update();
            base.Update(gameTime);
        }

        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(new Color(.41f, .41f, .45f, 1));

            if (displayEntities)
                ModelDrawer.Draw(Camera.ViewMatrix, Camera.ProjectionMatrix);

            if (displayConstraints)
                ConstraintDrawer.Draw(Camera.ViewMatrix, Camera.ProjectionMatrix);

            LineDrawer.LightingEnabled = false;
            LineDrawer.VertexColorEnabled = true;
            LineDrawer.World = Matrix.Identity;
            LineDrawer.View = Camera.ViewMatrix;
            LineDrawer.Projection = Camera.ProjectionMatrix;

            #region Contact Drawing

            //Draw collisions.
            if (displayContacts)
            {
                collisionNormalList.Clear();
                int contactCount = 0;
                foreach (INarrowPhasePair pair in currentSimulation.Space.NarrowPhase.Pairs)
                {
                    var pairHandler = pair as CollidablePairHandler;
                    if (pairHandler != null)
                    {
                        foreach (ContactInformation information in pairHandler.Contacts)
                        {
                            contactCount++;
                            collisionNormalList.Add(new VertexPositionColor(information.Contact.Position, Color.White));
                            collisionNormalList.Add(new VertexPositionColor(information.Contact.Position + information.Contact.Normal * information.Contact.PenetrationDepth, Color.Red));
                            collisionNormalList.Add(new VertexPositionColor(information.Contact.Position + information.Contact.Normal * information.Contact.PenetrationDepth, Color.White));
                            collisionNormalList.Add(new VertexPositionColor(information.Contact.Position + information.Contact.Normal * (information.Contact.PenetrationDepth + .3f), Color.White));
                        }
                    }
                }

                if (contactCount > 0)
                {
                    foreach (EffectPass pass in LineDrawer.CurrentTechnique.Passes)
                    {
                        pass.Apply();

                        GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, collisionNormalList.ToArray(), 0, collisionNormalList.Count / 2);
                    }
                }

            }

            #endregion

            #region bounding box drawing

            if (displayBoundingBoxes)
            {
                if (currentSimulation.Space.Entities.Count > 0)
                {
                    boundingBoxLines.Clear();

                    foreach (Entity e in currentSimulation.Space.Entities)
                    {
                        Vector3[] boundingBoxCorners = e.CollisionInformation.BoundingBox.GetCorners();
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], Color.DarkRed));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], Color.DarkRed));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], Color.DarkRed));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], Color.DarkRed));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], Color.DarkRed));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], Color.DarkRed));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], Color.DarkRed));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], Color.DarkRed));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], Color.DarkRed));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], Color.DarkRed));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], Color.DarkRed));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], Color.DarkRed));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], Color.DarkRed));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], Color.DarkRed));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], Color.DarkRed));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], Color.DarkRed));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], Color.DarkRed));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], Color.DarkRed));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], Color.DarkRed));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], Color.DarkRed));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], Color.DarkRed));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], Color.DarkRed));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], Color.DarkRed));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], Color.DarkRed));
                    }
                    foreach (EffectPass pass in LineDrawer.CurrentTechnique.Passes)
                    {
                        pass.Apply();
                        GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, boundingBoxLines.ToArray(), 0, currentSimulation.Space.Entities.Count * 12);
                    }
                }
            }

            #endregion

            #region Simulation Island Drawing

            if (displaySimulationIslands)
            {
                if (currentSimulation.Space.Entities.Count > 0)
                {
                    boundingBoxLines.Clear();
                    BoundingBox box;

                    for (int i = 0; i < currentSimulation.Space.DeactivationManager.SimulationIslands.Count; i++)
                    {
                        SimulationIsland s = currentSimulation.Space.DeactivationManager.SimulationIslands[i];

                        box = new BoundingBox(new Vector3(float.MaxValue, float.MaxValue, float.MaxValue),
                                              new Vector3(-float.MaxValue, -float.MaxValue, -float.MaxValue));
                        for (int j = 0; j < s.Members.Count; j++)
                        {
                            var e = s.Members[j] as Entity;
                            if (e != null)
                            {
                                box = BoundingBox.CreateMerged(box, e.CollisionInformation.BoundingBox);
                            }
                        }



                        Color colorToUse = s.IsActive ? Color.DarkGoldenrod : Color.DarkGray;
                        Vector3[] boundingBoxCorners = box.GetCorners();
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], colorToUse));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], colorToUse));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], colorToUse));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], colorToUse));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], colorToUse));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], colorToUse));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], colorToUse));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], colorToUse));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], colorToUse));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], colorToUse));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], colorToUse));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], colorToUse));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], colorToUse));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], colorToUse));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], colorToUse));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], colorToUse));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], colorToUse));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], colorToUse));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], colorToUse));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], colorToUse));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], colorToUse));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], colorToUse));

                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], colorToUse));
                        boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], colorToUse));
                    }

                    if (currentSimulation.Space.DeactivationManager.SimulationIslands.Count > 0)
                        foreach (EffectPass pass in LineDrawer.CurrentTechnique.Passes)
                        {
                            pass.Apply();
                            GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, boundingBoxLines.ToArray(), 0, currentSimulation.Space.DeactivationManager.SimulationIslands.Count * 12);
                        }
                }
            }

            #endregion

            #region UI Drawing

            UIDrawer.Begin();
            if (displayUI)
            {
                FPStotalSinceLast += gameTime.ElapsedGameTime.TotalSeconds;
                FPStotalFramesSinceLast++;
                if (gameTime.TotalGameTime.TotalSeconds - FPSlastTime > .25 && gameTime.ElapsedGameTime.TotalSeconds > 0)
                {
                    double avg = FPStotalSinceLast / FPStotalFramesSinceLast;
                    FPSlastTime = gameTime.TotalGameTime.TotalSeconds;
                    FPStoDisplay = Math.Round(1 / avg, 1);
                    averagePhysicsTime = Math.Round(1000 * currentSimulation.PhysicsTime, 1);
                    FPStotalSinceLast = 0;
                    FPStotalFramesSinceLast = 0;
                }

                DataTextDrawer.Draw("FPS: ", FPStoDisplay, new Vector2(50, 450));
                DataTextDrawer.Draw("Physics Time (ms): ", averagePhysicsTime, new Vector2(50, 467));
                DataTextDrawer.Draw("Collision Pairs: ", currentSimulation.Space.NarrowPhase.Pairs.Count, new Vector2(50, 484));
                int countActive = 0;

                for (int i = 0; i < currentSimulation.Space.Entities.Count; i++)
                {
                    if (currentSimulation.Space.Entities[i].IsActive)
                        countActive++;
                }
                DataTextDrawer.Draw("Active Objects: ", countActive, new Vector2(50, 501));
#if !WINDOWS
                DataTextDrawer.Draw("Press Start for Controls", new Vector2(50, 518));
#else
                DataTextDrawer.Draw("Press F1 for Controls", new Vector2(50, 518));
#endif

                TinyTextDrawer.Draw("Current Simulation: ", currentSimulationIndex, new Vector2(600, 500));
                TinyTextDrawer.Draw(currentSimulation.Name, new Vector2(620, 514));

                currentSimulation.DrawUI();
            }
            if (displayMenu)
            {
                UIDrawer.Draw(controlsMenu, new Rectangle(0, 0, 800, 600), Color.White);
            }
            UIDrawer.End();

            #endregion

            //This doesn't actually draw the elements in the demo (that's the modeldrawer's job),
            //but some demos can specify their own extra stuff to draw.
            currentSimulation.Draw();

            base.Draw(gameTime);
        }
    }
}