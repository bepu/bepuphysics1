using BEPUphysics;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.Character;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysicsDemos.SampleCode;
using BEPUphysicsDrawer.Lines;
using BEPUphysics.CollisionRuleManagement;
using System;
using BEPUphysicsDemos.AlternateMovement;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Ray = BEPUutilities.Ray;
using Vector2 = BEPUutilities.Vector2;
using Vector3 = BEPUutilities.Vector3;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Superclass of the normal demos that let the user
    /// shoot spheres, grab things, create vehicles, 
    /// and walk around as characters.
    /// </summary>
    public abstract class StandardDemo : Demo
    {
        protected FreeCameraControlScheme freeCameraControlScheme;
        protected CharacterControllerInput character;
        protected float grabDistance;
        protected MotorizedGrabSpring grabber;
        protected LineDisplayObjectBase grabberGraphic;
        protected Entity kapow;
        protected Explosion kapowMaker;
        protected VehicleInput vehicle;

        protected Texture2D whitePixel;

        public StandardDemo(DemosGame game)
            : base(game)
        {
            freeCameraControlScheme = new FreeCameraControlScheme(10, game.Camera, game);

            //Creates the player character (C).
            character = new CharacterControllerInput(Space, game.Camera, game);

            //Creates the drivable vehicle (V).
            var wheelModel = game.Content.Load<Model>("carWheel");
            var wheelTexture = game.Content.Load<Texture2D>("wheel");
            whitePixel = game.Content.Load<Texture2D>("whitePixel");
            vehicle = new VehicleInput(new Vector3(10000, 0, 0), Space, game.Camera, game, game.ModelDrawer, wheelModel, wheelTexture);
            Space.ForceUpdater.Gravity = new Vector3(0, -9.81f, 0f); //If left unset, the default value is (0,0,0).

            //Create the tossable ball.
            kapow = new Sphere(new Vector3(11000, 0, 0), .6f, 20);
            kapowMaker = new Explosion(Vector3.Zero, 400, 15, Space);
            //Create the right-click grab spring.
            grabber = new MotorizedGrabSpring();
            grabberGraphic = game.ConstraintDrawer.Add(grabber);
            grabberGraphic.IsDrawing = false;
            Space.Add(grabber);
            Space.Add(kapow);


            //IMPORTANT PERFORMANCE NOTE:
            //  BEPUphysics uses an iterative system to solve constraints.  You can tell it to do more or less iterations.
            //  Less iterations is faster; more iterations makes the result more accurate.
            //
            //  The amount of iterations needed for a simulation varies.  The "Wall" and "Pyramid" simulations are each fairly
            //  solver intensive, but as few as 4 iterations can be used with acceptable results.
            //  The "Jenga" simulation usually needs a few more iterations for stability; 7-9 is a good minimum.
            //
            //  The Dogbot demo shows how accuracy can smoothly increase with more iterations.
            //  With very few iterations (1-3), it has slightly jaggier movement, as if the parts used to construct it were a little cheap.
            //  As you give it a few more iterations, the motors and constraints get more and more robust.
            //  
            //  Many simulations can work perfectly fine with very few iterations, 
            //  and using a low number of iterations can substantially improve performance.
            //
            //  To change the number of iterations used, uncomment and change the following line (10 iterations is the default):

            //Space.Solver.IterationLimit = 10;

            rayCastFilter = RayCastFilter;
        }

        //The raycast filter limits the results retrieved from the Space.RayCast while grabbing.
        Func<BroadPhaseEntry, bool> rayCastFilter;
        bool RayCastFilter(BroadPhaseEntry entry)
        {
            return entry != character.CharacterController.Body.CollisionInformation && entry.CollisionRules.Personal <= CollisionRule.Normal;
        }

        public override void Update(float dt)
        {
            #region Kapow-Shooter Input

            //Update kapow-shooter
            if (!vehicle.IsActive)
#if !WINDOWS
                if (Game.GamePadInput.IsButtonDown(Buttons.RightTrigger))
#else
                if (Game.MouseInput.LeftButton == ButtonState.Pressed)
#endif
                {
                    if (character.IsActive) //Keep the ball out of the character's body if its being used.
                        kapow.Position = Game.Camera.Position + Game.Camera.WorldMatrix.Forward * 3;
                    else
                        kapow.Position = Game.Camera.Position + Game.Camera.WorldMatrix.Forward;
                    kapow.AngularVelocity = Vector3.Zero;
                    kapow.LinearVelocity = Game.Camera.WorldMatrix.Forward * 30;
                }

            #endregion

            #region Grabber Input

            //Update grabber

#if !WINDOWS
            if (Game.GamePadInput.IsButtonDown(Buttons.RightShoulder) && !grabber.IsUpdating)
#else
            if (Game.MouseInput.RightButton == ButtonState.Pressed && !grabber.IsGrabbing)
#endif
            {
                //Find the earliest ray hit
                RayCastResult raycastResult;
                if (Space.RayCast(new Ray(Game.Camera.Position, Game.Camera.WorldMatrix.Forward), 1000, rayCastFilter, out raycastResult))
                {
                    var entityCollision = raycastResult.HitObject as EntityCollidable;
                    //If there's a valid ray hit, then grab the connected object!
                    if (entityCollision != null && entityCollision.Entity.IsDynamic)
                    {
                        grabber.Setup(entityCollision.Entity, raycastResult.HitData.Location);
                        grabberGraphic.IsDrawing = true;
                        grabDistance = raycastResult.HitData.T;
                    }
                }

            }
#if !WINDOWS
            if (Game.GamePadInput.IsButtonDown(Buttons.RightShoulder) && grabber.IsUpdating)
#else
            else if (Game.MouseInput.RightButton == ButtonState.Pressed && grabber.IsUpdating)
#endif
            {
                grabber.GoalPosition = Game.Camera.Position + Game.Camera.WorldMatrix.Forward * grabDistance;
            }
#if !WINDOWS
            if (!Game.GamePadInput.IsButtonDown(Buttons.RightShoulder) && grabber.IsUpdating)
#else
            else if (Game.MouseInput.RightButton == ButtonState.Released && grabber.IsUpdating)
#endif
            {
                grabber.Release();
                grabberGraphic.IsDrawing = false;
            }

            #endregion

            #region Control State Input

#if !WINDOWS
            if (!vehicle.IsActive && Game.WasButtonPressed(Buttons.LeftTrigger))
            {
                kapowMaker.Position = kapow.Position;
                kapowMaker.Explode();
            }
            if (Game.WasButtonPressed(Buttons.A))
            {//Toggle character perspective.
                if (!character.IsActive)
                {
                    vehicle.Deactivate();
                    character.Activate();
                }
                else
                    character.Deactivate();
            }
            if (Game.WasButtonPressed(Buttons.B))
            {//Toggle vehicle perspective.
                if (!vehicle.IsActive)
                {
                    character.Deactivate();
                    vehicle.Activate();
                }
                else
                    vehicle.Deactivate();
            }
#else
            if (!vehicle.IsActive && Game.WasKeyPressed(Keys.Space))
            {
                //Detonate the bomb
                kapowMaker.Position = kapow.Position;
                kapowMaker.Explode();
            }
            if (Game.WasKeyPressed(Keys.C))
            {
                //Toggle character perspective.
                if (!character.IsActive)
                {
                    vehicle.Deactivate();
                    character.Activate();
                }
                else
                    character.Deactivate();
            }


            if (Game.WasKeyPressed(Keys.V))
            {
                //Toggle vehicle perspective.
                if (!vehicle.IsActive)
                {
                    character.Deactivate();
                    vehicle.Activate(Game.Camera.Position);
                }
                else
                    vehicle.Deactivate();
            }
#endif

            #endregion

            base.Update(dt); //Base.update updates the space, which needs to be done before the camera is updated.

            character.Update(dt, Game.PreviousKeyboardInput, Game.KeyboardInput, Game.PreviousGamePadInput, Game.GamePadInput);
            vehicle.Update(dt, Game.KeyboardInput, Game.GamePadInput);
            //If neither are active, just use the default camera movement style.
            if (!character.IsActive && !vehicle.IsActive)
                freeCameraControlScheme.Update(dt);
        }

        public override void CleanUp()
        {
            //Wouldn't want the character or vehicle to own the camera after we switch.
            character.Deactivate();
            vehicle.Deactivate();
            base.CleanUp();
        }

        public override void DrawUI()
        {
#if !WINDOWS
            if (Game.GamePadInput.IsButtonDown(Buttons.RightShoulder))
#else
            if (Game.MouseInput.RightButton == ButtonState.Pressed)
#endif
                Game.UIDrawer.Draw(whitePixel, new Microsoft.Xna.Framework.Rectangle(Game.Graphics.PreferredBackBufferWidth / 2, Game.Graphics.PreferredBackBufferHeight / 2, 3, 3), Microsoft.Xna.Framework.Color.LightBlue);
        }
    }
}