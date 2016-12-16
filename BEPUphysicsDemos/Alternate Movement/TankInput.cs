using System.Collections.Generic;
using BEPUphysics;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.Vehicle;
using BEPUphysicsDrawer.Models;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionShapes;
using BEPUutilities;
using ConversionHelper;

namespace BEPUphysicsDemos.AlternateMovement
{
    /// <summary>
    /// Handles input and movement of a vehicle in the game.
    /// Acts as the 'front end' for the bookkeeping and math of the vehicle within the physics engine.
    /// </summary>
    public class TankInput
    {
        /// <summary>
        /// Wheels belonging to the left track of the tank.
        /// </summary>
        private readonly List<Wheel> leftTrack = new List<Wheel>();

        /// <summary>
        /// Wheels belonging to the right track of the tank.
        /// </summary>
        private readonly List<Wheel> rightTrack = new List<Wheel>();

        /// <summary>
        /// Speed that the vehicle tries towreach when moving backward.
        /// </summary>
        public float BackwardSpeed = -15;

        /// <summary>
        /// Default coefficient of sliding friction on an individual wheel in the tank track.
        /// </summary>
        public float BaseSlidingFriction;


        /// <summary>
        /// Speed that the vehicle tries to reach when moving forward.
        /// </summary>
        public float ForwardSpeed = 15;

        /// <summary>
        /// Whether or not to use the vehicle's input.
        /// </summary>
        public bool IsActive;

        /// <summary>
        /// Default maximum force that an individual wheel in a tank track can exert.
        /// </summary>
        public float MaximumDriveForce;

        /// <summary>
        /// Draws the body and wheels.
        /// </summary>
        public ModelDrawer ModelDrawer;

        /// <summary>
        /// Owning space of the vehicle.
        /// </summary>
        public Space Space;

        /// <summary>
        /// Physics representation of the vehicle.
        /// </summary>
        public Vehicle Vehicle;

        /// <summary>
        /// List of graphical representations of the wheels on the vehicle.
        /// </summary>
        public List<DisplayModel> WheelModels;

        float FrictionBlender(float wheelFriction, float supportFriction, bool isKineticFriction, Wheel wheel)
        {
            //The default friction blender is multiplicative.  This tank had its coefficients designed for averaged coefficients.
            //So, let's average the friction coefficients!
            //Try to fiddle with the configuration and this blender to see how you like other approaches.
            return (wheelFriction + supportFriction) / 2;
        }

        /// <summary>
        /// Gets the camera control scheme ued by this input manager.
        /// </summary>
        public ChaseCameraControlScheme CameraControlScheme { get; private set; }


        /// <summary>
        /// Constructs the front end and the internal physics representation of the vehicle.
        /// </summary>
        /// <param name="position">Position of the tank.</param>
        /// <param name="owningSpace">Space to add the vehicle to.</param>
        /// <param name="camera">Camera to attach to the vehicle.</param>
        /// <param name="game">Running game.</param>
        /// <param name="drawer">Drawer used to draw the tank.</param>
        /// <param name="wheelModel">Model to use for the 'wheels' of the tank.</param>
        /// <param name="wheelTexture">Texture of the wheels on the tank.</param>
        public TankInput(Vector3 position, Space owningSpace, Camera camera, DemosGame game, ModelDrawer drawer, Model wheelModel, Texture2D wheelTexture)
        {
            var bodies = new List<CompoundShapeEntry>()
            {
                new CompoundShapeEntry(new BoxShape(4f, 1, 8), new Vector3(0, 0, 0), 500),
                new CompoundShapeEntry(new BoxShape(3, .7f, 4f), new Vector3(0, .5f + .35f, .5f), 1)
            };
            var body = new CompoundBody(bodies, 501);
            body.CollisionInformation.LocalPosition = new Vector3(0, .5f, 0);
            body.Position = (position); //At first, just keep it out of the way.
            Vehicle = new Vehicle(body);

            #region RaycastWheelShapes

            //The wheel model used is not aligned initially with how a wheel would normally look, so rotate them.
            MaximumDriveForce = 1800;
            BaseSlidingFriction = 3;

            Matrix wheelGraphicRotation = Matrix.CreateFromAxisAngle(Vector3.Forward, MathHelper.PiOver2);
            for (int i = 0; i < 6; i++)
            {
                var toAdd = new Wheel(
                    new RaycastWheelShape(.375f, wheelGraphicRotation),
                    new WheelSuspension(2000, 300f, Vector3.Down, 1.3f, new Vector3(-1.9f, 0, -2.9f + i * 1.15f)),
                    new WheelDrivingMotor(10, MaximumDriveForce, MaximumDriveForce),
                    new WheelBrake(7, 7, 1.0f),
                    new WheelSlidingFriction(BaseSlidingFriction, BaseSlidingFriction));
                toAdd.DrivingMotor.GripFrictionBlender = FrictionBlender;
                toAdd.Brake.FrictionBlender = FrictionBlender;
                toAdd.SlidingFriction.FrictionBlender = FrictionBlender;
                Vehicle.AddWheel(toAdd);
                leftTrack.Add(toAdd);
            }
            for (int i = 0; i < 6; i++)
            {
                var toAdd = new Wheel(
                    new RaycastWheelShape(.375f, wheelGraphicRotation),
                    new WheelSuspension(2000, 300f, Vector3.Down, 1.3f, new Vector3(1.9f, 0, -2.9f + i * 1.15f)),
                    new WheelDrivingMotor(10, 2000, 1000),
                    new WheelBrake(7, 7, 1.0f),
                    new WheelSlidingFriction(BaseSlidingFriction, BaseSlidingFriction));
                toAdd.DrivingMotor.GripFrictionBlender = FrictionBlender;
                toAdd.Brake.FrictionBlender = FrictionBlender;
                toAdd.SlidingFriction.FrictionBlender = FrictionBlender;
                Vehicle.AddWheel(toAdd);
                rightTrack.Add(toAdd);
            }

            #endregion

            foreach (Wheel wheel in Vehicle.Wheels)
            {
                //This is a cosmetic setting that makes it looks like the car doesn't have antilock brakes.
                wheel.Shape.FreezeWheelsWhileBraking = true;

                //By default, wheels use as many iterations as the space.  By lowering it,
                //performance can be improved at the cost of a little accuracy.
                wheel.Suspension.SolverSettings.MaximumIterationCount = 1;
                wheel.Brake.SolverSettings.MaximumIterationCount = 1;
                wheel.SlidingFriction.SolverSettings.MaximumIterationCount = 1;
                wheel.DrivingMotor.SolverSettings.MaximumIterationCount = 1;
            }

            Space = owningSpace;

            Space.Add(Vehicle);
            ModelDrawer = drawer;
            DisplayModel model;
            WheelModels = new List<DisplayModel>();
            for (int k = 0; k < Vehicle.Wheels.Count; k++)
            {
                Vehicle.Wheels[k].Shape.Detector.Tag = "noDisplayObject";
                model = new DisplayModel(wheelModel, ModelDrawer);
                ModelDrawer.Add(model);
                WheelModels.Add(model);
                model.Texture = wheelTexture;
            }



            CameraControlScheme = new ChaseCameraControlScheme(Vehicle.Body, new Vector3(0, 0.6f, 0), true, 10, camera, game);

        }

        /// <summary>
        /// Gives the vehicle control over the camera and movement input.
        /// </summary>
        public void Activate(Vector3 position)
        {
            if (!IsActive)
            {
                IsActive = true;
                //Put the vehicle where the camera is.
                Vehicle.Body.Position = position;
                Vehicle.Body.LinearVelocity = Vector3.Zero;
                Vehicle.Body.AngularVelocity = Vector3.Zero;
                Vehicle.Body.Orientation = Quaternion.Identity;
            }
        }

        /// <summary>
        /// Returns input control to the camera.
        /// </summary>
        public void Deactivate()
        {
            if (IsActive)
            {
                IsActive = false;
            }
        }


        /// <summary>
        /// Handles the input and movement of the character.
        /// </summary>
        /// <param name="dt">Time since last frame in simulation seconds.</param>
        /// <param name="keyboardInput">Keyboard state.</param>
        /// <param name="gamePadInput">Gamepad state.</param>
        public void Update(float dt, KeyboardState keyboardInput, GamePadState gamePadInput)
        {
            //Update the wheel's graphics.
            for (int k = 0; k < WheelModels.Count; k++)
            {
                WheelModels[k].WorldTransform = Vehicle.Wheels[k].Shape.WorldTransform;
            }

            if (IsActive)
            {
                CameraControlScheme.Update(dt);

                //The reason for the more complicated handling of turning is that real tanks'
                //treads target a certain speed and will apply positive or negative forces
                //to reach it.

                //The normal Vehicle class is slightly different.  If you're rolling down a hill
                //with a target velocity of 30 and you're actually going 40, the vehicle doesn't
                //try to slow down.  It won't have to apply any force to reach its goal, and lets
                //itself coast faster.

                //To change direction while moving, a tank can slow down one track.  Friction will
                //force a pivot.  Slowing down one track on this vehicle doesn't do anything
                //because the wheels will happily roll as fast as the other track, even if not
                //applying any driving force.

                //To overcome this difference, the opposite track actually tries to drive backward.
                //This forces the vehicle wheels to actually do work to slow down the track.
                //Going at full speed and reversing a track's direction can be a little jarring, so
                //its maximum force is modified dynamically to make it feel more correct.
#if XBOX360
                float speed = gamePadInput.Triggers.Right * ForwardSpeed + gamePadInput.Triggers.Left * BackwardSpeed;
                foreach (Wheel wheel in Vehicle.Wheels)
                {
                    wheel.DrivingMotor.TargetSpeed = speed;
                    wheel.DrivingMotor.MaximumForwardForce = MaximumDriveForce;
                    wheel.DrivingMotor.MaximumBackwardForce = MaximumDriveForce;
                    wheel.SlidingFriction.KineticCoefficient = BaseSlidingFriction;
                    wheel.SlidingFriction.StaticCoefficient = BaseSlidingFriction;
                }
                //Thumbsticks can have small values even when left alone, so allow a little margin.
                const float stickMargin = .1f;
                //"Approximately stationary" is good enough to turn normally.  Pick a reasonable tolerance.
                const float fullTurnSpeedLimit = 1;
                if (speed > fullTurnSpeedLimit)
                {

                    if (gamePadInput.ThumbSticks.Left.X < -stickMargin)
                    {
                        foreach (Wheel wheel in leftTrack)
                        {
                            //Tell one of the tracks to reverse direction, but don't let it 
                            //run at full force.  This helps prevents wild spinouts and encourages
                            //more 'tanky' movement.
                            wheel.DrivingMotor.TargetSpeed = -gamePadInput.ThumbSticks.Left.X * BackwardSpeed;
                            wheel.DrivingMotor.MaximumBackwardForce = MaximumDriveForce / 2;
                        }

                        //It's possible to configure the tank in such a way
                        //that you won't have to use separate sliding frictions while turning,
                        //but cheating is a lot easier.
                        ReduceSlidingFriction();
                    }
                    if (gamePadInput.ThumbSticks.Left.X > stickMargin)
                    {
                        foreach (Wheel wheel in rightTrack)
                        {
                            wheel.DrivingMotor.TargetSpeed = gamePadInput.ThumbSticks.Left.X * BackwardSpeed;
                            wheel.DrivingMotor.MaximumBackwardForce = MaximumDriveForce / 2;

                        }
                        ReduceSlidingFriction();
                    }
                }
                else if (speed < -fullTurnSpeedLimit)
                {
                    if (gamePadInput.ThumbSticks.Left.X > stickMargin)
                    {
                        foreach (Wheel wheel in leftTrack)
                        {
                            wheel.DrivingMotor.TargetSpeed = gamePadInput.ThumbSticks.Left.X * ForwardSpeed;
                            wheel.DrivingMotor.MaximumForwardForce = MaximumDriveForce / 2;
                        }
                        ReduceSlidingFriction();
                    }
                    if (gamePadInput.ThumbSticks.Left.X < -stickMargin)
                    {
                        foreach (Wheel wheel in rightTrack)
                        {
                            wheel.DrivingMotor.TargetSpeed = -gamePadInput.ThumbSticks.Left.X * ForwardSpeed;
                            wheel.DrivingMotor.MaximumForwardForce = MaximumDriveForce / 2;

                        }
                        ReduceSlidingFriction();
                    }
                }
                else
                {
                    if (gamePadInput.ThumbSticks.Left.X < 0)
                    {
                        //Turn left
                        foreach (Wheel wheel in leftTrack)
                        {
                            wheel.DrivingMotor.TargetSpeed = -gamePadInput.ThumbSticks.Left.X * BackwardSpeed / 5;
                        }
                        foreach (Wheel wheel in rightTrack)
                        {
                            wheel.DrivingMotor.TargetSpeed = -gamePadInput.ThumbSticks.Left.X * ForwardSpeed / 5;
                        }
                        ReduceSlidingFriction();
                    }
                    if (gamePadInput.ThumbSticks.Left.X > 0)
                    {
                        //Turn right
                        foreach (Wheel wheel in leftTrack)
                        {
                            wheel.DrivingMotor.TargetSpeed = gamePadInput.ThumbSticks.Left.X * ForwardSpeed / 5;
                        }
                        foreach (Wheel wheel in rightTrack)
                        {
                            wheel.DrivingMotor.TargetSpeed = gamePadInput.ThumbSticks.Left.X * BackwardSpeed / 5;
                        }
                        ReduceSlidingFriction();
                    }
                }

                if (gamePadInput.IsButtonDown(Buttons.LeftStick))
                    foreach (Wheel wheel in Vehicle.Wheels)
                    {
                        wheel.Brake.IsBraking = true;
                    }
                else
                    foreach (Wheel wheel in Vehicle.Wheels)
                    {
                        wheel.Brake.IsBraking = false;
                    }
#else
                //Reset properties to defaults.
                foreach (Wheel wheel in Vehicle.Wheels)
                {
                    wheel.Brake.IsBraking = false;
                    wheel.DrivingMotor.MaximumForwardForce = MaximumDriveForce;
                    wheel.DrivingMotor.MaximumBackwardForce = MaximumDriveForce;
                    wheel.SlidingFriction.KineticCoefficient = BaseSlidingFriction;
                    wheel.SlidingFriction.StaticCoefficient = BaseSlidingFriction;
                }

                List<Wheel> wheelsToAccelerate;
                if (keyboardInput.IsKeyDown(Keys.E))
                {
                    if (keyboardInput.IsKeyDown(Keys.S))
                    {
                        //Turn left while going forward
                        foreach (Wheel wheel in rightTrack)
                        {
                            wheel.DrivingMotor.TargetSpeed = ForwardSpeed;
                        }
                        foreach (Wheel wheel in leftTrack)
                        {
                            //Tell one of the tracks to reverse direction, but don't let it 
                            //run at full force.  This helps prevents wild spinouts and encourages
                            //more 'tanky' movement.
                            wheel.DrivingMotor.TargetSpeed = BackwardSpeed;
                            wheel.DrivingMotor.MaximumBackwardForce = MaximumDriveForce / 3;
                        }
                        //It's possible to configure the tank in such a way
                        //that you won't have to use separate sliding frictions while turning,
                        //but cheating is a lot easier.
                        ReduceSlidingFriction();
                    }
                    else if (keyboardInput.IsKeyDown(Keys.F))
                    {
                        //Turn right while going forward            
                        foreach (Wheel wheel in leftTrack)
                        {
                            wheel.DrivingMotor.TargetSpeed = ForwardSpeed;
                        }
                        foreach (Wheel wheel in rightTrack)
                        {
                            wheel.DrivingMotor.TargetSpeed = BackwardSpeed;
                            wheel.DrivingMotor.MaximumBackwardForce = MaximumDriveForce / 3;
                        }
                        ReduceSlidingFriction();
                    }
                    else
                    {
                        foreach (Wheel wheel in Vehicle.Wheels)
                        {
                            wheel.DrivingMotor.TargetSpeed = ForwardSpeed;
                        }
                    }

                }
                else if (keyboardInput.IsKeyDown(Keys.D))
                {
                    if (keyboardInput.IsKeyDown(Keys.F))
                    {
                        //Turn right while going back
                        foreach (var wheel in rightTrack)
                        {
                            wheel.DrivingMotor.TargetSpeed = BackwardSpeed;
                        }
                        foreach (Wheel wheel in leftTrack)
                        {
                            wheel.DrivingMotor.TargetSpeed = ForwardSpeed;
                            wheel.DrivingMotor.MaximumForwardForce = MaximumDriveForce / 3;
                        }
                        ReduceSlidingFriction();
                    }
                    else if (keyboardInput.IsKeyDown(Keys.S))
                    {
                        //Turn left while going back
                        foreach (var wheel in leftTrack)
                        {
                            wheel.DrivingMotor.TargetSpeed = BackwardSpeed;
                        }
                        foreach (Wheel wheel in rightTrack)
                        {
                            wheel.DrivingMotor.TargetSpeed = ForwardSpeed;
                            wheel.DrivingMotor.MaximumForwardForce = MaximumDriveForce / 3;
                        }
                        ReduceSlidingFriction();
                    }
                    else
                    {
                        foreach (Wheel wheel in Vehicle.Wheels)
                        {
                            wheel.DrivingMotor.TargetSpeed = BackwardSpeed;
                        }
                    }
      
                }
                else if (keyboardInput.IsKeyDown(Keys.S))
                {
                    //Turn left
                    foreach (Wheel wheel in leftTrack)
                    {
                        wheel.DrivingMotor.TargetSpeed = BackwardSpeed / 5;
                    }
                    foreach (Wheel wheel in rightTrack)
                    {
                        wheel.DrivingMotor.TargetSpeed = ForwardSpeed / 5;
                    }
                    ReduceSlidingFriction();
                }
                else if (keyboardInput.IsKeyDown(Keys.F))
                {
                    //Turn right
                    foreach (Wheel wheel in leftTrack)
                    {
                        wheel.DrivingMotor.TargetSpeed = ForwardSpeed / 5;
                    }
                    foreach (Wheel wheel in rightTrack)
                    {
                        wheel.DrivingMotor.TargetSpeed = BackwardSpeed / 5;
                    }
                    ReduceSlidingFriction();
                }
                else
                {
                    //Idle
                    foreach (Wheel wheel in Vehicle.Wheels)
                    {
                        wheel.DrivingMotor.TargetSpeed = 0;
                    }
                }
                if (keyboardInput.IsKeyDown(Keys.Space))
                {
                    //Brake
                    foreach (Wheel wheel in Vehicle.Wheels)
                    {
                        wheel.Brake.IsBraking = true;
                    }
                }


#endif
            }
            else
            {
                //Parking brake
                foreach (Wheel wheel in Vehicle.Wheels)
                {
                    wheel.Brake.IsBraking = true;
                    //Don't want the car to keep trying to drive.
                    wheel.DrivingMotor.TargetSpeed = 0;
                }
            }
        }

        private void ReduceSlidingFriction()
        {
            ////If you want to make turning while moving faster, you can enable this.
            ////Careful- with sliding friction reduction, the tank can twirl pretty fast!
            //foreach (Wheel wheel in Vehicle.Wheels)
            //{
            //    wheel.SlidingFriction.StaticCoefficient = BaseSlidingFriction * .8f;
            //    wheel.SlidingFriction.KineticCoefficient = BaseSlidingFriction * .8f;
            //}
        }
    }
}