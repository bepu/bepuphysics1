using System;
using System.Diagnostics;
using BEPUphysics.Collidables;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.Materials;

namespace BEPUphysicsDemos.Demos.Tests
{
    /// <summary>
    /// For random tests and general fiddling.
    /// </summary>
    /// <remarks>
    /// This demo type is initially excluded from the main list in the DemosGame.
    /// To access it while playing the demos, add an entry to the demoTypes array for this TestDemo.
    /// </remarks>
    public class ReverseTrikeDemo : StandardDemo
    {
        private readonly RevoluteMotor drivingMotor1;
        private readonly RevoluteMotor drivingMotor2;
        private readonly RevoluteMotor steeringMotor1;
        private readonly RevoluteMotor steeringMotor2;
        private float driveSpeed = 30;
        private float maximumTurnAngle = MathHelper.Pi * .3f;

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public ReverseTrikeDemo(DemosGame game)
            : base(game)
        {
            game.Camera.Position = new Vector3(0, 2, 15);
            game.Camera.Yaw = 0;
            game.Camera.Pitch = 0;

            Space.Add(new Box(new Vector3(0, -5, 0), 20, 1, 20));

            var body = new Box(new Vector3(0, 0, 0), 2, 1, 3, 10);
            body.CollisionInformation.LocalPosition = new Vector3(0, .8f, 0);
            Space.Add(body);

            #region First Wheel

            var wheel = new Cylinder(body.Position + new Vector3(-1.3f, 0, -1.5f), .2f, .5f, 4);
            wheel.Material = new Material(1.5f, 1.5f, 0);
            wheel.Orientation = Quaternion.CreateFromAxisAngle(Vector3.Forward, MathHelper.PiOver2);

            //Preventing the occasional pointless collision pair can speed things up.
            CollisionRules.AddRule(body, wheel, CollisionRule.NoBroadPhase);

            //Connect the wheel to the body.
            var ballSocketJoint = new BallSocketJoint(body, wheel, wheel.Position);
            var swivelHingeAngularJoint = new SwivelHingeAngularJoint(body, wheel, Vector3.Up, Vector3.Right);
            //Motorize the wheel.
            drivingMotor1 = new RevoluteMotor(body, wheel, Vector3.Left);
            drivingMotor1.Settings.VelocityMotor.Softness = .2f;
            //Let it roll when the user isn't giving specific commands.
            drivingMotor1.IsActive = false;
            steeringMotor1 = new RevoluteMotor(body, wheel, Vector3.Up);
            steeringMotor1.Settings.Mode = MotorMode.Servomechanism;
            //The constructor makes a guess about how to set up the constraint.
            //It can't always be right since it doesn't have all the information;
            //in this case, it chooses the basis and test axis incorrectly.
            //This leads to a 'flipping' behavior when the wheel is rolling
            //(the test axis is 'rolling' with the wheel, and passes over
            //a singularity which causes a flip).

            //To fix this, we configure the constraint directly.
            //The basis is aligned with how the wheel is set up; we choose 'up' as 
            //the motorized axis, and right/forward to define the angle measurement plane.
            //The test axis is set to be perpendicular to the wheel's rotation so that
            //it only measures the steering angle.

            //If you're curious, the angle measurement is just a Math.Atan2.
            //The current world test axis is dotted against the two plane axes (Right and Forward here).
            //This gives an x and y value.  These can be plugged into Atan2 just like when
            //you compute an angle on a normal 2d graph.
            steeringMotor1.Basis.SetWorldAxes(Vector3.Up, Vector3.Right, Vector3.Forward);
            steeringMotor1.TestAxis = Vector3.Right;


            //Add the wheel and connection to the space.
            Space.Add(wheel);
            Space.Add(ballSocketJoint);
            Space.Add(swivelHingeAngularJoint);
            Space.Add(drivingMotor1);
            Space.Add(steeringMotor1);

            #endregion

            #region Second Wheel

            wheel = new Cylinder(body.Position + new Vector3(1.3f, 0, -1.5f), .2f, .5f, 4);
            wheel.Material = new Material(1.5f, 1.5f, 0);
            wheel.Orientation = Quaternion.CreateFromAxisAngle(Vector3.Forward, MathHelper.PiOver2);


            //Preventing the occasional pointless collision pair can speed things up.
            CollisionRules.AddRule(body, wheel, CollisionRule.NoBroadPhase);

            //Connect the wheel to the body.
            ballSocketJoint = new BallSocketJoint(body, wheel, wheel.Position);
            swivelHingeAngularJoint = new SwivelHingeAngularJoint(body, wheel, Vector3.Up, Vector3.Right);
            //Motorize the wheel.
            drivingMotor2 = new RevoluteMotor(body, wheel, Vector3.Left);
            drivingMotor2.Settings.VelocityMotor.Softness = .2f;
            //Let it roll when the user isn't giving specific commands.
            drivingMotor2.IsActive = false;
            steeringMotor2 = new RevoluteMotor(body, wheel, Vector3.Up);
            steeringMotor2.Settings.Mode = MotorMode.Servomechanism;
            //Configure the motor.  See wheel 1 for more description.
            steeringMotor2.Basis.SetWorldAxes(Vector3.Up, Vector3.Right, Vector3.Forward);
            steeringMotor2.TestAxis = Vector3.Right;


            //Add the wheel and connection to the space.
            Space.Add(wheel);
            Space.Add(ballSocketJoint);
            Space.Add(swivelHingeAngularJoint);
            Space.Add(drivingMotor2);
            Space.Add(steeringMotor2);

            #endregion

            #region Third Wheel

            wheel = new Cylinder(body.Position + new Vector3(0, -.3f, 1.5f), .2f, .5f, 4);
            wheel.Material = new Material(1.5f, 1.5f, 0);
            wheel.Orientation = Quaternion.CreateFromAxisAngle(Vector3.Forward, MathHelper.PiOver2);

            //Preventing the occasional pointless collision pair can speed things up.
            CollisionRules.AddRule(body, wheel, CollisionRule.NoBroadPhase);

            //Connect the wheel to the body.
            ballSocketJoint = new BallSocketJoint(body, wheel, wheel.Position);
            //Notice that the third wheel isn't a swivel hinge, it's just a revolute axis.
            //This lets it roll, but prevents flopping around like the wheels of a grocery cart.
            //Could have used a RevoluteJoint solver group here, but this shows it's possible to do
            //the same things without using the combo-constraints.
            var revoluteAngularJoint = new RevoluteAngularJoint(body, wheel, Vector3.Right);

            //Add the wheel and connection to the space.
            Space.Add(wheel);
            Space.Add(ballSocketJoint);
            Space.Add(revoluteAngularJoint);

            #endregion

            int xLength = 256;
            int zLength = 256;

            float xSpacing = 8f;
            float zSpacing = 8f;
            var heights = new float[xLength, zLength];
            for (int i = 0; i < xLength; i++)
            {
                for (int j = 0; j < zLength; j++)
                {
                    float x = i - xLength / 2;
                    float z = j - zLength / 2;
                    //heights[i,j] = (float)(x * y / 1000f);
                    heights[i, j] = (float)(10 * (Math.Sin(x / 8) + Math.Sin(z / 8)));
                    //heights[i,j] = 3 * (float)Math.Sin(x * y / 100f);
                    //heights[i,j] = (x * x * x * y - y * y * y * x) / 1000f;
                }
            }
            //Create the terrain.
            var terrain = new Terrain(heights, new AffineTransform(
                new Vector3(xSpacing, 1, zSpacing), 
                Quaternion.Identity, 
                new Vector3(-xLength * xSpacing / 2, -10, -zLength * zSpacing / 2)));
            Space.Add(terrain);

            game.ModelDrawer.Add(terrain);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Reverse Trike"; }
        }

        public override void Update(float dt)
        {
            if (Game.KeyboardInput.IsKeyDown(Keys.NumPad8))
            {
                //Go forward
                drivingMotor1.Settings.VelocityMotor.GoalVelocity = driveSpeed;
                drivingMotor2.Settings.VelocityMotor.GoalVelocity = driveSpeed;
                //The driving motors are disabled when no button is pressed, so need to turn it on.
                drivingMotor1.IsActive = true;
                drivingMotor2.IsActive = true;
            }
            else if (Game.KeyboardInput.IsKeyDown(Keys.NumPad5))
            {
                //Go backward
                drivingMotor1.Settings.VelocityMotor.GoalVelocity = -driveSpeed;
                drivingMotor2.Settings.VelocityMotor.GoalVelocity = -driveSpeed;
                //The driving motors are disabled when no button is pressed, so need to turn it on.
                drivingMotor1.IsActive = true;
                drivingMotor2.IsActive = true;
            }
            else
            {
                //Let it roll.
                drivingMotor1.IsActive = false;
                drivingMotor2.IsActive = false;
            }

            if (Game.KeyboardInput.IsKeyDown(Keys.NumPad4))
            {
                //Turn left
                steeringMotor1.Settings.Servo.Goal = maximumTurnAngle;
                steeringMotor2.Settings.Servo.Goal = maximumTurnAngle;
            }
            else if (Game.KeyboardInput.IsKeyDown(Keys.NumPad6))
            {
                //Turn right
                steeringMotor1.Settings.Servo.Goal = -maximumTurnAngle;
                steeringMotor2.Settings.Servo.Goal = -maximumTurnAngle;
            }
            else
            {
                //Face forward
                steeringMotor1.Settings.Servo.Goal = 0;
                steeringMotor2.Settings.Servo.Goal = 0;
            }

            base.Update(dt);
        }

        public override void DrawUI()
        {
            base.DrawUI();
        }
    }
}