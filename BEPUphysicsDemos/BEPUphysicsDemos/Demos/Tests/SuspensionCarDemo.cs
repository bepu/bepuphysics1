using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using BEPUphysics.Entities;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.TwoEntity.JointLimits;
using Microsoft.Xna.Framework.Input;
using System;
using BEPUphysics.MathExtensions;
using BEPUphysics.Collidables;
namespace BEPUphysicsDemos.Demos.Tests
{
    /// <summary>
    /// For random tests and general fiddling.
    /// </summary>
    /// <remarks>
    /// This demo type is initially excluded from the main list in the DemosGame.
    /// To access it while playing the demos, add an entry to the demoTypes array for this TestDemo.
    /// </remarks>
    public class SuspensionCarDemo : StandardDemo
    {
        private readonly RevoluteMotor drivingMotor1;
        private readonly RevoluteMotor drivingMotor2;
        private readonly RevoluteMotor steeringMotor1;
        private readonly RevoluteMotor steeringMotor2;
        private float driveSpeed = 40;
        private float maximumTurnAngle = MathHelper.Pi * .2f;

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public SuspensionCarDemo(DemosGame game)
            : base(game)
        {
            game.Camera.Position = new Vector3(0, 2, 15);
            game.Camera.Yaw = 0;
            game.Camera.Pitch = 0;

            Space.Add(new Box(new Vector3(0, -5, 0), 20, 1, 20));

            var body = new Box(new Vector3(0, 0, 0), 4, .5f, 5, 20);
            //body.CenterOfMassOffset = new Vector3(0, -.8f, 0);
            Space.Add(body);

            AddWheel(new Vector3(-1.8f, -1, 2.1f), body, out drivingMotor1, out steeringMotor1);
            AddWheel(new Vector3(1.8f, -1, 2.1f), body, out drivingMotor1, out  steeringMotor1);
            //Just overwrite the driving motor/steering motor from the previous ones, these last two are the real ones we want.
            AddWheel(new Vector3(-1.8f, -1, -2.1f), body, out drivingMotor1, out steeringMotor1);
            AddWheel(new Vector3(1.8f, -1, -2.1f), body, out drivingMotor2, out steeringMotor2);

            //x and y, in terms of heightmaps, refer to their local x and y coordinates.  In world space, they correspond to x and z.
            //Setup the heights of the terrain.
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

            //terrain.Thickness = 5; //Uncomment this and shoot some things at the bottom of the terrain! They'll be sucked up through the ground.

            Space.Add(terrain);

            game.ModelDrawer.Add(terrain);
        }

        void AddWheel(Vector3 wheelOffset, Entity body, out RevoluteMotor drivingMotor, out RevoluteMotor steeringMotor)
        {
            var wheel = new Cylinder(body.Position + wheelOffset, .2f, .5f, 10);
            wheel.Material.KineticFriction = 2.5f;
            wheel.Material.StaticFriction = 2.5f;
            wheel.Orientation = Quaternion.CreateFromAxisAngle(Vector3.Forward, MathHelper.PiOver2);

            //Preventing the occasional pointless collision pair can speed things up.
            CollisionRules.AddRule(wheel, body, CollisionRule.NoBroadPhase);

            //Connect the wheel to the body.
            var pointOnLineJoint = new PointOnLineJoint(body, wheel, wheel.Position, Vector3.Down, wheel.Position);
            var suspensionLimit = new LinearAxisLimit(body, wheel, wheel.Position, wheel.Position, Vector3.Down, -1, 0);
            //This linear axis motor will give the suspension its springiness by pushing the wheels outward.
            var suspensionSpring = new LinearAxisMotor(body, wheel, wheel.Position, wheel.Position, Vector3.Down);
            suspensionSpring.Settings.Mode = MotorMode.Servomechanism;
            suspensionSpring.Settings.Servo.Goal = 0;
            suspensionSpring.Settings.Servo.SpringSettings.StiffnessConstant = 300;
            suspensionSpring.Settings.Servo.SpringSettings.DampingConstant = 70;

            var swivelHingeAngularJoint = new SwivelHingeAngularJoint(body, wheel, Vector3.Up, Vector3.Right);
            //Motorize the wheel.
            drivingMotor = new RevoluteMotor(body, wheel, Vector3.Left);
            drivingMotor.Settings.VelocityMotor.Softness = .2f;
            drivingMotor.Settings.MaximumForce = 150;
            //Let it roll when the user isn't giving specific commands.
            drivingMotor.IsActive = false;
            steeringMotor = new RevoluteMotor(body, wheel, Vector3.Up);
            steeringMotor.Settings.Mode = MotorMode.Servomechanism;
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
            steeringMotor.Basis.SetWorldAxes(Vector3.Up, Vector3.Right, Vector3.Forward);
            steeringMotor.TestAxis = Vector3.Right;


            //Add the wheel and connection to the space.
            Space.Add(wheel);
            Space.Add(pointOnLineJoint);
            Space.Add(suspensionLimit);
            Space.Add(suspensionSpring);
            Space.Add(swivelHingeAngularJoint);
            Space.Add(drivingMotor);
            Space.Add(steeringMotor);

        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Suspension Car Demo"; }
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