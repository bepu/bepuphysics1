using BEPUphysics.Constraints.SolverGroups;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;
using BEPUphysics.Entities;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.TwoEntity.JointLimits;
using Microsoft.Xna.Framework.Input;
using System;
using BEPUphysics.BroadPhaseEntries;

namespace BEPUphysicsDemos.Demos.Extras
{
    /// <summary>
    /// <para>A car built from constraints and entities (as opposed to using the Vehicle class) drives around on a terrain and 
    /// sometimes a box.</para>
    /// <para>This shows a slightly different approach to building a constraint car compared to SuspensionCarDemo; this one has entities representing the suspension.</para>
    /// </summary>
    /// <remarks>
    /// This demo type is initially excluded from the main list in the DemosGame.
    /// To access it while playing the demos, add an entry to the demoTypes array for this TestDemo.
    /// </remarks>
    public class SuspensionCarDemo2 : StandardDemo
    {
        private readonly RevoluteMotor drivingMotor1;
        private readonly RevoluteMotor drivingMotor2;
        private readonly RevoluteMotor steeringMotor1;
        private readonly RevoluteMotor steeringMotor2;
        private float driveSpeed = 80;
        private float maximumTurnAngle = MathHelper.Pi * .2f;

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public SuspensionCarDemo2(DemosGame game)
            : base(game)
        {
            game.Camera.Position = new Vector3(0, 2, 15);

            Space.Add(new Box(new Vector3(0, -5, 0), 20, 1, 20));

            var body = new Box(new Vector3(0, 0, 0), 2, .5f, 3, 20);
            body.CollisionInformation.LocalPosition = new Vector3(0, 1f, 0);
            Space.Add(body);
            
            AddBackWheel(new Vector3(-1f, .55f, 1.3f), body, true);
            AddBackWheel(new Vector3(1f, .55f, 1.3f), body, false);
            Box suspensionLeg1, suspensionLeg2;
            AddDriveWheel(new Vector3(-1f, .55f, -1.3f), body, true, out drivingMotor1, out steeringMotor1, out suspensionLeg1);
            AddDriveWheel(new Vector3(1f, .55f, -1.3f), body, false, out drivingMotor2, out steeringMotor2, out suspensionLeg2);

            //Add a stabilizer so that the wheels can't point different directions.
            var steeringStabilizer = new RevoluteAngularJoint(suspensionLeg1, suspensionLeg2, Vector3.Right);
            Space.Add(steeringStabilizer);


            //x and y, in terms of heightmaps, refer to their local x and y coordinates.  In world space, they correspond to x and z.
            //Setup the heights of the terrain.
            int xLength = 180;
            int zLength = 180;

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

        void AddBackWheel(Vector3 suspensionOffset, Entity body, bool leftSide)
        {


            var suspensionLeg = new Box(body.Position + suspensionOffset, 0.25f, 0.8f, 0.25f, 10);
            const float horizontalWheelOffset = 0.2f;

            var wheel = new Cylinder(suspensionLeg.Position + new Vector3(leftSide ? -horizontalWheelOffset : horizontalWheelOffset, -suspensionLeg.HalfHeight, 0), .2f, .3f, 5f);
            wheel.Material.KineticFriction = 2.5f;
            wheel.Material.StaticFriction = 3.5f;
            wheel.Orientation = Quaternion.CreateFromAxisAngle(Vector3.Forward, MathHelper.PiOver2);

            //Preventing the occasional pointless collision pair can speed things up.
            CollisionRules.AddRule(wheel, body, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(wheel, suspensionLeg, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(suspensionLeg, body, CollisionRule.NoBroadPhase);

            //Connect the suspension to the body.
            var bodyToSuspension = new PrismaticJoint(body, suspensionLeg, suspensionLeg.Position, Vector3.Down, suspensionLeg.Position);
            bodyToSuspension.Motor.Settings.Mode = MotorMode.Servomechanism;
            bodyToSuspension.Motor.IsActive = true;
            bodyToSuspension.Motor.Settings.Servo.SpringSettings.Stiffness = 300;
            bodyToSuspension.Motor.Settings.Servo.SpringSettings.Damping = 70;

            bodyToSuspension.Limit.IsActive = true;
            bodyToSuspension.Limit.Minimum = -0.5f;
            bodyToSuspension.Limit.Maximum = 0;

            //Connect the wheel to the suspension.
            var suspensionToWheel = new RevoluteJoint(suspensionLeg, wheel, wheel.Position, Vector3.Right);

            //Add the wheel and connection to the space.
            Space.Add(wheel);
            Space.Add(suspensionLeg);
            Space.Add(bodyToSuspension);
            Space.Add(suspensionToWheel);
        }

        void AddDriveWheel(Vector3 suspensionOffset, Entity body, bool leftSide, out RevoluteMotor drivingMotor, out RevoluteMotor steeringMotor, out Box suspensionLeg)
        {

            suspensionLeg = new Box(body.Position + suspensionOffset, 0.25f, 0.8f, 0.25f, 10);

            const float horizontalWheelOffset = 0.2f;

            var wheel = new Cylinder(suspensionLeg.Position + new Vector3(leftSide ? -horizontalWheelOffset : horizontalWheelOffset, -suspensionLeg.HalfHeight, 0), .2f, .3f, 5f);
            wheel.Material.KineticFriction = 2.5f;
            wheel.Material.StaticFriction = 3.5f;
            wheel.Orientation = Quaternion.CreateFromAxisAngle(Vector3.Forward, MathHelper.PiOver2);

            //Preventing the occasional pointless collision pair can speed things up.
            CollisionRules.AddRule(wheel, body, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(wheel, suspensionLeg, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(suspensionLeg, body, CollisionRule.NoBroadPhase);

            //Connect the suspension to the body.
            var bodyToSuspension = new LineSliderJoint(body, suspensionLeg, suspensionLeg.Position, Vector3.Down, suspensionLeg.Position);
            bodyToSuspension.Limit.IsActive = true;
            bodyToSuspension.Limit.Minimum = -0.5f;
            bodyToSuspension.Limit.Maximum = 0;

            //This linear axis motor will give the suspension its springiness by pushing the wheels outward.
            bodyToSuspension.Motor.IsActive = true;
            bodyToSuspension.Motor.Settings.Mode = MotorMode.Servomechanism;
            bodyToSuspension.Motor.Settings.Servo.Goal = 0;
            bodyToSuspension.Motor.Settings.Servo.SpringSettings.Stiffness = 300;
            bodyToSuspension.Motor.Settings.Servo.SpringSettings.Damping = 70;
            

            steeringMotor = new RevoluteMotor(body, suspensionLeg, Vector3.Up);
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
            steeringMotor.Basis.SetWorldAxes(Vector3.Up, Vector3.Right);
            steeringMotor.TestAxis = Vector3.Right;

            //To make the steering a little more responsive, set a base speed at which error gets corrected.
            //This works on top of the default error reduction implied by the constraint's spring constants.
            steeringMotor.Settings.Servo.BaseCorrectiveSpeed = 1;


            //The revolute motor is weaker than some other types of constraints and maintaining a goal in the presence of extremely fast rotation and integration issues.
            //Laying a revolute limit on top of it can help mitigate the problem.
            var steeringConstraint = new RevoluteLimit(body, suspensionLeg, Vector3.Up, Vector3.Right, -maximumTurnAngle, maximumTurnAngle);

            //Connect the wheel to the suspension.
            var suspensionToWheel = new RevoluteJoint(suspensionLeg, wheel, wheel.Position, Vector3.Right);
            drivingMotor = suspensionToWheel.Motor;
            //The driving motor's default, created by the RevoluteJoint constructor above, chose the axis of rotation such that negatives values made the car go forward and vice versa.
            //Swap it around so that the positive values make the car roll forward instead!
            drivingMotor.Basis.SetWorldAxes(Vector3.Left, Vector3.Forward);
            drivingMotor.TestAxis = Vector3.Forward;
            drivingMotor.Settings.VelocityMotor.Softness = .3f;
            drivingMotor.Settings.MaximumForce = 100;


            //Add the wheel and connection to the space.
            Space.Add(wheel);
            Space.Add(suspensionLeg);
            Space.Add(bodyToSuspension);
            Space.Add(drivingMotor);
            Space.Add(steeringMotor);
            Space.Add(steeringConstraint);
            Space.Add(suspensionToWheel);

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


    }
}