using System.Collections.Generic;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;
using BEPUphysics.Entities;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.TwoEntity.JointLimits;
using Microsoft.Xna.Framework.Input;
using System;

namespace BEPUphysicsDemos.Demos.Extras
{
    /// <summary>
    /// A car built from constraints and entities (as opposed to using the Vehicle class) drives around on a terrain and 
    /// sometimes a box.
    /// </summary>
    /// <remarks>
    /// This demo type is initially excluded from the main list in the DemosGame.
    /// To access it while playing the demos, add an entry to the demoTypes array for this TestDemo.
    /// </remarks>
    public class TankDemo : StandardDemo
    {
        private List<RevoluteMotor> leftTreadMotors;
        private List<RevoluteMotor> rightTreadMotors;

        private float driveSpeed = 30;

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public TankDemo(DemosGame game)
            : base(game)
        {
            game.Camera.Position = new Vector3(0, 2, 15);

            Space.Add(new Box(new Vector3(0, -5, 0), 20, 1, 20));

            var body = new Box(new Vector3(0, 0, 0), 4, .5f, 5, 20);
            body.CollisionInformation.LocalPosition = new Vector3(0, .8f, 0);
            Space.Add(body);


            var treadDescription = new TreadSegmentDescription
                {
                    Width = 0.5f,
                    Radius = 0.5f,
                    Mass = 1,
                    Friction = 2.5f,
                    MotorSoftness = 0,//0.3f,
                    MotorMaximumForce = 30,
                    SuspensionDamping = 70,
                    SuspensionStiffness = 300,
                    SuspensionLength = 1
                };

            leftTreadMotors = CreateTread(body, new Vector3(-1.8f, -.2f, -2.1f), 5, 1, treadDescription);
            rightTreadMotors = CreateTread(body, new Vector3(1.8f, -.2f, -2.1f), 5, 1, treadDescription);
            

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
                    heights[i, j] = (float)(20 * (Math.Sin(x / 8) + Math.Sin(z / 8)));
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

        private struct TreadSegmentDescription
        {
            public float Mass;
            public float Width;
            public float Radius;
            public float Friction;
            public float SuspensionLength;
            public float SuspensionStiffness;
            public float SuspensionDamping;
            public float MotorSoftness;
            public float MotorMaximumForce;
        }


        private List<RevoluteMotor> CreateTread(Entity tankBody, Vector3 offsetToFrontOfTread, int segmentCount, float spacing, TreadSegmentDescription treadSegmentDescription)
        {
            var segmentEntities = new List<Entity>();
            var segmentMotors = new List<RevoluteMotor>();
            Vector3 nextSegmentPosition = tankBody.Position + offsetToFrontOfTread;
            //The front of the tread includes the radius of the first segment.
            nextSegmentPosition.Z += treadSegmentDescription.Radius * 0.5f;
            for (int i = 0; i < segmentCount; ++i)
            {
                Entity segment;
                RevoluteMotor motor;
                CreateTreadSegment(nextSegmentPosition, tankBody, treadSegmentDescription, out segment, out motor);
                segmentEntities.Add(segment);
                segmentMotors.Add(motor);

                //The tread offset starts at the front of the vehicle and moves backward.
                nextSegmentPosition.Z += spacing;
            }


            for (int i = 1; i < segmentCount; ++i)
            {
                //Create constraints linking the segments together to ensure that the power of one motor is felt by other segments.
                Space.Add(new NoRotationJoint(segmentEntities[i - 1], segmentEntities[i]));
                //Don't let the tread segments collide.
                CollisionRules.AddRule(segmentEntities[i - 1], segmentEntities[i], CollisionRule.NoBroadPhase);
            }

            //Note: You can organize this in different ways. For example, you could have one motor which drives one wheel, which
            //in turn drives other wheels through these NoRotationJoints.

            //In such a one-motor model, it may be a good idea for stability to bind all wheels directly to the drive wheel with 
            //NoRotationJoints rather than using a chain of one wheel to the next.

            //Per-wheel drive motors are used in this example just because it is slightly more intuitive at a glance.
            //Each segment is no different than the others.

            return segmentMotors;
        }

        private void CreateTreadSegment(Vector3 segmentPosition, Entity body, TreadSegmentDescription treadSegmentDescription, out Entity segment, out RevoluteMotor motor)
        {
            segment = new Cylinder(segmentPosition, treadSegmentDescription.Width, treadSegmentDescription.Radius, treadSegmentDescription.Mass);

            segment.Material.KineticFriction = treadSegmentDescription.Friction;
            segment.Material.StaticFriction = treadSegmentDescription.Friction;
            segment.Orientation = Quaternion.CreateFromAxisAngle(Vector3.Forward, MathHelper.PiOver2);

            //Preventing the occasional pointless collision pair can speed things up.
            CollisionRules.AddRule(segment, body, CollisionRule.NoBroadPhase);

            //Connect the wheel to the body.
            var pointOnLineJoint = new PointOnLineJoint(body, segment, segment.Position, Vector3.Down, segment.Position);
            var suspensionLimit = new LinearAxisLimit(body, segment, segment.Position, segment.Position, Vector3.Down, -treadSegmentDescription.SuspensionLength, 0);
            //This linear axis motor will give the suspension its springiness by pushing the wheels outward.
            var suspensionSpring = new LinearAxisMotor(body, segment, segment.Position, segment.Position, Vector3.Down);
            suspensionSpring.Settings.Mode = MotorMode.Servomechanism;
            suspensionSpring.Settings.Servo.Goal = 0;
            suspensionSpring.Settings.Servo.SpringSettings.StiffnessConstant = treadSegmentDescription.SuspensionStiffness;
            suspensionSpring.Settings.Servo.SpringSettings.DampingConstant = treadSegmentDescription.SuspensionDamping;

            var revoluteJoint = new RevoluteAngularJoint(body, segment, Vector3.Right);
            //Make the joint extremely rigid.  There are going to be extreme conditions when the wheels get up to speed;
            //we don't want the forces involved to torque the wheel off the frame!
            revoluteJoint.SpringSettings.DampingConstant *= segment.Mass * 50;
            revoluteJoint.SpringSettings.StiffnessConstant *= segment.Mass * 50;
            //Motorize the wheel.
            motor = new RevoluteMotor(body, segment, Vector3.Left);
            motor.Settings.VelocityMotor.Softness = treadSegmentDescription.MotorSoftness;
            motor.Settings.MaximumForce = treadSegmentDescription.MotorMaximumForce;


            //Add the wheel and connection to the space.
            Space.Add(segment);
            Space.Add(pointOnLineJoint);
            Space.Add(suspensionLimit);
            Space.Add(suspensionSpring);
            Space.Add(revoluteJoint);
            Space.Add(motor);
        }


        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Tank Demo"; }
        }

        private void SetTreadGoal(float goal, List<RevoluteMotor> treadMotors)
        {
            foreach (var motor in treadMotors)
            {
                motor.Settings.VelocityMotor.GoalVelocity = goal;
            }
        }

        public override void Update(float dt)
        {
            if (Game.KeyboardInput.IsKeyDown(Keys.NumPad8))
            {
                if (Game.KeyboardInput.IsKeyDown(Keys.NumPad4))
                {
                    //Turn left while still allowing forward motion
                    SetTreadGoal(driveSpeed, rightTreadMotors);
                    SetTreadGoal(driveSpeed * 0.15f, leftTreadMotors);
                }
                else if (Game.KeyboardInput.IsKeyDown(Keys.NumPad6))
                {
                    //Turn right while still allowing forward motion
                    SetTreadGoal(driveSpeed * 0.15f, rightTreadMotors);
                    SetTreadGoal(driveSpeed, leftTreadMotors);
                }
                else
                {
                    //Go forward
                    SetTreadGoal(driveSpeed, rightTreadMotors);
                    SetTreadGoal(driveSpeed, leftTreadMotors);
                }
            }
            else if (Game.KeyboardInput.IsKeyDown(Keys.NumPad5))
            {

                if (Game.KeyboardInput.IsKeyDown(Keys.NumPad4))
                {
                    //Turn left while still allowing backward motion
                    SetTreadGoal(-driveSpeed, rightTreadMotors);
                    SetTreadGoal(-driveSpeed * 0.15f, leftTreadMotors);
                }
                else if (Game.KeyboardInput.IsKeyDown(Keys.NumPad6))
                {
                    //Turn right while still allowing backward motion
                    SetTreadGoal(-driveSpeed * 0.15f, rightTreadMotors);
                    SetTreadGoal(-driveSpeed, leftTreadMotors);
                }
                else
                {
                    //Go backward
                    SetTreadGoal(-driveSpeed, rightTreadMotors);
                    SetTreadGoal(-driveSpeed, leftTreadMotors);
                }
            }
            else
            {
                if (Game.KeyboardInput.IsKeyDown(Keys.NumPad4))
                {
                    //Turn left
                    SetTreadGoal(driveSpeed, rightTreadMotors);
                    SetTreadGoal(-driveSpeed, leftTreadMotors);
                }
                else if (Game.KeyboardInput.IsKeyDown(Keys.NumPad6))
                {
                    //Turn right
                    SetTreadGoal(-driveSpeed, rightTreadMotors);
                    SetTreadGoal(driveSpeed, leftTreadMotors);
                }
                else
                {
                    //Stop
                    SetTreadGoal(0, rightTreadMotors);
                    SetTreadGoal(0, leftTreadMotors);
                }
            }




            base.Update(dt);
        }


    }
}