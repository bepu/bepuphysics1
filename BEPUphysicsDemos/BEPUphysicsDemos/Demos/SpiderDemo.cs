using System.Collections.Generic;
using BEPUphysics.Constraints.SolverGroups;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;
using BEPUphysics.Entities;
using BEPUphysics.CollisionRuleManagement;
using Microsoft.Xna.Framework.Input;
using System;
using BEPUphysics.BroadPhaseEntries;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// A robot with multiple individually controllable legs qwops around.
    /// </summary>
    /// <remarks>
    /// This demo type is initially excluded from the main list in the DemosGame.
    /// To access it while playing the demos, add an entry to the demoTypes array for this demo.
    /// </remarks>
    public class SpiderDemo : StandardDemo
    {
        private List<RevoluteJoint> legJoints;

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public SpiderDemo(DemosGame game)
            : base(game)
        {
            game.Camera.Position = new Vector3(0, 2, 15);

            Space.Add(new Box(new Vector3(0, -5, 0), 20, 1, 20));

            BuildBot(new Vector3(0, 3, 0), out legJoints);


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

        void BuildBot(Vector3 position, out List<RevoluteJoint> legJoints)
        {
            var body = new Box(position + new Vector3(0, 2.5f, 0), 2, 3, 2, 30);
            Space.Add(body);

            legJoints = new List<RevoluteJoint>();

            //Plop four legs on the spider.
            BuildLeg(body, new RigidTransform(body.Position + new Vector3(-.8f, -1, -.8f), Quaternion.CreateFromAxisAngle(Vector3.Up, MathHelper.PiOver4 * 3)), legJoints);
            BuildLeg(body, new RigidTransform(body.Position + new Vector3(-.8f, -1, .8f), Quaternion.CreateFromAxisAngle(Vector3.Up, MathHelper.PiOver4 * 5)), legJoints);
            BuildLeg(body, new RigidTransform(body.Position + new Vector3(.8f, -1, -.8f), Quaternion.CreateFromAxisAngle(Vector3.Up, MathHelper.PiOver4)), legJoints);
            BuildLeg(body, new RigidTransform(body.Position + new Vector3(.8f, -1, .8f), Quaternion.CreateFromAxisAngle(Vector3.Up, MathHelper.PiOver4 * 7)), legJoints);
        }

        void BuildLeg(Entity body, RigidTransform legTransform, List<RevoluteJoint> legJoints)
        {
            //Build the leg in local space.
            var upperLeg = new Box(new Vector3(.75f, 0, 0), 1.5f, .6f, .6f, 7);
            var lowerLeg = new Box(new Vector3(1.2f, -.75f, 0), .6f, 1.5f, .6f, 7);

            //Increase the feetfriction to make walking easier.
            lowerLeg.Material.KineticFriction = 3;
            lowerLeg.Material.StaticFriction = 3;

            Space.Add(upperLeg);
            Space.Add(lowerLeg);

            //Pull the leg entities into world space.
            upperLeg.WorldTransform *= legTransform.Matrix;
            lowerLeg.WorldTransform *= legTransform.Matrix;

            //Don't let the leg pieces interact with each other.
            CollisionRules.AddRule(upperLeg, body, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(upperLeg, lowerLeg, CollisionRule.NoBroadPhase);

            //Connect the body to the upper leg.
            var bodyToUpper = new RevoluteJoint(body, upperLeg, legTransform.Position, legTransform.OrientationMatrix.Forward);
            //Both the motor and limit need to be activated for this leg.
            bodyToUpper.Limit.IsActive = true;
            //While the angular joint doesn't need any extra configuration, the limit does in order to ensure that its interpretation of angles matches ours.
            bodyToUpper.Limit.LocalTestAxis = Vector3.Left;
            bodyToUpper.Limit.Basis.SetWorldAxes(legTransform.OrientationMatrix.Forward, legTransform.OrientationMatrix.Left);
            bodyToUpper.Limit.MinimumAngle = -MathHelper.PiOver4;
            bodyToUpper.Limit.MaximumAngle = MathHelper.PiOver4;
            //Similarly, the motor needs configuration.
            bodyToUpper.Motor.IsActive = true;
            bodyToUpper.Motor.LocalTestAxis = Vector3.Left;
            bodyToUpper.Motor.Basis.SetWorldAxes(legTransform.OrientationMatrix.Forward, legTransform.OrientationMatrix.Left);
            bodyToUpper.Motor.Settings.Mode = MotorMode.Servomechanism;
            //Weaken the spring to prevent it from launching too much.
            bodyToUpper.Motor.Settings.Servo.SpringSettings.Stiffness *= .01f;
            bodyToUpper.Motor.Settings.Servo.SpringSettings.Damping *= .01f;
            Space.Add(bodyToUpper);

            //Connect the upper leg to the lower leg.
            var upperToLower = new RevoluteJoint(upperLeg, lowerLeg, legTransform.Position - legTransform.OrientationMatrix.Left * 1.2f, legTransform.OrientationMatrix.Forward);
            //Both the motor and limit need to be activated for this leg.
            upperToLower.Limit.IsActive = true;
            //While the angular joint doesn't need any extra configuration, the limit does in order to ensure that its interpretation of angles matches ours.
            upperToLower.Limit.LocalTestAxis = Vector3.Down;
            upperToLower.Limit.Basis.SetWorldAxes(legTransform.OrientationMatrix.Forward, legTransform.OrientationMatrix.Down);
            upperToLower.Limit.MinimumAngle = -MathHelper.PiOver4;
            upperToLower.Limit.MaximumAngle = MathHelper.PiOver4;
            //Similarly, the motor needs configuration.
            upperToLower.Motor.IsActive = true;
            upperToLower.Motor.LocalTestAxis = Vector3.Down;
            upperToLower.Motor.Basis.SetWorldAxes(legTransform.OrientationMatrix.Forward, legTransform.OrientationMatrix.Down);
            upperToLower.Motor.Settings.Mode = MotorMode.Servomechanism;
            //Weaken the spring to prevent it from launching too much.
            upperToLower.Motor.Settings.Servo.SpringSettings.Stiffness *= .01f;
            upperToLower.Motor.Settings.Servo.SpringSettings.Damping *= .01f;
            Space.Add(upperToLower);

            legJoints.Add(upperToLower);
            legJoints.Add(bodyToUpper);

        }


        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "QWOPbot"; }
        }

        public override void DrawUI()
        {
            base.DrawUI();
            Game.DataTextDrawer.Draw("QWRT to retract, OPKL to extend.  Good luck!", new Microsoft.Xna.Framework.Vector2(50, 50));
        }

        public override void Update(float dt)
        {
            //Extend the legs!
            if (Game.KeyboardInput.IsKeyDown(Keys.Q))
            {
                Extend(legJoints[0], dt);
                Extend(legJoints[1], dt);
            }
            if (Game.KeyboardInput.IsKeyDown(Keys.W))
            {
                Extend(legJoints[2], dt);
                Extend(legJoints[3], dt);
            }
            if (Game.KeyboardInput.IsKeyDown(Keys.R))
            {
                Extend(legJoints[4], dt);
                Extend(legJoints[5], dt);
            }
            if (Game.KeyboardInput.IsKeyDown(Keys.T))
            {
                Extend(legJoints[6], dt);
                Extend(legJoints[7], dt);
            }

            //Retract the legs!
            if (Game.KeyboardInput.IsKeyDown(Keys.O))
            {
                Retract(legJoints[0], dt);
                Retract(legJoints[1], dt);
            }
            if (Game.KeyboardInput.IsKeyDown(Keys.P))
            {
                Retract(legJoints[2], dt);
                Retract(legJoints[3], dt);
            }
            if (Game.KeyboardInput.IsKeyDown(Keys.K))
            {
                Retract(legJoints[4], dt);
                Retract(legJoints[5], dt);
            }
            if (Game.KeyboardInput.IsKeyDown(Keys.L))
            {
                Retract(legJoints[6], dt);
                Retract(legJoints[7], dt);
            }

            base.Update(dt);
        }

        void Extend(RevoluteJoint joint, float dt)
        {
            float extensionSpeed = 2;
            joint.Motor.Settings.Servo.Goal = MathHelper.Clamp(joint.Motor.Settings.Servo.Goal + extensionSpeed * dt, joint.Limit.MinimumAngle, joint.Limit.MaximumAngle);
        }

        void Retract(RevoluteJoint joint, float dt)
        {
            float retractionSpeed = 2;
            joint.Motor.Settings.Servo.Goal = MathHelper.Clamp(joint.Motor.Settings.Servo.Goal - retractionSpeed * dt, joint.Limit.MinimumAngle, joint.Limit.MaximumAngle);
        }


    }
}