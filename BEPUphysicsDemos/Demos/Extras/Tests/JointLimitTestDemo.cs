using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.Constraints.TwoEntity.JointLimits;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    class JointLimitTestDemo : StandardDemo
    {
        public JointLimitTestDemo(DemosGame game)
            : base(game)
        {
            float bounciness = 1;
            float baseMass = 100;
            float armMass = 10;
            //DistanceLimit
            Box boxA = new Box(new Vector3(-21, 4, 0), 3, 3, 3, baseMass);
            Box boxB = new Box(boxA.Position + new Vector3(0, 5, 0), 1, 4, 1, armMass);
            CollisionRules.AddRule(boxA, boxB, CollisionRule.NoBroadPhase);
            boxB.ActivityInformation.IsAlwaysActive = true;

            var distanceLimit = new DistanceLimit(boxA, boxB, boxA.Position, boxB.Position - new Vector3(0, 2, 0), 1, 6);
            distanceLimit.Bounciness = bounciness;

            Space.Add(boxA);
            Space.Add(boxB);
            Space.Add(distanceLimit);

            //EllipseSwingLimit
            boxA = new Box(new Vector3(-14, 4, 0), 3, 3, 3, baseMass);
            boxB = new Box(boxA.Position + new Vector3(0, 5, 0), 1, 4, 1, armMass);
            CollisionRules.AddRule(boxA, boxB, CollisionRule.NoBroadPhase);
            boxB.ActivityInformation.IsAlwaysActive = true;

            var ballSocketJoint = new BallSocketJoint(boxA, boxB, boxB.Position + new Vector3(0, -2, 0));
            var ellipseSwingLimit = new EllipseSwingLimit(boxA, boxB, Vector3.Up, MathHelper.Pi / 1.5f, MathHelper.Pi / 3);
            ellipseSwingLimit.Bounciness = bounciness;

            Space.Add(boxA);
            Space.Add(boxB);
            Space.Add(ballSocketJoint);
            Space.Add(ellipseSwingLimit);

            //LinearAxisLimit
            boxA = new Box(new Vector3(-7, 4, 0), 3, 3, 3, baseMass);
            boxB = new Box(boxA.Position + new Vector3(0, 5, 0), 1, 4, 1, armMass);
            CollisionRules.AddRule(boxA, boxB, CollisionRule.NoBroadPhase);
            boxB.ActivityInformation.IsAlwaysActive = true;

            var pointOnLineJoint = new PointOnLineJoint(boxA, boxB, boxA.Position, Vector3.Up, boxB.Position + new Vector3(0, -2, 0));
            var linearAxisLimit = new LinearAxisLimit(boxA, boxB, boxA.Position, boxB.Position + new Vector3(0, -2, 0), Vector3.Up, 0, 4);
            linearAxisLimit.Bounciness = bounciness;

            Space.Add(boxA);
            Space.Add(boxB);
            Space.Add(pointOnLineJoint);
            Space.Add(linearAxisLimit);

            //RevoluteLimit
            boxA = new Box(new Vector3(0, 4, 0), 3, 3, 3, baseMass);
            boxB = new Box(boxA.Position + new Vector3(0, 5, 0), 1, 4, 1, armMass);
            CollisionRules.AddRule(boxA, boxB, CollisionRule.NoBroadPhase);
            boxB.ActivityInformation.IsAlwaysActive = true;

            ballSocketJoint = new BallSocketJoint(boxA, boxB, boxB.Position + new Vector3(0, -2, 0));
            var revoluteAngularJoint = new RevoluteAngularJoint(boxA, boxB, Vector3.Forward);
            var revoluteLimit = new RevoluteLimit(boxA, boxB, Vector3.Forward, Vector3.Up, -MathHelper.PiOver4, MathHelper.PiOver4);
            revoluteLimit.Bounciness = bounciness;

            Space.Add(boxA);
            Space.Add(boxB);
            Space.Add(ballSocketJoint);
            Space.Add(revoluteAngularJoint);
            Space.Add(revoluteLimit);

            //SwingLimit
            boxA = new Box(new Vector3(7, 4, 0), 3, 3, 3, baseMass);
            boxB = new Box(boxA.Position + new Vector3(0, 5, 0), 1, 4, 1, armMass);
            CollisionRules.AddRule(boxA, boxB, CollisionRule.NoBroadPhase);
            boxB.ActivityInformation.IsAlwaysActive = true;

            ballSocketJoint = new BallSocketJoint(boxA, boxB, boxB.Position + new Vector3(0, -2, 0));
            var swingLimit = new SwingLimit(boxA, boxB, Vector3.Up, Vector3.Up, MathHelper.PiOver4);
            swingLimit.Bounciness = bounciness;

            Space.Add(boxA);
            Space.Add(boxB);
            Space.Add(ballSocketJoint);
            Space.Add(swingLimit);

            //TwistLimit
            boxA = new Box(new Vector3(14, 4, 0), 3, 3, 3, baseMass);
            boxB = new Box(boxA.Position + new Vector3(0, 5, 0), 1, 4, 1, armMass);
            CollisionRules.AddRule(boxA, boxB, CollisionRule.NoBroadPhase);
            boxB.ActivityInformation.IsAlwaysActive = true;

            ballSocketJoint = new BallSocketJoint(boxA, boxB, boxB.Position + new Vector3(0, -2, 0));
            revoluteAngularJoint = new RevoluteAngularJoint(boxA, boxB, Vector3.Up);
            var twistLimit = new TwistLimit(boxA, boxB, Vector3.Up, Vector3.Up, -MathHelper.PiOver4, MathHelper.PiOver4);
            twistLimit.Bounciness = bounciness;

            Space.Add(boxA);
            Space.Add(boxB);
            Space.Add(ballSocketJoint);
            Space.Add(revoluteAngularJoint);
            Space.Add(twistLimit);

            Space.Add(new Box(new Vector3(0, 0, 0), 60, 1, 60));
            game.Camera.Position = new Vector3(0, 6, 15);
        }

        public override string Name
        {
            get { return "Joint Limit Behavior Test"; }
        }
    }
}
