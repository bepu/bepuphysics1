using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
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

            //EllipseSwingLimit
            Box box = new Box(new Vector3(0, 25, 0), 1, 4, 1, 10);
            Box boxA = new Box(new Vector3(0, 21, 0), 3, 3, 3, 80);
            Space.Add(boxA);
            Space.Add(box);
            var ballSocketJoint = new BallSocketJoint(boxA, box, box.Position + new Vector3(0, -2, 0));
            Space.Add(ballSocketJoint);
            var angularJoint = new EllipseSwingLimit(boxA, box, Vector3.Up, MathHelper.Pi / 1.8f, MathHelper.Pi / 4);
            angularJoint.Bounciness = 1;
            Space.Add(angularJoint);
            box.ActivityInformation.IsAlwaysActive = true;

            box.Orientation = Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathHelper.Pi / 8) * Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), MathHelper.Pi / 8);

            Space.Add(new Box(new Vector3(0, 0, 0), 40, 1, 40));
            game.Camera.Position = new Vector3(0, 6, 15);
        }

        public override string Name
        {
            get { return "Joint Limit Behavior Test"; }
        }
    }
}
