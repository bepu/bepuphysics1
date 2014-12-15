using BEPUphysics.Constraints.TwoEntity.JointLimits;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class TwistTestDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public TwistTestDemo(DemosGame game)
            : base(game)
        {
            var a = new Box(new Vector3(-2, 2, 0), 1, 2, 2, 5);
            var b = new Box(new Vector3(2, 2, 0), 1, 2, 2, 5);
            b.Orientation = Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), MathHelper.PiOver4);
            Space.Add(a);
            Space.Add(b);

            var twistJoint = new TwistJoint(a, b, a.OrientationMatrix.Right, b.OrientationMatrix.Right);
            var twistMotor = new TwistMotor(a, b, a.OrientationMatrix.Right, b.OrientationMatrix.Right);
            twistMotor.Settings.Mode = MotorMode.Servomechanism;

            //Space.Add(twistJoint);
            Space.Add(twistMotor);

            var ballSocketJoint = new BallSocketJoint(a, b, (a.Position + b.Position) * 0.5f);
            var swingLimit = new SwingLimit(a, b, a.OrientationMatrix.Right, a.OrientationMatrix.Right, MathHelper.PiOver2);

            Space.Add(ballSocketJoint);
            Space.Add(swingLimit);





            Box ground = new Box(new Vector3(0, -.5f, 0), 50, 1, 50);
            Space.Add(ground);
            game.Camera.Position = new Vector3(0, 6, 15);
        }


        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Wall"; }
        }
    }
}