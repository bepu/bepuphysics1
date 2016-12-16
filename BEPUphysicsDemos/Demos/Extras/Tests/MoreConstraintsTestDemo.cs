using BEPUphysics.Constraints.SolverGroups;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class MoreConstraintsTestDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public MoreConstraintsTestDemo(DemosGame game)
            : base(game)
        {

            Box boxA = new Box(new Vector3(0, 5, 0), 1, 2, 1, 10);
            Box boxB = new Box(new Vector3(0, 8, 0), 1, 2, 1, 10);
            boxA.Orientation = Quaternion.CreateFromYawPitchRoll(0, MathHelper.PiOver4, 0);
            boxB.Orientation = Quaternion.CreateFromYawPitchRoll(MathHelper.PiOver4, 0, 0);

            WeldJoint weld = new WeldJoint(boxA, boxB);

            Space.Add(boxA);
            Space.Add(boxB);
            Space.Add(weld);


            boxA = new Box(new Vector3(3, 5, 0), 1, 2, 1, 10);
            boxB = new Box(new Vector3(3, 8, 0), 1, 2, 1, 10);
            boxA.Orientation = Quaternion.CreateFromYawPitchRoll(0, MathHelper.PiOver4, 0);
            boxB.Orientation = Quaternion.CreateFromYawPitchRoll(MathHelper.PiOver4, 0, 0);

            BallSocketJoint ballSocket = new BallSocketJoint(boxA, boxB, (boxA.Position + boxB.Position) / 2);
            AngularMotor angularMotor = new AngularMotor(boxA, boxB);
            angularMotor.Settings.Mode = MotorMode.Servomechanism;


            Space.Add(boxA);
            Space.Add(boxB);
            Space.Add(ballSocket);
            Space.Add(angularMotor);


            Box ground = new Box(new Vector3(0, 0, 0), 10, 1, 10);

            Space.Add(ground);

            game.Camera.Position = new Vector3(0, 6, 15);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "More Constraint Testin'"; }
        }
    }
}