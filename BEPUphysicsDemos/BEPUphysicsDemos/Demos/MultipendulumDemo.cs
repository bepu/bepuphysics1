using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// A two-body pendulum hits a stack of boxes.
    /// </summary>
    public class MultipendulumDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public MultipendulumDemo(DemosGame game)
            : base(game)
        {
            //Create the blocks in the pendulum.
            var link1 = new Box(new Vector3(3, 5, 0), 1.1f, 1.1f, 1.1f, 2);
            var link2 = new Box(new Vector3(6, 5, 0), 1.1f, 1.1f, 1.1f, 2);
            Space.Add(link1);
            Space.Add(link2);
            var ground = new Box(new Vector3(3, -3, 0), 20, 1, 10);
            Space.Add(ground);
            //Connect them together.
            Space.Add(new BallSocketJoint(link1, link2, (link1.Position + link2.Position) / 2));
            Space.Add(new BallSocketJoint(link1, ground, new Vector3(0, 5, 0)));
            Entity toAdd;
            //Create a target stack.
            for (int k = 0; k < 4; k++)
            {
                toAdd = new Box(new Vector3(-2f, -1f + 2 * k, 0), 1.5f, 1.5f, 1.5f, 1);
                Space.Add(toAdd);
            }
            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(0, 2, 20);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Multipendulum"; }
        }
    }
}