using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using BEPUphysics.Constraints.SolverGroups;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// A long string of blocks connected by joints.
    /// </summary>
    public class BridgeDemo : StandardDemo
    {


        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public BridgeDemo(DemosGame game)
            : base(game)
        {
            //Form a long chain of planks connected by revolute joints.
            //The revolute joints control the three linear degrees of freedom and two angular degrees of freedom.
            //The third allowed angular degree of freedom allows the bridge to flex like a rope bridge.
            Vector3 startPosition = new Vector3(0, 0, 0);
            var startPlatform = new Box(startPosition - new Vector3(0, 0, 3.2f), 8, .5f, 8);
            Space.Add(startPlatform);
            Vector3 offset = new Vector3(0, 0, 1.7f);
            Box previousLink = startPlatform;
            Vector3 position = new Vector3();
            for (int i = 1; i <= 200; i++)
            {
                position = startPosition + offset * i;
                Box link = new Box(position, 4.5f, .3f, 1.5f, 50);
                Space.Add(link);
                Space.Add(new RevoluteJoint(previousLink, link, position - offset * .5f, Vector3.Right));

                previousLink = link;
            }
            var endPlatform = new Box(position - new Vector3(0, 0, -4.8f), 8, .5f, 8);
            Space.Add(endPlatform);

            Space.Add(new RevoluteJoint(previousLink, endPlatform, position + offset * .5f, Vector3.Right));


            game.Camera.Position = startPosition + new Vector3(0, 1, offset.Z * 200 + 5);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Bridge"; }
        }
    }
}