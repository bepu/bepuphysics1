using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;

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
            //Form a long chain of cubes, connected by ball socket joints.

            float baseHeight = 85;
            float linkSeparation = 2f;
            int numLinks = 200;
            float xOffset = -(numLinks + 1) * linkSeparation / 2;
            Box link;
            Box previousLink = null;
            Entity ground = new Box(new Vector3(0, -3, 0), 10, 10, 10);
            Space.Add(ground);
            BallSocketJoint ballSocketJointToAdd;
            for (int k = 0; k < numLinks; k++)
            {
                link = new Box(new Vector3(xOffset + linkSeparation * (k + 1), baseHeight, 0), 1.1f, 1.1f, 1.1f, 10);
                Space.Add(link);
                if (k == 0)
                {
                    ballSocketJointToAdd = new BallSocketJoint(link, ground, new Vector3(xOffset, baseHeight, 0));
                    ballSocketJointToAdd.SpringSettings.StiffnessConstant = 90000;
                    ballSocketJointToAdd.SpringSettings.DampingConstant = 8000f;
                    Space.Add(ballSocketJointToAdd);
                }
                else if (k == numLinks - 1)
                {
                    ballSocketJointToAdd = new BallSocketJoint(link, previousLink, (link.Position + previousLink.Position) / 2);
                    ballSocketJointToAdd.SpringSettings.StiffnessConstant = 90000;
                    ballSocketJointToAdd.SpringSettings.DampingConstant = 8000f;
                    Space.Add(ballSocketJointToAdd);
                    ballSocketJointToAdd = new BallSocketJoint(link, ground, new Vector3(xOffset + (numLinks + 1) * linkSeparation, baseHeight, 0));
                    ballSocketJointToAdd.SpringSettings.StiffnessConstant = 90000;
                    ballSocketJointToAdd.SpringSettings.DampingConstant = 8000f;
                    Space.Add(ballSocketJointToAdd);
                }
                else
                {
                    ballSocketJointToAdd = new BallSocketJoint(link, previousLink, (link.Position + previousLink.Position) / 2);
                    ballSocketJointToAdd.SpringSettings.StiffnessConstant = 90000;
                    ballSocketJointToAdd.SpringSettings.DampingConstant = 8000f;
                    Space.Add(ballSocketJointToAdd);
                }
                previousLink = link;
            }


            game.Camera.Position = new Vector3(-180, 70, 300);
            game.Camera.Yaw = MathHelper.ToRadians(-24);
            game.Camera.Pitch = MathHelper.ToRadians(-5);
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