using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using System.Collections.Generic;
using BEPUphysics.CollisionShapes;
using BEPUphysics.Constraints.SolverGroups;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Constraints.TwoEntity.Motors;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class WallDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public WallDemo(DemosGame game)
            : base(game)
        {
            int width = 10;
            int height = 10;
            float blockWidth = 2f;
            float blockHeight = 1f;
            float blockLength = 1f;

            for (int i = 0; i < width; i++)
            {
                for (int j = 0; j < height; j++)
                {
                    var toAdd =
                        new Box(
                            new Vector3(
                                i * blockWidth + .5f * blockWidth * (j % 2) - width * blockWidth * .5f,
                                blockHeight * .5f + j * (blockHeight),
                                0),
                            blockWidth, blockHeight, blockLength, 10);
                    Space.Add(toAdd);
                }
            }

            Box tankBody = new Box(new Vector3(0, 5, 10), 3.5f, 2, 6, 30);
            CompoundBody turret = new CompoundBody(
                new List<CompoundShapeEntry>() 
                {
                    new CompoundShapeEntry(new BoxShape(.5f, .5f, 4), tankBody.Position + new Vector3(0, 1.5f, 2), 5),
                    new CompoundShapeEntry(new BoxShape(1.5f, .7f, 2f), tankBody.Position + new Vector3(0, 1.5f, -1), 5)
                }, 10);
            RevoluteJoint axisJoint = new RevoluteJoint(tankBody, turret, tankBody.Position + new Vector3(0, 1, -1), Vector3.Up);
            axisJoint.Motor.IsActive = true;
            axisJoint.Motor.Settings.VelocityMotor.GoalVelocity = 1;

            axisJoint.Motor.Settings.Mode = MotorMode.Servomechanism;
            axisJoint.Motor.Settings.Servo.Goal = MathHelper.PiOver4;

            Space.Add(tankBody);
            Space.Add(turret);
            Space.Add(axisJoint);

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