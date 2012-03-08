using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Poseable humanoid with joint friction.
    /// </summary>
    public class ActionFigureDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public ActionFigureDemo(DemosGame game)
            : base(game)
        {
            //Make a simple, poseable action figure.  This isn't a full featured 'ragdoll' really;
            //ragdolls usually have specific joint limits and appropriate kinds of joints rather than all
            //ball socket joints.  This demo could be modified into a 'proper' ragdoll.
            Entity body = new Box(new Vector3(0, 5, 0), 1.5f, 2, 1, 10);
            Space.Add(body);

            Entity head = new Sphere(body.Position + new Vector3(0, 2, 0), .5f, 5);
            Space.Add(head);

            //Connect the head to the body.
            Space.Add(new BallSocketJoint(body, head, head.Position + new Vector3(0, -.9f, 0)));
            //Angular motors can be used to simulate friction when their goal velocity is 0.
            var angularMotor = new AngularMotor(body, head);
            angularMotor.Settings.MaximumForce = 150; //The maximum force of 'friction' in this joint.
            Space.Add(angularMotor);

            //Make the first arm.
            var upperLimb = new Box(body.Position + new Vector3(-1.6f, .8f, 0), 1, .5f, .5f, 5);
            Space.Add(upperLimb);

            var lowerLimb = new Box(upperLimb.Position + new Vector3(-1.4f, 0, 0), 1, .5f, .5f, 5);
            Space.Add(lowerLimb);

            //Connect the body to the upper arm.
            Space.Add(new BallSocketJoint(body, upperLimb, upperLimb.Position + new Vector3(.7f, 0, 0)));
            angularMotor = new AngularMotor(body, upperLimb);
            angularMotor.Settings.MaximumForce = 250;
            Space.Add(angularMotor);


            //Connect the upper arm to the lower arm.
            Space.Add(new BallSocketJoint(upperLimb, lowerLimb, upperLimb.Position + new Vector3(-.7f, 0, 0)));
            angularMotor = new AngularMotor(upperLimb, lowerLimb);
            angularMotor.Settings.MaximumForce = 150;
            Space.Add(angularMotor);

            //Make the second arm.
            upperLimb = new Box(body.Position + new Vector3(1.6f, .8f, 0), 1, .5f, .5f, 5);
            Space.Add(upperLimb);

            lowerLimb = new Box(upperLimb.Position + new Vector3(1.4f, 0, 0), 1, .5f, .5f, 5);
            Space.Add(lowerLimb);

            //Connect the body to the upper arm.
            Space.Add(new BallSocketJoint(body, upperLimb, upperLimb.Position + new Vector3(-.7f, 0, 0)));
            //Angular motors can be used to simulate friction when their goal velocity is 0.
            angularMotor = new AngularMotor(body, upperLimb);
            angularMotor.Settings.MaximumForce = 250; //The maximum force of 'friction' in this joint.
            Space.Add(angularMotor);


            //Connect the upper arm to the lower arm.
            Space.Add(new BallSocketJoint(upperLimb, lowerLimb, upperLimb.Position + new Vector3(.7f, 0, 0)));
            angularMotor = new AngularMotor(upperLimb, lowerLimb);
            angularMotor.Settings.MaximumForce = 150;
            Space.Add(angularMotor);

            //Make the first leg.
            upperLimb = new Box(body.Position + new Vector3(-.6f, -2.1f, 0), .5f, 1.3f, .5f, 8);
            Space.Add(upperLimb);

            lowerLimb = new Box(upperLimb.Position + new Vector3(0, -1.7f, 0), .5f, 1.3f, .5f, 8);
            Space.Add(lowerLimb);

            //Connect the body to the upper arm.
            Space.Add(new BallSocketJoint(body, upperLimb, upperLimb.Position + new Vector3(0, .9f, 0)));
            //Angular motors can be used to simulate friction when their goal velocity is 0.
            angularMotor = new AngularMotor(body, upperLimb);
            angularMotor.Settings.MaximumForce = 350; //The maximum force of 'friction' in this joint.
            Space.Add(angularMotor);


            //Connect the upper arm to the lower arm.
            Space.Add(new BallSocketJoint(upperLimb, lowerLimb, upperLimb.Position + new Vector3(0, -.9f, 0)));
            angularMotor = new AngularMotor(upperLimb, lowerLimb);
            angularMotor.Settings.MaximumForce = 250;
            Space.Add(angularMotor);

            //Make the second leg.
            upperLimb = new Box(body.Position + new Vector3(.6f, -2.1f, 0), .5f, 1.3f, .5f, 8);
            Space.Add(upperLimb);

            lowerLimb = new Box(upperLimb.Position + new Vector3(0, -1.7f, 0), .5f, 1.3f, .5f, 8);
            Space.Add(lowerLimb);

            //Connect the body to the upper arm.
            Space.Add(new BallSocketJoint(body, upperLimb, upperLimb.Position + new Vector3(0, .9f, 0)));
            //Angular motors can be used to simulate friction when their goal velocity is 0.
            angularMotor = new AngularMotor(body, upperLimb);
            angularMotor.Settings.MaximumForce = 350; //The maximum force of 'friction' in this joint.
            Space.Add(angularMotor);


            //Connect the upper arm to the lower arm.
            Space.Add(new BallSocketJoint(upperLimb, lowerLimb, upperLimb.Position + new Vector3(0, -.9f, 0)));
            angularMotor = new AngularMotor(upperLimb, lowerLimb);
            angularMotor.Settings.MaximumForce = 250;
            Space.Add(angularMotor);

            //Add some ground.
            Space.Add(new Box(new Vector3(0, -3.5f, 0), 40f, 1, 40f));

            game.Camera.Position = new Vector3(0, 5, 25);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Action Figure"; }
        }
    }
}