using BEPUphysics.Constraints.SolverGroups;
using BEPUphysics.Constraints.TwoEntity.JointLimits;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using System.Collections.Generic;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// A one-armed guy flops on the ground.  Involves a detailed ragdoll-like limb with appropriate limits.
    /// </summary>
    public class UnfortunateGuyDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public UnfortunateGuyDemo(DemosGame game)
            : base(game)
        {
            Entity ground = new Box(Vector3.Zero, 10, 1, 10);
            Space.Add(ground);

            //Rather than add basically redundant code for every limb, this demo
            //focuses on a single arm, showing some extra details and limits.

            //Make the torso.
            var bodies = new List<CompoundShapeEntry>()
            {
                new CompoundShapeEntry(new BoxShape(2, 1.5f, 1), new Vector3(-1, 3, 0), 50),
                new CompoundShapeEntry(new SphereShape(.45f), new Vector3(.4f, 3, 0), 1),
                new CompoundShapeEntry(new SphereShape(.25f), new Vector3(-1.9f, 3.5f, 0), 1),
                new CompoundShapeEntry(new SphereShape(.25f), new Vector3(-1.9f, 2.5f, 0), 1),
                new CompoundShapeEntry(new SphereShape(.25f), new Vector3(-.3f, 2.3f, 0), 1)
            };

            var torso = new CompoundBody(bodies, 54);
            Space.Add(torso);

            //Make the upper arm.
            Entity upperArm = new Box(torso.Position + new Vector3(1, 1.4f, 0), .4f, 1.2f, .4f, 8);
            Space.Add(upperArm);


            //A ball socket joint will handle the linear degrees of freedom between the two entities.
            var ballSocketJoint = new BallSocketJoint(torso, upperArm, torso.Position + new Vector3(1, .7f, 0));
            Space.Add(ballSocketJoint);

            //Shoulders don't have a simple limit.  The EllipseSwingLimit allows angles within an ellipse, which is closer to how some joints function
            //than just flat planes (like two RevoluteLimits) or a single angle (like SwingLimits).
            var swingLimit = new EllipseSwingLimit(torso, upperArm, Vector3.Up, MathHelper.PiOver2, MathHelper.PiOver4 * 3);
            Space.Add(swingLimit);

            //Upper arms can't spin around forever.
            var twistLimit = new TwistLimit(torso, upperArm, Vector3.Up, Vector3.Up, -MathHelper.PiOver4 / 2, MathHelper.PiOver4);
            twistLimit.SpringSettings.StiffnessConstant = 100;
            twistLimit.SpringSettings.DampingConstant = 100;
            Space.Add(twistLimit);


            //Make the lower arm.
            Entity lowerArm = new Box(upperArm.Position + new Vector3(0, 1.4f, 0), .35f, 1.3f, .35f, 8);
            Space.Add(lowerArm);


            var elbow = new SwivelHingeJoint(upperArm, lowerArm, upperArm.Position + new Vector3(0, .6f, 0), Vector3.Forward);
            //Forearm can twist a little.
            elbow.TwistLimit.IsActive = true;
            elbow.TwistLimit.MinimumAngle = -MathHelper.PiOver4 / 2;
            elbow.TwistLimit.MaximumAngle = MathHelper.PiOver4 / 2;
            elbow.TwistLimit.SpringSettings.DampingConstant = 100;
            elbow.TwistLimit.SpringSettings.StiffnessConstant = 100;


            //The elbow is like a hinge, but it can't hyperflex.
            elbow.HingeLimit.IsActive = true;
            elbow.HingeLimit.MinimumAngle = 0;
            elbow.HingeLimit.MaximumAngle = MathHelper.Pi * .7f;
            Space.Add(elbow);

            Entity hand = new Box(lowerArm.Position + new Vector3(0, .9f, 0), .4f, .55f, .25f, 3);
            Space.Add(hand);

            ballSocketJoint = new BallSocketJoint(lowerArm, hand, lowerArm.Position + new Vector3(0, .7f, 0));
            Space.Add(ballSocketJoint);

            //Wrists can use an ellipse limit too.
            swingLimit = new EllipseSwingLimit(lowerArm, hand, Vector3.Up, MathHelper.PiOver4, MathHelper.PiOver2);
            Space.Add(swingLimit);

            //Allow a little extra twist beyond the forearm.
            twistLimit = new TwistLimit(lowerArm, hand, Vector3.Up, Vector3.Up, -MathHelper.PiOver4 / 2, MathHelper.PiOver4 / 2);
            twistLimit.SpringSettings.StiffnessConstant = 100;
            twistLimit.SpringSettings.DampingConstant = 100;
            Space.Add(twistLimit);

            //The hand is pretty floppy without some damping.
            var angularMotor = new AngularMotor(lowerArm, hand);
            angularMotor.Settings.VelocityMotor.Softness = .5f;
            Space.Add(angularMotor);

            //Make sure the parts of the arm don't collide.
            CollisionRules.AddRule(torso, upperArm, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(lowerArm, upperArm, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(lowerArm, hand, CollisionRule.NoBroadPhase);

            game.Camera.Position = new Vector3(0, 4, 20);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Unfortunate One-armed Guy"; }
        }
    }
}