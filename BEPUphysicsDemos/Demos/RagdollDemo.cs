using System.Collections.Generic;
using BEPUphysics;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.Constraints;
using BEPUphysics.Constraints.SolverGroups;
using BEPUphysics.Constraints.TwoEntity.JointLimits;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;
using BEPUutilities.DataStructures;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Poseable humanoid with joint friction.
    /// </summary>
    public class RagdollDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public RagdollDemo(DemosGame game)
            : base(game)
        {
            int numRows = 8;
            int numColumns = 3;
            float xSpacing = 5;
            float zSpacing = 5;
            for (int i = 0; i < numRows; i++)
            {
                for (int j = 0; j < numColumns; j++)
                {
                    Ragdoll ragdoll = new Ragdoll();
                    //Transform and add every bone.
                    foreach (var bone in ragdoll.Bones)
                    {
                        bone.WorldTransform *= Matrix.CreateTranslation(new Vector3(
                                                         i * xSpacing - (numRows - 1) * xSpacing / 2,
                                                         i - 1.5f,
                                                         j * zSpacing - (numColumns - 1) * zSpacing / 2));
                        Space.Add(bone);
                    }

                    //Add every constraint.
                    foreach (var joint in ragdoll.Joints)
                    {
                        Space.Add(joint);
                    }
                }
            }

            //Add some ground.
            Space.Add(new Box(new Vector3(0, -3.5f, 0), 50f, 1, 50f));

            game.Camera.Position = new Vector3(0, 5, -35);
            game.Camera.ViewDirection = new Vector3(0, 0, 1);
        }

        public override void DrawUI()
        {
            Game.DataTextDrawer.Draw("Press J to toggle joint visualization.", new Microsoft.Xna.Framework.Vector2(40, 40));
            base.DrawUI();
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Ragdolls"; }
        }
    }

    /// <summary>
    /// Basic humanoid ragdoll.
    /// </summary>
    /// <remarks>
    /// <para>
    /// The following creates the physical parts of a ragdoll (bones and joints).
    /// This is only one possible setup for a ragdoll.
    /// </para>
    /// <para>
    /// This example uses AngularMotors to simulate damping between ragdoll bones.
    /// A cheaper, though technically less 'realistic' approach, would be to simply
    /// change the angular damping property of the involved entities.
    /// </para>
    /// <para>
    /// Because of the large number of constraints and the focus on functional realism,
    /// this type of ragdoll should only be used sparingly.  If you need a dozen or more
    /// ragdolls going at once, particularly on the Xbox360, it may be a good idea to
    /// cut down the realism in favor of performance.
    /// </para>
    /// <para>
    /// The following example does not include a possible optimization for multithreaded
    /// simulations.  Since the solver cannot update a entity from more than one constraint
    /// at a time (to avoid data corruption), many overlapping constraints will make the solver
    /// slow down a bit.  A simulation composed of just a bunch of these ragdolls will incur some 
    /// multithreading overhead due to the overlaps.
    /// </para>
    /// <para>
    /// By batching the constraints between any two bones in the ragdoll into a single "SolverGroup"
    /// object, the solver only sees a single constraint instead of 3-5 different ones.  This can improve
    /// multithreaded performance, but was excluded here for the sake of simplicity.
    /// </para>
    /// <para>
    /// Some of the constraints, like SwivelHingeJoint, are in fact SolverGroups themselves.
    /// They contain a variety of configurable sub-constraints.   
    /// </para>
    /// </remarks>
    public class Ragdoll
    {
        //List format of ragdoll components for easy iteration.
        List<Entity> bones = new List<Entity>();
        List<SolverUpdateable> joints = new List<SolverUpdateable>();

        /// <summary>
        /// Gets the bones of the ragdoll.
        /// </summary>
        public ReadOnlyList<Entity> Bones
        {
            get { return new ReadOnlyList<Entity>(bones); }
        }

        /// <summary>
        /// Gets the joints of the ragdoll.
        /// </summary>
        public ReadOnlyList<SolverUpdateable> Joints
        {
            get { return new ReadOnlyList<SolverUpdateable>(joints); }
        }


        public Ragdoll()
        {

            #region Ragdoll Entities
            //Create the ragdoll's bones.
            var pelvis = new Box(Vector3.Zero, .5f, .28f, .33f, 20);
            var torsoBottom = new Box(pelvis.Position + new Vector3(0, .3f, 0), .42f, .48f, .3f, 15);
            var torsoTop = new Box(torsoBottom.Position + new Vector3(0, .3f, 0), .5f, .38f, .32f, 20);

            var neck = new Box(torsoTop.Position + new Vector3(0, .2f, .04f), .19f, .24f, .2f, 5);
            var head = new Sphere(neck.Position + new Vector3(0, .22f, -.04f), .19f, 7);

            var leftUpperArm = new Box(torsoTop.Position + new Vector3(-.46f, .1f, 0), .52f, .19f, .19f, 6);
            var leftForearm = new Box(leftUpperArm.Position + new Vector3(-.5f, 0, 0), .52f, .18f, .18f, 5);
            var leftHand = new Box(leftForearm.Position + new Vector3(-.35f, 0, 0), .28f, .13f, .22f, 4);

            var rightUpperArm = new Box(torsoTop.Position + new Vector3(.46f, .1f, 0), .52f, .19f, .19f, 6);
            var rightForearm = new Box(rightUpperArm.Position + new Vector3(.5f, 0, 0), .52f, .18f, .18f, 5);
            var rightHand = new Box(rightForearm.Position + new Vector3(.35f, 0, 0), .28f, .13f, .22f, 4);

            var leftThigh = new Box(pelvis.Position + new Vector3(-.15f, -.4f, 0), .23f, .63f, .23f, 10);
            var leftShin = new Box(leftThigh.Position + new Vector3(0, -.6f, 0), .21f, .63f, .21f, 7);
            var leftFoot = new Box(leftShin.Position + new Vector3(0, -.35f, -.1f), .23f, .15f, .43f, 5);

            var rightThigh = new Box(pelvis.Position + new Vector3(.15f, -.4f, 0), .23f, .63f, .23f, 10);
            var rightShin = new Box(rightThigh.Position + new Vector3(0, -.6f, 0), .21f, .63f, .21f, 7);
            var rightFoot = new Box(rightShin.Position + new Vector3(0, -.35f, -.1f), .23f, .15f, .43f, 5);
            #endregion

            #region Bone List
            //Make a convenient list of all of the bones.
            bones.Add(pelvis);
            bones.Add(torsoBottom);
            bones.Add(torsoTop);
            bones.Add(neck);
            bones.Add(head);
            bones.Add(leftUpperArm);
            bones.Add(leftForearm);
            bones.Add(leftHand);
            bones.Add(rightUpperArm);
            bones.Add(rightForearm);
            bones.Add(rightHand);
            bones.Add(leftThigh);
            bones.Add(leftShin);
            bones.Add(leftFoot);
            bones.Add(rightThigh);
            bones.Add(rightShin);
            bones.Add(rightFoot);
            #endregion

            #region Collision Rules
            //Prevent adjacent limbs from colliding.
            CollisionRules.AddRule(pelvis, torsoBottom, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(torsoBottom, torsoTop, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(torsoTop, neck, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(neck, head, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(head, torsoTop, CollisionRule.NoBroadPhase);

            CollisionRules.AddRule(torsoTop, leftUpperArm, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(leftUpperArm, leftForearm, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(leftForearm, leftHand, CollisionRule.NoBroadPhase);

            CollisionRules.AddRule(torsoTop, rightUpperArm, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(rightUpperArm, rightForearm, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(rightForearm, rightHand, CollisionRule.NoBroadPhase);

            CollisionRules.AddRule(pelvis, leftThigh, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(leftThigh, leftShin, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(leftThigh, torsoBottom, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(leftShin, leftFoot, CollisionRule.NoBroadPhase);

            CollisionRules.AddRule(pelvis, rightThigh, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(rightThigh, rightShin, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(rightThigh, torsoBottom, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(rightShin, rightFoot, CollisionRule.NoBroadPhase);
            #endregion

            //Create the constraints between the bones.
            #region Pelvis up to Head Constraints
            var pelvisToTorsoBottomBallSocketJoint = new BallSocketJoint(pelvis, torsoBottom, pelvis.Position + new Vector3(0, .1f, 0));
            var pelvisToTorsoBottomTwistLimit = new TwistLimit(pelvis, torsoBottom, Vector3.Up, Vector3.Up, -MathHelper.Pi / 6, MathHelper.Pi / 6);
            var pelvisToTorsoBottomSwingLimit = new SwingLimit(pelvis, torsoBottom, Vector3.Up, Vector3.Up, MathHelper.Pi / 6);
            var pelvisToTorsoBottomMotor = new AngularMotor(pelvis, torsoBottom);
            pelvisToTorsoBottomMotor.Settings.VelocityMotor.Softness = .05f;

            var torsoBottomToTorsoTopBallSocketJoint = new BallSocketJoint(torsoBottom, torsoTop, torsoBottom.Position + new Vector3(0, .25f, 0));
            var torsoBottomToTorsoTopSwingLimit = new SwingLimit(torsoBottom, torsoTop, Vector3.Up, Vector3.Up, MathHelper.Pi / 6);
            var torsoBottomToTorsoTopTwistLimit = new TwistLimit(torsoBottom, torsoTop, Vector3.Up, Vector3.Up, -MathHelper.Pi / 6, MathHelper.Pi / 6);
            var torsoBottomToTorsoTopMotor = new AngularMotor(torsoBottom, torsoTop);
            torsoBottomToTorsoTopMotor.Settings.VelocityMotor.Softness = .05f;

            var torsoTopToNeckBallSocketJoint = new BallSocketJoint(torsoTop, neck, torsoTop.Position + new Vector3(0, .15f, .05f));
            var torsoTopToNeckSwingLimit = new SwingLimit(torsoTop, neck, Vector3.Up, Vector3.Up, MathHelper.Pi / 6);
            var torsoTopToNeckTwistLimit = new TwistLimit(torsoTop, neck, Vector3.Up, Vector3.Up, -MathHelper.Pi / 8, MathHelper.Pi / 8);
            var torsoTopToNeckMotor = new AngularMotor(torsoTop, neck);
            torsoTopToNeckMotor.Settings.VelocityMotor.Softness = .1f;

            var neckToHeadBallSocketJoint = new BallSocketJoint(neck, head, neck.Position + new Vector3(0, .1f, .05f));
            var neckToHeadTwistLimit = new TwistLimit(neck, head, Vector3.Up, Vector3.Up, -MathHelper.Pi / 8, MathHelper.Pi / 8);
            var neckToHeadSwingLimit = new SwingLimit(neck, head, Vector3.Up, Vector3.Up, MathHelper.Pi / 6);
            var neckToHeadMotor = new AngularMotor(neck, head);
            neckToHeadMotor.Settings.VelocityMotor.Softness = .1f;
            #endregion

            #region Left Arm
            var torsoTopToLeftArmBallSocketJoint = new BallSocketJoint(torsoTop, leftUpperArm, torsoTop.Position + new Vector3(-.3f, .1f, 0));
            var torsoTopToLeftArmEllipseLimit = new EllipseSwingLimit(torsoTop, leftUpperArm, Vector3.Left, MathHelper.Pi * .75f, MathHelper.PiOver2);
            var torsoTopToLeftArmTwistLimit = new TwistLimit(torsoTop, leftUpperArm, Vector3.Left, Vector3.Left, -MathHelper.PiOver2, MathHelper.PiOver2);
            var torsoTopToLeftArmMotor = new AngularMotor(torsoTop, leftUpperArm);
            torsoTopToLeftArmMotor.Settings.VelocityMotor.Softness = .2f;

            var leftUpperArmToLeftForearmSwivelHingeJoint = new SwivelHingeJoint(leftUpperArm, leftForearm, leftUpperArm.Position + new Vector3(-.28f, 0, 0), Vector3.Up);
            leftUpperArmToLeftForearmSwivelHingeJoint.HingeLimit.IsActive = true;
            leftUpperArmToLeftForearmSwivelHingeJoint.TwistLimit.IsActive = true;
            leftUpperArmToLeftForearmSwivelHingeJoint.TwistLimit.MinimumAngle = -MathHelper.Pi / 8;
            leftUpperArmToLeftForearmSwivelHingeJoint.TwistLimit.MaximumAngle = MathHelper.Pi / 8;
            leftUpperArmToLeftForearmSwivelHingeJoint.HingeLimit.MinimumAngle = -MathHelper.Pi * .8f;
            leftUpperArmToLeftForearmSwivelHingeJoint.HingeLimit.MaximumAngle = 0;
            //The SwivelHingeJoint has motors, but they are separately defined for twist/bending.
            //The AngularMotor covers all degrees of freedom.
            var leftUpperArmToLeftForearmMotor = new AngularMotor(leftUpperArm, leftForearm);
            leftUpperArmToLeftForearmMotor.Settings.VelocityMotor.Softness = .3f;

            var leftForearmToLeftHandBallSocketJoint = new BallSocketJoint(leftForearm, leftHand, leftForearm.Position + new Vector3(-.2f, 0, 0));
            var leftForearmToLeftHandEllipseSwingLimit = new EllipseSwingLimit(leftForearm, leftHand, Vector3.Left, MathHelper.PiOver2, MathHelper.Pi / 6);
            var leftForearmToLeftHandTwistLimit = new TwistLimit(leftForearm, leftHand, Vector3.Left, Vector3.Left, -MathHelper.Pi / 6, MathHelper.Pi / 6);
            var leftForearmToLeftHandMotor = new AngularMotor(leftForearm, leftHand);
            leftForearmToLeftHandMotor.Settings.VelocityMotor.Softness = .4f;
            #endregion

            #region Right Arm
            var torsoTopToRightArmBallSocketJoint = new BallSocketJoint(torsoTop, rightUpperArm, torsoTop.Position + new Vector3(.3f, .1f, 0));
            var torsoTopToRightArmEllipseLimit = new EllipseSwingLimit(torsoTop, rightUpperArm, Vector3.Right, MathHelper.Pi * .75f, MathHelper.PiOver2);
            var torsoTopToRightArmTwistLimit = new TwistLimit(torsoTop, rightUpperArm, Vector3.Right, Vector3.Right, -MathHelper.PiOver2, MathHelper.PiOver2);
            var torsoTopToRightArmMotor = new AngularMotor(torsoTop, rightUpperArm);
            torsoTopToRightArmMotor.Settings.VelocityMotor.Softness = .2f;

            var rightUpperArmToRightForearmSwivelHingeJoint = new SwivelHingeJoint(rightUpperArm, rightForearm, rightUpperArm.Position + new Vector3(.28f, 0, 0), Vector3.Up);
            rightUpperArmToRightForearmSwivelHingeJoint.HingeLimit.IsActive = true;
            rightUpperArmToRightForearmSwivelHingeJoint.TwistLimit.IsActive = true;
            rightUpperArmToRightForearmSwivelHingeJoint.TwistLimit.MinimumAngle = -MathHelper.Pi / 8;
            rightUpperArmToRightForearmSwivelHingeJoint.TwistLimit.MaximumAngle = MathHelper.Pi / 8;
            rightUpperArmToRightForearmSwivelHingeJoint.HingeLimit.MinimumAngle = 0;
            rightUpperArmToRightForearmSwivelHingeJoint.HingeLimit.MaximumAngle = MathHelper.Pi * .8f;
            //The SwivelHingeJoint has motors, but they are separately defined for twist/bending.
            //The AngularMotor covers all degrees of freedom.
            var rightUpperArmToRightForearmMotor = new AngularMotor(rightUpperArm, rightForearm);
            rightUpperArmToRightForearmMotor.Settings.VelocityMotor.Softness = .3f;

            var rightForearmToRightHandBallSocketJoint = new BallSocketJoint(rightForearm, rightHand, rightForearm.Position + new Vector3(.2f, 0, 0));
            var rightForearmToRightHandEllipseSwingLimit = new EllipseSwingLimit(rightForearm, rightHand, Vector3.Right, MathHelper.PiOver2, MathHelper.Pi / 6);
            var rightForearmToRightHandTwistLimit = new TwistLimit(rightForearm, rightHand, Vector3.Right, Vector3.Right, -MathHelper.Pi / 6, MathHelper.Pi / 6);
            var rightForearmToRightHandMotor = new AngularMotor(rightForearm, rightHand);
            rightForearmToRightHandMotor.Settings.VelocityMotor.Softness = .4f;
            #endregion

            #region Left Leg
            var pelvisToLeftThighBallSocketJoint = new BallSocketJoint(pelvis, leftThigh, pelvis.Position + new Vector3(-.15f, -.1f, 0));
            var pelvisToLeftThighEllipseSwingLimit = new EllipseSwingLimit(pelvis, leftThigh, Vector3.Normalize(new Vector3(-.2f, -1, -.6f)), MathHelper.Pi * .7f, MathHelper.PiOver4);
            pelvisToLeftThighEllipseSwingLimit.LocalTwistAxisB = Vector3.Down;
            var pelvisToLeftThighTwistLimit = new TwistLimit(pelvis, leftThigh, Vector3.Down, Vector3.Down, -MathHelper.Pi / 6, MathHelper.Pi / 6);
            var pelvisToLeftThighMotor = new AngularMotor(pelvis, leftThigh);
            pelvisToLeftThighMotor.Settings.VelocityMotor.Softness = .1f;

            var leftThighToLeftShinRevoluteJoint = new RevoluteJoint(leftThigh, leftShin, leftThigh.Position + new Vector3(0, -.3f, 0), Vector3.Right);
            leftThighToLeftShinRevoluteJoint.Limit.IsActive = true;
            leftThighToLeftShinRevoluteJoint.Limit.MinimumAngle = -MathHelper.Pi * .8f;
            leftThighToLeftShinRevoluteJoint.Limit.MaximumAngle = 0;
            leftThighToLeftShinRevoluteJoint.Motor.IsActive = true;
            leftThighToLeftShinRevoluteJoint.Motor.Settings.VelocityMotor.Softness = .2f;

            var leftShinToLeftFootBallSocketJoint = new BallSocketJoint(leftShin, leftFoot, leftShin.Position + new Vector3(0, -.3f, 0));
            var leftShinToLeftFootSwingLimit = new SwingLimit(leftShin, leftFoot, Vector3.Forward, Vector3.Forward, MathHelper.Pi / 8);
            var leftShinToLeftFootTwistLimit = new TwistLimit(leftShin, leftFoot, Vector3.Down, Vector3.Forward, -MathHelper.Pi / 8, MathHelper.Pi / 8);
            var leftShinToLeftFootMotor = new AngularMotor(leftShin, leftFoot);
            leftShinToLeftFootMotor.Settings.VelocityMotor.Softness = .2f;

            #endregion

            #region Right Leg
            var pelvisToRightThighBallSocketJoint = new BallSocketJoint(pelvis, rightThigh, pelvis.Position + new Vector3(.15f, -.1f, 0));
            var pelvisToRightThighEllipseSwingLimit = new EllipseSwingLimit(pelvis, rightThigh, Vector3.Normalize(new Vector3(.2f, -1, -.6f)), MathHelper.Pi * .7f, MathHelper.PiOver4);
            pelvisToRightThighEllipseSwingLimit.LocalTwistAxisB = Vector3.Down;
            var pelvisToRightThighTwistLimit = new TwistLimit(pelvis, rightThigh, Vector3.Down, Vector3.Down, -MathHelper.Pi / 6, MathHelper.Pi / 6);
            var pelvisToRightThighMotor = new AngularMotor(pelvis, rightThigh);
            pelvisToRightThighMotor.Settings.VelocityMotor.Softness = .1f;

            var rightThighToRightShinRevoluteJoint = new RevoluteJoint(rightThigh, rightShin, rightThigh.Position + new Vector3(0, -.3f, 0), Vector3.Right);
            rightThighToRightShinRevoluteJoint.Limit.IsActive = true;
            rightThighToRightShinRevoluteJoint.Limit.MinimumAngle = -MathHelper.Pi * .8f;
            rightThighToRightShinRevoluteJoint.Limit.MaximumAngle = 0;
            rightThighToRightShinRevoluteJoint.Motor.IsActive = true;
            rightThighToRightShinRevoluteJoint.Motor.Settings.VelocityMotor.Softness = .2f;

            var rightShinToRightFootBallSocketJoint = new BallSocketJoint(rightShin, rightFoot, rightShin.Position + new Vector3(0, -.3f, 0));
            var rightShinToRightFootSwingLimit = new SwingLimit(rightShin, rightFoot, Vector3.Forward, Vector3.Forward, MathHelper.Pi / 8);
            var rightShinToRightFootTwistLimit = new TwistLimit(rightShin, rightFoot, Vector3.Down, Vector3.Forward, -MathHelper.Pi / 8, MathHelper.Pi / 8);
            var rightShinToRightFootMotor = new AngularMotor(rightShin, rightFoot);
            rightShinToRightFootMotor.Settings.VelocityMotor.Softness = .2f;

            #endregion

            #region Joint List
            //Collect the joints.
            joints.Add(pelvisToTorsoBottomBallSocketJoint);
            joints.Add(pelvisToTorsoBottomTwistLimit);
            joints.Add(pelvisToTorsoBottomSwingLimit);
            joints.Add(pelvisToTorsoBottomMotor);

            joints.Add(torsoBottomToTorsoTopBallSocketJoint);
            joints.Add(torsoBottomToTorsoTopTwistLimit);
            joints.Add(torsoBottomToTorsoTopSwingLimit);
            joints.Add(torsoBottomToTorsoTopMotor);

            joints.Add(torsoTopToNeckBallSocketJoint);
            joints.Add(torsoTopToNeckTwistLimit);
            joints.Add(torsoTopToNeckSwingLimit);
            joints.Add(torsoTopToNeckMotor);

            joints.Add(neckToHeadBallSocketJoint);
            joints.Add(neckToHeadTwistLimit);
            joints.Add(neckToHeadSwingLimit);
            joints.Add(neckToHeadMotor);

            joints.Add(torsoTopToLeftArmBallSocketJoint);
            joints.Add(torsoTopToLeftArmEllipseLimit);
            joints.Add(torsoTopToLeftArmTwistLimit);
            joints.Add(torsoTopToLeftArmMotor);

            joints.Add(leftUpperArmToLeftForearmSwivelHingeJoint);
            joints.Add(leftUpperArmToLeftForearmMotor);

            joints.Add(leftForearmToLeftHandBallSocketJoint);
            joints.Add(leftForearmToLeftHandEllipseSwingLimit);
            joints.Add(leftForearmToLeftHandTwistLimit);
            joints.Add(leftForearmToLeftHandMotor);

            joints.Add(torsoTopToRightArmBallSocketJoint);
            joints.Add(torsoTopToRightArmEllipseLimit);
            joints.Add(torsoTopToRightArmTwistLimit);
            joints.Add(torsoTopToRightArmMotor);

            joints.Add(rightUpperArmToRightForearmSwivelHingeJoint);
            joints.Add(rightUpperArmToRightForearmMotor);

            joints.Add(rightForearmToRightHandBallSocketJoint);
            joints.Add(rightForearmToRightHandEllipseSwingLimit);
            joints.Add(rightForearmToRightHandTwistLimit);
            joints.Add(rightForearmToRightHandMotor);

            joints.Add(pelvisToLeftThighBallSocketJoint);
            joints.Add(pelvisToLeftThighEllipseSwingLimit);
            joints.Add(pelvisToLeftThighTwistLimit);
            joints.Add(pelvisToLeftThighMotor);

            joints.Add(leftThighToLeftShinRevoluteJoint);

            joints.Add(leftShinToLeftFootBallSocketJoint);
            joints.Add(leftShinToLeftFootSwingLimit);
            joints.Add(leftShinToLeftFootTwistLimit);
            joints.Add(leftShinToLeftFootMotor);

            joints.Add(pelvisToRightThighBallSocketJoint);
            joints.Add(pelvisToRightThighEllipseSwingLimit);
            joints.Add(pelvisToRightThighTwistLimit);
            joints.Add(pelvisToRightThighMotor);

            joints.Add(rightThighToRightShinRevoluteJoint);

            joints.Add(rightShinToRightFootBallSocketJoint);
            joints.Add(rightShinToRightFootSwingLimit);
            joints.Add(rightShinToRightFootTwistLimit);
            joints.Add(rightShinToRightFootMotor);
            #endregion


        }
    }
}