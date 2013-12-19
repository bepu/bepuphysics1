#if WINDOWS
using System;
using System.Collections.Generic;
using BEPUik;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.Constraints.TwoEntity.JointLimits;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.EntityStateManagement;
using BEPUphysicsDrawer.Models;
using BEPUutilities;
using System.Diagnostics;
using ConversionHelper;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using MathHelper = BEPUutilities.MathHelper;
using Matrix = BEPUutilities.Matrix;
using Quaternion = BEPUutilities.Quaternion;
using Ray = BEPUutilities.Ray;
using Vector2 = BEPUutilities.Vector2;
using Vector3 = BEPUutilities.Vector3;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    public class StateControlGroup
    {
        private struct ControlEntry
        {
            internal Vector3 GrabOffset;
            internal StateControl Control;
        }

        private float distanceToTarget;
        private Camera camera;
        public List<Control> Controls { get; set; }

        public bool IsActive
        {
            get { return stateControls.Count > 0; }
        }

        public StateControlGroup(Camera camera, List<Control> controls)
        {
            this.camera = camera;
            this.Controls = controls;
        }

        private List<ControlEntry> stateControls = new List<ControlEntry>();
        private Stack<StateControl> stateControlsPool = new Stack<StateControl>();

        public void ClearStateControls()
        {
            foreach (var entry in stateControls)
            {
                GiveBack(entry.Control);
            }
            stateControls.Clear();
        }
        StateControl GetControl(Bone bone)
        {
            var control = stateControlsPool.Count > 0 ? stateControlsPool.Pop() : new StateControl();
            control.TargetBone = bone;
            Controls.Add(control);
            return control;
        }
        void GiveBack(StateControl control)
        {
            Controls.Remove(control);
            control.TargetBone = null;
            stateControlsPool.Push(control);
        }

        public void TryToAddBone(Bone bone, Vector3 grabbedLocation)
        {
            bool alreadyConstrainingBone = false;
            for (int i = 0; i < stateControls.Count; i++)
            {
                var entry = stateControls[i];
                entry.GrabOffset = grabbedLocation - entry.Control.TargetBone.Position;
                stateControls[i] = entry;
                if (entry.Control.TargetBone == bone)
                {
                    alreadyConstrainingBone = true;
                }
            }
            if (!alreadyConstrainingBone)
            {
                //Add a new control to the group for this bone.
                var entry = new ControlEntry { Control = GetControl(bone), GrabOffset = grabbedLocation - bone.Position };
                stateControls.Add(entry);
            }
            distanceToTarget = Vector3.Dot(camera.WorldMatrix.Forward, grabbedLocation - camera.Position);
        }

        public void UpdateGoals()
        {
            var newGoal = camera.WorldMatrix.Forward * distanceToTarget + camera.Position;
            foreach (var entry in stateControls)
            {
                entry.Control.LinearMotor.TargetPosition = newGoal - entry.GrabOffset;
            }
        }
    }

    /// <summary>
    /// Test environment for the inverse kinematics solver.
    /// </summary>
    public class InverseKinematicsTestDemo : Demo
    {
        private bool usingIK;

        private IKSolver solver = new IKSolver();

        private FreeCameraControlScheme cameraControl;

        private DragControl dragControl = new DragControl();
        private float distanceToGrabbedBone;
        private StateControlGroup stateControlGroup;



        private double elapsedTime;

        private struct BoneRelationship
        {
            public Bone Bone;
            public DisplayModel DisplayBone;
            public Entity Entity;
            public Quaternion LocalRotationBoneToEntity;

            public BoneRelationship(Bone bone, Entity entity)
            {
                Bone = bone;
                Entity = entity;
                DisplayBone = null;
                LocalRotationBoneToEntity = Quaternion.Concatenate(entity.Orientation, Quaternion.Conjugate(bone.Orientation));
            }
        }

        private InstancedModelDrawer drawer;
        private Texture2D whitePixel;

        private List<BoneRelationship> bones = new List<BoneRelationship>();
        private List<Control> controls = new List<Control>();
        private List<IKJoint> joints = new List<IKJoint>();

        void BuildChain(Vector3 position)
        {
            //Set up a bone chain.
            int linkCount = 100;
            var previousBoneEntity = new Cylinder(position, 1, .2f, 10);
            var previousBone = new Bone(previousBoneEntity.Position, previousBoneEntity.Orientation, previousBoneEntity.Radius, previousBoneEntity.Height);
            bones.Add(new BoneRelationship(previousBone, previousBoneEntity));
            Space.Add(previousBoneEntity);

            for (int i = 1; i < linkCount; i++)
            {
                var boneEntity = new Cylinder(previousBone.Position + new Vector3(0, 1, 0), 1, .2f, 10);
                var bone = new Bone(boneEntity.Position, boneEntity.Orientation, boneEntity.Radius, boneEntity.Height);
                bones.Add(new BoneRelationship(bone, boneEntity));
                Space.Add(boneEntity);

                //Make a relationship between the two bones and entities.
                CollisionRules.AddRule(previousBoneEntity, boneEntity, CollisionRule.NoBroadPhase);
                Vector3 anchor = (previousBoneEntity.Position + boneEntity.Position) / 2;
                var dynamicsBallSocketJoint = new BallSocketJoint(previousBoneEntity, boneEntity, anchor);
                var dynamicsAngularFriction = new AngularMotor(previousBoneEntity, boneEntity);
                dynamicsAngularFriction.Settings.Mode = MotorMode.VelocityMotor;
                dynamicsAngularFriction.Settings.MaximumForce = 200;
                Space.Add(dynamicsBallSocketJoint);
                Space.Add(dynamicsAngularFriction);
                var ikBallSocketJoint = new IKBallSocketJoint(previousBone, bone, anchor); //(the joint is auto-added to the bones; no solver-add is needed)
                joints.Add(ikBallSocketJoint);

                previousBone = bone;
                previousBoneEntity = boneEntity;
            }
        }


        void BuildStick(Vector3 position)
        {
            //Set up a bone chain.
            float fullLength = 20;
            int linkCount = 20;
            float linkLength = fullLength / linkCount;
            float linkRadius = linkLength * 0.2f;
            var previousBoneEntity = new Cylinder(position, linkLength, linkRadius, 100);
            var previousBone = new Bone(previousBoneEntity.Position, previousBoneEntity.Orientation, previousBoneEntity.Radius, previousBoneEntity.Height);
            bones.Add(new BoneRelationship(previousBone, previousBoneEntity));
            Space.Add(previousBoneEntity);

            for (int i = 1; i < linkCount; i++)
            {
                var boneEntity = new Cylinder(previousBone.Position + new Vector3(0, linkLength, 0), linkLength, linkRadius, 100);
                var bone = new Bone(boneEntity.Position, boneEntity.Orientation, boneEntity.Radius, boneEntity.Height);
                bones.Add(new BoneRelationship(bone, boneEntity));
                Space.Add(boneEntity);

                //Make a relationship between the two bones and entities.
                CollisionRules.AddRule(previousBoneEntity, boneEntity, CollisionRule.NoBroadPhase);
                Vector3 anchor = (previousBoneEntity.Position + boneEntity.Position) / 2;
                var dynamicsBallSocketJoint = new BallSocketJoint(previousBoneEntity, boneEntity, anchor);
                var dynamicsAngularFriction = new NoRotationJoint(previousBoneEntity, boneEntity);
                Space.Add(dynamicsBallSocketJoint);
                Space.Add(dynamicsAngularFriction);
                var ikBallSocketJoint = new IKBallSocketJoint(previousBone, bone, anchor); //(the joint is auto-added to the bones; no solver-add is needed)
                var ikAngularJoint = new IKAngularJoint(previousBone, bone);
                joints.Add(ikBallSocketJoint);
                joints.Add(ikAngularJoint);

                previousBone = bone;
                previousBoneEntity = boneEntity;
            }
        }
        
        void BuildActionFigure(Vector3 position)
        {
            //Make a simple, poseable action figure, like the ActionFigureDemo.
            Entity body = new Box(position, 1.5f, 2, 1, 10);
            Space.Add(body);

            Entity head = new Sphere(body.Position + new Vector3(0, 2, 0), .5f, 5);
            Space.Add(head);

            //Connect the head to the body.
            var headBodyBallSocketAnchor = head.Position + new Vector3(0, -.75f, 0);
            Space.Add(new BallSocketJoint(body, head, headBodyBallSocketAnchor));
            //Angular motors can be used to simulate friction when their goal velocity is 0.
            var angularMotor = new AngularMotor(body, head);
            angularMotor.Settings.MaximumForce = 150; //The maximum force of 'friction' in this joint.
            Space.Add(angularMotor);

            //Make the first arm.
            var upperLeftArm = new Box(body.Position + new Vector3(-1.6f, .8f, 0), 1, .5f, .5f, 5);
            Space.Add(upperLeftArm);

            var lowerLeftArm = new Box(upperLeftArm.Position + new Vector3(-1.4f, 0, 0), 1, .5f, .5f, 5);
            Space.Add(lowerLeftArm);

            var leftHand = new Box(lowerLeftArm.Position + new Vector3(-.8f, 0, 0), 0.5f, 0.3f, 0.5f, 4);
            Space.Add(leftHand);

            //Connect the body to the upper arm.
            var bodyUpperLeftArmBallSocketAnchor = upperLeftArm.Position + new Vector3(.7f, 0, 0);
            Space.Add(new BallSocketJoint(body, upperLeftArm, bodyUpperLeftArmBallSocketAnchor));
            angularMotor = new AngularMotor(body, upperLeftArm);
            angularMotor.Settings.MaximumForce = 250;
            Space.Add(angularMotor);

            //Connect the upper arm to the lower arm.
            var upperLeftArmLowerLeftArmBallSocketAnchor = upperLeftArm.Position + new Vector3(-.7f, 0, 0);
            Space.Add(new BallSocketJoint(upperLeftArm, lowerLeftArm, upperLeftArmLowerLeftArmBallSocketAnchor));
            angularMotor = new AngularMotor(upperLeftArm, lowerLeftArm);
            angularMotor.Settings.MaximumForce = 150;
            Space.Add(angularMotor);

            //Connect the lower arm to the hand.
            var lowerLeftArmLeftHandBallSocketAnchor = lowerLeftArm.Position + new Vector3(-.5f, 0, 0);
            Space.Add(new BallSocketJoint(lowerLeftArm, leftHand, lowerLeftArmLeftHandBallSocketAnchor));
            angularMotor = new AngularMotor(lowerLeftArm, leftHand);
            angularMotor.Settings.MaximumForce = 150;
            Space.Add(angularMotor);

            //Make the second arm.
            var upperRightArm = new Box(body.Position + new Vector3(1.6f, .8f, 0), 1, .5f, .5f, 5);
            Space.Add(upperRightArm);

            var lowerRightArm = new Box(upperRightArm.Position + new Vector3(1.4f, 0, 0), 1, .5f, .5f, 5);
            Space.Add(lowerRightArm);

            var rightHand = new Box(lowerRightArm.Position + new Vector3(.8f, 0, 0), 0.5f, 0.3f, 0.5f, 4);
            Space.Add(rightHand);

            //Connect the body to the upper arm.
            var bodyUpperRightArmBallSocketAnchor = upperRightArm.Position + new Vector3(-.7f, 0, 0);
            Space.Add(new BallSocketJoint(body, upperRightArm, bodyUpperRightArmBallSocketAnchor));
            //Angular motors can be used to simulate friction when their goal velocity is 0.
            angularMotor = new AngularMotor(body, upperRightArm);
            angularMotor.Settings.MaximumForce = 250; //The maximum force of 'friction' in this joint.
            Space.Add(angularMotor);

            //Connect the upper arm to the lower arm.
            var upperRightArmLowerRightArmBallSocketAnchor = upperRightArm.Position + new Vector3(.7f, 0, 0);
            Space.Add(new BallSocketJoint(upperRightArm, lowerRightArm, upperRightArmLowerRightArmBallSocketAnchor));
            angularMotor = new AngularMotor(upperRightArm, lowerRightArm);
            angularMotor.Settings.MaximumForce = 150;
            Space.Add(angularMotor);

            //Connect the lower arm to the hand.
            var lowerRightArmRightHandBallSocketAnchor = lowerRightArm.Position + new Vector3(.5f, 0, 0);
            Space.Add(new BallSocketJoint(lowerRightArm, rightHand, lowerRightArmRightHandBallSocketAnchor));
            angularMotor = new AngularMotor(lowerRightArm, rightHand);
            angularMotor.Settings.MaximumForce = 150;
            Space.Add(angularMotor);

            //Make the first leg.
            var upperLeftLeg = new Box(body.Position + new Vector3(-.6f, -2.1f, 0), .5f, 1.3f, .5f, 8);
            Space.Add(upperLeftLeg);

            var lowerLeftLeg = new Box(upperLeftLeg.Position + new Vector3(0, -1.7f, 0), .5f, 1.3f, .5f, 8);
            Space.Add(lowerLeftLeg);

            var leftFoot = new Box(lowerLeftLeg.Position + new Vector3(0, -.25f - 0.65f, 0.25f), .5f, .4f, 1, 8);
            Space.Add(leftFoot);

            //Connect the body to the upper leg.
            var bodyUpperLeftLegBallSocketAnchor = upperLeftLeg.Position + new Vector3(0, .9f, 0);
            Space.Add(new BallSocketJoint(body, upperLeftLeg, bodyUpperLeftLegBallSocketAnchor));
            //Angular motors can be used to simulate friction when their goal velocity is 0.
            angularMotor = new AngularMotor(body, upperLeftLeg);
            angularMotor.Settings.MaximumForce = 350; //The maximum force of 'friction' in this joint.
            Space.Add(angularMotor);

            //Connect the upper leg to the lower leg.
            var upperLeftLegLowerLeftLegBallSocketAnchor = upperLeftLeg.Position + new Vector3(0, -.9f, 0);
            Space.Add(new BallSocketJoint(upperLeftLeg, lowerLeftLeg, upperLeftLegLowerLeftLegBallSocketAnchor));
            angularMotor = new AngularMotor(upperLeftLeg, lowerLeftLeg);
            angularMotor.Settings.MaximumForce = 250;
            Space.Add(angularMotor);

            //Connect the lower leg to the foot.
            var lowerLeftLegLeftFootBallSocketAnchor = lowerLeftLeg.Position + new Vector3(0, -.65f, 0);
            Space.Add(new BallSocketJoint(lowerLeftLeg, leftFoot, lowerLeftLegLeftFootBallSocketAnchor));
            angularMotor = new AngularMotor(lowerLeftLeg, leftFoot);
            angularMotor.Settings.MaximumForce = 250;
            Space.Add(angularMotor);

            //Make the second leg.
            var upperRightLeg = new Box(body.Position + new Vector3(.6f, -2.1f, 0), .5f, 1.3f, .5f, 8);
            Space.Add(upperRightLeg);

            var lowerRightLeg = new Box(upperRightLeg.Position + new Vector3(0, -1.7f, 0), .5f, 1.3f, .5f, 8);
            Space.Add(lowerRightLeg);

            var rightFoot = new Box(lowerRightLeg.Position + new Vector3(0, -.25f - 0.65f, 0.25f), .5f, .4f, 1, 8);
            Space.Add(rightFoot);

            //Connect the body to the upper leg.
            var bodyUpperRightLegBallSocketAnchor = upperRightLeg.Position + new Vector3(0, .9f, 0);
            Space.Add(new BallSocketJoint(body, upperRightLeg, bodyUpperRightLegBallSocketAnchor));
            //Angular motors can be used to simulate friction when their goal velocity is 0.
            angularMotor = new AngularMotor(body, upperRightLeg);
            angularMotor.Settings.MaximumForce = 350; //The maximum force of 'friction' in this joint.
            Space.Add(angularMotor);

            //Connect the upper leg to the lower leg.
            var upperRightLegLowerRightLegBallSocketAnchor = upperRightLeg.Position + new Vector3(0, -.9f, 0);
            Space.Add(new BallSocketJoint(upperRightLeg, lowerRightLeg, upperRightLegLowerRightLegBallSocketAnchor));
            angularMotor = new AngularMotor(upperRightLeg, lowerRightLeg);
            angularMotor.Settings.MaximumForce = 250;
            Space.Add(angularMotor);

            //Connect the lower leg to the foot.
            var lowerRightLegRightFootBallSocketAnchor = lowerRightLeg.Position + new Vector3(0, -.65f, 0);
            Space.Add(new BallSocketJoint(lowerRightLeg, rightFoot, lowerRightLegRightFootBallSocketAnchor));
            angularMotor = new AngularMotor(lowerRightLeg, rightFoot);
            angularMotor.Settings.MaximumForce = 250;
            Space.Add(angularMotor);

            //Set up collision rules.
            CollisionRules.AddRule(head, body, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(body, upperLeftArm, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(upperLeftArm, lowerLeftArm, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(lowerLeftArm, leftHand, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(body, upperRightArm, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(upperRightArm, lowerRightArm, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(lowerRightArm, rightHand, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(body, upperLeftLeg, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(upperLeftLeg, lowerLeftLeg, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(lowerLeftLeg, leftFoot, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(body, upperRightLeg, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(upperRightLeg, lowerRightLeg, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(lowerRightLeg, rightFoot, CollisionRule.NoBroadPhase);



            //IK version!
            Bone bodyBone = new Bone(body.Position, Quaternion.Identity, .75f, 2);
            Bone headBone = new Bone(head.Position, Quaternion.Identity, .4f, .8f);
            Bone upperLeftArmBone = new Bone(upperLeftArm.Position, Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), MathHelper.PiOver2), .25f, 1);
            Bone lowerLeftArmBone = new Bone(lowerLeftArm.Position, Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), MathHelper.PiOver2), .25f, 1);
            Bone upperRightArmBone = new Bone(upperRightArm.Position, Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), MathHelper.PiOver2), .25f, 1);
            Bone lowerRightArmBone = new Bone(lowerRightArm.Position, Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), MathHelper.PiOver2), .25f, 1);
            Bone leftHandBone = new Bone(leftHand.Position, Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), MathHelper.PiOver2), .2f, .5f);
            Bone rightHandBone = new Bone(rightHand.Position, Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), MathHelper.PiOver2), .2f, .5f);
            Bone upperLeftLegBone = new Bone(upperLeftLeg.Position, Quaternion.Identity, .25f, 1.3f);
            Bone lowerLeftLegBone = new Bone(lowerLeftLeg.Position, Quaternion.Identity, .25f, 1.3f);
            Bone leftFootBone = new Bone(leftFoot.Position, Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathHelper.PiOver2), .25f, 1);
            Bone upperRightLegBone = new Bone(upperRightLeg.Position, Quaternion.Identity, .25f, 1.3f);
            Bone lowerRightLegBone = new Bone(lowerRightLeg.Position, Quaternion.Identity, .25f, 1.3f);
            Bone rightFootBone = new Bone(rightFoot.Position, Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathHelper.PiOver2), .25f, 1);

            bones.Add(new BoneRelationship(bodyBone, body));
            bones.Add(new BoneRelationship(headBone, head));
            bones.Add(new BoneRelationship(upperLeftArmBone, upperLeftArm));
            bones.Add(new BoneRelationship(lowerLeftArmBone, lowerLeftArm));
            bones.Add(new BoneRelationship(upperRightArmBone, upperRightArm));
            bones.Add(new BoneRelationship(lowerRightArmBone, lowerRightArm));
            bones.Add(new BoneRelationship(leftHandBone, leftHand));
            bones.Add(new BoneRelationship(rightHandBone, rightHand));
            bones.Add(new BoneRelationship(upperLeftLegBone, upperLeftLeg));
            bones.Add(new BoneRelationship(lowerLeftLegBone, lowerLeftLeg));
            bones.Add(new BoneRelationship(leftFootBone, leftFoot));
            bones.Add(new BoneRelationship(upperRightLegBone, upperRightLeg));
            bones.Add(new BoneRelationship(lowerRightLegBone, lowerRightLeg));
            bones.Add(new BoneRelationship(rightFootBone, rightFoot));

            //[We don't care about the return values here. A bit weird, but the constructor puts the reference where it needs to go.]
            joints.Add(new IKBallSocketJoint(bodyBone, headBone, headBodyBallSocketAnchor));
            joints.Add(new IKSwingLimit(bodyBone, headBone, Vector3.Up, Vector3.Up, MathHelper.PiOver2));
            joints.Add(new IKTwistLimit(bodyBone, headBone, Vector3.Up, Vector3.Up, MathHelper.PiOver2));

            //Left arm
            joints.Add(new IKBallSocketJoint(bodyBone, upperLeftArmBone, bodyUpperLeftArmBallSocketAnchor));
            joints.Add(new IKSwingLimit(bodyBone, upperLeftArmBone, Vector3.Normalize(new Vector3(-1, 0, .3f)), Vector3.Left, MathHelper.Pi * .65f));
            joints.Add(new IKTwistLimit(bodyBone, upperLeftArmBone, Vector3.Left, Vector3.Left, MathHelper.PiOver2) { Rigidity = 0.08f });
            joints.Add(new IKBallSocketJoint(upperLeftArmBone, lowerLeftArmBone, upperLeftArmLowerLeftArmBallSocketAnchor));
            joints.Add(new IKSwivelHingeJoint(upperLeftArmBone, lowerLeftArmBone, Vector3.Up, Vector3.Left));
            joints.Add(new IKSwingLimit(upperLeftArmBone, lowerLeftArmBone, Vector3.Normalize(new Vector3(-0.23f, 0, .97f)), Vector3.Left, MathHelper.Pi * 0.435f));
            joints.Add(new IKTwistLimit(upperLeftArmBone, lowerLeftArmBone, Vector3.Left, Vector3.Left, MathHelper.PiOver4) { Rigidity = 0.08f });
            joints.Add(new IKBallSocketJoint(lowerLeftArmBone, leftHandBone, lowerLeftArmLeftHandBallSocketAnchor));
            joints.Add(new IKSwingLimit(lowerLeftArmBone, leftHandBone, Vector3.Left, Vector3.Left, MathHelper.PiOver2));
            joints.Add(new IKTwistLimit(lowerLeftArmBone, leftHandBone, Vector3.Left, Vector3.Left, MathHelper.PiOver4) { Rigidity = 0.08f });

            //Right arm
            joints.Add(new IKBallSocketJoint(bodyBone, upperRightArmBone, bodyUpperRightArmBallSocketAnchor));
            joints.Add(new IKSwingLimit(bodyBone, upperRightArmBone, Vector3.Normalize(new Vector3(1, 0, .3f)), Vector3.Right, MathHelper.Pi * .65f));
            joints.Add(new IKTwistLimit(bodyBone, upperRightArmBone, Vector3.Right, Vector3.Right, MathHelper.PiOver2) { Rigidity = 0.08f });
            joints.Add(new IKBallSocketJoint(upperRightArmBone, lowerRightArmBone, upperRightArmLowerRightArmBallSocketAnchor));
            joints.Add(new IKSwivelHingeJoint(upperRightArmBone, lowerRightArmBone, Vector3.Up, Vector3.Right));
            joints.Add(new IKSwingLimit(upperRightArmBone, lowerRightArmBone, Vector3.Normalize(new Vector3(0.23f, 0, .97f)), Vector3.Right, MathHelper.Pi * 0.435f));
            joints.Add(new IKTwistLimit(upperRightArmBone, lowerRightArmBone, Vector3.Right, Vector3.Right, MathHelper.PiOver4) { Rigidity = 0.08f });
            joints.Add(new IKBallSocketJoint(lowerRightArmBone, rightHandBone, lowerRightArmRightHandBallSocketAnchor));
            joints.Add(new IKSwingLimit(lowerRightArmBone, rightHandBone, Vector3.Right, Vector3.Right, MathHelper.PiOver2));
            joints.Add(new IKTwistLimit(lowerRightArmBone, rightHandBone, Vector3.Right, Vector3.Right, MathHelper.PiOver4) { Rigidity = 0.08f });

            //Left Leg
            joints.Add(new IKBallSocketJoint(bodyBone, upperLeftLegBone, bodyUpperLeftLegBallSocketAnchor));
            joints.Add(new IKSwingLimit(bodyBone, upperLeftLegBone, Vector3.Normalize(new Vector3(-.3f, -1, .6f)), Vector3.Down, MathHelper.Pi * 0.6f));
            joints.Add(new IKTwistLimit(bodyBone, upperLeftLegBone, Vector3.Up, Vector3.Up, MathHelper.PiOver4) { MeasurementAxisA = Vector3.Normalize(new Vector3(-1, 0, 1)), Rigidity = 0.08f });
            joints.Add(new IKBallSocketJoint(upperLeftLegBone, lowerLeftLegBone, upperLeftLegLowerLeftLegBallSocketAnchor));
            joints.Add(new IKSwivelHingeJoint(upperLeftLegBone, lowerLeftLegBone, Vector3.Left, Vector3.Down));
            joints.Add(new IKTwistLimit(upperLeftLegBone, lowerLeftLegBone, Vector3.Up, Vector3.Up, MathHelper.Pi * .1f) { Rigidity = 0.08f });
            joints.Add(new IKSwingLimit(upperLeftLegBone, lowerLeftLegBone, Vector3.Normalize(new Vector3(0, -.23f, -.97f)), Vector3.Down, MathHelper.Pi * 0.435f));
            joints.Add(new IKBallSocketJoint(lowerLeftLegBone, leftFootBone, lowerLeftLegLeftFootBallSocketAnchor));
            joints.Add(new IKTwistJoint(lowerLeftLegBone, leftFootBone, Vector3.Down, Vector3.Down) { Rigidity = 0.08f });
            joints.Add(new IKSwingLimit(lowerLeftLegBone, leftFootBone, Vector3.Normalize(new Vector3(0, -1, -.3f)), Vector3.Down, MathHelper.Pi * 0.22f));

            //Right leg
            joints.Add(new IKBallSocketJoint(bodyBone, upperRightLegBone, bodyUpperRightLegBallSocketAnchor));
            joints.Add(new IKSwingLimit(bodyBone, upperRightLegBone, Vector3.Normalize(new Vector3(.3f, -1, .6f)), Vector3.Down, MathHelper.Pi * 0.6f));
            joints.Add(new IKTwistLimit(bodyBone, upperRightLegBone, Vector3.Up, Vector3.Up, MathHelper.PiOver4) { MeasurementAxisA = Vector3.Normalize(new Vector3(1, 0, 1)), Rigidity = 0.08f });
            joints.Add(new IKBallSocketJoint(upperRightLegBone, lowerRightLegBone, upperRightLegLowerRightLegBallSocketAnchor));
            joints.Add(new IKSwivelHingeJoint(upperRightLegBone, lowerRightLegBone, Vector3.Right, Vector3.Down));
            joints.Add(new IKTwistLimit(upperRightLegBone, lowerRightLegBone, Vector3.Up, Vector3.Up, MathHelper.Pi * .1f) { Rigidity = 0.08f });
            joints.Add(new IKSwingLimit(upperRightLegBone, lowerRightLegBone, Vector3.Normalize(new Vector3(0, -.23f, -.97f)), Vector3.Down, MathHelper.Pi * 0.435f));
            joints.Add(new IKBallSocketJoint(lowerRightLegBone, rightFootBone, lowerRightLegRightFootBallSocketAnchor));
            joints.Add(new IKTwistJoint(lowerRightLegBone, rightFootBone, Vector3.Down, Vector3.Down) { Rigidity = 0.08f });
            joints.Add(new IKSwingLimit(lowerRightLegBone, rightFootBone, Vector3.Normalize(new Vector3(0, -1, -.3f)), Vector3.Down, MathHelper.Pi * 0.22f));






        }

        void BuildCyclicMesh(Vector3 position)
        {
            int widthCount = 5;
            int heightCount = 5;
            var boneMesh = new BoneRelationship[widthCount, heightCount];
            for (int i = 0; i < widthCount; i++)
            {
                for (int j = 0; j < heightCount; j++)
                {
                    var bonePosition = position + new Vector3(i, j, 0);
                    boneMesh[i, j] = new BoneRelationship(
                        new Bone(bonePosition, Quaternion.Identity, .4f, .8f),
                        new Box(bonePosition, .8f, .8f, .8f, 10));
                    Space.Add(boneMesh[i, j].Entity);
                    bones.Add(boneMesh[i, j]);
                }
            }

            for (int i = 0; i < widthCount; i++)
            {
                for (int j = 0; j < heightCount; j++)
                {
                    if (i > 0)
                    {
                        var boneA = boneMesh[i, j];
                        var boneB = boneMesh[i - 1, j];
                        var anchor = (boneA.Entity.Position + boneB.Entity.Position) / 2;
                        var dynamicsJoint = new BallSocketJoint(boneA.Entity, boneB.Entity, anchor);
                        CollisionRules.AddRule(boneA.Entity, boneB.Entity, CollisionRule.NoBroadPhase);
                        Space.Add(dynamicsJoint);
                        joints.Add(new IKBallSocketJoint(boneA.Bone, boneB.Bone, anchor));
                    }
                    if (j > 0)
                    {
                        var boneA = boneMesh[i, j];
                        var boneB = boneMesh[i, j - 1];
                        var anchor = (boneA.Entity.Position + boneB.Entity.Position) / 2;
                        var dynamicsJoint = new BallSocketJoint(boneA.Entity, boneB.Entity, anchor);
                        CollisionRules.AddRule(boneA.Entity, boneB.Entity, CollisionRule.NoBroadPhase);
                        Space.Add(dynamicsJoint);
                        joints.Add(new IKBallSocketJoint(boneA.Bone, boneB.Bone, anchor));
                    }
                    if (i > 0 && j > 0)
                    {
                        var boneA = boneMesh[i, j];
                        var boneB = boneMesh[i - 1, j - 1];
                        var anchor = (boneA.Entity.Position + boneB.Entity.Position) / 2;
                        var dynamicsJoint = new BallSocketJoint(boneA.Entity, boneB.Entity, anchor);
                        CollisionRules.AddRule(boneA.Entity, boneB.Entity, CollisionRule.NoBroadPhase);
                        Space.Add(dynamicsJoint);
                        joints.Add(new IKBallSocketJoint(boneA.Bone, boneB.Bone, anchor));
                    }

                    if (i < widthCount - 1 && j > 0)
                    {
                        var boneA = boneMesh[i, j];
                        var boneB = boneMesh[i + 1, j - 1];
                        var anchor = (boneA.Entity.Position + boneB.Entity.Position) / 2;
                        var dynamicsJoint = new BallSocketJoint(boneA.Entity, boneB.Entity, anchor);
                        CollisionRules.AddRule(boneA.Entity, boneB.Entity, CollisionRule.NoBroadPhase);
                        Space.Add(dynamicsJoint);
                        joints.Add(new IKBallSocketJoint(boneA.Bone, boneB.Bone, anchor));
                    }
                }
            }

            int limbCount = 4;

            var previous = boneMesh[boneMesh.GetLength(0) / 2, 0];

            for (int i = 0; i < limbCount; i++)
            {
                Bone bone = new Bone(previous.Bone.Position + new Vector3(0, -0.8f, 0), Quaternion.Identity, .2f, .4f);
                Entity entity = new Box(bone.Position, .4f, .4f, .4f, 10);
                Space.Add(entity);
                var anchor = (entity.Position + previous.Entity.Position) / 2;
                Space.Add(new BallSocketJoint(entity, previous.Entity, anchor));
                joints.Add(new IKBallSocketJoint(bone, previous.Bone, anchor));
                CollisionRules.AddRule(entity, previous.Entity, CollisionRule.NoBroadPhase);
                previous = new BoneRelationship(bone, entity);
                bones.Add(previous);
            }

            previous = boneMesh[boneMesh.GetLength(0) / 2, boneMesh.GetLength(1) - 1];

            for (int i = 0; i < limbCount; i++)
            {
                Bone bone = new Bone(previous.Bone.Position + new Vector3(0, 0.8f, 0), Quaternion.Identity, .2f, .4f);
                Entity entity = new Box(bone.Position, .4f, .4f, .4f, 10);
                Space.Add(entity);
                var anchor = (entity.Position + previous.Entity.Position) / 2;
                Space.Add(new BallSocketJoint(entity, previous.Entity, anchor));
                CollisionRules.AddRule(entity, previous.Entity, CollisionRule.NoBroadPhase);
                joints.Add(new IKBallSocketJoint(bone, previous.Bone, anchor));
                previous = new BoneRelationship(bone, entity);
                bones.Add(previous);
            }

            previous = boneMesh[0, boneMesh.GetLength(1) / 2];

            for (int i = 0; i < limbCount; i++)
            {
                Bone bone = new Bone(previous.Bone.Position + new Vector3(-.8f, 0, 0), Quaternion.Identity, .2f, .4f);
                Entity entity = new Box(bone.Position, .4f, .4f, .4f, 10);
                Space.Add(entity);
                var anchor = (entity.Position + previous.Entity.Position) / 2;
                Space.Add(new BallSocketJoint(entity, previous.Entity, anchor));
                CollisionRules.AddRule(entity, previous.Entity, CollisionRule.NoBroadPhase);
                joints.Add(new IKBallSocketJoint(bone, previous.Bone, anchor));
                previous = new BoneRelationship(bone, entity);
                bones.Add(previous);
            }


            previous = boneMesh[boneMesh.GetLength(0) - 1, boneMesh.GetLength(1) / 2];

            for (int i = 0; i < limbCount; i++)
            {
                Bone bone = new Bone(previous.Bone.Position + new Vector3(0.8f, 0, 0), Quaternion.Identity, .2f, .4f);
                Entity entity = new Box(bone.Position, .4f, .4f, .4f, 10);
                Space.Add(entity);
                var anchor = (entity.Position + previous.Entity.Position) / 2;
                Space.Add(new BallSocketJoint(entity, previous.Entity, anchor));
                CollisionRules.AddRule(entity, previous.Entity, CollisionRule.NoBroadPhase);
                joints.Add(new IKBallSocketJoint(bone, previous.Bone, anchor));
                previous = new BoneRelationship(bone, entity);
                bones.Add(previous);
            }

        }

        void BuildJointTest(Vector3 position)
        {

            Bone a, b;
            a = new Bone(position + new Vector3(0, 5, 0), Quaternion.Identity, .5f, 1);
            b = new Bone(position + new Vector3(0, 7, 0), Quaternion.Identity, .5f, 1);
            //var ikJoint = new IKBallSocketJoint(a, b, (a.Position + b.Position) * 0.5f);
            //var ikLimit = new IKSwingLimit(a, b, Vector3.Up, Vector3.Up, MathHelper.PiOver2);
            //var ikRevolute = new IKRevoluteJoint(a, b, Vector3.Right);
            //var ikSwivelHingeJoint = new IKSwivelHingeJoint(a, b, Vector3.Right, Vector3.Up);
            //var ikAngularJoint = new IKAngularJoint(a, b);
            //var ikTwistLimit = new IKTwistLimit(a, b, Vector3.Up, Vector3.Up, MathHelper.PiOver2);
            //var ikDistanceLimit = new IKDistanceLimit(a, b, a.Position + new Vector3(0, 0.5f, 0), b.Position + new Vector3(0, -0.5f, 0), 1f, 4);
            //var ikLinearAxisLimit = new IKLinearAxisLimit(a, b, a.Position + new Vector3(0, 0.5f, 0), Vector3.Up, b.Position + new Vector3(0, -0.5f, 0), 0, 4);
            //var ikTwistJoint = new IKTwistJoint(a, b, Vector3.Up, Vector3.Up);
            //var ikPointOnLineJoint = new IKPointOnLineJoint(a, b, a.Position + new Vector3(0, 0.5f, 0), Vector3.Up, b.Position - new Vector3(0, 0.5f, 0));
            var ikPointOnPlaneJoint = new IKPointOnPlaneJoint(a, b, a.Position + new Vector3(0, 1f, 0), Vector3.Up, b.Position - new Vector3(0, 1f, 0));

            //ikPointOnLineJoint.Softness = 0;
            //ikPointOnLineJoint.ErrorCorrectionFactor = 0;
            //solver.VelocitySubiterationCount = 10;

            var entityA = new Cylinder(a.Position, 1, 0.5f, 10);
            var entityB = new Cylinder(b.Position, 1, 0.5f, 10);
            entityB.Orientation = b.Orientation;
            //var joint = new BallSocketJoint(entityA, entityB, (a.Position + b.Position) * 0.5f);
            //var limit = new SwingLimit(entityA, entityB, Vector3.Up, Vector3.Up, MathHelper.PiOver2);
            //var revolute = new RevoluteAngularJoint(entityA, entityB, Vector3.Right);
            //var swivelHingeJoint = new SwivelHingeAngularJoint(entityA, entityB, Vector3.Right, Vector3.Up);
            //var angularJoint = new NoRotationJoint(entityA, entityB);
            //var twistLimit = new TwistLimit(entityA, entityB, Vector3.Up, Vector3.Up, -MathHelper.PiOver2, MathHelper.PiOver2);
            //var distanceLimit = new DistanceLimit(entityA, entityB, ikDistanceLimit.AnchorA, ikDistanceLimit.AnchorB, ikDistanceLimit.MinimumDistance, ikDistanceLimit.MaximumDistance);
            //var linearAxisLimit = new LinearAxisLimit(entityA, entityB, ikLinearAxisLimit.LineAnchor, ikLinearAxisLimit.AnchorB, ikLinearAxisLimit.LineDirection, ikLinearAxisLimit.MinimumDistance, ikLinearAxisLimit.MaximumDistance);
            //var twistJoint = new TwistJoint(entityA, entityB, Vector3.Up, Vector3.Up);
            //var pointOnLineJoint = new PointOnLineJoint(entityA, entityB, ikPointOnLineJoint.LineAnchor, ikPointOnLineJoint.LineDirection, ikPointOnLineJoint.AnchorB);
            var pointOnPlaneJoint = new PointOnPlaneJoint(entityA, entityB, ikPointOnPlaneJoint.PlaneAnchor, ikPointOnPlaneJoint.PlaneNormal, ikPointOnPlaneJoint.AnchorB);

            Space.Add(entityA);
            Space.Add(entityB);
            //Space.Add(joint);
            //Space.Add(limit);
            //Space.Add(revolute);
            //Space.Add(swivelHingeJoint);
            //Space.Add(angularJoint);
            //Space.Add(twistLimit);
            //Space.Add(distanceLimit);
            //Space.Add(linearAxisLimit);
            //Space.Add(twistJoint);
            //Space.Add(pointOnLineJoint);
            Space.Add(pointOnPlaneJoint);
            bones.Add(new BoneRelationship(a, entityA));
            bones.Add(new BoneRelationship(b, entityB));
        }

        void BuildRoboArmThing(Vector3 position)
        {
            //Make the IK representation
            Bone baseBone = new Bone(position, Quaternion.Identity, 1, 3);
            Bone upperArm = new Bone(baseBone.Position + new Vector3(0, 1.5f + 2, 0), Quaternion.Identity, .4f, 4);
            Bone lowerArm = new Bone(upperArm.Position + new Vector3(0, 2 + 1, 0), Quaternion.Identity, .7f, 2);
            Bone bonkDevice = new Bone(lowerArm.Position + new Vector3(0, 5, 0), Quaternion.Identity, .6f, 1.2f);

            joints.Add(new IKBallSocketJoint(baseBone, upperArm, baseBone.Position + new Vector3(0, 1.5f, 0)));
            joints.Add(new IKSwingLimit(baseBone, upperArm, Vector3.Up, Vector3.Up, MathHelper.PiOver4));
            joints.Add(new IKBallSocketJoint(upperArm, lowerArm, upperArm.Position + new Vector3(0, 2f, 0)));
            joints.Add(new IKRevoluteJoint(upperArm, lowerArm, Vector3.Forward));
            joints.Add(new IKSwingLimit(upperArm, lowerArm, Vector3.Up, Vector3.Up, MathHelper.PiOver4));
            joints.Add(new IKPointOnLineJoint(lowerArm, bonkDevice, lowerArm.Position, Vector3.Up, bonkDevice.Position));
            joints.Add(new IKAngularJoint(lowerArm, bonkDevice));
            joints.Add(new IKLinearAxisLimit(lowerArm, bonkDevice, lowerArm.Position, Vector3.Up, bonkDevice.Position, 1.6f, 5));

            //Make the dynamics representation
            Entity baseEntity = new Cylinder(baseBone.Position, baseBone.Height, baseBone.Radius, 10);
            Entity upperArmEntity = new Cylinder(upperArm.Position, upperArm.Height, upperArm.Radius, 7);
            Entity lowerArmEntity = new Cylinder(lowerArm.Position, lowerArm.Height, lowerArm.Radius, 5);
            Entity bonkDeviceEntity = new Cylinder(bonkDevice.Position, bonkDevice.Height, bonkDevice.Radius, 3);
            bonkDeviceEntity.Orientation = bonkDevice.Orientation;

            Space.Add(baseEntity);
            Space.Add(upperArmEntity);
            Space.Add(lowerArmEntity);
            Space.Add(bonkDeviceEntity);

            Space.Add(new BallSocketJoint(baseEntity, upperArmEntity, baseBone.Position + new Vector3(0, 1.5f, 0)));
            Space.Add(new SwingLimit(baseEntity, upperArmEntity, Vector3.Up, Vector3.Up, MathHelper.PiOver4));
            Space.Add(new BallSocketJoint(upperArmEntity, lowerArmEntity, upperArm.Position + new Vector3(0, 2f, 0)));
            Space.Add(new RevoluteAngularJoint(upperArmEntity, lowerArmEntity, Vector3.Forward));
            Space.Add(new SwingLimit(upperArmEntity, lowerArmEntity, Vector3.Up, Vector3.Up, MathHelper.PiOver4));
            Space.Add(new PointOnLineJoint(lowerArmEntity, bonkDeviceEntity, lowerArm.Position, Vector3.Up, bonkDevice.Position));
            var motor = new AngularMotor(lowerArmEntity, bonkDeviceEntity);
            motor.Settings.Mode = MotorMode.Servomechanism;
            Space.Add(motor);
            Space.Add(new LinearAxisLimit(lowerArmEntity, bonkDeviceEntity, lowerArm.Position, bonkDevice.Position, Vector3.Up, 1.6f, 5));

            CollisionRules.AddRule(baseEntity, upperArmEntity, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(upperArmEntity, lowerArmEntity, CollisionRule.NoBroadPhase);
            CollisionRules.AddRule(lowerArmEntity, bonkDeviceEntity, CollisionRule.NoBroadPhase);


            //Relate the two!
            bones.Add(new BoneRelationship(baseBone, baseEntity));
            bones.Add(new BoneRelationship(upperArm, upperArmEntity));
            bones.Add(new BoneRelationship(lowerArm, lowerArmEntity));
            bones.Add(new BoneRelationship(bonkDevice, bonkDeviceEntity));

        }

        void BuildRing(Vector3 position)
        {
            int incrementCount = 20;
            float radius = 5;
            float anglePerIncrement = MathHelper.TwoPi / incrementCount;
            Bone[] bonesList = new Bone[incrementCount];
            for (int i = 0; i < incrementCount; i++)
            {
                Vector3 bonePosition;
#if !WINDOWS
                bonePosition = new Vector3();
#endif
                bonePosition.X = (float)Math.Cos(anglePerIncrement * i);
                bonePosition.Y = 0;
                bonePosition.Z = (float)Math.Sin(anglePerIncrement * i);
                bonePosition = bonePosition * radius + position;
                bonesList[i] = new Bone(bonePosition,
                                     Quaternion.Concatenate(Quaternion.CreateFromAxisAngle(Vector3.Right, MathHelper.PiOver2), Quaternion.CreateFromAxisAngle(Vector3.Up, -anglePerIncrement * i)),
                                     0.5f,
                                     MathHelper.Pi * radius * 2 / incrementCount);
            }

            for (int i = 0; i < bonesList.Length; i++)
            {
                var boneA = bonesList[i];
                var boneB = bonesList[(i + 1) % incrementCount];
                var upA = Quaternion.Transform(Vector3.Up, boneA.Orientation);
                var upB = Quaternion.Transform(Vector3.Up, boneB.Orientation);
                joints.Add(new IKBallSocketJoint(boneA, boneB, (boneA.Position + upA * boneB.HalfHeight + boneB.Position - upB * boneB.HalfHeight) * .5f));
                joints.Add(new IKSwingLimit(boneA, boneB, upA, upB, MathHelper.Pi * .5f));
            }
            Cylinder[] boneEntitiesList = new Cylinder[incrementCount];
            for (int i = 0; i < incrementCount; i++)
            {
                var boneEntity = new Cylinder(new MotionState { Position = bonesList[i].Position, Orientation = bonesList[i].Orientation }, bonesList[i].Height, bonesList[i].Radius, 10);
                bones.Add(new BoneRelationship(bonesList[i], boneEntity));
                boneEntitiesList[i] = boneEntity;
                Space.Add(boneEntity);
            }

            for (int i = 0; i < incrementCount; i++)
            {
                var boneA = boneEntitiesList[i];
                var boneB = boneEntitiesList[(i + 1) % incrementCount];
                var upA = Quaternion.Transform(Vector3.Up, boneA.Orientation);
                var upB = Quaternion.Transform(Vector3.Up, boneB.Orientation);
                var joint = new BallSocketJoint(boneA, boneB, (boneA.Position + upA * boneB.Height * 0.5f + boneB.Position - upB * boneB.Height * 0.5f) * .5f);
                var swingLimit = new SwingLimit(boneA, boneB, upA, upB, MathHelper.Pi * .5f);
                Space.Add(swingLimit);
                Space.Add(joint);
                CollisionRules.AddRule(boneA, boneB, CollisionRule.NoBroadPhase);
            }
        }

        void BuildTinyTest(Vector3 position)
        {

            Bone[] bonesList = new Bone[2];



            bonesList[0] = new Bone(position + new Vector3(0, 10, 0),
                     Quaternion.Identity,
                     0.05f,
                     0.2f);

            bonesList[1] = new Bone(position + new Vector3(0, 12, 0),
                     Quaternion.Identity,
                     0.05f * 100,
                     0.2f * 100);


            Cylinder[] boneEntitiesList = new Cylinder[2];
            boneEntitiesList[0] = new Cylinder(bonesList[0].Position, bonesList[0].Radius, bonesList[0].Height, 10);
            boneEntitiesList[1] = new Cylinder(bonesList[1].Position, bonesList[1].Radius, bonesList[1].Height, 10);

            bones.Add(new BoneRelationship(bonesList[0], boneEntitiesList[0]));
            bones.Add(new BoneRelationship(bonesList[1], boneEntitiesList[1]));

        }



        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public InverseKinematicsTestDemo(DemosGame game)
            : base(game)
        {

            cameraControl = new FreeCameraControlScheme(10, game.Camera, game);
            whitePixel = game.Content.Load<Texture2D>("whitePixel");
            game.Camera.Position = new Vector3(0, 3, 5);
            Box ground = new Box(new Vector3(0, 0, 0), 30, 1, 30);
            Space.Add(ground);
            Space.ForceUpdater.Gravity = new Vector3(0, -9.81f, 0);

            stateControlGroup = new StateControlGroup(game.Camera, controls);


            drawer = new InstancedModelDrawer(game);

            solver.ActiveSet.UseAutomass = true;
            solver.AutoscaleControlImpulses = true;
            solver.AutoscaleControlMaximumForce = float.MaxValue;
            solver.TimeStepDuration = .1f;
            solver.ControlIterationCount = 100;
            solver.FixerIterationCount = 10;
            solver.VelocitySubiterationCount = 3;


            BuildChain(new Vector3(-5, 2, 0));

            BuildStick(new Vector3(-10, 2, 0));

            BuildActionFigure(new Vector3(5, 6, -8));
            BuildActionFigure(new Vector3(5, 6, -3));
            BuildActionFigure(new Vector3(5, 6, 3));
            BuildActionFigure(new Vector3(5, 6, 8));

            BuildCyclicMesh(new Vector3(-5, 5, -5));


            //BuildJointTest(new Vector3(0, 5, 0));
            //BuildTinyTest(new Vector3(0, 15, 0));

            BuildRoboArmThing(new Vector3(0, 5, 0));

            BuildRing(new Vector3(0, 10, 8));



            //Create the display objects.
            Model cylinder = game.Content.Load<Model>("cylinder");
            for (int i = 0; i < bones.Count; i++)
            {
                var displayBone = new DisplayModel(cylinder, drawer);
                var completedBone = bones[i];
                completedBone.DisplayBone = displayBone;
                bones[i] = completedBone;
                drawer.Add(displayBone);
            }
        }

        public override void Update(float dt)
        {
            cameraControl.Update(dt);

            if (Game.WasKeyPressed(Keys.T))
            {
                usingIK = !usingIK;
                //Upon swapping, update the positions of the entities to the state of the IK or vice versa.
                if (usingIK)
                {
                    foreach (var bone in bones)
                    {
                        bone.Bone.Position = bone.Entity.Position;
                        bone.Bone.Orientation = Quaternion.Concatenate(Quaternion.Conjugate(bone.LocalRotationBoneToEntity), bone.Entity.Orientation);
                    }
                }
                else
                {
                    foreach (var bone in bones)
                    {
                        bone.Entity.Position = bone.Bone.Position;
                        bone.Entity.Orientation = Quaternion.Concatenate(bone.LocalRotationBoneToEntity, bone.Bone.Orientation);
                        bone.Entity.AngularVelocity = new Vector3();
                        bone.Entity.LinearVelocity = new Vector3();
                    }
                }
            }

            if (usingIK)
            {
                //Check for pinning!
                if (Game.MouseInput.LeftButton == ButtonState.Pressed && Game.PreviousMouseInput.LeftButton == ButtonState.Released && Game.MouseInput.RightButton == ButtonState.Released)
                {
                    //Try to pin a bone.
                    BoneRelationship hitBone;
                    Vector3 hitPosition;
                    if (RayCastBones(new Ray(Game.Camera.Position, Game.Camera.WorldMatrix.Forward), out hitBone, out hitPosition)) //Can't control pinned bones.
                    {
                        //Found one!
                        hitBone.Bone.Pinned = !hitBone.Bone.Pinned;
                    }
                }

                //Manage bone grabbery.
                if ((Game.MouseInput.RightButton == ButtonState.Pressed && Game.PreviousMouseInput.RightButton == ButtonState.Released) || //Either a new click, or
                    (Game.MouseInput.RightButton == ButtonState.Pressed && dragControl.TargetBone == null && !stateControlGroup.IsActive)) //we do not yet have a grab, but we want one.
                {
                    //This is a new click. Try to grab.
                    BoneRelationship hitBone;
                    Vector3 hitPosition;
                    if (RayCastBones(new Ray(Game.Camera.Position, Game.Camera.WorldMatrix.Forward), out hitBone, out hitPosition) && !hitBone.Bone.Pinned)
                    {
                        if (Game.KeyboardInput.IsKeyDown(Keys.LeftShift))
                        {
                            //Disable the drag control.
                            controls.Remove(dragControl);
                            dragControl.TargetBone = null;
                            //Grab with a state control.
                            stateControlGroup.TryToAddBone(hitBone.Bone, hitPosition);
                        }
                        else
                        {
                            //Disable the state controls.
                            stateControlGroup.ClearStateControls();
                            //Grab a bone with the drag control.
                            dragControl.TargetBone = hitBone.Bone;
                            controls.Add(dragControl);
                            dragControl.LinearMotor.Offset = hitPosition - hitBone.Bone.Position;
                            distanceToGrabbedBone = Vector3.Dot(Game.Camera.WorldMatrix.Forward, hitPosition - Game.Camera.Position);
                        }
                    }
                    else
                    {
                        //We missed! Clear controls.
                        stateControlGroup.ClearStateControls();
                        controls.Remove(dragControl);
                        dragControl.TargetBone = null;
                    }

                }
                if (Game.MouseInput.RightButton == ButtonState.Pressed)
                {
                    //We may be dragging something. If we are, update it.
                    if (dragControl.TargetBone != null)
                        dragControl.LinearMotor.TargetPosition = Game.Camera.Position + Game.Camera.WorldMatrix.Forward * distanceToGrabbedBone;
                    else if (stateControlGroup.IsActive)
                        stateControlGroup.UpdateGoals();

                    if (dragControl.TargetBone != null || stateControlGroup.IsActive)
                    {
                        long start = Stopwatch.GetTimestamp();
                        solver.Solve(controls);
                        long end = Stopwatch.GetTimestamp();
                        elapsedTime = (end - start) / (double)Stopwatch.Frequency;
                    }

                }
                else
                {
                    elapsedTime = 0;
                    dragControl.TargetBone = null;
                    controls.Remove(dragControl);

                }


                if (Game.MouseInput.XButton1 == ButtonState.Pressed)
                {
                    solver.Solve(joints);
                }

                //Update the positions of the bone graphics.
                for (int i = 0; i < bones.Count; i++)
                {
                    Matrix localTransform;
                    Matrix.CreateScale(bones[i].Bone.Radius, bones[i].Bone.Height, bones[i].Bone.Radius, out localTransform);
                    Matrix transform;
                    Matrix.CreateFromQuaternion(ref bones[i].Bone.Orientation, out transform);
                    Matrix.Multiply(ref localTransform, ref transform, out transform);
                    transform.Translation = bones[i].Bone.Position;
                    bones[i].DisplayBone.WorldTransform = transform;

                    if (bones[i].Bone.Pinned)
                        bones[i].DisplayBone.Color = new Microsoft.Xna.Framework.Vector3(0, 0, 1);
                    else
                    {
                        bones[i].DisplayBone.Color = new Microsoft.Xna.Framework.Vector3(bones[i].Bone.Mass / 2, 0, 0);
                    }
                }


            }
            else
                base.Update(dt);
        }




        public override void DrawUI()
        {
            Game.DataTextDrawer.Draw("IK Controls:", new Microsoft.Xna.Framework.Vector2(970, 20));
            Game.TinyTextDrawer.Draw(" T: Toggle Dynamics/IK", new Microsoft.Xna.Framework.Vector2(970, 43));
            Game.TinyTextDrawer.Draw(" Right click: Drag", new Microsoft.Xna.Framework.Vector2(970, 57));
            Game.TinyTextDrawer.Draw(" Left shift + Right click: Multiselect Move", new Microsoft.Xna.Framework.Vector2(970, 71));
            Game.TinyTextDrawer.Draw(" Left click: Pin/unpin bone", new Microsoft.Xna.Framework.Vector2(970, 85));

            Game.DataTextDrawer.Draw("Current mode: ", new Microsoft.Xna.Framework.Vector2(970, 113));
            if (usingIK)
                Game.DataTextDrawer.Draw("IK", new Microsoft.Xna.Framework.Vector2(1110, 113));
            else
                Game.DataTextDrawer.Draw("Dynamics", new Microsoft.Xna.Framework.Vector2(1110, 113));

            if (usingIK)
                Game.TinyTextDrawer.Draw(" Solving time (ms): ", elapsedTime * 1000, 2, new Microsoft.Xna.Framework.Vector2(970, 141));
            Game.UIDrawer.Draw(whitePixel, new Rectangle(Game.Graphics.PreferredBackBufferWidth / 2, Game.Graphics.PreferredBackBufferHeight / 2, 3, 3), Color.LightBlue);

        }

        public override void Draw()
        {
            if (usingIK)
            {
                drawer.Draw(Game.Camera.ViewMatrix, Game.Camera.ProjectionMatrix);
            }
            base.Draw();
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Inverse Kinematics"; }
        }

        private bool RayCastBones(Ray ray, out BoneRelationship hitBone, out Vector3 hitPosition)
        {
            float t = float.MaxValue;
            hitBone = new BoneRelationship();
            hitPosition = new Vector3();
            for (int i = 0; i < bones.Count; i++)
            {
                var bone = bones[i].Bone;
                RayHit hit;
                if (RayCast(bone, ray, out hit) && hit.T < t)
                {
                    t = hit.T;
                    hitPosition = hit.Location;
                    hitBone = bones[i];
                }
            }
            return t < float.MaxValue;
        }
        public bool RayCast(Bone bone, Ray ray, out RayHit hit)
        {
            var transform = new RigidTransform(bone.Position, bone.Orientation);
            //Put the ray into local space.
            Quaternion conjugate;
            Quaternion.Conjugate(ref transform.Orientation, out conjugate);
            Ray localRay;
            Vector3.Subtract(ref ray.Position, ref transform.Position, out localRay.Position);
            Quaternion.Transform(ref localRay.Position, ref conjugate, out localRay.Position);
            Quaternion.Transform(ref ray.Direction, ref conjugate, out localRay.Direction);

            var halfHeight = bone.HalfHeight;
            var radius = bone.Radius;
            //Check for containment.
            if (localRay.Position.Y >= -halfHeight && localRay.Position.Y <= halfHeight && localRay.Position.X * localRay.Position.X + localRay.Position.Z * localRay.Position.Z <= radius * radius)
            {
                //It's inside!
                hit.T = 0;
                hit.Location = localRay.Position;
                hit.Normal = new Vector3(hit.Location.X, 0, hit.Location.Z);
                float normalLengthSquared = hit.Normal.LengthSquared();
                if (normalLengthSquared > 1e-9f)
                    Vector3.Divide(ref hit.Normal, (float)Math.Sqrt(normalLengthSquared), out hit.Normal);
                else
                    hit.Normal = new Vector3();
                //Pull the hit into world space.
                Quaternion.Transform(ref hit.Normal, ref transform.Orientation, out hit.Normal);
                RigidTransform.Transform(ref hit.Location, ref transform, out hit.Location);
                return true;
            }

            //Project the ray direction onto the plane where the cylinder is a circle.
            //The projected ray is then tested against the circle to compute the time of impact.
            //That time of impact is used to compute the 3d hit location.
            Vector2 planeDirection = new Vector2(localRay.Direction.X, localRay.Direction.Z);
            float planeDirectionLengthSquared = planeDirection.LengthSquared();

            if (planeDirectionLengthSquared < Toolbox.Epsilon)
            {
                //The ray is nearly parallel with the axis.
                //Skip the cylinder-sides test.  We're either inside the cylinder and won't hit the sides, or we're outside
                //and won't hit the sides.  
                if (localRay.Position.Y > halfHeight)
                    goto upperTest;
                if (localRay.Position.Y < -halfHeight)
                    goto lowerTest;


                hit = new RayHit();
                return false;

            }
            Vector2 planeOrigin = new Vector2(localRay.Position.X, localRay.Position.Z);
            float dot;
            Vector2.Dot(ref planeDirection, ref planeOrigin, out dot);
            float closestToCenterT = -dot / planeDirectionLengthSquared;

            Vector2 closestPoint;
            Vector2.Multiply(ref planeDirection, closestToCenterT, out closestPoint);
            Vector2.Add(ref planeOrigin, ref closestPoint, out closestPoint);
            //How close does the ray come to the circle?
            float squaredDistance = closestPoint.LengthSquared();
            if (squaredDistance > radius * radius)
            {
                //It's too far!  The ray cannot possibly hit the capsule.
                hit = new RayHit();
                return false;
            }



            //With the squared distance, compute the distance backward along the ray from the closest point on the ray to the axis.
            float backwardsDistance = radius * (float)Math.Sqrt(1 - squaredDistance / (radius * radius));
            float tOffset = backwardsDistance / (float)Math.Sqrt(planeDirectionLengthSquared);

            hit.T = closestToCenterT - tOffset;

            //Compute the impact point on the infinite cylinder in 3d local space.
            Vector3.Multiply(ref localRay.Direction, hit.T, out hit.Location);
            Vector3.Add(ref hit.Location, ref localRay.Position, out hit.Location);

            //Is it intersecting the cylindrical portion of the capsule?
            if (hit.Location.Y <= halfHeight && hit.Location.Y >= -halfHeight)
            {
                //Yup!
                hit.Normal = new Vector3(hit.Location.X, 0, hit.Location.Z);
                float normalLengthSquared = hit.Normal.LengthSquared();
                if (normalLengthSquared > 1e-9f)
                    Vector3.Divide(ref hit.Normal, (float)Math.Sqrt(normalLengthSquared), out hit.Normal);
                else
                    hit.Normal = new Vector3();
                //Pull the hit into world space.
                Quaternion.Transform(ref hit.Normal, ref transform.Orientation, out hit.Normal);
                RigidTransform.Transform(ref hit.Location, ref transform, out hit.Location);
                return true;
            }

            if (hit.Location.Y < halfHeight)
                goto lowerTest;
        upperTest:
            //Nope! It may be intersecting the ends of the cylinder though.
            //We're above the cylinder, so cast a ray against the upper cap.
            if (localRay.Direction.Y > -1e-9)
            {
                //Can't hit the upper cap if the ray isn't pointing down.
                hit = new RayHit();
                return false;
            }
            float t = (halfHeight - localRay.Position.Y) / localRay.Direction.Y;
            Vector3 planeIntersection;
            Vector3.Multiply(ref localRay.Direction, t, out planeIntersection);
            Vector3.Add(ref localRay.Position, ref planeIntersection, out planeIntersection);
            if (planeIntersection.X * planeIntersection.X + planeIntersection.Z * planeIntersection.Z < radius * radius + 1e-9)
            {
                //Pull the hit into world space.
                Quaternion.Transform(ref Toolbox.UpVector, ref transform.Orientation, out hit.Normal);
                RigidTransform.Transform(ref planeIntersection, ref transform, out hit.Location);
                hit.T = t;
                return true;
            }
            //No intersection! We can't be hitting the other sphere, so it's over!
            hit = new RayHit();
            return false;

        lowerTest:
            //Is it intersecting the bottom cap?
            if (localRay.Direction.Y < 1e-9)
            {
                //Can't hit the bottom cap if the ray isn't pointing up.
                hit = new RayHit();
                return false;
            }
            t = (-halfHeight - localRay.Position.Y) / localRay.Direction.Y;
            Vector3.Multiply(ref localRay.Direction, t, out planeIntersection);
            Vector3.Add(ref localRay.Position, ref planeIntersection, out planeIntersection);
            if (planeIntersection.X * planeIntersection.X + planeIntersection.Z * planeIntersection.Z < radius * radius + 1e-9)
            {
                //Pull the hit into world space.
                Quaternion.Transform(ref Toolbox.DownVector, ref transform.Orientation, out hit.Normal);
                RigidTransform.Transform(ref planeIntersection, ref transform, out hit.Location);
                hit.T = t;
                return true;
            }
            //No intersection! We can't be hitting the other sphere, so it's over!
            hit = new RayHit();
            return false;

        }
    }
}
#endif