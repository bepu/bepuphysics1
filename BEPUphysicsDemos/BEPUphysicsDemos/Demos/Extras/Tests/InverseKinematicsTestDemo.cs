using System;
using System.Collections.Generic;
using BEPUphysics;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.Constraints.TwoEntity.JointLimits;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.MathExtensions;
using BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics;
using BEPUphysicsDrawer.Models;
using Microsoft.Xna.Framework;
using System.Diagnostics;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Test environment for the inverse kinematics solver.
    /// </summary>
    public class InverseKinematicsTestDemo : Demo
    {
        private bool usingIK;

        private IKSolver solver = new IKSolver();

        private StateControl stateControl = new StateControl();
        private DragControl dragControl = new DragControl();
        private Control currentControl;
        private float distanceToGrabbedBone;
        private Vector3 grabOffset; //Used by the stateControl.




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
                Space.Add(dynamicsBallSocketJoint);
                var ikBallSocketJoint = new IKBallSocketJoint(previousBone, bone, anchor); //(the joint is auto-added to the bones; not solver-add is needed)

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
            Space.Add(new BallSocketJoint(body, head, head.Position + new Vector3(0, -.9f, 0)));
            //Angular motors can be used to simulate friction when their goal velocity is 0.
            var angularMotor = new AngularMotor(body, head);
            angularMotor.Settings.MaximumForce = 150; //The maximum force of 'friction' in this joint.
            Space.Add(angularMotor);

            //Make the first arm.
            var upperLeftArm = new Box(body.Position + new Vector3(-1.6f, .8f, 0), 1, .5f, .5f, 5);
            Space.Add(upperLeftArm);

            var lowerLeftArm = new Box(upperLeftArm.Position + new Vector3(-1.4f, 0, 0), 1, .5f, .5f, 5);
            Space.Add(lowerLeftArm);

            //Connect the body to the upper arm.
            Space.Add(new BallSocketJoint(body, upperLeftArm, upperLeftArm.Position + new Vector3(.7f, 0, 0)));
            angularMotor = new AngularMotor(body, upperLeftArm);
            angularMotor.Settings.MaximumForce = 250;
            Space.Add(angularMotor);


            //Connect the upper arm to the lower arm.
            Space.Add(new BallSocketJoint(upperLeftArm, lowerLeftArm, upperLeftArm.Position + new Vector3(-.7f, 0, 0)));
            angularMotor = new AngularMotor(upperLeftArm, lowerLeftArm);
            angularMotor.Settings.MaximumForce = 150;
            Space.Add(angularMotor);

            //Make the second arm.
            var upperRightArm = new Box(body.Position + new Vector3(1.6f, .8f, 0), 1, .5f, .5f, 5);
            Space.Add(upperRightArm);

            var lowerRightArm = new Box(upperRightArm.Position + new Vector3(1.4f, 0, 0), 1, .5f, .5f, 5);
            Space.Add(lowerRightArm);

            //Connect the body to the upper arm.
            Space.Add(new BallSocketJoint(body, upperRightArm, upperRightArm.Position + new Vector3(-.7f, 0, 0)));
            //Angular motors can be used to simulate friction when their goal velocity is 0.
            angularMotor = new AngularMotor(body, upperRightArm);
            angularMotor.Settings.MaximumForce = 250; //The maximum force of 'friction' in this joint.
            Space.Add(angularMotor);


            //Connect the upper arm to the lower arm.
            Space.Add(new BallSocketJoint(upperRightArm, lowerRightArm, upperRightArm.Position + new Vector3(.7f, 0, 0)));
            angularMotor = new AngularMotor(upperRightArm, lowerRightArm);
            angularMotor.Settings.MaximumForce = 150;
            Space.Add(angularMotor);

            //Make the first leg.
            var upperLeftLeg = new Box(body.Position + new Vector3(-.6f, -2.1f, 0), .5f, 1.3f, .5f, 8);
            Space.Add(upperLeftLeg);

            var lowerLeftLeg = new Box(upperLeftLeg.Position + new Vector3(0, -1.7f, 0), .5f, 1.3f, .5f, 8);
            Space.Add(lowerLeftLeg);

            //Connect the body to the upper arm.
            Space.Add(new BallSocketJoint(body, upperLeftLeg, upperLeftLeg.Position + new Vector3(0, .9f, 0)));
            //Angular motors can be used to simulate friction when their goal velocity is 0.
            angularMotor = new AngularMotor(body, upperLeftLeg);
            angularMotor.Settings.MaximumForce = 350; //The maximum force of 'friction' in this joint.
            Space.Add(angularMotor);


            //Connect the upper arm to the lower arm.
            Space.Add(new BallSocketJoint(upperLeftLeg, lowerLeftLeg, upperLeftLeg.Position + new Vector3(0, -.9f, 0)));
            angularMotor = new AngularMotor(upperLeftLeg, lowerLeftLeg);
            angularMotor.Settings.MaximumForce = 250;
            Space.Add(angularMotor);

            //Make the second leg.
            var upperRightLeg = new Box(body.Position + new Vector3(.6f, -2.1f, 0), .5f, 1.3f, .5f, 8);
            Space.Add(upperRightLeg);

            var lowerRightLeg = new Box(upperRightLeg.Position + new Vector3(0, -1.7f, 0), .5f, 1.3f, .5f, 8);
            Space.Add(lowerRightLeg);

            //Connect the body to the upper arm.
            Space.Add(new BallSocketJoint(body, upperRightLeg, upperRightLeg.Position + new Vector3(0, .9f, 0)));
            //Angular motors can be used to simulate friction when their goal velocity is 0.
            angularMotor = new AngularMotor(body, upperRightLeg);
            angularMotor.Settings.MaximumForce = 350; //The maximum force of 'friction' in this joint.
            Space.Add(angularMotor);


            //Connect the upper arm to the lower arm.
            Space.Add(new BallSocketJoint(upperRightLeg, lowerRightLeg, upperRightLeg.Position + new Vector3(0, -.9f, 0)));
            angularMotor = new AngularMotor(upperRightLeg, lowerRightLeg);
            angularMotor.Settings.MaximumForce = 250;
            Space.Add(angularMotor);


            //IK version!
            Bone bodyBone = new Bone(body.Position, Quaternion.Identity, .75f, 2);
            Bone headBone = new Bone(head.Position, Quaternion.Identity, .4f, .8f);
            Bone upperLeftArmBone = new Bone(upperLeftArm.Position, Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), MathHelper.PiOver2), .25f, 1);
            Bone lowerLeftArmBone = new Bone(lowerLeftArm.Position, Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), MathHelper.PiOver2), .25f, 1);
            Bone upperRightArmBone = new Bone(upperRightArm.Position, Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), MathHelper.PiOver2), .25f, 1);
            Bone lowerRightArmBone = new Bone(lowerRightArm.Position, Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), MathHelper.PiOver2), .25f, 1);
            Bone upperLeftLegBone = new Bone(upperLeftLeg.Position, Quaternion.Identity, .25f, 1.3f);
            Bone lowerLeftLegBone = new Bone(lowerLeftLeg.Position, Quaternion.Identity, .25f, 1.3f);
            Bone upperRightLegBone = new Bone(upperRightLeg.Position, Quaternion.Identity, .25f, 1.3f);
            Bone lowerRightLegBone = new Bone(lowerRightLeg.Position, Quaternion.Identity, .25f, 1.3f);

            bones.Add(new BoneRelationship(bodyBone, body));
            bones.Add(new BoneRelationship(headBone, head));
            bones.Add(new BoneRelationship(upperLeftArmBone, upperLeftArm));
            bones.Add(new BoneRelationship(lowerLeftArmBone, lowerLeftArm));
            bones.Add(new BoneRelationship(upperRightArmBone, upperRightArm));
            bones.Add(new BoneRelationship(lowerRightArmBone, lowerRightArm));
            bones.Add(new BoneRelationship(upperLeftLegBone, upperLeftLeg));
            bones.Add(new BoneRelationship(lowerLeftLegBone, lowerLeftLeg));
            bones.Add(new BoneRelationship(upperRightLegBone, upperRightLeg));
            bones.Add(new BoneRelationship(lowerRightLegBone, lowerRightLeg));

            //[We don't care about the return values here. A bit weird, but the constructor puts the reference where it needs to go.]
            new IKBallSocketJoint(bodyBone, headBone, headBone.Position + new Vector3(0, -.9f, 0));

            new IKBallSocketJoint(bodyBone, upperLeftArmBone, upperLeftArmBone.Position + new Vector3(.7f, 0, 0));
            new IKBallSocketJoint(upperLeftArmBone, lowerLeftArmBone, upperLeftArmBone.Position + new Vector3(-.7f, 0, 0));

            new IKBallSocketJoint(bodyBone, upperRightArmBone, upperRightArmBone.Position + new Vector3(-.7f, 0, 0));
            new IKBallSocketJoint(upperRightArmBone, lowerRightArmBone, upperRightArmBone.Position + new Vector3(.7f, 0, 0));

            new IKBallSocketJoint(bodyBone, upperLeftLegBone, upperLeftLegBone.Position + new Vector3(0, .9f, 0));
            new IKBallSocketJoint(upperLeftLegBone, lowerLeftLegBone, upperLeftLegBone.Position + new Vector3(0, -.9f, 0));

            new IKBallSocketJoint(bodyBone, upperRightLegBone, upperRightLegBone.Position + new Vector3(0, .9f, 0));
            new IKBallSocketJoint(upperRightLegBone, lowerRightLegBone, upperRightLegBone.Position + new Vector3(0, -.9f, 0));



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
                        var ikJoint = new IKBallSocketJoint(boneA.Bone, boneB.Bone, anchor);
                    }
                    if (j > 0)
                    {
                        var boneA = boneMesh[i, j];
                        var boneB = boneMesh[i, j - 1];
                        var anchor = (boneA.Entity.Position + boneB.Entity.Position) / 2;
                        var dynamicsJoint = new BallSocketJoint(boneA.Entity, boneB.Entity, anchor);
                        CollisionRules.AddRule(boneA.Entity, boneB.Entity, CollisionRule.NoBroadPhase);
                        Space.Add(dynamicsJoint);
                        var ikJoint = new IKBallSocketJoint(boneA.Bone, boneB.Bone, anchor);
                    }
                    if (i > 0 && j > 0)
                    {
                        var boneA = boneMesh[i, j];
                        var boneB = boneMesh[i - 1, j - 1];
                        var anchor = (boneA.Entity.Position + boneB.Entity.Position) / 2;
                        var dynamicsJoint = new BallSocketJoint(boneA.Entity, boneB.Entity, anchor);
                        CollisionRules.AddRule(boneA.Entity, boneB.Entity, CollisionRule.NoBroadPhase);
                        Space.Add(dynamicsJoint);
                        var ikJoint = new IKBallSocketJoint(boneA.Bone, boneB.Bone, anchor);
                    }

                    if (i < widthCount - 1 && j > 0)
                    {
                        var boneA = boneMesh[i, j];
                        var boneB = boneMesh[i + 1, j - 1];
                        var anchor = (boneA.Entity.Position + boneB.Entity.Position) / 2;
                        var dynamicsJoint = new BallSocketJoint(boneA.Entity, boneB.Entity, anchor);
                        CollisionRules.AddRule(boneA.Entity, boneB.Entity, CollisionRule.NoBroadPhase);
                        Space.Add(dynamicsJoint);
                        var ikJoint = new IKBallSocketJoint(boneA.Bone, boneB.Bone, anchor);
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
                var ikJoint = new IKBallSocketJoint(bone, previous.Bone, anchor);
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
                var ikJoint = new IKBallSocketJoint(bone, previous.Bone, anchor);
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
                var ikJoint = new IKBallSocketJoint(bone, previous.Bone, anchor);
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
                var ikJoint = new IKBallSocketJoint(bone, previous.Bone, anchor);
                previous = new BoneRelationship(bone, entity);
                bones.Add(previous);
            }

        }

        void BuildJointTest(Vector3 position)
        {
            ////DISTANCE JOINT

            //Bone a, b;
            //a = new Bone(new Vector3(0, 5, 0), Quaternion.Identity, .5f, 1);
            //b = new Bone(new Vector3(0, 7, 0), Quaternion.Identity, .5f, 1);
            //var ikDistanceJoint = new IKDistanceJoint(a, b, a.Position + new Vector3(0, .5f, 0), b.Position - new Vector3(0, .5f, 0));

            //var entityA = new Cylinder(a.Position, 1, 0.5f, 10);
            //var entityB = new Cylinder(b.Position, 1, 0.5f, 10);
            //var distanceJoint = new DistanceJoint(entityA, entityB, ikDistanceJoint.AnchorA, ikDistanceJoint.AnchorB);
            //Space.Add(entityA);
            //Space.Add(entityB);
            //Space.Add(distanceJoint);
            //bones.Add(new BoneRelationship(a, entityA));
            //bones.Add(new BoneRelationship(b, entityB));

            //SWING LIMIT
            //solver.FixerIterationCount = 0;
            //solver.ControlIterationCount = 1;

            Bone a, b;
            a = new Bone(new Vector3(0, 5, 0), Quaternion.Identity, .5f, 1);
            b = new Bone(new Vector3(0, 7, 0), Quaternion.Identity, .5f, 1);
            var ikJoint = new IKBallSocketJoint(a, b, (a.Position + b.Position) * 0.5f);
            var ikLimit = new IKSwingLimit(a, b, Vector3.Up, Vector3.Up, MathHelper.PiOver2);
            var ikRevolute = new IKRevoluteJoint(a, b, Vector3.Right);

            var entityA = new Cylinder(a.Position, 1, 0.5f, 10);
            var entityB = new Cylinder(b.Position, 1, 0.5f, 10);
            var joint = new BallSocketJoint(entityA, entityB, (a.Position + b.Position) * 0.5f);
            var limit = new SwingLimit(entityA, entityB, Vector3.Up, Vector3.Up, MathHelper.PiOver2);
            var revolute = new RevoluteAngularJoint(entityA, entityB, Vector3.Right);
            Space.Add(entityA);
            Space.Add(entityB);
            Space.Add(joint);
            Space.Add(limit);
            Space.Add(revolute);
            bones.Add(new BoneRelationship(a, entityA));
            bones.Add(new BoneRelationship(b, entityB));
        }

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public InverseKinematicsTestDemo(DemosGame game)
            : base(game)
        {
            whitePixel = game.Content.Load<Texture2D>("whitePixel");
            game.Camera.Position = new Vector3(0, 3, 5);
            Box ground = new Box(new Vector3(0, 0, 0), 30, 1, 30);
            Space.Add(ground);
            Space.ForceUpdater.Gravity = new Vector3(0, -9.81f, 0);

            drawer = new InstancedModelDrawer(game);

            solver.ActiveSet.UseAutomass = true;
            solver.AutoscaleControlImpulses = true;
            solver.AutoscaleControlMaximumForce = 7;

            BuildChain(new Vector3(-5, 2, 0));

            BuildActionFigure(new Vector3(5, 5, -8));
            BuildActionFigure(new Vector3(5, 5, -3));
            BuildActionFigure(new Vector3(5, 5, 3));
            BuildActionFigure(new Vector3(5, 5, 8));

            BuildCyclicMesh(new Vector3(-5, 5, -5));


            BuildJointTest(new Vector3(0, 5, 0));




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
                //Manage bone grabbery.
                if (Game.MouseInput.RightButton == ButtonState.Pressed)
                {
                    BoneRelationship hitBone;
                    Vector3 hitPosition;
                    if ((currentControl == null || !currentControl.IsActive) && //Don't bother raycasting if we are holding a bone.
                        RayCastBones(new Ray(Game.Camera.Position, Game.Camera.WorldMatrix.Forward), out hitBone, out hitPosition) &&
                        !hitBone.Bone.Pinned) //Can't control pinned bones.
                    {
                        //We grabbed a bone!
                        if (Game.KeyboardInput.IsKeyDown(Keys.LeftShift))
                        {
                            //Use a full linear+angular state controller.
                            stateControl.TargetBone = hitBone.Bone;
                            solver.Add(stateControl);
                            grabOffset = hitPosition - hitBone.Bone.Position;
                            stateControl.AngularMotor.TargetOrientation = hitBone.Bone.Orientation;
                            currentControl = stateControl;

                        }
                        else
                        {
                            //Use a point-grab-puller-controller thing.
                            dragControl.TargetBone = hitBone.Bone;
                            solver.Add(dragControl);
                            dragControl.LinearMotor.Offset = hitPosition - hitBone.Bone.Position;
                            currentControl = dragControl;

                        }

                        distanceToGrabbedBone = Vector3.Dot(Game.Camera.WorldMatrix.Forward, hitPosition - Game.Camera.Position);
                    }

                    if (currentControl != null && currentControl.IsActive)
                    {
                        //If the mouse control is active, then update the mouse control's goal location.
                        if (currentControl == dragControl)
                            dragControl.LinearMotor.TargetPosition = Game.Camera.Position + Game.Camera.WorldMatrix.Forward * distanceToGrabbedBone;
                        else
                            stateControl.LinearMotor.TargetPosition = Game.Camera.Position + Game.Camera.WorldMatrix.Forward * distanceToGrabbedBone - grabOffset;


                        //Solve for the new bone positions and orientations.
                        long start = Stopwatch.GetTimestamp();
                        solver.Solve();
                        long end = Stopwatch.GetTimestamp();
                        elapsedTime = (end - start) / (double)Stopwatch.Frequency;


                    }
                }
                else
                {
                    elapsedTime = 0;
                    if (currentControl != null)
                        solver.Remove(currentControl); //Disable the control if it's active.

                    if (Game.MouseInput.LeftButton == ButtonState.Pressed && Game.PreviousMouseInput.LeftButton == ButtonState.Released)
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
                        bones[i].DisplayBone.Color = new Vector3(0, 0, 1);
                    else
                    {
                        bones[i].DisplayBone.Color = new Vector3(bones[i].Bone.Mass / 2, 0, 0);
                    }
                }


            }
            else
                base.Update(dt);
        }




        public override void DrawUI()
        {
            Game.DataTextDrawer.Draw("IK Controls:", new Vector2(970, 20));
            Game.TinyTextDrawer.Draw(" T: Toggle Dynamics/IK", new Vector2(970, 43));
            Game.TinyTextDrawer.Draw(" Right click: Drag", new Vector2(970, 57));
            Game.TinyTextDrawer.Draw(" Left shift + Right click: Move", new Vector2(970, 71));
            Game.TinyTextDrawer.Draw(" Left click: Pin/unpin bone", new Vector2(970, 85));

            Game.DataTextDrawer.Draw("Current mode: ", new Vector2(970, 113));
            if (usingIK)
                Game.DataTextDrawer.Draw("IK", new Vector2(1110, 113));
            else
                Game.DataTextDrawer.Draw("Dynamics", new Vector2(1110, 113));

            if (usingIK)
                Game.TinyTextDrawer.Draw(" Solving time (ms): ", elapsedTime * 1000, 2, new Vector2(970, 141));
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
            Vector3.Transform(ref localRay.Position, ref conjugate, out localRay.Position);
            Vector3.Transform(ref ray.Direction, ref conjugate, out localRay.Direction);

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
                Vector3.Transform(ref hit.Normal, ref transform.Orientation, out hit.Normal);
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
                Vector3.Transform(ref hit.Normal, ref transform.Orientation, out hit.Normal);
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
                Vector3.Transform(ref Toolbox.UpVector, ref transform.Orientation, out hit.Normal);
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
                Vector3.Transform(ref Toolbox.DownVector, ref transform.Orientation, out hit.Normal);
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