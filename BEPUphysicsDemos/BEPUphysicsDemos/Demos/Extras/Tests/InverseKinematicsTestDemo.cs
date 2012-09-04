using System;
using System.Collections.Generic;
using BEPUphysics;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.Constraints.TwoEntity.Joints;
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
        }

        private InstancedModelDrawer drawer;
        private Texture2D whitePixel;

        private List<BoneRelationship> bones = new List<BoneRelationship>();

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

            #region Chain
            //Set up a bone chain.
            int linkCount = 100;
            var previousBoneEntity = new Cylinder(new Vector3(0, 2, 0), 1, .2f, 10);
            var previousBone = new Bone(previousBoneEntity.Position, previousBoneEntity.Orientation, previousBoneEntity.Radius, previousBoneEntity.Height, previousBoneEntity.Mass);
            bones.Add(new BoneRelationship { Bone = previousBone, Entity = previousBoneEntity });
            Space.Add(previousBoneEntity);

            for (int i = 1; i < linkCount; i++)
            {
                var boneEntity = new Cylinder(new Vector3(0, previousBoneEntity.Position.Y + 1, 0), 1, .2f, 10);
                var bone = new Bone(boneEntity.Position, boneEntity.Orientation, boneEntity.Radius, boneEntity.Height, boneEntity.Mass);
                bones.Add(new BoneRelationship { Bone = bone, Entity = boneEntity });
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
            #endregion



            //Create the display objects.
            Model cylinder = game.Content.Load<Model>("cylinder");
            for (int i = 0; i < bones.Count; i++)
            {
                var displayBone = new DisplayModel(cylinder, drawer);
                bones[i] = new BoneRelationship { Bone = bones[i].Bone, Entity = bones[i].Entity, DisplayBone = displayBone };
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
                        bone.Bone.Orientation = bone.Entity.Orientation;
                    }
                }
                else
                {
                    foreach (var bone in bones)
                    {
                        bone.Entity.Position = bone.Bone.Position;
                        bone.Entity.Orientation = bone.Bone.Orientation;
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

                            stateControl.LinearMotor.MaximumImpulse = hitBone.Bone.Mass * 5f / (solver.FixerIterationCount + solver.ControlIterationCount);
                            stateControl.AngularMotor.MaximumImpulse = hitBone.Bone.Mass * 5f / (solver.FixerIterationCount + solver.ControlIterationCount);
                        }
                        else
                        {
                            //Use a point-grab-puller-controller thing.
                            dragControl.TargetBone = hitBone.Bone;
                            solver.Add(dragControl);
                            dragControl.LinearMotor.Offset = hitPosition - hitBone.Bone.Position;
                            currentControl = dragControl;

                            dragControl.LinearMotor.MaximumImpulse = hitBone.Bone.Mass * 5f / (solver.FixerIterationCount + solver.ControlIterationCount);
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
                            if (hitBone.Bone.Pinned)
                                hitBone.DisplayBone.Color = new Vector3(0, 0, 1);
                            else
                                hitBone.DisplayBone.Color = new Vector3(0, 0, 0);
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
            Game.TinyTextDrawer.Draw(" Right click + Left shift: Move", new Vector2(970, 71));
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