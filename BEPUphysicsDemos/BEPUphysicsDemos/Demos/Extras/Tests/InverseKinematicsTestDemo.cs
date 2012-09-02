using System;
using System.Collections.Generic;
using BEPUphysics;
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
        private InstancedModelDrawer drawer;

        private struct BoneRelationship
        {
            public Bone Bone;
            public DisplayModel DisplayBone;
            public Entity Entity;
        }

        private List<BoneRelationship> bones = new List<BoneRelationship>();

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public InverseKinematicsTestDemo(DemosGame game)
            : base(game)
        {
            Box ground = new Box(new Vector3(0, 0, 0), 30, 1, 30);
            Space.Add(ground);

            drawer = new InstancedModelDrawer(game);

            //Set up the 

            //Create the display
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
            }
            if (usingIK)
            {
                //Try to grab a bone.
                if (Game.MouseInput.RightButton == ButtonState.Pressed)
                {
                    Bone hitBone;
                    Vector3 hitPosition;
                    if(RayCastBones(new Ray(Game.Camera.Position, Game.Camera.WorldMatrix.Forward), out hitBone, out hitPosition))
                    {
                        //We grabbed a bone!
                    }
                }
                //Solve for the new bone positions and orientations.
                solver.Solve();

                //Update the positions of the bone graphics.
                for (int i = 0; i < bones.Count; i++)
                {
                    Matrix transform;
                    Matrix.CreateFromQuaternion(ref bones[i].Bone.Orientation, out transform);
                    transform.Translation = bones[i].Bone.Position;
                    bones[i].DisplayBone.WorldTransform = transform;
                }
            }
            else
                base.Update(dt);
        }

        private bool RayCastBones(Ray ray, out Bone hitBone, out Vector3 hitPosition)
        {
            float t = float.MaxValue;
            hitBone = null;
            hitPosition = new Vector3();
            for (int i = 0; i < bones.Count; i++)
            {
                var bone = bones[i].Bone;
                RayHit hit;
                if (RayCast(bone, ray, out hit) && hit.T < t)
                {
                    t = hit.T;
                    hitPosition = hit.Location;
                    hitBone = bone;
                }
            }
            return t < float.MaxValue;
        }

        bool RayCast(Bone bone, Ray ray, out RayHit hit)
        {
            hit = new RayHit();
            return false;
        }


        public override void DrawUI()
        {
            Game.DataTextDrawer.Draw("IK Controls:", new Vector2(1000, 20));
            Game.TinyTextDrawer.Draw(" T: Toggle Dynamics/IK", new Vector2(1000, 43));
            Game.TinyTextDrawer.Draw(" Right click: Grab and pull", new Vector2(1000, 57));
            Game.TinyTextDrawer.Draw(" W and move mouse: Rotate grabbed bone", new Vector2(1000, 71));
            Game.TinyTextDrawer.Draw(" Left click: Pin/unpin bone", new Vector2(1000, 85));

            Game.DataTextDrawer.Draw("Current mode: ", new Vector2(1000, 113));
            if (usingIK)
                Game.DataTextDrawer.Draw("IK", new Vector2(1140, 113));
            else
                Game.DataTextDrawer.Draw("Dynamics", new Vector2(1140, 113));

            base.DrawUI();
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
    }
}