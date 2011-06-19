using BEPUphysics.Collidables;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.PositionUpdating;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;
using System.Diagnostics;
using System.Collections.Generic;
using System;
using BEPUphysics.MathExtensions;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.DataStructures;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.CollisionShapes;
using BEPUphysics.Settings;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Constraints;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using BEPUphysics.Constraints.SolverGroups;
using BEPUphysics.Constraints.TwoEntity.Motors;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class MobileMeshDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public MobileMeshDemo(DemosGame game)
            : base(game)
        {


            ShapeDistributionInformation info;
            Vector3[] vertices;
            int[] indices;

            TriangleMesh.GetVerticesAndIndicesFromModel(game.Content.Load<Model>("playground"), out vertices, out indices);
            AffineTransform transform = new AffineTransform(new Vector3(1, 1, 1), Quaternion.Identity, new Vector3(0, -30, 0));
            //InstancedMesh mesh = new InstancedMesh(new InstancedMeshShape(vertices, indices), transform);
            //StaticMesh mesh = new StaticMesh(vertices, indices, transform);
            //mesh.Sidedness = TriangleSidedness.Counterclockwise;
            Entity mesh = new Entity(new MobileMeshShape(vertices, indices, transform, MobileMeshSolidity.Counterclockwise, out info));

            Space.Add(mesh);
            //game.ModelDrawer.Add(mesh);

            //var transform = new AffineTransform(new Vector3(5, 1, 5), Quaternion.Identity, new Vector3(0, -30, 0));
            //float[,] heights = new float[128, 128];
            //for (int i = 0; i < 128; i++)
            //{
            //    for (int j = 0; j < 128; j++)
            //    {
            //        heights[i, j] = 5 * (float)(Math.Sin(i / 5f) + Math.Cos(j / 5f));
            //    }

            //}
            //Terrain terrain = new Terrain(heights, transform);
            //terrain.Thickness = 1000;
            //Space.Add(terrain);
            //game.ModelDrawer.Add(terrain);

            TriangleMesh.GetVerticesAndIndicesFromModel(game.Content.Load<Model>("tube"), out vertices, out indices);
            //MotionSettings.DefaultPositionUpdateMode = PositionUpdateMode.Continuous;
            //transform = AffineTransform.Identity;// new AffineTransform(new Vector3(1, 1, 1), Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathHelper.Pi), new Vector3(0, 0, 0));
            //transform = new AffineTransform(new Vector3(.02f, .02f, .02f), Quaternion.Identity, new Vector3(0, 0, 0));
            transform = new AffineTransform(new Vector3(1f, 1f, 1f), Quaternion.Identity, new Vector3(0, 0, 0));
            var shape = new MobileMeshShape(vertices, indices, transform, MobileMeshSolidity.Solid, out info);
            //shape.Sidedness = TriangleSidedness.Counterclockwise;

            //Space.ForceUpdater.Gravity = new Vector3();

            Space.Remove(kapow);
            kapow = new Sphere(new Vector3(10000, 0, 0), .4f, 1);
            Space.Add(kapow);
            Matrix3X3 inertia;
            float mass = 10;
            //inertia = new Matrix3X3();
            //inertia.M11 = mass;
            //inertia.M22 = mass;
            //inertia.M33 = mass;
            Matrix3X3.Multiply(ref info.VolumeDistribution, mass * InertiaHelper.InertiaTensorScale, out inertia);
            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    var entityMesh = new Entity(shape, mass, inertia, info.Volume);
                    //entityMesh.CollisionInformation.ImproveBoundaryBehavior = true;
                    entityMesh.IsAlwaysActive = true;
                    entityMesh.Position = new Vector3(i * 25, 5000, j * 25);
                    entityMesh.LinearVelocity = new Vector3(0, -5000, 0);
                    entityMesh.PositionUpdateMode = PositionUpdateMode.Continuous;
                    //CollisionRules.AddRule(entityMesh, kapow, CollisionRule.NoSolver);
                    //entityMesh.AngularVelocity = new Vector3(5, 0, 0);
                    //entityMesh.LinearVelocity = new Vector3(1, 0, 0);
                    //entityMesh.Material.KineticFriction = 0;
                    //entityMesh.Material.StaticFriction = 0;
                    //entityMesh.LocalInertiaTensorInverse = new Matrix3X3();
                    Space.Add(entityMesh);
                    //entityMesh.IsAlwaysActive = true;
                }
            }
            for (int j = 0; j < 0; j++)
            {
                var toAdd = new Box(new Vector3(0, 35 + j * -.1f, 0), 2, 2, 2, 1);
                //toAdd.Material.KineticFriction = 0;
                //toAdd.Material.StaticFriction = 0;
                Space.Add(toAdd);
            }

            //Space.Add(new Box(new Vector3(0, -10, 0), 1, 1, 1));
            game.Camera.Position = new Vector3(0, -10, 5);
            game.Camera.Yaw = 0;
            game.Camera.Pitch = 0;


        }


        public override void Update(float dt)
        {
            base.Update(dt);
        }


        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "MobileMesh"; }
        }
    }
}