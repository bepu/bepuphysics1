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
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUphysics.CollisionTests;
using BEPUphysics;
using BEPUphysics.EntityStateManagement;
using BEPUphysics.ResourceManagement;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class TestDemo4 : StandardDemo
    {
        void MakeCar(Vector3 basePosition)
        {
            var compound = new CompoundBody(
                    new List<CompoundShapeEntry>()
                            {
                                new CompoundShapeEntry(new BoxShape(.9f,.9f,.9f), basePosition + new Vector3(-1,0,0)),
                                new CompoundShapeEntry( new BoxShape(.9f,.9f,.9f),basePosition +  new Vector3(1,0,0))
                            }, 10
            );

            Matrix3X3 wheelOrientation = Matrix3X3.CreateFromAxisAngle(Vector3.Right, MathHelper.PiOver2);

            Cylinder wheel1 = new Cylinder(basePosition + new Vector3(-1, 0, 1), .2f, .5f, 3);
            wheel1.OrientationMatrix = wheelOrientation;
            Cylinder wheel2 = new Cylinder(basePosition + new Vector3(-1, 0, -1), .2f, .5f, 3);
            wheel2.OrientationMatrix = wheelOrientation;
            Cylinder wheel3 = new Cylinder(basePosition + new Vector3(1, 0, 1), .2f, .5f, 3);
            wheel3.OrientationMatrix = wheelOrientation;
            Cylinder wheel4 = new Cylinder(basePosition + new Vector3(1, 0, -1), .2f, .5f, 3);
            wheel4.OrientationMatrix = wheelOrientation;

            RevoluteJoint revoluteJoint1 = new RevoluteJoint(wheel1, compound, basePosition + new Vector3(-1, 0, 0), Vector3.Forward);
            RevoluteJoint revoluteJoint2 = new RevoluteJoint(wheel2, compound, basePosition + new Vector3(-1, 0, 0), Vector3.Forward);
            RevoluteJoint revoluteJoint3 = new RevoluteJoint(wheel3, compound, basePosition + new Vector3(1, 0, 0), Vector3.Forward);
            RevoluteJoint revoluteJoint4 = new RevoluteJoint(wheel4, compound, basePosition + new Vector3(1, 0, 0), Vector3.Forward);

            Space.Add(wheel1);
            Space.Add(wheel2);
            Space.Add(wheel3);
            Space.Add(wheel4);

            Space.Add(revoluteJoint1);
            Space.Add(revoluteJoint2);
            Space.Add(revoluteJoint3);
            Space.Add(revoluteJoint4);

            Space.Add(compound);
        }

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public TestDemo4(DemosGame game)
            : base(game)
        {
            Vector3[] vertices = new Vector3[] { new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 0, 1), new Vector3(1, 0, 1), new Vector3(0, 0, 2), new Vector3(1, 0, 2) };
            int[] indices = new int[] { 0, 1, 2, 
                                        1, 3, 2, 
                                        2, 3, 4, 
                                        3, 5, 4 };
            StaticMesh mesh = new StaticMesh(vertices, indices, new AffineTransform(new Vector3(5, 5, 5), Quaternion.Identity, new Vector3(0, 0, 0)));
            mesh.Sidedness = TriangleSidedness.Counterclockwise;
            mesh.ImproveBoundaryBehavior = false;
            game.ModelDrawer.Add(mesh.Mesh);
            Space.Add(mesh);

            var box = new Box(new Vector3(0, 10, 0), 1, 1, 1, 1);
            box.Material.KineticFriction = 0;
            box.Material.StaticFriction = 0;
            mesh.Material.KineticFriction = 0;
            mesh.Material.StaticFriction = 0;
            Space.Add(box);



            Space.Add(new Box(new Vector3(0, -5, 0), 10, 1, 10));

        }






        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Test4"; }
        }

    }
}