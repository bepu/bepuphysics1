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
using BEPUphysics.Materials;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class TestDemo4 : StandardDemo
    {

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public TestDemo4(DemosGame game)
            : base(game)
        {
            Terrain terrain = new Terrain(new float[128, 128], 
                new AffineTransform(new Vector3(1, 5, 1), Quaternion.Identity, new Vector3()));
            Space.Add(terrain);
            game.ModelDrawer.Add(terrain);
            Material m = new Material();
            MaterialManager.MaterialInteractions.Add(new MaterialPair(m, terrain.Material), new InteractionProperties() { StaticFriction = 0, KineticFriction = 0, Bounciness = 0 });
            for (int i = 0; i < 100; i++)
            {
                var c = new Cylinder(new Vector3(0, 3, 1 + i * 2), .2f, .35f, 1);
                //c.CollisionInformation.Shape.CollisionMargin = .19f;
                c.LinearVelocity = new Vector3(7 + i * .5f, 0, 0);
                c.Material = m;
                Space.Add(c);
            }
            game.Camera.Position = new Vector3(10, 10, 30);
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