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


        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public TestDemo4(DemosGame game)
            : base(game)
        {
            Box a, b;
            b = new Box(new Vector3(4, 10, 5), 1, 1, 1, 1);
            a = new Box(new Vector3(4, 10.2f, 7), 1, 1, 1);

            SwivelHingeJoint joint = new SwivelHingeJoint(a, b, new Vector3(4, 11.975f, 7), new Vector3(0, -1, 0));
            Space.Add(a);
            Space.Add(b);
            Space.Add(joint);

            Space.Add(new Box(new Vector3(0, 0, 0), 10, 1, 10));

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