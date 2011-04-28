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

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class TestDemo2 : StandardDemo
    {


        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public TestDemo2(DemosGame game)
            : base(game)
        {
            var shapeA = new BoxShape(1, 1, 1);
            shapeA.CollisionMargin = 0;
            var shapeB = new BoxShape(1, 1, 1);
            shapeB.CollisionMargin = 0;

            var transformA = new RigidTransform(new Vector3(0,0,0));
            var transformB = new RigidTransform(new Vector3(.5f, .5f,0));
            Vector3 overlap;
            bool overlapped = MPRTesting.GetLocalOverlapPosition(shapeA, shapeB, ref transformB, out overlap);
            Vector3 normal;
            float depth;
            Vector3 direction = Vector3.Up + Vector3.Right;
            MPRTesting.LocalSurfaceCast(shapeA, shapeB, ref transformB, ref direction, out depth, out normal);

            ContactData contactData;
            bool overlappedOld = MPRToolbox.AreObjectsColliding(shapeA, shapeB, ref transformA, ref transformB, out contactData);

            //Random rand = new Random(0);
            //for (int i = 0; i < 10000000; i++)
            //{
            //    transformA = new RigidTransform(new Vector3((float)rand.NextDouble() * 10 - 5, (float)rand.NextDouble() * 10 - 5, (float)rand.NextDouble() * 10 - 5),
            //        Quaternion.CreateFromYawPitchRoll((float)rand.NextDouble() * 1000, (float)rand.NextDouble() * 1000, (float)rand.NextDouble() * 1000));
            //    transformB = new RigidTransform(new Vector3((float)rand.NextDouble() * 10 - 5, (float)rand.NextDouble() * 10 - 5, (float)rand.NextDouble() * 10 - 5),
            //        Quaternion.CreateFromYawPitchRoll((float)rand.NextDouble() * 1000, (float)rand.NextDouble() * 1000, (float)rand.NextDouble() * 1000));

            //    overlapped = MPRTesting.GetOverlapPosition(shapeA, shapeB, ref transformA, ref transformB, out overlap);

            //    overlappedOld = MPRToolbox.AreObjectsColliding(shapeA, shapeB, ref transformA, ref transformB, out contactData);

            //    if (overlapped && !overlappedOld &&
            //        (!MPRToolbox.IsPointInsideShape(ref overlap, shapeA, ref transformA) ||
            //        !MPRToolbox.IsPointInsideShape(ref overlap, shapeB, ref transformB)))
            //        Debug.WriteLine("Break.");
            //    if (overlappedOld && !overlapped &&
            //        (!MPRToolbox.IsPointInsideShape(ref contactData.Position, shapeA, ref transformA) ||
            //        !MPRToolbox.IsPointInsideShape(ref contactData.Position, shapeB, ref transformB)))
            //        Debug.WriteLine("Break.");
            //    if (overlapped && overlappedOld &&
            //        (!MPRToolbox.IsPointInsideShape(ref overlap, shapeA, ref transformA) ||
            //        !MPRToolbox.IsPointInsideShape(ref overlap, shapeB, ref transformB) ||
            //        !MPRToolbox.IsPointInsideShape(ref contactData.Position, shapeA, ref transformA) ||
            //        !MPRToolbox.IsPointInsideShape(ref contactData.Position, shapeB, ref transformB)))
            //        Debug.WriteLine("Break.");
            //}

        }







        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Test2"; }
        }

    }
}