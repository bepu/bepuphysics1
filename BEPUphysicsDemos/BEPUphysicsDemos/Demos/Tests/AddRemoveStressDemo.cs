using BEPUphysics.Entities.Prefabs;
using System;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using System.Collections.Generic;
using BEPUphysics.NarrowPhaseSystems;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.Demos.Tests
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class AddRemoveStressDemo : StandardDemo
    {


        float width = 15;
        float height = 15;
        float length = 15;
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public AddRemoveStressDemo(DemosGame game)
            : base(game)
        {

            NarrowPhaseHelper.Factories.BoxBox.EnsureCount(5000);
            NarrowPhaseHelper.Factories.CompoundCompound.EnsureCount(5000);
            Space.Remove(vehicle.Vehicle);


            for (int i = 0; i < 1000; i++)
            {
                var toAdd =
                    new CompoundBody(new List<CompoundShapeEntry>() 
                    {
                        new CompoundShapeEntry(new BoxShape(1,1,1), 
                        new Vector3(
                            (float)random.NextDouble() * width,
                            (float)random.NextDouble() * height,
                            (float)random.NextDouble() * length), 1)
                    }, 10);
                toAdd.IsAffectedByGravity = false;
                Space.Add(toAdd);

            }

            //Box ground = new Box(new Vector3(0, -.5f, 0), 50, 1, 50);
            //Space.Add(ground);
            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(0, 6, 15);
        }

        Random random = new Random();
        public override void Update(float dt)
        {
            for (int i = 0; i < 20; i++)
            {
                var entity = Space.Entities[random.Next(Space.Entities.Count)];
                Space.Remove(entity);
                entity.Position = new Vector3(
                            (float)random.NextDouble() * width,
                            (float)random.NextDouble() * height,
                            (float)random.NextDouble() * length);
                Space.Add(entity);
            }
            base.Update(dt);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Wall"; }
        }
    }
}