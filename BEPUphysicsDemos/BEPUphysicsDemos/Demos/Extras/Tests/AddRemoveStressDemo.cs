using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using System;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using System.Collections.Generic;
using BEPUphysics.NarrowPhaseSystems;
using BEPUphysics.Threading;
using BEPUutilities;
using BEPUutilities.DataStructures;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class AddRemoveStressDemo : StandardDemo
    {


        float width = 15f;
        float height = 15f;
        float length = 15f;
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

                var position = new Vector3(
                    (float)(random.NextDouble() - 0.5) * width,
                    (float)(random.NextDouble() - 0.5) * height,
                    (float)(random.NextDouble() - 0.5) * length);
                var toAdd =
                    new CompoundBody(new List<CompoundShapeEntry>
                    {
                        new CompoundShapeEntry(new BoxShape(1,1,1), position, 1)
                    }, 10);
                //var toAdd = new Box(position, 1, 1, 1, 1);
                toAdd.IsAffectedByGravity = false;
                toAdd.LinearVelocity = 3 * Vector3.Normalize(toAdd.Position);
                Space.Add(toAdd);
                addedEntities.Add(toAdd);

            }

            //Box ground = new Box(new Vector3(0, -.5f, 0), 50, 1, 50);
            //Space.Add(ground);
            game.Camera.Position = new Vector3(0, 6, 15);
        }

        Random random = new Random();
        private RawList<Entity> addedEntities = new RawList<Entity>();
        private RawList<Entity> removedEntities = new RawList<Entity>();

        public override void Update(float dt)
        {
            for (int i = removedEntities.Count - 1; i >= 0; --i)
            {
                if (random.NextDouble() < 0.2)
                {
                    var entity = removedEntities[i];
                    addedEntities.Add(entity);
                    Space.Add(entity);
                    removedEntities.FastRemoveAt(i);
                }
            }
            for (int i = addedEntities.Count - 1; i >= 0; --i)
            {
                if (random.NextDouble() < 0.02)
                {
                    var entity = addedEntities[i];
                    removedEntities.Add(entity);
                    Space.Remove(entity);
                    addedEntities.FastRemoveAt(i);
                }
            }

            if (Game.MouseInput.MiddleButton != Microsoft.Xna.Framework.Input.ButtonState.Pressed)
            {
                for (int i = 0; i < 20; i++)
                {
                    var entity = addedEntities[random.Next(addedEntities.Count)];
                    entity.Position = new Vector3(
                        (float)(random.NextDouble() - 0.5f) * width,
                        (float)(random.NextDouble() - 0.5f) * height,
                        (float)(random.NextDouble() - 0.5f) * length);
                }
            }
            base.Update(dt);


        }
        
        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Add-remove Stress Test"; }
        }
    }
}