using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using System;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using System.Collections.Generic;
using BEPUphysics.NarrowPhaseSystems;
using BEPUutilities;
using BEPUutilities.DataStructures;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class AddRemoveStressDemo : StandardDemo
    {
        private Vector3 GetRandomPosition(Random random)
        {
            return new Vector3(
                       (float)(random.NextDouble() - 0.5) * width,
                       (float)(random.NextDouble() - 0.5) * height,
                       (float)(random.NextDouble() - 0.5) * length);
        }

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
            Space.Remove(vehicle.Vehicle);

            var compoundShape = new CompoundShape(new List<CompoundShapeEntry>
                {
                    new CompoundShapeEntry(new BoxShape(1, 1, 1), new Vector3(0, 1, 0), 1),
                    new CompoundShapeEntry(new BoxShape(2, 1, 2), new Vector3(), 1),
                    new CompoundShapeEntry(new BoxShape(1, 1, 1), new Vector3(0, -1, 0), 1)
                });
            for (int i = 0; i < 300; ++i)
            {
                var toAdd = new Entity(compoundShape, 10);
                addedEntities.Add(toAdd);
            }

            var boxShape = new BoxShape(1, 1, 1);
            for (int i = 0; i < 300; ++i)
            {
                var toAdd = new Entity(boxShape, 10);
                addedEntities.Add(toAdd);
            }

            Vector3[] vertices;
            int[] indices;
            ModelDataExtractor.GetVerticesAndIndicesFromModel(game.Content.Load<Model>("cube"), out vertices, out indices);
            var mobileMeshShape = new MobileMeshShape(vertices, indices, new AffineTransform(Matrix3x3.CreateScale(1, 2, 1), new Vector3()), MobileMeshSolidity.Counterclockwise);
            for (int i = 0; i < 300; ++i)
            {
                var toAdd = new Entity(mobileMeshShape, 10);
                addedEntities.Add(toAdd);
            }

            for (int i = 0; i < addedEntities.Count; ++i)
            {
                var entity = addedEntities[i];
                entity.Gravity = new Vector3();
                entity.Position = GetRandomPosition(random);
                entity.LinearVelocity = 3 * Vector3.Normalize(entity.Position);
                Space.Add(entity);
            }


            var playgroundModel = game.Content.Load<Model>("playground");
            ModelDataExtractor.GetVerticesAndIndicesFromModel(playgroundModel, out vertices, out indices);
            var staticMesh = new StaticMesh(vertices, indices, new AffineTransform(Matrix3x3.CreateFromAxisAngle(Vector3.Up, MathHelper.Pi), new Vector3(0, -30, 0)));
            staticMesh.Sidedness = TriangleSidedness.Counterclockwise;

            Space.Add(staticMesh);
            game.ModelDrawer.Add(staticMesh);

            game.Camera.Position = new Vector3(0, 6, 15);
        }

        Random random = new Random(5);
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
                for (int i = 0; i < Math.Max(1, (int)(addedEntities.Count * 0.02f)); i++)
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