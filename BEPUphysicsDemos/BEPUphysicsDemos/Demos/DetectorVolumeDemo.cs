using System.Collections.Generic;
using BEPUphysics.Collidables;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.DataStructures;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysicsDrawer.Models;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Box changes colors as it interacts with a detector volume.
    /// </summary>
    public class DetectorVolumeDemo : StandardDemo
    {
        //Box to change colors when it hits the detector volume.
        private ModelDisplayObject displayBox;
        private DetectorVolume detectorVolume;
        private Entity testEntity;

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public DetectorVolumeDemo(DemosGame game)
            : base(game)
        {
            var model = game.Content.Load<Model>("tube");
            Vector3[] modelVertices;
            int[] modelIndices;
            TriangleMesh.GetVerticesAndIndicesFromModel(model, out modelVertices, out modelIndices);
            detectorVolume = new DetectorVolume(new TriangleMesh(new StaticMeshData(modelVertices, modelIndices)));
            Space.Add(detectorVolume);

            game.ModelDrawer.Add(detectorVolume.TriangleMesh);

            //The detector volume works on all of the entity types:
            //Convexes!
            testEntity = new Box(new Vector3(0, -10, 0), 1, 1, 1);

            //Compounds!
            //var bodies = new List<CompoundShapeEntry>
            //{
            //    new CompoundShapeEntry(new SphereShape(.5f), new Vector3(0, -8.2f, 0), 1),
            //    new CompoundShapeEntry(new SphereShape(.5f), new Vector3(0, -9f, 0), 1),
            //    new CompoundShapeEntry(new SphereShape(.5f), new Vector3(0, -9.8f, 0), 1)
            //};
            //testEntity = new CompoundBody(bodies);

            //Mobile meshes!
            //model = game.Content.Load<Model>("tube");
            //TriangleMesh.GetVerticesAndIndicesFromModel(model, out modelVertices, out modelIndices);
            //testEntity = new MobileMesh(modelVertices, modelIndices, new AffineTransform(new Vector3(.2f, .2f, .2f), Quaternion.Identity, new Vector3(0, -10, 0)), MobileMeshSolidity.Solid);

            testEntity.Tag = "noDisplayObject";
            displayBox = game.ModelDrawer.Add(testEntity);
            SetColor(0);
            game.ModelDrawer.IsWireframe = true;
            testEntity.LinearVelocity = new Vector3(0, 1, 0);
            Space.Add(testEntity);

            //The color of the entity will change based upon events.
            //The events don't pay attention to the causing events, so you can toss a ball through the volume to recolor the entity.
            detectorVolume.EntityBeganTouching += BeganTouching;
            detectorVolume.EntityStoppedTouching += StoppedTouching;
            detectorVolume.VolumeBeganContainingEntity += BeganContaining;
            detectorVolume.VolumeStoppedContainingEntity += StoppedContaining;

            Space.ForceUpdater.Gravity = new Vector3();
            game.Camera.Position = new Vector3(0, 0, 22);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Detector Volume"; }
        }

        private void StoppedContaining(DetectorVolume volume, Entity entity)
        {
            SetColor(3);
        }

        private void BeganContaining(DetectorVolume volume, Entity entity)
        {
            SetColor(2);
        }

        private void StoppedTouching(DetectorVolume volume, Entity toucher)
        {
            SetColor(0);
        }

        private void BeganTouching(DetectorVolume volume, Entity toucher)
        {
            SetColor(1);
        }

        void SetColor(int index)
        {
            //The model drawer requires a reset to change the color.
            Game.ModelDrawer.Remove(testEntity);
            displayBox = Game.ModelDrawer.GetDisplayObject(testEntity);
            displayBox.TextureIndex = index;
            Game.ModelDrawer.Add(displayBox);
        }

        public override void DrawUI()
        {
            //Here's an alternate method of keeping track of changes: check the pairs list!
            //This will check specifically for our test entity, ignoring any other flying objects that might have been tossed through.
            DetectorVolumePairHandler pair;
            if (detectorVolume.Pairs.TryGetValue(testEntity, out pair))
            {
                if (pair.Containing)
                    Game.DataTextDrawer.Draw("Contained", new Vector2(50, 50));
                else if (pair.Touching)
                    Game.DataTextDrawer.Draw("Touching", new Vector2(50, 50));
                else
                    Game.DataTextDrawer.Draw("Separated", new Vector2(50, 50));
            }
            else
                Game.DataTextDrawer.Draw("Separated", new Vector2(50, 50));
            base.DrawUI();
        }

        public override void CleanUp()
        {
            Game.ModelDrawer.IsWireframe = false;
            base.CleanUp();
        }
    }
}