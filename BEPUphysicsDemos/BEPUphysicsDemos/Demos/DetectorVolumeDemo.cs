using BEPUphysics.DataStructures;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.UpdateableSystems;
using BEPUphysicsDrawer.Models;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Box changes colors as it interacts with a detector volume.
    /// </summary>
    public class DetectorVolumeDemo : StandardDemo
    {
        //Box to change colors when it hits the detector volume.
        private readonly ModelDisplayObjectBase displayBox;

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public DetectorVolumeDemo(DemosGame game)
            : base(game)
        {
            var tubeModel = game.Content.Load<Model>("tube");
            Vector3[] modelVertices;
            int[] modelIndices;
            ModelDataExtractor.GetVerticesAndIndicesFromModel(tubeModel, out modelVertices, out modelIndices);
            var detectorVolume = new DetectorVolume(new StaticMeshData(modelVertices, modelIndices), Space.BroadPhase.QueryAccelerator);
            Space.Add(detectorVolume);

            game.ModelDrawer.Add(detectorVolume.TriangleMesh);

            Entity toAdd = new Box(new Vector3(0, -10, 0), 1, 1, 1);
            toAdd.Tag = "noDisplayObject";
            displayBox = game.ModelDrawer.Add(toAdd);
            displayBox.TextureIndex = 0;
            game.ModelDrawer.IsWireframe = true;
            toAdd.LinearVelocity = new Vector3(0, 1, 0);
            Space.Add(toAdd); //Create the ground

            detectorVolume.EntityBeginsTouching += BeginsTouching;
            detectorVolume.EntityStopsTouching += StopsTouching;
            detectorVolume.VolumeBeginsContainingEntity += BeginsContaining;
            detectorVolume.VolumeStopsContainingEntity += StopsContaining;

            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(0, 0, 22);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Detector Volume"; }
        }

        private void StopsContaining(DetectorVolume volume, Entity entity)
        {
            displayBox.TextureIndex = 3;
        }

        private void BeginsContaining(DetectorVolume volume, Entity entity)
        {
            displayBox.TextureIndex = 2;
        }

        private void StopsTouching(Entity toucher, DetectorVolume volume)
        {
            displayBox.TextureIndex = 0;
        }

        private void BeginsTouching(Entity toucher, DetectorVolume volume)
        {
            displayBox.TextureIndex = 1;
        }

        public override void CleanUp()
        {
            Game.ModelDrawer.IsWireframe = false;
            base.CleanUp();
        }
    }
}