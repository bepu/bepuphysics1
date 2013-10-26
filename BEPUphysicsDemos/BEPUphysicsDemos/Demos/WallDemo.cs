using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;
using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class WallDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public WallDemo(DemosGame game)
            : base(game)
        {
            int width = 10;
            int height = 10;
            float blockWidth = 2f;
            float blockHeight = 1f;
            float blockLength = 1f;

            for (int i = 0; i < width; i++)
            {
                for (int j = 0; j < height; j++)
                {
                    var toAdd =
                        new Box(
                            new Vector3(
                                i * blockWidth + .5f * blockWidth * (j % 2) - width * blockWidth * .5f,
                                blockHeight * .5f + j * (blockHeight),
                                0),
                            blockWidth, blockHeight, blockLength, 10);
                    Space.Add(toAdd);
                }
            }

            Box ground = new Box(new Vector3(0, -.5f, 0), 50, 1, 50);
            Space.Add(ground);
            game.Camera.Position = new Vector3(0, 6, 15);

            //Vector3[] vertices = new[]
            //    {
            //        new Vector3(0, 0, 0),
            //        new Vector3(0, 0, 1),
            //        new Vector3(0, 1, 0),
            //        new Vector3(0, 1, 1),
            //        new Vector3(1, 0, 0),
            //        new Vector3(1, 0, 1),
            //        new Vector3(1, 1, 0),
            //        new Vector3(1, 1, 1),
            //    };
            //for (int i = 0; i < vertices.Length; ++i)
            //    vertices[i] *= 4;
            //var shape = new ConvexHullShape(vertices);


            //var shape = new ConeShape(1, 1);
            //var shape = new CylinderShape(1, 1);
            var shape = new BoxShape(1, 1, 1);

            float volume;
            var distribution = InertiaHelper.ComputeVolumeDistribution(shape, out volume);


            var samples = CommonResources.GetVectorList();
            if (samples.Capacity < InertiaHelper.SampleDirections.Length)
                samples.Capacity = InertiaHelper.SampleDirections.Length;
            for (int i = 0; i < InertiaHelper.SampleDirections.Length; ++i)
            {
                shape.GetLocalExtremePointWithoutMargin(ref InertiaHelper.SampleDirections[i], out samples.Elements[i]);
            }

            var triangles = CommonResources.GetIntList();
            ConvexHullHelper.GetConvexHull(samples, triangles);

            float newVolume;
            Vector3 newCenter;
            Matrix3x3 newDistribution;
            InertiaHelper.ComputeShapeDistribution(samples, triangles, out newCenter, out newVolume, out newDistribution);

            var minimumRadius = InertiaHelper.ComputeMinimumRadius(samples, triangles, ref newCenter);
            CommonResources.GiveBack(samples);
            CommonResources.GiveBack(triangles);

            Space.Add(new Entity(shape, 1) { Position = new Vector3(0, 12, 0) });

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