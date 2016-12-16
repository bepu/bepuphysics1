using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using BEPUutilities;
using System.Diagnostics;
using System;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUutilities.DataStructures;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class BooleanConvexTestDemo : StandardDemo
    {


        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public BooleanConvexTestDemo(DemosGame game)
            : base(game)
        {
            var random = new Random();

            int numberOfConfigurations = 1000;
            int numberOfTestsPerConfiguration = 10000;


            float size = 2;
            var aPositionBounds = new BoundingBox(new Vector3(-size, -size, -size), new Vector3(size, size, size));
            var bPositionBounds = new BoundingBox(new Vector3(-size, -size, -size), new Vector3(size, size, size));

            size = 1;
            var aShapeBounds = new BoundingBox(new Vector3(-size, -size, -size), new Vector3(size, size, size));
            var bShapeBounds = new BoundingBox(new Vector3(-size, -size, -size), new Vector3(size, size, size));
            int pointsInA = 10;
            int pointsInB = 10;



            RawList<Vector3> points = new RawList<Vector3>();

            long accumulatedMPR = 0;
            long accumulatedGJK = 0;
            long accumulatedGJKSeparatingAxis = 0;


            for (int i = 0; i < numberOfConfigurations; i++)
            {
                //Create two convex hull shapes.
                for (int j = 0; j < pointsInA; j++)
                {
                    Vector3 point;
                    GetRandomPointInBoundingBox(random, ref aShapeBounds, out point);
                    points.Add(point);
                }
                var a = new ConvexHullShape(points);
                points.Clear();
                for (int j = 0; j < pointsInB; j++)
                {
                    Vector3 point;
                    GetRandomPointInBoundingBox(random, ref bShapeBounds, out point);
                    points.Add(point);
                }
                var b = new ConvexHullShape(points);
                points.Clear();

                //Generate some random tranforms for the shapes.
                RigidTransform aTransform;
                var axis = Vector3.Normalize(new Vector3((float)((random.NextDouble() - .5f) * 2), (float)((random.NextDouble() - .5f) * 2), (float)((random.NextDouble() - .5f) * 2)));
                var angle = (float)random.NextDouble() * MathHelper.TwoPi;
                Quaternion.CreateFromAxisAngle(ref axis, angle, out aTransform.Orientation);
                GetRandomPointInBoundingBox(random, ref aPositionBounds, out aTransform.Position);

                RigidTransform bTransform;
                axis = Vector3.Normalize(new Vector3((float)((random.NextDouble() - .5f) * 2), (float)((random.NextDouble() - .5f) * 2), (float)((random.NextDouble() - .5f) * 2)));
                angle = (float)random.NextDouble() * MathHelper.TwoPi;
                Quaternion.CreateFromAxisAngle(ref axis, angle, out bTransform.Orientation);
                GetRandomPointInBoundingBox(random, ref bPositionBounds, out bTransform.Position);

                //Perform MPR tests.
                //Warm up the cache a bit.
                MPRToolbox.AreShapesOverlapping(a, b, ref aTransform, ref bTransform);
                long start = Stopwatch.GetTimestamp();
                for (int j = 0; j < numberOfTestsPerConfiguration; j++)
                {
                    if (MPRToolbox.AreShapesOverlapping(a, b, ref aTransform, ref bTransform))
                        overlapsMPR++;
                }
                long end = Stopwatch.GetTimestamp();
                accumulatedMPR += end - start;

                //Perform GJK tests.
                //Warm up the cache a bit.
                GJKToolbox.AreShapesIntersecting(a, b, ref aTransform, ref bTransform);
                start = Stopwatch.GetTimestamp();
                for (int j = 0; j < numberOfTestsPerConfiguration; j++)
                {
                    if (GJKToolbox.AreShapesIntersecting(a, b, ref aTransform, ref bTransform))
                        overlapsGJK++;
                }
                end = Stopwatch.GetTimestamp();
                accumulatedGJK += end - start;

                //Perform GJK Separating Axis tests.
                //Warm up the cache a bit.
                Vector3 localSeparatingAxis = Vector3.Up;
                GJKToolbox.AreShapesIntersecting(a, b, ref aTransform, ref bTransform, ref localSeparatingAxis);
                start = Stopwatch.GetTimestamp();
                for (int j = 0; j < numberOfTestsPerConfiguration; j++)
                {
                    if (GJKToolbox.AreShapesIntersecting(a, b, ref aTransform, ref bTransform, ref localSeparatingAxis))
                        overlapsGJKSeparatingAxis++;
                }
                end = Stopwatch.GetTimestamp();
                accumulatedGJKSeparatingAxis += end - start;

            }

            //Compute the actual time per test.
            long denominator = Stopwatch.Frequency * numberOfConfigurations * numberOfTestsPerConfiguration;
            timeMPR = (double)accumulatedMPR / denominator;
            timeGJK = (double)accumulatedGJK / denominator;
            timeGJKSeparatingAxis = (double)accumulatedGJKSeparatingAxis / denominator;
        }

        void GetRandomPointInBoundingBox(Random random, ref BoundingBox box, out Vector3 point)
        {
#if XBOX360
            point = new Vector3();
#endif
            point.X = (float)(random.NextDouble() * (box.Max.X - box.Min.X) + box.Min.X);
            point.Y = (float)(random.NextDouble() * (box.Max.Y - box.Min.Y) + box.Min.Y);
            point.Z = (float)(random.NextDouble() * (box.Max.Z - box.Min.Z) + box.Min.Z);
        }


        private double timeMPR;
        private double timeGJK;
        private double timeGJKSeparatingAxis;


        private long overlapsMPR;
        private long overlapsGJK;
        private long overlapsGJKSeparatingAxis;

        public override void DrawUI()
        {
            Game.DataTextDrawer.Draw("MPR Time (ns): ", timeMPR * 1e9, 0, new Microsoft.Xna.Framework.Vector2(40, 40));
            Game.DataTextDrawer.Draw("GJK Time (ns): ", timeGJK * 1e9, 0, new Microsoft.Xna.Framework.Vector2(40, 70));
            Game.DataTextDrawer.Draw("GJK Separating Axis Time (ns): ", timeGJKSeparatingAxis * 1e9, 0, new Microsoft.Xna.Framework.Vector2(40, 100));

            Game.DataTextDrawer.Draw("MPR overlaps: ", overlapsMPR, new Microsoft.Xna.Framework.Vector2(40, 150));
            Game.DataTextDrawer.Draw("GJK overlaps: ", overlapsGJK, new Microsoft.Xna.Framework.Vector2(40, 180));
            Game.DataTextDrawer.Draw("GJK Separating Axis overlaps: ", overlapsGJKSeparatingAxis, new Microsoft.Xna.Framework.Vector2(40, 210));
            base.DrawUI();
        }



        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Boolean Convex Test"; }
        }

    }
}