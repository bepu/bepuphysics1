using System;
using System.Collections.Generic;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Non-standard shapes like MinkowskiSums, WrappedBodies, and ConvexHulls.
    /// </summary>
    public class FancyShapesDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public FancyShapesDemo(DemosGame game)
            : base(game)
        {



            var points = new List<Vector3>();

            //Setup a random distribution in a cube and compute a convex hull.
            var random = new Random();
            for (int k = 0; k < 40; k++)
            {
                points.Add(new Vector3(3 * (float)random.NextDouble(), 5 * (float)random.NextDouble(), 3 * (float)random.NextDouble()));
            }
            var convexHull = new ConvexHull(new Vector3(0, 7, 0), points, 10);

            Space.Add(convexHull);

            points.Clear();

            
            //Create another random distribution, but this time with more points.
            points.Clear();
            for (int k = 0; k < 400; k++)
            {
                points.Add(new Vector3(1 * (float)random.NextDouble(), 3 * (float)random.NextDouble(), 1 * (float)random.NextDouble()));
            }
            convexHull = new ConvexHull(new Vector3(4, 7, 0), points, 10);
            Space.Add(convexHull);


            //Minkowski Sums are fancy 'combinations' of objects, where the result is the sum of the individual points making up shapes.
            //Think of it as sweeping one shape around and through another; a sphere and a box would produce a rounded-edge box.
            var minkowskiSum = new MinkowskiSum(new Vector3(4, -3, 0),
                    new OrientedConvexShapeEntry(new BoxShape(2, 2, 2)),
                    new OrientedConvexShapeEntry(new ConeShape(2, 2)), 10);
            Space.Add(minkowskiSum);

            minkowskiSum = new MinkowskiSum(new Vector3(0, 3, 0),
                    new OrientedConvexShapeEntry(Quaternion.CreateFromYawPitchRoll(1, 2, 3), new ConeShape(1, 1)),
                    new OrientedConvexShapeEntry(new TriangleShape(Vector3.Zero, Vector3.Right, Vector3.Forward)), 1);
            Space.Add(minkowskiSum);

            //Note how this minkowski sum is composed of a cylinder, and another minkowski sum shape.
            minkowskiSum = new MinkowskiSum(new Vector3(-4, 10, 0),
                    new OrientedConvexShapeEntry(new CylinderShape(1, 2)),
                    new OrientedConvexShapeEntry(new MinkowskiSumShape(
                        new OrientedConvexShapeEntry(new TriangleShape(new Vector3(1, 1, 1), new Vector3(-2, 0, 0), new Vector3(0, -1, 0))),
                        new OrientedConvexShapeEntry(new BoxShape(.3f, 1, .3f)))), 10);
            Space.Add(minkowskiSum);

            //Minkowski sums can also be used on more than two shapes at once.  The two-shape constructor is just a convenience wrapper.


            //Wrapped objects use an implicit convex hull around a set of shapes.

            //Oblique cone:
            var cone = new List<ConvexShapeEntry>()
            {
                new ConvexShapeEntry(new CylinderShape(0, 1)),
                new ConvexShapeEntry(new RigidTransform(new Vector3(1f, 2, 0)), new SphereShape(0)) 
            };
            Space.Add(new WrappedBody(new Vector3(-5, 0, 0), cone, 10));



            //Rather odd shape:
            var oddShape = new List<ConvexShapeEntry>();
            var bottom = new ConvexShapeEntry(new Vector3(-2, 2, 0), new SphereShape(2));
            var middle = new ConvexShapeEntry(
                new RigidTransform(
                    new Vector3(-2, 3, 0),
                    Quaternion.CreateFromAxisAngle(Vector3.Right, (float)Math.PI / 6)),
                    new CylinderShape(0, 3));
            var top = new ConvexShapeEntry(new Vector3(-2, 4, 0), new SphereShape(1f));
            oddShape.Add(bottom);
            oddShape.Add(middle);
            oddShape.Add(top);
            Space.Add(new WrappedBody(new Vector3(-3, 4, 0), oddShape, 10));

            //Transformable shapes can be any other kind of convex primitive transformed by any affine transformation.
            Matrix3X3 transform;
            transform = Matrix3X3.Identity;
            transform.M23 = .5f;
            transform.M13 = .5f;
            var transformable = new TransformableEntity(new Vector3(0, 0, 4), new BoxShape(1, 1, 1), transform, 10);
            Space.Add(transformable);
            

            Space.Add(new Box(new Vector3(0, -10, 0), 70, 5, 70));

            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(0, 0, 30);

        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Fancy Shapes"; }
        }
    }
}