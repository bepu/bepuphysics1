using BEPUphysics.Entities.Prefabs;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUutilities;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Test class for performance analysis.
    /// </summary>
    public class TerrainTestDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public TerrainTestDemo(DemosGame game)
            : base(game)
        {
            //x and y, in terms of heightmaps, refer to their local x and y coordinates.  In world space, they correspond to x and z.
            //Setup the heights of the terrain.
            //int xLength = 384;
            //int zLength = 384;

            //float xSpacing = .5f;
            //float zSpacing = .5f;
            //var heights = new float[xLength, zLength];
            //for (int i = 0; i < xLength; i++)
            //{
            //    for (int j = 0; j < zLength; j++)
            //    {
            //        float x = i - xLength / 2;
            //        float z = j - zLength / 2;
            //        heights[i, j] = (float)(10 * (Math.Sin(x / 8) + Math.Sin(z / 8)));
            //    }
            //}
            ////Create the terrain.
            //var terrain = new Terrain(heights, new AffineTransform(
            //        new Vector3(xSpacing, 1, zSpacing),
            //        Quaternion.Identity,
            //        new Vector3(-xLength * xSpacing / 2, 0, -zLength * zSpacing / 2)));

            //Space.Add(terrain);
            //game.ModelDrawer.Add(terrain);

            for (int i = 0; i < 7; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    Space.Add(new Box(new Vector3(i * 50, -50, j * 50), 50, 20, 50));
                }
            }


            for (int i = 0; i < 20; i++)
            {
                for (int j = 0; j < 20; j++)
                {
                    CompoundBody cb = new CompoundBody(new CompoundShapeEntry[] 
                        {
                            new CompoundShapeEntry(new CapsuleShape(2, 1), new Vector3(0, 0,0)),
                            new CompoundShapeEntry(new CapsuleShape(2, 1), new Vector3(2, 0,0)),
                            new CompoundShapeEntry(new CapsuleShape(2, 1), new Vector3(4, 0,0)),
                            new CompoundShapeEntry(new CapsuleShape(2, 1), new Vector3(6, 0,0)),
                            new CompoundShapeEntry(new CapsuleShape(2, 1), new Vector3(8, 0,0)),
                            new CompoundShapeEntry(new CapsuleShape(2, 1), new Vector3(10, 0,0)),
                            new CompoundShapeEntry(new CapsuleShape(2, 1), new Vector3(12, 0,0)),
                        }, 10);
                    cb.Position = new Vector3(i * 15, 20, j * 5);

                    //CompoundBody cb = new CompoundBody(new CompoundShapeEntry[] 
                    //{
                    //    new CompoundShapeEntry(new SphereShape(1), new Vector3(0, 0,0)),
                    //    new CompoundShapeEntry(new SphereShape(1), new Vector3(2, -2,0)),
                    //    new CompoundShapeEntry(new SphereShape(1), new Vector3(4, 0,0)),
                    //    new CompoundShapeEntry(new SphereShape(1), new Vector3(6, -2,0)),
                    //    new CompoundShapeEntry(new SphereShape(1), new Vector3(8, 0,0)),
                    //    new CompoundShapeEntry(new SphereShape(1), new Vector3(10, -2,0)),
                    //    new CompoundShapeEntry(new SphereShape(1), new Vector3(12, 0,0)),
                    //}, 10);
                    //cb.Position = new Vector3(i * 15, 20, j * 5);

                    //CompoundBody cb = new CompoundBody(new CompoundShapeEntry[] 
                    //{
                    //    new CompoundShapeEntry(new CapsuleShape(0, 1), new Vector3(0, 0,0)),
                    //    new CompoundShapeEntry(new CapsuleShape(0, 1), new Vector3(2, -2,0)),
                    //    new CompoundShapeEntry(new CapsuleShape(0, 1), new Vector3(4, 0,0)),
                    //    new CompoundShapeEntry(new CapsuleShape(0, 1), new Vector3(6, -2,0)),
                    //    new CompoundShapeEntry(new CapsuleShape(0, 1), new Vector3(8, 0,0)),
                    //    new CompoundShapeEntry(new CapsuleShape(0, 1), new Vector3(10, -2,0)),
                    //    new CompoundShapeEntry(new CapsuleShape(0, 1), new Vector3(12, 0,0)),
                    //}, 10);
                    //cb.Position = new Vector3(i * 15, 20, j * 5);

                    cb.ActivityInformation.IsAlwaysActive = true;
                    cb.AngularVelocity = new Vector3(.01f, 0, 0);

                    Space.Add(cb);
                }
            }



            game.Camera.Position = new Vector3(0, 30, 20);

        }





        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Terrain"; }
        }
    }
}