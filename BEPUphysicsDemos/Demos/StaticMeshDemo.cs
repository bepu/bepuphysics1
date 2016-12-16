using BEPUutilities;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.Entities.Prefabs;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// A nice driveable landscape.
    /// </summary>
    public class StaticMeshDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public StaticMeshDemo(DemosGame game)
            : base(game)
        {
            //Load in mesh data and create the collision mesh.
            Vector3[] staticTriangleVertices;
            int[] staticTriangleIndices;

            var playgroundModel = game.Content.Load<Model>("playground");
            //This is a little convenience method used to extract vertices and indices from a model.
            //It doesn't do anything special; any approach that gets valid vertices and indices will work.
            ModelDataExtractor.GetVerticesAndIndicesFromModel(playgroundModel, out staticTriangleVertices, out staticTriangleIndices);
            var staticMesh = new StaticMesh(staticTriangleVertices, staticTriangleIndices, new AffineTransform(Matrix3x3.CreateFromAxisAngle(Vector3.Up, MathHelper.Pi), new Vector3(0, -10, 0)));
            staticMesh.Sidedness = TriangleSidedness.Counterclockwise;

            Space.Add(staticMesh);
            game.ModelDrawer.Add(staticMesh);


            //Dump some boxes on top of it for fun.
            int numColumns = 8;
            int numRows = 8;
            int numHigh = 1;
            float separation = 8;
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                    for (int k = 0; k < numHigh; k++)
                    {
                        var toAdd = new Box(
                            new Vector3(
                            separation * i - numRows * separation / 2,
                            30f + k * separation,
                            separation * j - numColumns * separation / 2),
                            2, 2, 2, 15);
                        Space.Add(toAdd);
                    }



            game.Camera.Position = new Vector3(0, 10, 40);


        }



        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Static Mesh"; }
        }



    }
}