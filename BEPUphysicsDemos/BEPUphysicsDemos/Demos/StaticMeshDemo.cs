using BEPUphysics.Collidables;
using BEPUphysics.DataStructures;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.MathExtensions;
using BEPUphysics.CollisionShapes.ConvexShapes;
using System.Diagnostics;
using BEPUphysics.Settings;
using BEPUphysics.Materials;
using BEPUphysics.Constraints.SingleEntity;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// A nice driveble landscape.
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
            //Load in mesh data and create the group.
            Vector3[] staticTriangleVertices;
            int[] staticTriangleIndices;

            var playgroundModel = game.Content.Load<Model>("playground");
            //This load method wraps the TriangleMesh.GetVerticesAndIndicesFromModel method 
            //to output vertices of type StaticTriangleGroupVertex instead of TriangleMeshVertex or simply Vector3.
            ModelDataExtractor.GetVerticesAndIndicesFromModel(playgroundModel, out staticTriangleVertices, out staticTriangleIndices);
            var staticMesh = new StaticMesh(staticTriangleVertices, staticTriangleIndices, new AffineTransform(Matrix3X3.CreateFromAxisAngle(Vector3.Up, MathHelper.Pi), new Vector3(0, -10, 0)));
            staticMesh.Sidedness = TriangleSidedness.Counterclockwise;

            Space.Add(staticMesh);
            game.ModelDrawer.Add(staticMesh);
           


            //Dump some boxes on top of it for fun.
            int numColumns = 8;
            int numRows = 8;
            int numHigh = 1;
            float separation = 8;
            Entity toAdd;
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                    for (int k = 0; k < numHigh; k++)
                    {
                        toAdd = new Box(
                            new Vector3(
                            separation * i - numRows * separation / 2,
                            30f + k * separation,
                            separation * j - numColumns * separation / 2),
                            2, 2, 2, 15);
                    
                        Space.Add(toAdd);
                    }



            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(0, 10, 40);


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