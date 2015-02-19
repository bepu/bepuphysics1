using BEPUphysics.Entities.Prefabs;
using BEPUutilities;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.CollisionShapes;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Checks the mobile mesh solidity sidedness calculation.
    /// </summary>
    public class MobileMeshSolidityTestDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public MobileMeshSolidityTestDemo(DemosGame game)
            : base(game)
        {
            Vector3[] vertices;
            int[] indices;

            //Hardcoded box
            vertices = new Vector3[] 
            {
                new Vector3(0.5f, 0.5f, 0.5f),
                new Vector3(0.5f,0.5f,-0.5f),
                new Vector3(-0.5000001f,0.5f ,-0.4999999f),
                new Vector3(-0.4999998f ,0.5f ,0.5000002f),
                new Vector3(-0.4999998f ,-0.5f ,0.5000002f),
                new Vector3(-0.5000001f ,-0.5f ,-0.4999999f),
                new Vector3(0.5f ,-0.5f ,-0.5f),
                new Vector3(0.5f ,-0.5f ,0.5f),
                new Vector3(0.5f ,0.5f ,0.5f),
                new Vector3(0.5f ,-0.5f ,0.5f),
                new Vector3(0.5f ,-0.5f ,-0.5f),
                new Vector3(0.5f ,0.5f ,-0.5f),
                new Vector3(0.5f,0.5f ,-0.5f),
                new Vector3(0.5f ,-0.5f ,-0.5f),
                new Vector3(-0.5000001f ,-0.5f ,-0.4999999f),
                new Vector3(-0.5000001f,0.5f ,-0.4999999f),
                new Vector3(-0.5000001f ,0.5f ,-0.4999999f),
                new Vector3(-0.5000001f ,-0.5f ,-0.4999999f),
                new Vector3(-0.4999998f ,-0.5f ,0.5000002f),
                new Vector3(-0.4999998f ,0.5f ,0.5000002f),
                new Vector3(-0.4999998f,0.5f ,0.5000002f),
                new Vector3(-0.4999998f ,-0.5f ,0.5000002f),
                new Vector3(0.5f ,-0.5f,0.5f) ,
                new Vector3(0.5f ,0.5f ,0.5f)
            };

            indices = new[] 
            {
                2, 1, 0,
                3, 2, 0,
                6, 5 ,4,
                7, 6 ,4,
                10, 9, 8,
                11, 10, 8,
                14, 13, 12,
                15, 14, 12,
                18, 17, 16,
                19, 18, 16,
                22, 21, 20,
                23, 22, 20
            };

            var mesh = new MobileMesh(vertices, indices, AffineTransform.Identity, MobileMeshSolidity.Solid, 10);
            Space.Add(mesh);

            //Tube
            ModelDataExtractor.GetVerticesAndIndicesFromModel(game.Content.Load<Model>("tube"), out vertices, out indices);
            mesh = new MobileMesh(vertices, indices, AffineTransform.Identity, MobileMeshSolidity.Solid, 10);
            mesh.Position = new Vector3(-10, 10, 0);
            Space.Add(mesh);

            //Cube
            ModelDataExtractor.GetVerticesAndIndicesFromModel(game.Content.Load<Model>("cube"), out vertices, out indices);
            mesh = new MobileMesh(vertices, indices, AffineTransform.Identity, MobileMeshSolidity.Solid, 10);
            mesh.Position = new Vector3(10, 0, 0);
            Space.Add(mesh);

            //Guy
            ModelDataExtractor.GetVerticesAndIndicesFromModel(game.Content.Load<Model>("guy"), out vertices, out indices);
            mesh = new MobileMesh(vertices, indices, AffineTransform.Identity, MobileMeshSolidity.Solid, 10);
            mesh.Position = new Vector3(0, 0, 10);
            Space.Add(mesh);

            //Barrel Platform
            ModelDataExtractor.GetVerticesAndIndicesFromModel(game.Content.Load<Model>("barrelandplatform"), out vertices, out indices);
            mesh = new MobileMesh(vertices, indices, AffineTransform.Identity, MobileMeshSolidity.Solid, 10);
            mesh.Position = new Vector3(0, 0, -10);
            Space.Add(mesh);

            //FloaterTube
            ModelDataExtractor.GetVerticesAndIndicesFromModel(game.Content.Load<Model>("tube"), out vertices, out indices);
            mesh = new MobileMesh(vertices, indices, new AffineTransform(new Vector3(1, 1, 1), Quaternion.Identity, new Vector3(0, 0, 0)), MobileMeshSolidity.Solid);
            mesh.Position = new Vector3(5, 18, 0);
            Space.Add(mesh);

            //Float a box through the last mesh to check contact generation controllably.
            var solidityTester = new Box(new Vector3(5, 8, 0), 1, 1, 1);
            solidityTester.LinearVelocity = new Vector3(0, 1, 0);
            CollisionRules.AddRule(solidityTester, mesh, CollisionRule.NoSolver);
            Space.Add(solidityTester);


            Space.Add(new Box(new Vector3(0, -5, 0), 50, 1, 50));

            game.Camera.Position = new Vector3(0, 10, 20);

        }





        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Solidity Test"; }
        }
    }
}