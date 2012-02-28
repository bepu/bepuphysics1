using BEPUphysics.Collidables;
using BEPUphysics.DataStructures;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.MathExtensions;
using BEPUphysics.CollisionShapes.ConvexShapes;
using System.Diagnostics;
using BEPUphysics.Settings;
using BEPUphysics.Materials;
using BEPUphysics.Constraints.SingleEntity;
using Microsoft.Xna.Framework.Input;
using BEPUphysics.CollisionShapes;

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
            //Load in mesh data and create the collision mesh.
            Vector3[] staticTriangleVertices;
            int[] staticTriangleIndices;

            var playgroundModel = game.Content.Load<Model>("playground");
            //This is a little convenience method used to extract vertices and indices from a model.
            //It doesn't do anything special; any approach that gets valid vertices and indices will work.
            TriangleMesh.GetVerticesAndIndicesFromModel(playgroundModel, out staticTriangleVertices, out staticTriangleIndices);
            //staticTriangleVertices = new Vector3[] { new Vector3(-10, 0, -10), new Vector3(0, 0, -10), new Vector3(10, 0, -10),
            //                                         new Vector3(-10, 0, 0), new Vector3(0, 0, 0), new Vector3(10, 0, 0),
            //                                         new Vector3(-10, 0, 10), new Vector3(0, 0, 10), new Vector3(10, 0, 10) };
            //staticTriangleIndices = new int[] { 0, 1, 4, 
            //                                    1, 2, 5, 
            //                                    0, 4, 3, 
            //                                    1, 5, 4, 
            //                                    3, 4, 7, 
            //                                    4, 5, 8, 
            //                                    3, 7, 6, 
            //                                    4, 8, 7 };
            var staticMesh = new StaticMesh(staticTriangleVertices, staticTriangleIndices, new AffineTransform(Matrix3X3.CreateFromAxisAngle(Vector3.Up, MathHelper.Pi), new Vector3(0, -10, 0)));
            staticMesh.Sidedness = TriangleSidedness.Counterclockwise;

            Space.Add(staticMesh);
            game.ModelDrawer.Add(staticMesh);




            ////Dump some boxes on top of it for fun.
            //int numColumns = 8;
            //int numRows = 8;
            //int numHigh = 1;
            //float separation = 8;
            //Entity toAdd;
            //for (int i = 0; i < numRows; i++)
            //    for (int j = 0; j < numColumns; j++)
            //        for (int k = 0; k < numHigh; k++)
            //        {
            //            toAdd = new Box(
            //                new Vector3(
            //                separation * i - numRows * separation / 2,
            //                30f + k * separation,
            //                separation * j - numColumns * separation / 2),
            //                2, 2, 2, 15);
            //            Space.Add(toAdd);
            //        }

            //CollisionDetectionSettings.ContactMinimumSeparationDistance = 0;



            Cylinder cylinder = new Cylinder(new Vector3(0, .3f, 0), 1.7f, .2f, 10);
            cylinder.CollisionInformation.Shape.CollisionMargin = .1f;
            //cylinder.PositionUpdateMode = BEPUphysics.PositionUpdating.PositionUpdateMode.Continuous;
            cylinder.LocalInertiaTensorInverse = new Matrix3X3();
            //Space.Add(cylinder);

            int numColumns = 4;
            int numRows = 4;
            int numHigh = 1;
            float separation = 8;

            var ball = game.Content.Load<Model>("360 thick Sphere");
            TriangleMesh.GetVerticesAndIndicesFromModel(ball, out staticTriangleVertices, out staticTriangleIndices);

            MobileMesh toAdd;
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                    for (int k = 0; k < numHigh; k++)
                    {
                        toAdd = new MobileMesh(staticTriangleVertices, staticTriangleIndices,
                        new AffineTransform(new Vector3(0.01f, 0.01f, 0.01f),
                         Quaternion.Identity,
                         new Vector3(
                            (separation * i - numRows * separation / 2) - 200,
                            30f + k * separation,
                            separation * j - numColumns * separation / 2)),
                         MobileMeshSolidity.Counterclockwise, 10);
                        Space.Add(toAdd);
                    }


            game.Camera.Position = new Vector3(0, 1, 6);


        }

        public override void Update(float dt)
        {
            if (Keyboard.GetState().IsKeyDown(Keys.P))
            {
                Debug.WriteLine("asdf");
            }
            else
                base.Update(dt);
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