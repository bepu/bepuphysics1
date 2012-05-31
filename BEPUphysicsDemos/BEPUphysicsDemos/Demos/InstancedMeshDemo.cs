using System;
using BEPUphysics.Collidables;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.DataStructures;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Drops some objects onto a field of transformed mesh instances.
    /// Instanced meshes are a low-memory usag approach to creating
    /// lots of similar static collision objects.
    /// </summary>
    public class InstancedMeshDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public InstancedMeshDemo(DemosGame game)
            : base(game)
        {



            Vector3[] vertices;
            int[] indices;
            ModelDataExtractor.GetVerticesAndIndicesFromModel(game.Content.Load<Model>("guy"), out vertices, out indices);
            var meshShape = new InstancedMeshShape(vertices, indices);

            var random = new Random();
            for (int i = 0; i < 10; i++)
            {
                for (int j = 0; j < 10; j++)
                {
                    //Create a transform and the instance of the mesh.
                    var transform = new AffineTransform(
                        new Vector3((float)random.NextDouble() * 6 + .5f, (float)random.NextDouble() * 6 + .5f, (float)random.NextDouble() * 6 + .5f), 
                         Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble())), (float)random.NextDouble() * 100),
                        new Vector3(i * 2, 3, j * 2));
                    var mesh = new InstancedMesh(meshShape, transform);
                    //Making the triangles one-sided makes collision detection a bit more robust, since the backsides of triangles won't try to collide with things
                    //and 'pull' them back into the mesh.
                    mesh.Sidedness = TriangleSidedness.Counterclockwise;
                    Space.Add(mesh);
                    game.ModelDrawer.Add(mesh);

                }
            }

            for (int i = 0; i < 5; i++)
            {
                for (int j = 0; j < 5; j++)
                {
                    //Drop a box on the mesh.
                    Space.Add(new Box(new Vector3((i + 1) * 4, 10, (j + 1) * 4), 1, 1, 1, 10));
                }
            }

            Space.Add(new Box(new Vector3(10, 0, 10), 20, 1, 20));

            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(10, 6, 30);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Instanced Meshes"; }
        }
    }
}