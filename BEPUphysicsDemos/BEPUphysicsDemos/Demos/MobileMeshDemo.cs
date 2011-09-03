using BEPUphysics.Collidables;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.PositionUpdating;
using Microsoft.Xna.Framework.Input;
using System.Diagnostics;
using System.Collections.Generic;
using System;
using BEPUphysics.MathExtensions;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.DataStructures;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.CollisionShapes;
using BEPUphysics.Settings;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Constraints;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using BEPUphysics.Constraints.SolverGroups;
using BEPUphysics.Constraints.TwoEntity.Motors;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class MobileMeshDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public MobileMeshDemo(DemosGame game)
            : base(game)
        {


            Vector3[] vertices;
            int[] indices;

            //Create a big hollow sphere (squished into an ellipsoid).
            ModelDataExtractor.GetVerticesAndIndicesFromModel(game.Content.Load<Model>("hollowsphere"), out vertices, out indices);
            var transform = new AffineTransform(new Vector3(.06f, .04f, .06f), Quaternion.Identity, new Vector3(0, 0, 0));

            //Note that meshes can also be made solid (MobileMeshSolidity.Solid).  This gives meshes a solid collidable volume, instead of just
            //being thin shells.  However, enabling solidity is more expensive.
            var mesh = new MobileMesh(vertices, indices, transform, MobileMeshSolidity.Counterclockwise);
            mesh.Position = new Vector3(0, 0, 0);
            //Make the mesh spin a bit!
            mesh.AngularVelocity = new Vector3(0, 1, 0);
            Space.Add(mesh);

            //Add another mobile mesh inside.
            ModelDataExtractor.GetVerticesAndIndicesFromModel(game.Content.Load<Model>("tube"), out vertices, out indices);
            transform = new AffineTransform(new Vector3(1, 1, 1), Quaternion.Identity, new Vector3(0, 0, 0));
            mesh = new MobileMesh(vertices, indices, transform, MobileMeshSolidity.Counterclockwise, 10);
            mesh.Position = new Vector3(0, 10, 0);
            Space.Add(mesh);

            //Create a bunch of boxes.
#if WINDOWS
            int numColumns = 5;
            int numRows = 5;
            int numHigh = 5;
#else
            //Keep the simulation a bit smaller on the xbox.
            int numColumns = 4;
            int numRows = 4;
            int numHigh = 4;
#endif
            float separation = 1.5f;


            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                    for (int k = 0; k < numHigh; k++)
                    {
                        Space.Add(new Box(new Vector3(separation * i, k * separation, separation * j), 1, 1, 1, 5));
                    }

            //Space.Add(new Box(new Vector3(0, -10, 0), 1, 1, 1));
            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(0, -10, 5);
            game.Camera.Yaw = 0;
            game.Camera.Pitch = 0;


        }


        public override void Update(float dt)
        {
            base.Update(dt);
        }


        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "MobileMeshes"; }
        }
    }
}