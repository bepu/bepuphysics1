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
using BEPUphysicsDemos.AlternateMovement.SphereCharacter;
using System.Collections.Generic;
using System;
using BEPUphysicsDemos.AlternateMovement.Character;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// A nice driveble landscape.
    /// </summary>
    public class CharacterStressTestDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public CharacterStressTestDemo(DemosGame game)
            : base(game)
        {
            //Load in mesh data and create the group.
            Vector3[] staticTriangleVertices;
            int[] staticTriangleIndices;

            var playgroundModel = game.Content.Load<Model>("playground");
            //This load method wraps the TriangleMesh.GetVerticesAndIndicesFromModel method 
            //to output vertices of type StaticTriangleGroupVertex instead of TriangleMeshVertex or simply Vector3.
            TriangleMesh.GetVerticesAndIndicesFromModel(playgroundModel, out staticTriangleVertices, out staticTriangleIndices);
            var staticMesh = new StaticMesh(staticTriangleVertices, staticTriangleIndices, new AffineTransform(Matrix3X3.CreateFromAxisAngle(Vector3.Up, MathHelper.Pi), new Vector3(0, -10, 0)));
            staticMesh.Sidedness = TriangleSidedness.Counterclockwise;

            Space.Add(staticMesh);
            game.ModelDrawer.Add(staticMesh);

      


            ////Dump some boxes on top of it for fun.
            int numColumns = 8;
            int numRows = 8;
            int numHigh = 1;
            float separation = 8;
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

            //Now drop the characters on it!
            numColumns = 8;
            numRows = 8;
            numHigh = 16;
            separation = 8;
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                    for (int k = 0; k < numHigh; k++)
                    {
                        var character = new CharacterController();
                        character.Body.Position =
                            new Vector3(
                            separation * i - numRows * separation / 2,
                            40f + k * separation,
                            separation * j - numColumns * separation / 2);

                        characters.Add(character);

                        Space.Add(character);
                    }


            game.Camera.Position = new Vector3(0, 10, 40);


        }

        List<CharacterController> characters = new List<CharacterController>();
        Random random = new Random();

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Character Stress Test"; }
        }

        public override void Update(float dt)
        {
            //Tell all the characters to run around randomly.
            for (int i = 0; i < characters.Count; i++)
            {
                characters[i].HorizontalMotionConstraint.MovementDirection = new Vector2((float)(random.NextDouble() * 2 - 1), (float)(random.NextDouble() * 2 - 1));
            }
            base.Update(dt);
        }



    }
}