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
using BEPUphysicsDemos.AlternateMovement.SimpleCharacter;

namespace BEPUphysicsDemos.Demos.Extras.Tests
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
            //This is a little convenience method used to extract vertices and indices from a model.
            //It doesn't do anything special; any approach that gets valid vertices and indices will work.
            TriangleMesh.GetVerticesAndIndicesFromModel(playgroundModel, out staticTriangleVertices, out staticTriangleIndices);
            var staticMesh = new StaticMesh(staticTriangleVertices, staticTriangleIndices, new AffineTransform(Matrix3X3.CreateFromAxisAngle(Vector3.Up, MathHelper.Pi), new Vector3(0, -10, 0)));
            staticMesh.Sidedness = TriangleSidedness.Counterclockwise;

            Space.Add(staticMesh);
            game.ModelDrawer.Add(staticMesh);





            //Now drop the characters on it!
            var numColumns = 8;
            var numRows = 8;
            var numHigh = 8;
            float separation = 16;

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

            //Now drop the ball-characters on it!
            numColumns = 8;
            numRows = 8;
            numHigh = 8;
            separation = 16;
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                    for (int k = 0; k < numHigh; k++)
                    {
                        var character = new SphereCharacterController();
                        character.Body.Position =
                            new Vector3(
                            separation * i - numRows * separation / 2,
                            48f + k * separation,
                            separation * j - numColumns * separation / 2);

                        sphereCharacters.Add(character);

                        Space.Add(character);
                    }


            game.Camera.Position = new Vector3(0, 10, 40);

            //Dump some boxes on top of the characters for fun.
            numColumns = 16;
            numRows = 16;
            numHigh = 1;
            separation = 8;
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                    for (int k = 0; k < numHigh; k++)
                    {
                        var toAdd = new Box(
                            new Vector3(
                            separation * i - numRows * separation / 2,
                            52f + k * separation,
                            separation * j - numColumns * separation / 2),
                            2, 2, 2, 15);

                        Space.Add(toAdd);
                    }
        }

        List<CharacterController> characters = new List<CharacterController>();
        List<SphereCharacterController> sphereCharacters = new List<SphereCharacterController>();
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
                if (random.NextDouble() < .01f)
                    characters[i].Jump();

                var next = random.NextDouble();
                if (next < .01)
                {
                    //Note: The character's graphic won't represent the crouching process properly since we're not remove/readding it.
                    if (next < .005f && characters[i].StanceManager.CurrentStance == Stance.Standing)
                        characters[i].StanceManager.DesiredStance = Stance.Crouching;
                    else
                        characters[i].StanceManager.DesiredStance = Stance.Standing;
                }
            }

            //Tell the sphere characters to run around too.
            for (int i = 0; i < sphereCharacters.Count; i++)
            {
                sphereCharacters[i].HorizontalMotionConstraint.MovementDirection = new Vector2((float)(random.NextDouble() * 2 - 1), (float)(random.NextDouble() * 2 - 1));
                if (random.NextDouble() < .01f)
                    sphereCharacters[i].Jump();
            }
            base.Update(dt);
        }



    }
}