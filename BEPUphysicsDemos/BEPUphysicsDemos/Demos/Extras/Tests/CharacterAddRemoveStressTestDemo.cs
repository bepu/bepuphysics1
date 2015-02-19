using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.Character;
using BEPUphysics.CollisionShapes;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;
using BEPUutilities.DataStructures;
using Microsoft.Xna.Framework.Graphics;
using System.Collections.Generic;
using System;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// A nice landscape full of ephemeral people.
    /// </summary>
    public class CharacterAddRemoveStressTestDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public CharacterAddRemoveStressTestDemo(DemosGame game)
            : base(game)
        {
            //Load in mesh data and create the group.
            Vector3[] staticTriangleVertices;
            int[] staticTriangleIndices;

            var playgroundModel = game.Content.Load<Model>("playground");
            //This is a little convenience method used to extract vertices and indices from a model.
            //It doesn't do anything special; any approach that gets valid vertices and indices will work.
            ModelDataExtractor.GetVerticesAndIndicesFromModel(playgroundModel, out staticTriangleVertices, out staticTriangleIndices);
            var meshShape = new InstancedMeshShape(staticTriangleVertices, staticTriangleIndices);
            var meshes = new List<Collidable>();

            var xSpacing = 400;
            var ySpacing = 400;
            var xCount = 3;
            var yCount = 3;
            for (int i = 0; i < xCount; i++)
            {
                for (int j = 0; j < yCount; j++)
                {
                    var staticMesh = new InstancedMesh(meshShape, new AffineTransform(Matrix3x3.Identity, new Vector3(-xSpacing * (xCount - 1) / 2 + i * xSpacing, 0, -ySpacing * (yCount - 1) / 2 + j * ySpacing)));
                    staticMesh.Sidedness = TriangleSidedness.Counterclockwise;
                    Space.Add(staticMesh);
                    //meshes.Add(staticMesh);
                    game.ModelDrawer.Add(staticMesh);
                }
            }
            //var group = new StaticGroup(meshes);
            //Space.Add(group);



            //Now drop the characters on it!
            var numColumns = 8;
            var numRows = 8;
            var numHigh = 8;
            float separation = 8;

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
            separation = 8;
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
            numColumns = 8;
            numRows = 8;
            numHigh = 8;
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
                            0.8f, 0.8f, 0.8f, 15);
                        toAdd.PositionUpdateMode = BEPUphysics.PositionUpdating.PositionUpdateMode.Continuous;

                        Space.Add(toAdd);
                    }
        }


        List<CharacterController> characters = new List<CharacterController>();
        List<SphereCharacterController> sphereCharacters = new List<SphereCharacterController>();
        Random random = new Random();
        private RawList<CharacterController> removedCharacters = new RawList<CharacterController>();
        private RawList<SphereCharacterController> removedSphereCharacters = new RawList<SphereCharacterController>();

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Character Add/Remove Test"; }
        }

        public override void Update(float dt)
        {
            //Add previously removed characters.
            for (int i = removedCharacters.Count - 1; i >= 0; --i)
            {
                if (random.NextDouble() < 0.2)
                {
                    Space.Add(removedCharacters[i]);
                    removedCharacters.FastRemoveAt(i);
                }
            }

            for (int i = removedSphereCharacters.Count - 1; i >= 0; --i)
            {
                if (random.NextDouble() < 0.2)
                {
                    Space.Add(removedSphereCharacters[i]);
                    removedSphereCharacters.FastRemoveAt(i);
                }
            }

            //Tell all the characters to run around randomly.
            for (int i = 0; i < characters.Count; i++)
            {
                if (characters[i].Space != null)
                {
                    if (random.NextDouble() < 0.02)
                    {
                        removedCharacters.Add(characters[i]);
                        Space.Remove(characters[i]);
                    }
                    else
                    {
                        characters[i].HorizontalMotionConstraint.MovementDirection = new Vector2((float) (random.NextDouble() * 2 - 1), (float) (random.NextDouble() * 2 - 1));
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
                }
            }

            //Tell the sphere characters to run around too.
            for (int i = 0; i < sphereCharacters.Count; i++)
            {
                if (sphereCharacters[i].Space != null)
                {
                    if (random.NextDouble() < 0.02)
                    {
                        removedSphereCharacters.Add(sphereCharacters[i]);
                        Space.Remove(sphereCharacters[i]);
                    }
                    else
                    {
                        sphereCharacters[i].HorizontalMotionConstraint.MovementDirection = new Vector2((float) (random.NextDouble() * 2 - 1), (float) (random.NextDouble() * 2 - 1));
                        if (random.NextDouble() < .01f)
                            sphereCharacters[i].Jump();
                    }
                }
            }


            base.Update(dt);
        }




    }
}