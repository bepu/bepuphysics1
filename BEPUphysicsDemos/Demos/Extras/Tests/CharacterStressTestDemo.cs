using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.Character;
using BEPUphysics.CollisionShapes;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;
using Microsoft.Xna.Framework.Graphics;
using System.Collections.Generic;
using System;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// A nice landscape full of strange people.
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
            ModelDataExtractor.GetVerticesAndIndicesFromModel(playgroundModel, out staticTriangleVertices, out staticTriangleIndices);
            var meshShape = new InstancedMeshShape(staticTriangleVertices, staticTriangleIndices);
            var meshes = new List<Collidable>();

            var xSpacing = 400;
            var ySpacing = 400;
            var xCount = 11;
            var yCount = 11;
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
            var numColumns = 16;
            var numRows = 16;
            var numHigh = 8;
            float separation = 64;

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
            numColumns = 16;
            numRows = 16;
            numHigh = 8;
            separation = 64;
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
            numHigh = 8;
            separation = 64;
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

        //public override void DrawUI()
        //{
        //    //Try compiling the library with the PROFILE symbol defined and using this!
        //    Game.DataTextDrawer.Draw("Time Step Stage Times: ", new Vector2(20, 10));

        //    Game.TinyTextDrawer.Draw("SpaceObjectBuffer: ", Space.SpaceObjectBuffer.Time * 1000, 2, new Vector2(20, 35));
        //    Game.TinyTextDrawer.Draw("Entity State Write Buffer: ", Space.EntityStateWriteBuffer.Time * 1000, 2, new Vector2(20, 50));
        //    Game.TinyTextDrawer.Draw("Deactivation: ", Space.DeactivationManager.Time * 1000, 2, new Vector2(20, 65));
        //    Game.TinyTextDrawer.Draw("ForceUpdater: ", Space.ForceUpdater.Time * 1000, 2, new Vector2(20, 80));
        //    Game.TinyTextDrawer.Draw("DuringForcesUpdateables: ", Space.DuringForcesUpdateables.Time * 1000, 2, new Vector2(20, 95));
        //    Game.TinyTextDrawer.Draw("Bounding Boxes: ", Space.BoundingBoxUpdater.Time * 1000, 2, new Vector2(20, 110));
        //    Game.TinyTextDrawer.Draw("BroadPhase: ", Space.BroadPhase.Time * 1000, 2, new Vector2(20, 125));
        //    Game.TinyTextDrawer.Draw("     Refit: ", (Space.BroadPhase as DynamicHierarchy).RefitTime * 1000, 2, new Vector2(20, 140));
        //    Game.TinyTextDrawer.Draw("     Overlap: ", (Space.BroadPhase as DynamicHierarchy).OverlapTime * 1000, 2, new Vector2(20, 155));
        //    Game.TinyTextDrawer.Draw("BeforeNarrowPhaseUpdateables: ", Space.BeforeNarrowPhaseUpdateables.Time * 1000, 2, new Vector2(20, 170));
        //    Game.TinyTextDrawer.Draw("NarrowPhase: ", Space.NarrowPhase.Time * 1000, 2, new Vector2(20, 185));
        //    Game.TinyTextDrawer.Draw("     Pair Updates: ", Space.NarrowPhase.PairUpdateTime * 1000, 2, new Vector2(20, 200));
        //    Game.TinyTextDrawer.Draw("     Flush New: ", Space.NarrowPhase.FlushNewPairsTime * 1000, 2, new Vector2(20, 215));
        //    Game.TinyTextDrawer.Draw("     Flush Solver Updateables: ", Space.NarrowPhase.FlushSolverUpdateableChangesTime * 1000, 2, new Vector2(20, 230));
        //    Game.TinyTextDrawer.Draw("     Stale Removal: ", Space.NarrowPhase.StaleOverlapRemovalTime * 1000, 2, new Vector2(20, 245));
        //    Game.TinyTextDrawer.Draw("BeforeSolverUpdateables: ", Space.BeforeSolverUpdateables.Time * 1000, 2, new Vector2(20, 260));
        //    Game.TinyTextDrawer.Draw("Solver: ", Space.Solver.Time * 1000, 2, new Vector2(20, 275));
        //    Game.TinyTextDrawer.Draw("BeforePositionUpdateUpdateables: ", Space.BeforePositionUpdateUpdateables.Time * 1000, 2, new Vector2(20, 290));
        //    Game.TinyTextDrawer.Draw("Position Update: ", Space.PositionUpdater.Time * 1000, 2, new Vector2(20, 305));
        //    Game.TinyTextDrawer.Draw("Read Buffers States Update: ", Space.BufferedStates.ReadBuffers.Time * 1000, 2, new Vector2(20, 320));
        //    Game.TinyTextDrawer.Draw("Deferred Event Dispatcher: ", Space.DeferredEventDispatcher.Time * 1000, 2, new Vector2(20, 335));
        //    Game.TinyTextDrawer.Draw("EndOfTimeStepUpdateables: ", Space.EndOfTimeStepUpdateables.Time * 1000, 2, new Vector2(20, 350));


        //    Game.DataTextDrawer.Draw("Total: ", Space.Time * 1000, 2, new Vector2(20, 375));
        //    base.DrawUI();
        //}




    }
}