using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.DataStructures;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysicsDrawer.Models;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.NarrowPhaseSystems.Pairs;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Put the fish in a barrel, and a box moves.
    /// Game of the year!
    /// </summary>
    public class FishInABarrelDemo : StandardDemo
    {
        //Some entities to keep track of for the demo's events.
        private readonly Collidable acceptedTrigger;
        private readonly Entity movedBox;

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public FishInABarrelDemo(DemosGame game)
            : base(game)
        {
            game.Camera.Position = new Vector3(0, 7, 30);

            var detector = new Box(new Vector3(0, 0, 0), 1.5f, 1.5f, 1.5f);
            detector.CollisionInformation.CollisionRules.Personal = CollisionRule.NoSolver;
            var acceptedTriggerEntity = new Box(new Vector3(5, 0, 0), 1.6f, .7f, .4f, 1);
            acceptedTrigger = acceptedTriggerEntity.CollisionInformation;

            detector.Tag = "noDisplayObject";
            acceptedTriggerEntity.Tag = "noDisplayObject";
            Space.Add(detector);
            Space.Add(acceptedTriggerEntity);

            var fish = game.Content.Load<Model>("fish");
            game.ModelDrawer.Add(new DisplayEntityModel(acceptedTriggerEntity, fish, game.ModelDrawer));

            var barrelAndPlatform = game.Content.Load<Model>("barrelAndPlatform");
            Vector3[] staticTriangleVertices;
            int[] staticTriangleIndices;
            TriangleMesh.GetVerticesAndIndicesFromModel(barrelAndPlatform, out staticTriangleVertices, out staticTriangleIndices);

            //Note that the final 'margin' parameter is optional, but can be used to specify a collision margin on triangles in the static triangle group.
            var fishDepositoryGroup = new StaticMesh(staticTriangleVertices, staticTriangleIndices);
            CollisionRules.AddRule(fishDepositoryGroup, detector, CollisionRule.NoBroadPhase);
            Space.Add(fishDepositoryGroup);
            game.ModelDrawer.Add(fishDepositoryGroup);


            movedBox = new Box(new Vector3(-4, 5, 0), 1, 1, 1, 1);
            detector.Space.Add(movedBox);
            detector.CollisionInformation.Events.InitialCollisionDetected += InitialCollisionDetected;
            detector.CollisionInformation.Events.CollisionEnded += CollisionEnded;

        }



        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Fish in a Barrel"; }
        }


        public void InitialCollisionDetected(EntityCollidable sender, Collidable other, CollidablePairHandler collisionPair)
        {
            if (other == acceptedTrigger)
            {
                //If the detector collided with the accepted trigger, move the box.
                movedBox.Position = new Vector3(4, 5, 0);
                movedBox.Orientation = Quaternion.Identity;
                movedBox.LinearVelocity = Vector3.Zero;
                movedBox.AngularVelocity = Vector3.Zero;

            }
        }

        public void CollisionEnded(EntityCollidable sender, Collidable other, CollidablePairHandler collisionPair)
        {
            if (other == acceptedTrigger)
            {
                //If the detector ceases to collide, get rid of the spawned box.
                movedBox.Position = new Vector3(-4, 5, 0);
                movedBox.Orientation = Quaternion.Identity;
                movedBox.LinearVelocity = Vector3.Zero;
                movedBox.AngularVelocity = Vector3.Zero;
            }
        }

        public override void DrawUI()
        {
            Game.DataTextDrawer.Draw("Put the fish in the barrel!", new Vector2(50, 50));
            base.DrawUI();
        }
    }
}