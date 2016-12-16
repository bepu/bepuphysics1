using System;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.BroadPhaseEntries.Events;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using ConversionHelper;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Piling up kinematic objects to test some corner cases related to dynamic->kinematic switching.
    /// </summary>
    public class AccumulationTestDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public AccumulationTestDemo(DemosGame game)
            : base(game)
        {
            //x and y, in terms of heightmaps, refer to their local x and y coordinates.  In world space, they correspond to x and z.
            //Setup the heights of the terrain.
            int xLength = 256;
            int zLength = 256;

            float xSpacing = .5f;
            float zSpacing = .5f;
            var heights = new float[xLength, zLength];
            for (int i = 0; i < xLength; i++)
            {
                for (int j = 0; j < zLength; j++)
                {
                    float x = i - xLength / 2;
                    float z = j - zLength / 2;
                    heights[i, j] = (float)(10 * (Math.Sin(x / 8) + Math.Sin(z / 8)));
                }
            }
            //Create the terrain.
            var terrain = new Terrain(heights, new AffineTransform(
                    new Vector3(xSpacing, 1, zSpacing),
                    Quaternion.Identity,
                    new Vector3(-xLength * xSpacing / 2, 0, -zLength * zSpacing / 2)));

            Space.Add(terrain);
            game.ModelDrawer.Add(terrain);




            eventHandler = (sender, other, pair) =>
            {

                var entityCollidable = other as EntityCollidable;
                if (entityCollidable == null || !entityCollidable.Entity.IsDynamic)
                {
                    sender.Events.RemoveAllEvents();
                    sender.Entity.LinearVelocity = new Vector3();
                    sender.Entity.AngularVelocity = new Vector3();
                    sender.Entity.BecomeKinematic();
                    sender.CollisionRules.Group = CollisionRules.DefaultDynamicCollisionGroup;
                }
            };

            game.Camera.Position = new Vector3(0, 30, 20);

        }

        InitialCollisionDetectedEventHandler<EntityCollidable> eventHandler;

        void Launch()
        {
            Sphere sphere = new Sphere(Game.Camera.Position, 1, 10);
            sphere.CollisionInformation.Events.InitialCollisionDetected += eventHandler;
            sphere.LinearVelocity = Game.Camera.WorldMatrix.Forward * 30;
            Space.Add(sphere);
            Game.ModelDrawer.Add(sphere);
        }


        public override void Update(float dt)
        {
#if XBOX360
            if(Game.GamePadInput.Triggers.Left > .5f)
#else
            if (Game.MouseInput.RightButton == Microsoft.Xna.Framework.Input.ButtonState.Pressed)
#endif
                Launch();
            base.Update(dt);

        }


        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Accumulation Test"; }
        }
    }
}