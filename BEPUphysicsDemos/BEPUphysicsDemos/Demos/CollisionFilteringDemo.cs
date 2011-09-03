using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Some objects pass through each other, showing one way in which collision rules function.
    /// </summary>
    public class CollisionFilteringDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public CollisionFilteringDemo(DemosGame game)
            : base(game)
        {
            Entity toAdd;
            toAdd = new Box(new Vector3(0, -.5f, 0), 50, 1, 50);
            Space.Add(toAdd);

            //Set up two stacks which go through each other
            var firstStackGroup = new CollisionGroup();
            var secondStackGroup = new CollisionGroup();
            //Adding this rule to the space's collision group rules will prevent entities belong to these two groups from generating collision pairs with each other.
            groupPair = new CollisionGroupPair(firstStackGroup, secondStackGroup);
            CollisionRules.CollisionGroupRules.Add(groupPair, CollisionRule.NoBroadPhase);

            for (int k = 0; k < 10; k++)
            {
                toAdd = new Box(
                    new Vector3(-4 + .12f * k, .5f + k, 0), 1f, 1f, 1f,
                    10);
                toAdd.CollisionInformation.CollisionRules.Group = firstStackGroup;
                Space.Add(toAdd);
                toAdd = new Box(new Vector3(4 - .12f * k, .5f + k, 0),
                                1f, 1f, 1f, 10);
                toAdd.CollisionInformation.CollisionRules.Group = secondStackGroup;
                Space.Add(toAdd);
            }
            //Add another two boxes which ignore each other using the specific entities method; they will still collide with the stacks since they will have the default dynamic collision group.
            toAdd = new Box(new Vector3(1, 3, 0), 1f, 4f, 2f, 10);
            var toAdd2 = new Box(new Vector3(-1, 3, 0), 1f, 4f, 2f, 15);
            CollisionRules.AddRule(toAdd, toAdd2, CollisionRule.NoBroadPhase);
            Space.Add(toAdd);
            Space.Add(toAdd2);
            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(0, 6, 20);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Collision Filtering"; }
        }

        CollisionGroupPair groupPair;
        public override void CleanUp()
        {
            base.CleanUp();
            //The CollisionGroupRules are static, so this dictionary would fill up
            //if we kept changing the simulation without cleaning it out.
            CollisionRules.CollisionGroupRules.Remove(groupPair);
        }
    }
}