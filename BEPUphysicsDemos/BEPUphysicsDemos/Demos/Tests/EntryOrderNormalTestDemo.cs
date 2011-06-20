using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using BEPUphysics.Entities;
using System.Diagnostics;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.Collidables;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class EntryOrderNormalTestDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public EntryOrderNormalTestDemo(DemosGame game)
            : base(game)
        {

            int numColumns = 5;
            int numRows = 5;
            int numHigh = 5;
            float separation = 3;

            Entity toAdd;

            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                    for (int k = 0; k < numHigh; k++)
                    {
                        Space.Add(new Box(new Vector3(separation * i, k * separation, separation * j), 1, 1, 1, 1));
                        Space.Add(new Sphere(new Vector3(separation * i, 10 + k * separation, separation * j), .5f, 1));
                        Space.Add(new Cone(new Vector3(separation * i, 20 + k * separation, separation * j), 1, .5f, 1));

                    }

            Space.Add(new Box(new Vector3(0, -5, 0), 100, 1, 100));

        }

        public override void Update(float dt)
        {
            base.Update(dt);
            foreach (var e in Space.Entities)
            {
                foreach (var pair in e.CollisionInformation.Pairs)
                {
                    foreach (var c in pair.Contacts)
                    {
                        float dot = Vector3.Dot(c.Contact.Normal, Vector3.Normalize(c.Contact.Position - (pair.BroadPhaseOverlap.EntryA as EntityCollidable).WorldTransform.Position));
                        if (c.Contact.PenetrationDepth < (((ConvexCollidable)pair.BroadPhaseOverlap.EntryA).Shape.CollisionMargin + ((ConvexCollidable)pair.BroadPhaseOverlap.EntryB).Shape.CollisionMargin) * .8f &&
                            dot < -.2f)
                            Debug.WriteLine("Breka.");

                    }
                }
            }
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Wall"; }
        }
    }
}