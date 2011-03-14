using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using System.Collections.Generic;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Compound bodies are created from other entities to make concave shapes.
    /// </summary>
    public class CompoundBodiesDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public CompoundBodiesDemo(DemosGame game)
            : base(game)
        {

            //Build the first body
            var bodies = new List<DynamicCompoundEntry>() 
            {
                new DynamicCompoundEntry(new SphereShape(.5f), new Vector3(0, 1, 0), 15),
                new DynamicCompoundEntry(new ConeShape(2, .5f), new Vector3(1, 1, 0), 15),
                new DynamicCompoundEntry(new SphereShape(.5f), new Vector3(-1, 1, 0), 15)
            };
            var cb1 = new CompoundBody(bodies);



            //Build the second body
            bodies = new List<DynamicCompoundEntry>() 
            {
                new DynamicCompoundEntry(new BoxShape(1,1,1), new Vector3(0, 3, 0), 2),
                new DynamicCompoundEntry(new BoxShape(1,1,1), new Vector3(1, 3.5f, 0), 2),
            };
            var cb2 = new CompoundBody(bodies);

            bodies = new List<DynamicCompoundEntry>();
            //Build the third Braum's-fry style body
            for (int k = 0; k < 7; k++)
            {
                bodies.Add(new DynamicCompoundEntry(new BoxShape(1, 1, 1), new Vector3(-4 + k * .7f, 2 + .7f * k, 2 + k * .2f), 1));
            }
            var cb3 = new CompoundBody(bodies);

            //Add them all to the space
            Space.Add(cb3);
            Space.Add(cb2);
            Space.Add(cb1);



            Space.Add(new Box(new Vector3(0, -.5f, 0), 10, 1, 10));
            game.Camera.Position = new Vector3(0, 3, 15);

        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Compound Bodies"; }
        }
    }
}