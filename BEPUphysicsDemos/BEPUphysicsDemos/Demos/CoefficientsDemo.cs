using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.Materials;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Boxes slide and spheres bounce.
    /// </summary>
    public class CoefficientsDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public CoefficientsDemo(DemosGame game)
            : base(game)
        {
            //The combine method changes how the properties of each entity in a collision are factored together.
            //Change them around a bit to alter the simulation.
            MaterialManager.FrictionBlendMethod = PropertyBlendMethod.Min; //Defaults to average
            MaterialManager.BouncinessBlendMethod = PropertyBlendMethod.Max;
            //Create the ground
            Entity toAdd = new Box(new Vector3(0, -.5f, 0), 20f, 1f, 20f);
            Space.Add(toAdd);
            //Bouncy balls 
            toAdd = new Sphere(new Vector3(-8, 10, 0), 1, 1);
            toAdd.Material = new Material(.8f, .8f, .95f);
            Space.Add(toAdd);
            toAdd = new Sphere(new Vector3(-5, 10, 0), 1, 1);
            toAdd.Material = new Material(.8f, .8f, .5f);
            Space.Add(toAdd);
            toAdd = new Sphere(new Vector3(-2, 10, 0), 1, 1);
            toAdd.Material = new Material(.8f, .8f, .25f);
            Space.Add(toAdd);
            //Slide-y boxes
            toAdd = new Box(new Vector3(9, 1, 9), 1, 1, 1, 1);
            toAdd.Material = new Material(0,0,0);
            toAdd.LinearVelocity = new Vector3(-5, 0, 0);
            Space.Add(toAdd);
            toAdd = new Box(new Vector3(9, 1, 5), 1, 1, 1, 1);
            toAdd.Material = new Material(.1f, .1f, 0);
            toAdd.LinearVelocity = new Vector3(-5, 0, 0);
            Space.Add(toAdd);
            toAdd = new Box(new Vector3(9, 1, -5), 1, 1, 1, 1);
            toAdd.Material = new Material(.2f, .2f, 0);
            toAdd.LinearVelocity = new Vector3(-5, 0, 0);
            Space.Add(toAdd);
            toAdd = new Box(new Vector3(9, 1, -9), 1, 1, 1, 1);
            toAdd.Material = new Material(.5f, .5f, 0);
            toAdd.LinearVelocity = new Vector3(-5, 0, 0);
            Space.Add(toAdd);
            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(0, 2, 30);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Bounciness and Friction"; }
        }
    }
}