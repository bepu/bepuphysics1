using BEPUphysics.Entities.Prefabs;
using BEPUphysics.PositionUpdating;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Demonstrates the difference between discrete collision detection and continuous collision detection.
    /// The default position update mode for objects is Discrete, which means they blindly move forward 
    /// based on their velocities every frame.  Continuous mode objects perform a sweep test based on the
    /// frame's motion, preventing the object from moving past a collision that would have otherwise significantly
    /// altered its trajectory.
    /// 
    /// When a continuous object hits a moving discrete object, the discrete object will still move all the way
    /// through its frame's motion and the continuous object will sweep against the discrete's final position.
    /// 
    /// There's a third update mode called "Passive," which is a midpoint between Discrete and Continuous.
    /// A discrete object won't perform sweep tests by itself, but it will listen to sweep tests requested by other
    /// Continuous objects.  The passive object will then only move up to the first point of contact, rather than 
    /// all the way through its frame's motion like a Discrete would.
    /// 
    /// The default mode that entities use can be changed in the MotionSettings.DefaultPositionUpdateMode field.
    /// </summary>
    public class DiscreteVsContinuousDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public DiscreteVsContinuousDemo(DemosGame game)
            : base(game)
        {
            //Create the discretely updated spheres.  These will fly through the ground due to their high speed.
            var toAdd = new Sphere(new Vector3(-6, 150, 0), .5f, 1);
            toAdd.LinearVelocity = new Vector3(0, -100, 0);
            Space.Add(toAdd);

            toAdd = new Sphere(new Vector3(-4, 150, 0), .25f, 1);
            toAdd.LinearVelocity = new Vector3(0, -100, 0);
            Space.Add(toAdd);

            toAdd = new Sphere(new Vector3(-2, 150, 0), .1f, 1);
            toAdd.LinearVelocity = new Vector3(0, -100, 0);
            Space.Add(toAdd);

            Space.Add(new Box(new Vector3(0, 0, 0), 20f, .1f, 20));


            //Create the continuously updated spheres.  These will hit the ground and stop.
            toAdd = new Sphere(new Vector3(6, 150, 0), .5f, 1);
            toAdd.LinearVelocity = new Vector3(0, -100, 0);
            toAdd.PositionUpdateMode = PositionUpdateMode.Continuous;
            Space.Add(toAdd);

            toAdd = new Sphere(new Vector3(4, 150, 0), .25f, 1);
            toAdd.LinearVelocity = new Vector3(0, -100, 0);
            toAdd.PositionUpdateMode = PositionUpdateMode.Continuous;
            Space.Add(toAdd);

            toAdd = new Sphere(new Vector3(2, 150, 0), .1f, 1);
            toAdd.LinearVelocity = new Vector3(0, -100, 0);
            toAdd.PositionUpdateMode = PositionUpdateMode.Continuous;
            Space.Add(toAdd);


            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(0, 0, 30);

        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Discrete vs Continuous"; }
        }
    }
}