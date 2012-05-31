using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.PositionUpdating;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// A set of jenga blocks.
    /// </summary>
    public class JengaDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public JengaDemo(DemosGame game)
            : base(game)
        {

            Space.Remove(kapow);
            //Have to shrink the ball a little to make it fit between jenga tower blocks.
            kapow = new Sphere(new Vector3(-11000, 0, 0), .2f, 20);
            kapow.PositionUpdateMode = PositionUpdateMode.Continuous; //The ball's really tiny! It will work better if it's handled continuously.
            Space.Add(kapow);
            int numBlocksTall = 18; //How many 'stories' tall.
            float blockWidth = 3f; //Total width/length of the tower.
            float blockHeight = 1 / 2f;
            Entity toAdd;

            //The default number of iterations is 10, which works fine, but this demo
            //is all about stability (it's jenga!).  Increase the iterations a bit.
            //Even though it's using twice as many iterations, it will early-out
            //before reaching the limit MOST of the time.
            //It's still pretty playable at around 7-8 max iterations, though.
            Space.Solver.IterationLimit = 20;

            for (int i = 0; i < numBlocksTall; i++)
            {
                if (i % 2 == 0)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        toAdd = new Box(new Vector3(
                                            j * (blockWidth / 3) - blockWidth / 3f,
                                            blockHeight / 2 + i * (blockHeight),
                                            0),
                                        blockWidth / 3, blockHeight, blockWidth, 10);
                        Space.Add(toAdd);
                    }
                }
                else
                {
                    for (int j = 0; j < 3; j++)
                    {
                        toAdd = new Box(new Vector3(
                                            0,
                                            blockHeight / 2 + (i) * (blockHeight),
                                            j * (blockWidth / 3) - blockWidth / 3f),
                                        blockWidth, blockHeight, blockWidth / 3, 10);
                        Space.Add(toAdd);

                    }
                }
            }
            Space.Add(new Box(new Vector3(0, -.5f, 0), 40, 1, 40));
            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(0, 5, 15);

        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Jenga"; }
        }
    }
}