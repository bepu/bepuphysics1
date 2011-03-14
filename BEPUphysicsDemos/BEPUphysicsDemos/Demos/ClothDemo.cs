using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using BEPUphysics.MathExtensions;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// A basic lattice of constraints acting like cloth.
    /// </summary>
    public class ClothDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public ClothDemo(DemosGame game)
            : base(game)
        {
            //Joints can also act like springs by modifying their springSettings.
            //Though using a bunch of DistanceJoint objects can be slower than just applying direct spring forces,
            //it is significantly more stable and allows rigid structures.
            //The extra stability can make it useful for cloth-like simulations.
            Entity latticePiece;
            DistanceJoint distanceJoint;

            int numColumns = 16;
            int numRows = 16;
            float xSpacing = 3.09f;
            float zSpacing = 3.09f;
            var lattice = new Entity[numRows,numColumns];
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                {
                    latticePiece = new Sphere(
                        new Vector3(
                            xSpacing * i - (numRows - 1) * xSpacing / 2f,
                            15.58f,
                            2 + zSpacing * j - (numColumns - 1) * zSpacing / 2f),
                        .3f, 10);

                    latticePiece.LocalInertiaTensorInverse = new Matrix3X3();
                    latticePiece.Tag = "noDisplayObject"; //The joint lines are visible enough; don't add a sphere model for this sphere.
                    lattice[i, j] = latticePiece;

                    Space.Add(latticePiece);
                }
            //The joints composing the cloth can have their max iterations set independently from the solver iterations.
            //More iterations (up to the solver's own max) will increase the quality at the cost of speed.
            int clothIterations = 1;
            //So while the above prevents joints from using more than 1 iteration, setting the solver's iteration limit can lower the
            //rest of the solving load (collisions).
            Space.Solver.IterationLimit = 2;

            //Loop through the grid and set up the joints.
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                {
                    if (i + 1 < numRows)
                    {
                        distanceJoint = new DistanceJoint(lattice[i, j], lattice[i + 1, j], lattice[i, j].Position, lattice[i + 1, j].Position);
                        distanceJoint.SpringSettings.StiffnessConstant = 1000;
                        distanceJoint.SpringSettings.DampingConstant = 1000;
                        distanceJoint.SolverSettings.MaximumIterations = clothIterations;
                        Space.Add(distanceJoint);

                        if (j + 1 < numColumns)
                        {
                            distanceJoint = new DistanceJoint(lattice[i, j], lattice[i + 1, j + 1], lattice[i, j].Position, lattice[i + 1, j + 1].Position);
                            distanceJoint.SpringSettings.StiffnessConstant = 1000;
                            distanceJoint.SpringSettings.DampingConstant = 1000;
                            distanceJoint.SolverSettings.MaximumIterations = clothIterations;
                            Space.Add(distanceJoint);
                        }
                        if (j - 1 >= 0)
                        {
                            distanceJoint = new DistanceJoint(lattice[i, j], lattice[i + 1, j - 1], lattice[i, j].Position, lattice[i + 1, j - 1].Position);
                            distanceJoint.SpringSettings.StiffnessConstant = 1000;
                            distanceJoint.SpringSettings.DampingConstant = 1000;
                            distanceJoint.SolverSettings.MaximumIterations = clothIterations;
                            Space.Add(distanceJoint);
                        }
                    }
                    if (j + 1 < numColumns)
                    {
                        distanceJoint = new DistanceJoint(lattice[i, j], lattice[i, j + 1], lattice[i, j].Position, lattice[i, j + 1].Position);
                        distanceJoint.SpringSettings.StiffnessConstant = 1000;
                        distanceJoint.SpringSettings.DampingConstant = 1000;
                        distanceJoint.SolverSettings.MaximumIterations = clothIterations;
                        Space.Add(distanceJoint);
                    }
                }

      


            //Add some ground.
            Space.Add(new Sphere(new Vector3(0, 0, 0), 10));
            Space.Add(new Box(new Vector3(0, -3.5f, 0), 80f, 1, 80f));

            game.Camera.Position = new Vector3(0, 5, 25);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Cloth Lattice"; }
        }
    }
}