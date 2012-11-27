using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.MathExtensions;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.NarrowPhaseSystems;

namespace BEPUphysicsDemos.Demos.Tests
{
    /// <summary>
    /// A basic lattice of constraints acting like cloth.
    /// </summary>
    public class SelfCollidingClothDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public SelfCollidingClothDemo(DemosGame game)
            : base(game)
        {
            //Joints can also act like springs by modifying their springSettings.
            //Though using a bunch of DistanceJoint objects can be slower than just applying direct spring forces,
            //it is significantly more stable and allows rigid structures.
            //The extra stability can make it useful for cloth-like simulations.
            Entity latticePiece;
            BallSocketJoint joint;

            NarrowPhaseHelper.Factories.BoxBox.Count = 10000;
            NarrowPhaseHelper.Factories.BoxSphere.Count = 5000;
            
            int numColumns = 70;
            int numRows = 70;
            float xSpacing = .5f;
            float zSpacing = .5f;
            var lattice = new Entity[numRows, numColumns];
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                {
                    latticePiece = new Box(
                        new Vector3(
                            xSpacing * i - (numRows - 1) * xSpacing / 2f,
                            15.58f,
                            2 + zSpacing * j - (numColumns - 1) * zSpacing / 2f),
                        xSpacing, .1f, zSpacing, 1);

                    //latticePiece.LocalInertiaTensorInverse = new Matrix3X3();
                    //latticePiece.Tag = "noDisplayObject"; //The joint lines are visible enough; don't add a sphere model for this sphere.
                    lattice[i, j] = latticePiece;
                    latticePiece.Material.KineticFriction = 0;

                    Space.Add(latticePiece);
                }
            //The joints composing the cloth can have their max iterations set independently from the solver iterations.
            //More iterations (up to the solver's own max) will increase the quality at the cost of speed.
            int clothIterations = 10;
            //So while the above prevents joints from using more than 1 iteration, setting the solver's iteration limit can lower the
            //rest of the solving load (collisions).
            Space.Solver.IterationLimit = 10;

            float damping = 5000, stiffness = 5000;
            float starchDamping = 5000, starchStiffness = 500;

            //Loop through the grid and set up the joints.
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                {
                    if (i == 0 && j + 1 < numColumns)
                    {
                        //Add in column connections for left edge.
                        joint = new BallSocketJoint(lattice[0, j], lattice[0, j + 1], lattice[0, j].Position + new Vector3(-xSpacing / 2, 0, zSpacing / 2));
                        joint.SpringSettings.DampingConstant = damping; joint.SpringSettings.StiffnessConstant = stiffness;
                        joint.SolverSettings.MaximumIterations = clothIterations;
                        Space.Add(joint);
                    }
                    if (i == numRows - 1 && j + 1 < numColumns)
                    {
                        //Add in column connections for right edge.
                        joint = new BallSocketJoint(lattice[numRows - 1, j], lattice[numRows - 1, j + 1], lattice[numRows - 1, j].Position + new Vector3(xSpacing / 2, 0, zSpacing / 2));
                        joint.SpringSettings.DampingConstant = damping; joint.SpringSettings.StiffnessConstant = stiffness;
                        joint.SolverSettings.MaximumIterations = clothIterations;
                        Space.Add(joint);
                    }
                    if (i + 1 < numRows && j == 0)
                    {
                        //Add in row connections for top edge.
                        joint = new BallSocketJoint(lattice[i, 0], lattice[i + 1, 0], lattice[i, 0].Position + new Vector3(xSpacing / 2, 0, -zSpacing / 2));
                        joint.SpringSettings.DampingConstant = damping; joint.SpringSettings.StiffnessConstant = stiffness;
                        joint.SolverSettings.MaximumIterations = clothIterations;
                        Space.Add(joint);
                    }
                    if (i + 1 < numRows && j == numColumns - 1)
                    {
                        //Add in row connections for bottom edge.
                        joint = new BallSocketJoint(lattice[i, numColumns - 1], lattice[i + 1, numColumns - 1], lattice[i, numColumns - 1].Position + new Vector3(xSpacing / 2, 0, zSpacing / 2));
                        joint.SpringSettings.DampingConstant = damping; joint.SpringSettings.StiffnessConstant = stiffness;
                        joint.SolverSettings.MaximumIterations = clothIterations;
                        Space.Add(joint);

                    }


                    if (i + 1 < numRows && j + 1 < numColumns)
                    {
                        //Add in interior connections.
                        joint = new BallSocketJoint(lattice[i, j], lattice[i + 1, j], lattice[i, j].Position + new Vector3(xSpacing / 2, 0, zSpacing / 2));
                        joint.SpringSettings.DampingConstant = damping; joint.SpringSettings.StiffnessConstant = stiffness;
                        joint.SolverSettings.MaximumIterations = clothIterations;
                        Space.Add(joint);

                        joint = new BallSocketJoint(lattice[i, j], lattice[i, j + 1], lattice[i, j].Position + new Vector3(xSpacing / 2, 0, zSpacing / 2));
                        joint.SpringSettings.DampingConstant = damping; joint.SpringSettings.StiffnessConstant = stiffness;
                        joint.SolverSettings.MaximumIterations = clothIterations;
                        Space.Add(joint);

                        joint = new BallSocketJoint(lattice[i, j], lattice[i + 1, j + 1], lattice[i, j].Position + new Vector3(xSpacing / 2, 0, zSpacing / 2));
                        joint.SpringSettings.DampingConstant = damping; joint.SpringSettings.StiffnessConstant = stiffness;
                        joint.SolverSettings.MaximumIterations = clothIterations;
                        Space.Add(joint);
                    }

                    if (i + 2 < numRows && j + 2 < numColumns)
                    {
                        //Add in skipping 'starch' connections.
                        joint = new BallSocketJoint(lattice[i, j], lattice[i + 2, j], lattice[i, j].Position + new Vector3(xSpacing, 0, zSpacing));
                        joint.SpringSettings.DampingConstant = starchDamping; joint.SpringSettings.StiffnessConstant = starchStiffness;
                        joint.SolverSettings.MaximumIterations = clothIterations;
                        Space.Add(joint);

                        joint = new BallSocketJoint(lattice[i, j], lattice[i, j + 2], lattice[i, j].Position + new Vector3(xSpacing, 0, zSpacing));
                        joint.SpringSettings.DampingConstant = starchDamping; joint.SpringSettings.StiffnessConstant = starchStiffness;
                        joint.SolverSettings.MaximumIterations = clothIterations;
                        Space.Add(joint);

                        joint = new BallSocketJoint(lattice[i, j], lattice[i + 2, j + 2], lattice[i, j].Position + new Vector3(xSpacing, 0, zSpacing));
                        joint.SpringSettings.DampingConstant = starchDamping; joint.SpringSettings.StiffnessConstant = starchStiffness;
                        joint.SolverSettings.MaximumIterations = clothIterations;
                        Space.Add(joint);
                    }

                    //Add in collision rules.
                    if (j - 1 >= 0)
                    {
                        if (i - 1 >= 0) CollisionRules.AddRule(lattice[i, j], lattice[i - 1, j - 1], CollisionRule.NoBroadPhase);
                        CollisionRules.AddRule(lattice[i, j], lattice[i, j - 1], CollisionRule.NoBroadPhase);
                        if (i + 1 < numRows) CollisionRules.AddRule(lattice[i, j], lattice[i + 1, j - 1], CollisionRule.NoBroadPhase);
                    }

                    if (i + 1 < numRows) CollisionRules.AddRule(lattice[i, j], lattice[i + 1, j], CollisionRule.NoBroadPhase);
                }




            //Add some ground.
            Space.Add(new Sphere(new Vector3(7, 0, 0), 10));
            Space.Add(new Box(new Vector3(0, -20.5f, 0), 100f, 10, 100f));

            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(0, 5, 25);
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