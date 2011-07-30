using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using BEPUphysics.DataStructures;
using BEPUphysics.MathExtensions;
using BEPUphysics.Collidables;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Entities;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.SolverGroups;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// A playground for the character controller to frolic in.
    /// </summary>
    public class CharacterTestDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public CharacterTestDemo(DemosGame game)
            : base(game)
        {

            //Load in mesh data for the environment.
            Vector3[] staticTriangleVertices;
            int[] staticTriangleIndices;


            var playgroundModel = game.Content.Load<Model>("CharacterControllerTestTerrain");
            //This load method wraps the TriangleMesh.GetVerticesAndIndicesFromModel method 
            //to output vertices of type StaticTriangleGroupVertex instead of TriangleMeshVertex or simply Vector3.
            TriangleMesh.GetVerticesAndIndicesFromModel(playgroundModel, out staticTriangleVertices, out staticTriangleIndices);
            //staticTriangleIndices = new int[] { 0, 2, 1, 0, 3, 2 };
            //staticTriangleVertices = new Vector3[] { new Vector3(-20, 0, -20), new Vector3(20, 0, -20), new Vector3(20, 0, 20), new Vector3(-20, -4, 20) };
            var staticMesh = new StaticMesh(staticTriangleVertices, staticTriangleIndices, new AffineTransform(new Vector3(.01f, .01f, .01f), Quaternion.Identity, new Vector3(0, 0, 0)));
            staticMesh.Sidedness = TriangleSidedness.Counterclockwise;

            Space.Add(staticMesh);
            game.ModelDrawer.Add(staticMesh);


            //Add a spinning blade for the character to ram itself into.
            var fanBase = new Cylinder(new Vector3(-10, .5f, 50), 1, 1);
            var fanBlade = new Box(fanBase.Position + new Vector3(0, .7f, 0), 5, .2f, 1, 5);
            var fanJoint = new RevoluteJoint(fanBase, fanBlade, (fanBase.Position + fanBlade.Position) * .5f, Vector3.Up);
            fanJoint.Motor.IsActive = true;
            fanJoint.Motor.Settings.VelocityMotor.GoalVelocity = 30;
            fanJoint.Motor.Settings.MaximumForce = 300;
            Space.Add(fanBase);
            Space.Add(fanBlade);
            Space.Add(fanJoint);

            //Add a bridge connecting the two towers.
            Vector3 startPosition = new Vector3(-19.3f, 10.5f - .25f, 23 - .85f);
            var startPlatform = new Box(startPosition - new Vector3(0, 0, 2.2f), 4, .5f, 6);
            Space.Add(startPlatform);
            Vector3 offset = new Vector3(0, 0, 1.7f);
            Box previousLink = startPlatform;
            Vector3 position = new Vector3();
            float stiffnessConstant = 35000;
            float dampingConstant = 50000;
            BallSocketJoint joint;
            for (int i = 1; i <= 7; i++)
            {
                position = startPosition + offset * i;
                Box link = new Box(position, 3, .3f, 1.5f, 50);
                link.LinearDamping = .1f;
                link.AngularDamping = .1f;
                Space.Add(link);
                joint = new BallSocketJoint(previousLink, link, position - offset * .5f + new Vector3(-1.5f, 0, 0));
                joint.SpringSettings.StiffnessConstant = stiffnessConstant;
                joint.SpringSettings.DampingConstant = dampingConstant;
                Space.Add(joint);
                joint = new BallSocketJoint(previousLink, link, position - offset * .5f + new Vector3(1.5f, 0, 0));
                joint.SpringSettings.StiffnessConstant = stiffnessConstant;
                joint.SpringSettings.DampingConstant = dampingConstant;
                Space.Add(joint); 
                previousLink = link;
            }
            var endPlatform = new Box(position - new Vector3(0, 0, -3.8f), 4, .5f, 6);
            Space.Add(endPlatform);
            joint = new BallSocketJoint(previousLink, endPlatform, position + offset * .5f + new Vector3(-1.5f, 0, 0));
            joint.SpringSettings.StiffnessConstant = stiffnessConstant;
            joint.SpringSettings.DampingConstant = dampingConstant;
            Space.Add(joint);
            joint = new BallSocketJoint(previousLink, endPlatform, position + offset * .5f + new Vector3(1.5f, 0, 0));
            joint.SpringSettings.StiffnessConstant = stiffnessConstant;
            joint.SpringSettings.DampingConstant = dampingConstant;
            Space.Add(joint); 




            game.Camera.Position = new Vector3(0, 10, 0);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Character Playground"; }
        }
    }
}