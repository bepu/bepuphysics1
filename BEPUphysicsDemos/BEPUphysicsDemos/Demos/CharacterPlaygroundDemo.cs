using BEPUphysics.Entities.Prefabs;
using BEPUphysics.DataStructures;
using BEPUphysics.MathExtensions;
using BEPUphysics.Collidables;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Entities;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.SolverGroups;
using BEPUphysics.Paths;
using BEPUphysics.Paths.PathFollowing;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Constraints;
using BEPUphysics.CollisionShapes;
using System.Collections.Generic;
using Microsoft.Xna.Framework.Graphics;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// A playground for the character controller to frolic in.
    /// </summary>
    public class CharacterPlaygroundDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public CharacterPlaygroundDemo(DemosGame game)
            : base(game)
        {

            game.Camera.Position = new Microsoft.Xna.Framework.Vector3(-10, 7, 5);
            game.Camera.Yaw = MathHelper.Pi;
            //Since this is the character playground, turn on the character by default.
            character.Activate();
            //Having the character body visible would be a bit distracting.
            character.CharacterController.Body.Tag = "noDisplayObject";

            //Load in mesh data for the environment.
            Vector3[] staticTriangleVertices;
            int[] staticTriangleIndices;


            var playgroundModel = game.Content.Load<Model>("CharacterControllerTestTerrain");
            //This load method wraps the TriangleMesh.GetVerticesAndIndicesFromModel method 
            //to output vertices of type StaticTriangleGroupVertex instead of TriangleMeshVertex or simply Vector3.
            ModelDataExtractor.GetVerticesAndIndicesFromModel(playgroundModel, out staticTriangleVertices, out staticTriangleIndices);
            var staticMesh = new StaticMesh(staticTriangleVertices, staticTriangleIndices, new AffineTransform(new Vector3(.01f, .01f, .01f), Quaternion.Identity, new Vector3(0, 0, 0)));
            staticMesh.Sidedness = TriangleSidedness.Counterclockwise;

            Space.Add(staticMesh);
            game.ModelDrawer.Add(staticMesh);



            //Add a spinning blade for the character to ram itself into.
            var fanBase = new Cylinder(new Vector3(-13, .5f, 50), 1.1f, 1);
            var fanBlade = new Box(fanBase.Position + new Vector3(0, .8f, 0), 5, .1f, 1f, 5);
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
            for (int i = 1; i <= 7; i++)
            {
                position = startPosition + offset * i;
                Box link = new Box(position, 3, .3f, 1.5f, 50);
                link.LinearDamping = .1f;
                link.AngularDamping = .1f;
                Space.Add(link);
                Space.Add(new RevoluteJoint(previousLink, link, position - offset * .5f, Vector3.Right));

                previousLink = link;
            }
            var endPlatform = new Box(position - new Vector3(0, 0, -3.8f), 4, .5f, 6);
            Space.Add(endPlatform);

            Space.Add(new RevoluteJoint(previousLink, endPlatform, position + offset * .5f, Vector3.Right));


            //Add in a floating platform controlled by a curve to serve as an elevator.
            Entity movingEntity = new Box(new Vector3(-10, 0, -10), 3, 1, 3);

            var positionCurve = new CardinalSpline3D();

            positionCurve.PreLoop = CurveEndpointBehavior.Mirror;
            positionCurve.PostLoop = CurveEndpointBehavior.Mirror;

            positionCurve.ControlPoints.Add(-1, new Vector3(-19.3f, 0, 43));
            positionCurve.ControlPoints.Add(0, new Vector3(-19.3f, 0, 43));
            positionCurve.ControlPoints.Add(2, new Vector3(-19.3f, 0, 43));
            positionCurve.ControlPoints.Add(3, new Vector3(-19.3f, 0, 43));
            positionCurve.ControlPoints.Add(4, new Vector3(-19.3f, 5, 43));
            positionCurve.ControlPoints.Add(5f, new Vector3(-19.3f, 10, 43));
            positionCurve.ControlPoints.Add(6f, new Vector3(-19.3f, 10, 43));
            positionCurve.ControlPoints.Add(8f, new Vector3(-19.3f, 10, 43));
            positionCurve.ControlPoints.Add(9f, new Vector3(-19.3f, 10, 43));

            elevatorMover = new EntityMover(movingEntity);
            Space.Add(elevatorMover);
            Space.Add(movingEntity);

            elevatorPath = positionCurve;

            //Add in another floating platform controlled by a curve for horizontal transport.
            movingEntity = new Box(new Vector3(-10, 0, -10), 2.5f, .5f, 2.5f);

            var platformCurve = new LinearInterpolationCurve3D();

            platformCurve.PreLoop = CurveEndpointBehavior.Mirror;
            platformCurve.PostLoop = CurveEndpointBehavior.Mirror;

            platformCurve.ControlPoints.Add(0, new Vector3(-1.75f, 10, 21.5f));
            platformCurve.ControlPoints.Add(2, new Vector3(-1.75f, 10, 21.5f));
            platformCurve.ControlPoints.Add(5, new Vector3(-1.75f, 10, 15.5f));
            platformCurve.ControlPoints.Add(10, new Vector3(-19.3f, 10, 15.5f));
            platformCurve.ControlPoints.Add(12, new Vector3(-19.3f, 10, 15.5f));
            platformCurve.ControlPoints.Add(15, new Vector3(-25, 10, 15.5f));
            platformCurve.ControlPoints.Add(22, new Vector3(-25, 10, 38));
            platformCurve.ControlPoints.Add(23, new Vector3(-22.75f, 10, 38));
            platformCurve.ControlPoints.Add(25, new Vector3(-22.75f, 10, 38));

            //Make it spin too.  That'll be fun.  Or something.
            var platformRotationCurve = new QuaternionSlerpCurve();
            platformRotationCurve.PreLoop = CurveEndpointBehavior.Mirror;
            platformRotationCurve.PostLoop = CurveEndpointBehavior.Mirror;
            platformRotationCurve.ControlPoints.Add(0, Quaternion.Identity);
            platformRotationCurve.ControlPoints.Add(15, Quaternion.Identity);
            platformRotationCurve.ControlPoints.Add(22, Quaternion.CreateFromAxisAngle(Vector3.Up, MathHelper.PiOver2));
            platformRotationCurve.ControlPoints.Add(25, Quaternion.CreateFromAxisAngle(Vector3.Up, MathHelper.PiOver2));

            platformMover = new EntityMover(movingEntity);
            platformRotator = new EntityRotator(movingEntity);
            Space.Add(platformMover);
            Space.Add(platformRotator);
            Space.Add(movingEntity);

            platformPath = platformCurve;
            platformOrientationPath = platformRotationCurve;

            //Add in a diving board.

            Box divingBoardBase = new Box(new Vector3(-9, 10, 39.3f), 5, 1, 3);
            Box divingBoard = new Box(divingBoardBase.Position + new Vector3(-2, 0, 3.5f), 1, .3f, 3, 5);
            var divingBoardJoint = new RevoluteJoint(divingBoardBase, divingBoard, divingBoard.Position + new Vector3(0, 0, -1.5f), Vector3.Right);
            divingBoardJoint.Motor.IsActive = true;
            divingBoardJoint.Motor.Settings.Mode = MotorMode.Servomechanism;
            divingBoardJoint.Motor.Settings.Servo.Goal = 0;
            divingBoardJoint.Motor.Settings.Servo.SpringSettings.StiffnessConstant = 5000;
            divingBoardJoint.Motor.Settings.Servo.SpringSettings.DampingConstant = 0;

            Space.Add(divingBoardBase);
            Space.Add(divingBoard);
            Space.Add(divingBoardJoint);


            //Add a second diving board for comparison.

            Box divingBoard2 = new Box(divingBoardBase.Position + new Vector3(2, 0, 5f), 1, .3f, 6, 5);
            var divingBoardJoint2 = new RevoluteJoint(divingBoardBase, divingBoard2, divingBoard2.Position + new Vector3(0, 0, -3), Vector3.Right);
            divingBoardJoint2.Motor.IsActive = true;
            divingBoardJoint2.Motor.Settings.Mode = MotorMode.Servomechanism;
            divingBoardJoint2.Motor.Settings.Servo.Goal = 0;
            divingBoardJoint2.Motor.Settings.Servo.SpringSettings.StiffnessConstant = 10000;
            divingBoardJoint2.Motor.Settings.Servo.SpringSettings.DampingConstant = 0;

            Space.Add(divingBoard2);
            Space.Add(divingBoardJoint2);

            //Add a seesaw for people to jump on.
            Box seesawBase = new Box(new Vector3(-7, .45f, 52), 1, .9f, .3f);
            Box seesawPlank = new Box(seesawBase.Position + new Vector3(0, .65f, 0), 1.2f, .2f, 6, 3);
            RevoluteJoint seesawJoint = new RevoluteJoint(seesawBase, seesawPlank, seesawPlank.Position, Vector3.Right);
            Space.Add(seesawJoint);
            Space.Add(seesawBase);
            Space.Add(seesawPlank);

            Space.Add(new Box(seesawPlank.Position + new Vector3(0, 1.3f, 2), 1, 1, 1, 5));


            //Add in some boxes to bump and jump on.
            int numColumns = 3;
            int numRows = 3;
            int numHigh = 3;
            float xSpacing = 1.01f;
            float ySpacing = 1.01f;
            float zSpacing = 1.01f;
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                    for (int k = 0; k < numHigh; k++)
                    {
                        Space.Add(new Box(new Vector3(
                                                 5 + xSpacing * i - (numRows - 1) * xSpacing / 2f,
                                                 1.58f + k * (ySpacing),
                                                 45 + zSpacing * j - (numColumns - 1) * zSpacing / 2f),
                                             .5f, .5f, .5f, 5));
                    }



            //Add a log to roll!
            //Make it a compound so some boxes can be added to let the player know it's actually spinning.
            CompoundBody log = new CompoundBody(new List<CompoundShapeEntry>()
            {
                new CompoundShapeEntry(new CylinderShape(4, 1.8f), Quaternion.CreateFromAxisAngle(Vector3.Forward, MathHelper.PiOver2), 20),
                new CompoundShapeEntry(new BoxShape(.5f, .5f, 3.7f),  new Vector3(1.75f, 0,0), 0),
                new CompoundShapeEntry(new BoxShape(.5f, 3.7f, .5f), new Vector3(1.75f, 0,0), 0),
                new CompoundShapeEntry(new BoxShape(.5f, .5f, 3.7f),  new Vector3(-1.75f, 0,0), 0),
                new CompoundShapeEntry(new BoxShape(.5f, 3.7f, .5f), new Vector3(-1.75f, 0,0), 0)
            }, 50);
            log.Position = new Vector3(-14.5f, 10, 41);
            log.AngularDamping = 0;


            RevoluteJoint logJointA = new RevoluteJoint(divingBoardBase, log, log.Position + new Vector3(2.5f, 0, 0), Vector3.Right);
            RevoluteJoint logJointB = new RevoluteJoint(endPlatform, log, log.Position + new Vector3(-2.5f, 0, 0), Vector3.Right);
            Space.Add(logJointA);
            Space.Add(logJointB);

            Space.Add(log);


            //Put some planks to stand on that show various slopes.
            int numPads = 10;
            for (int i = 0; i < numPads; i++)
            {
                offset = new Vector3(0, 0, 4);
                Box a = new Box(new Vector3(i * 1.5f + 3.5f, 10, 24), 1.5f, 1, 4);
                Box b = new Box(new Vector3(i * 1.5f + 3.5f, 10, 24), 1.5f, 1, 4);
                float angle = -i * MathHelper.PiOver2 / numPads;
                b.Orientation = Quaternion.CreateFromAxisAngle(Vector3.Right, angle);
                b.Position += offset * .5f + Vector3.Transform(offset * .5f, b.Orientation);

                Space.Add(a);
                Space.Add(b);
            }
        }

        EntityMover elevatorMover;
        Path<Vector3> elevatorPath;
        EntityMover platformMover;
        EntityRotator platformRotator;
        Path<Vector3> platformPath;
        Path<Quaternion> platformOrientationPath;
        double pathTime;


        public override void Update(float dt)
        {
            //Increment the time.  Note that the space's timestep is used
            //instead of the method's dt.  This is because the demos, by
            //default, update the space once each game update.  Using the
            //space's update time keeps things synchronized.
            //If the engine is using internal time stepping,
            //the passed in dt should be used instead (or put this logic into
            //an updateable that runs with space updates).
            pathTime += Space.TimeStepSettings.TimeStepDuration;
            elevatorMover.TargetPosition = elevatorPath.Evaluate(pathTime);
            platformMover.TargetPosition = platformPath.Evaluate(pathTime);
            platformRotator.TargetOrientation = platformOrientationPath.Evaluate(pathTime);
            base.Update(dt);
        }

        public override void DrawUI()
        {
#if XBOX360
            Game.DataTextDrawer.Draw("Press \"A\" to toggle the character.", new Microsoft.Xna.Framework.(50, 50));
#else
            Game.DataTextDrawer.Draw("Press \"C\" to toggle the character.", new Microsoft.Xna.Framework.Vector2(50, 50));
#endif
            base.DrawUI();
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