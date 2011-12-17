
using System;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysicsDemos.SampleCode;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionShapes.ConvexShapes;
using System.Collections.Generic;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// A spaceship blasts off into the sky (void).
    /// </summary>
    public class SpaceshipDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public SpaceshipDemo(DemosGame game)
            : base(game)
        {
            //Build the ship
            var shipFuselage = new CompoundShapeEntry(new CylinderShape(3, .7f), new Vector3(0, 5, 0), 4);
            var shipNose = new CompoundShapeEntry(new ConeShape(2, .7f), new Vector3(0, 7f, 0), 2);
            var shipWing = new CompoundShapeEntry(new BoxShape(5f, 2, .2f), new Vector3(0, 5, 0), 3);
            var shipThrusters = new CompoundShapeEntry(new ConeShape(1, .5f), new Vector3(0, 3.25f, 0), 1);

            var bodies = new List<CompoundShapeEntry>();
            bodies.Add(shipFuselage);
            bodies.Add(shipNose);
            bodies.Add(shipWing);
            bodies.Add(shipThrusters);

            var ship = new CompoundBody(bodies, 10);

            //Setup the launch pad and ramp
            Entity toAdd = new Box(new Vector3(10, 4, 0), 26, 1f, 6);
            Space.Add(toAdd);
            toAdd = new Box(new Vector3(32, 7.8f, 0), 20, 1, 6);
            toAdd.Orientation = Quaternion.CreateFromAxisAngle(Vector3.Forward, -(float)Math.PI / 8);
            Space.Add(toAdd);
            toAdd = new Box(new Vector3(32, 8.8f, -3.5f), 20, 1, 1);
            toAdd.Orientation = Quaternion.CreateFromAxisAngle(Vector3.Forward, -(float)Math.PI / 8);
            Space.Add(toAdd);
            toAdd = new Box(new Vector3(32, 8.8f, 3.5f), 20, 1, 1);
            toAdd.Orientation = Quaternion.CreateFromAxisAngle(Vector3.Forward, -(float)Math.PI / 8);
            Space.Add(toAdd);
            toAdd = new Box(new Vector3(-2.75f, 5.5f, 0), .5f, 2f, 3);
            Space.Add(toAdd);

            //Blast-off!
            ship.AngularDamping = .4f; //Helps keep the rocket on track for a little while longer :D
            var thruster = new Thruster(ship, new Vector3(0, -2, 0), new Vector3(0, 300, 0), 0);
            Space.Add(thruster);
            ship.Orientation = Quaternion.CreateFromAxisAngle(Vector3.Right, (float)Math.PI / 2) * Quaternion.CreateFromAxisAngle(Vector3.Forward, (float)Math.PI / 2);
            Space.Add(ship);


            game.Camera.Position = new Vector3(-14, 12, 25);
            game.Camera.Yaw = (float)Math.PI / -4;
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Spaceship"; }
        }
    }
}