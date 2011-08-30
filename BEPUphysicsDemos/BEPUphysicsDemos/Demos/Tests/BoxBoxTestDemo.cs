using System;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using System.Diagnostics;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Ring-shaped structure made of blocks.
    /// </summary>
    public class BoxBoxTestDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public BoxBoxTestDemo(DemosGame game)
            : base(game)
        {
            float blockWidth = 2;
            float blockHeight = 2;
            float blockLength = 6f;
            Entity toAdd;
            Space.Add(new Box(new Vector3(0, 0, 0), 20, 2f, 20));

            toAdd = new Box(new Vector3(0, 2,0), blockWidth, blockHeight, blockLength, 20);
            toAdd.ActivityInformation.IsAlwaysActive = true;
            toAdd.AllowStabilization = false;
            Space.Add(toAdd);

            game.Camera.Position = new Vector3(0, 2, 2);
        }

        public override void Update(float dt)
        {
            if (Game.WasKeyPressed(Microsoft.Xna.Framework.Input.Keys.P))
                Debug.WriteLine("break.");
            base.Update(dt);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Colosseum"; }
        }
    }
}