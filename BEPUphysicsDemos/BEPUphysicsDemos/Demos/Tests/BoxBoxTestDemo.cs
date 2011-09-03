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
            //float blockWidth = 2;
            //float blockHeight = 2;
            //float blockLength = 6f;
            //Entity toAdd;


            //toAdd = new Box(new Vector3(0, 2,0), blockWidth, blockHeight, blockLength, 20);
            //toAdd.ActivityInformation.IsAlwaysActive = true;
            //toAdd.AllowStabilization = false;
            //Space.Add(toAdd);

            int numColumns = 3;
            int numRows = 3;
            int numHigh = 30;
            float xSpacing = 1.01f;
            float ySpacing = 1.01f;
            float zSpacing = 1.01f;
            for (int i = 0; i < numRows; i++)
                for (int j = 0; j < numColumns; j++)
                    for (int k = 0; k < numHigh; k++)
                    {
                        Space.Add(new Box(new Vector3(
                                                 xSpacing * i - (numRows - 1) * xSpacing / 2f,
                                                 1f + k * (ySpacing),
                                                 zSpacing * j - (numColumns - 1) * zSpacing / 2f),
                                             .5f, .5f, .5f, 5));
                    }

            Space.Add(new Box(new Vector3(0, 0, 0), 20, 1f, 20));
            game.Camera.Position = new Vector3(0, 3, 10);
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