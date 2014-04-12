
using System;
using BEPUphysics.Constraints;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.Materials;
using BEPUphysics.Settings;
using BEPUutilities;
using Microsoft.Xna.Framework.Input;

namespace BEPUphysicsDemos.Demos.Extras
{
    /// <summary>
    /// Bunch of blocks arranged in a 3d pyramid, waiting to be blown up.
    /// </summary>
    public class SolidPyramidDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public SolidPyramidDemo(DemosGame game)
            : base(game)
        {
            float boxSize = 1f;
            int bottomBoxCount = 10;

            var ground = new Box(new Vector3(0, -.5f, 0), 40, 1, 40);
            Space.Add(ground);

            float spacing = 0.05f;

            float offset = -0.5f * ((bottomBoxCount - 1) * (boxSize + spacing));
            var origin = new Vector3(offset, -boxSize * 0.5f, offset);
            for (int heightIndex = 0; heightIndex < bottomBoxCount - 2; ++heightIndex)
            {
                var levelWidth = bottomBoxCount - heightIndex;
                float perBoxWidth = boxSize + spacing;
                //Move the origin for this level.
                origin.X += perBoxWidth * 0.5f;
                origin.Y += boxSize;
                origin.Z += perBoxWidth * 0.5f;

                for (int i = 0; i < levelWidth; ++i)
                {
                    for (int j = 0; j < levelWidth; ++j)
                    {
                        var position = new Vector3(
                            origin.X + i * perBoxWidth,
                            origin.Y,
                            origin.Z + j * perBoxWidth);

                        var box = new Box(position, boxSize, boxSize, boxSize, 20f);

                        Space.Add(box);
                    }
                }
            }

            game.Camera.Position = new Vector3(-bottomBoxCount * boxSize, 2, bottomBoxCount * boxSize);
            game.Camera.Yaw((float)Math.PI / -4f);
            game.Camera.Pitch((float)Math.PI / 9f);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Solid Pyramid"; }
        }
    }
}