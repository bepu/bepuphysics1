using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionShapes;
using System.Collections.Generic;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.MathExtensions;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.Entities;
using System;
using System.Diagnostics;

namespace BEPUphysicsDemos.Demos.Tests
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class MutableCompoundDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public MutableCompoundDemo(DemosGame game)
            : base(game)
        {
            int width = 3;
            int height = 3;
            int length = 10;
            float blockWidth = 1f;
            float blockHeight = 1f;
            float blockLength = 1f;



            for (int q = 0; q < 1; q++)
            {
                List<CompoundShapeEntry> shapes = new List<CompoundShapeEntry>();
                float totalWeight = 0;
                float density = 1;

                for (int i = 0; i < width; i++)
                {
                    for (int j = 0; j < height; j++)
                    {
                        for (int k = 0; k < length; k++)
                        {
                            float weight = density * blockWidth * blockLength * blockHeight;
                            totalWeight += weight;
                            shapes.Add(new CompoundShapeEntry(
                                new BoxShape(blockWidth, blockHeight, blockLength),
                                new Vector3(5 + q * 20 + i * blockWidth, j * blockHeight, k * blockLength), weight));
                        }
                    }
                }

                var compound = new CompoundBody(shapes, totalWeight);
                //compound.Orientation = Quaternion.CreateFromYawPitchRoll(1, 1, 1);
                //compound.AngularVelocity = new Vector3(0, 1, 0);
                Entity<CompoundCollidable> compound2, compound3;
                CompoundHelper.SplitCompound(x => x.ShapeIndex >= shapes.Count / 2, compound, out compound2);
                CompoundHelper.SplitCompound(x => x.ShapeIndex >= 3 * shapes.Count / 4, compound2, out compound3);



                //compound.ActivityInformation.IsAlwaysActive = true;
                //compound.IsAffectedByGravity = false;
                //compound2.ActivityInformation.IsAlwaysActive = true;
                //compound2.IsAffectedByGravity = false;
                //compound3.ActivityInformation.IsAlwaysActive = true;
                //compound3.IsAffectedByGravity = false;
                //compound.Tag = "noDisplayObject";
                Space.Add(compound);
                Space.Add(compound2);
                Space.Add(compound3);

            }

            Box ground = new Box(new Vector3(0, -4.5f, 0), 50, 1, 50);
            Space.Add(ground);
            game.Camera.Position = new Vector3(0, 6, 15);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Mutable Compound Test"; }
        }
    }
}