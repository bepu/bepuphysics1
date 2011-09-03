using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.DataStructures;
using BEPUphysics;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using Microsoft.Xna.Framework;
using ConversionHelper;

namespace BEPUphysicsDrawer.Lines
{
    /// <summary>
    /// Renders bounding boxes of entities.
    /// </summary>
    public class BoundingBoxDrawer
    {
        Game game;
        public BoundingBoxDrawer(Game game)
        {
            this.game = game;
        }
        
        RawList<VertexPositionColor> boundingBoxLines = new RawList<VertexPositionColor>();

        public void Draw(Effect effect, Space space)
        {
            if (space.Entities.Count > 0)
            {

                foreach (var e in space.Entities)
                {
                    Vector3[] boundingBoxCorners = MathConverter.Convert(e.CollisionInformation.BoundingBox.GetCorners());
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], Color.DarkRed));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], Color.DarkRed));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], Color.DarkRed));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], Color.DarkRed));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], Color.DarkRed));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], Color.DarkRed));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], Color.DarkRed));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], Color.DarkRed));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], Color.DarkRed));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], Color.DarkRed));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], Color.DarkRed));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], Color.DarkRed));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], Color.DarkRed));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], Color.DarkRed));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], Color.DarkRed));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], Color.DarkRed));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], Color.DarkRed));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], Color.DarkRed));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], Color.DarkRed));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], Color.DarkRed));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], Color.DarkRed));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], Color.DarkRed));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], Color.DarkRed));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], Color.DarkRed));
                }
                foreach (var pass in effect.CurrentTechnique.Passes)
                {
                    pass.Apply();
                    game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, boundingBoxLines.Elements, 0, space.Entities.Count * 12);
                }
                boundingBoxLines.Clear();
            }
        }
    }
}
