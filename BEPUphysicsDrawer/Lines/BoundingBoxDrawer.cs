using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.DataStructures;
using BEPUphysics;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using Microsoft.Xna.Framework;

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
                    Vector3[] boundingBoxCorners = e.CollisionInformation.BoundingBox.GetCorners();
                    var color = e.ActivityInformation.IsActive ? Color.DarkRed : new Color(150, 100, 100);
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], color));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], color));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], color));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], color));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], color));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], color));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], color));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], color));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], color));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], color));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], color));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], color));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], color));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], color));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], color));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], color));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], color));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], color));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], color));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], color));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], color));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], color));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], color));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], color));
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
