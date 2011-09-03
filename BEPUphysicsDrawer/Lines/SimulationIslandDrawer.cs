//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using Microsoft.Xna.Framework.Graphics;
//using BEPUphysics.DataStructures;
//using BEPUphysics;
//using BEPUphysics.NarrowPhaseSystems.Pairs;
//using Microsoft.Xna.Framework;
//using BEPUphysics.Entities;

//namespace BEPUphysicsDrawer.Lines
//{
//    /// <summary>
//    /// Renders bounding boxes of simulation islands.
//    /// </summary>
//    public class SimulationIslandDrawer
//    {
//        Game game;
//        public SimulationIslandDrawer(Game game)
//        {
//            this.game = game;
//        }
        
//        RawList<VertexPositionColor> boundingBoxLines = new RawList<VertexPositionColor>();

//        public void Draw(Effect effect, Space space)
//        {
//            if (space.Entities.Count > 0)
//            {
//                BoundingBox box;

//                for (int i = 0; i < space.DeactivationManager.SimulationIslands.Count; i++)
//                {
//                    var s = space.DeactivationManager.SimulationIslands[i];

//                    box = new BoundingBox(new Vector3(float.MaxValue, float.MaxValue, float.MaxValue),
//                                          new Vector3(-float.MaxValue, -float.MaxValue, -float.MaxValue));
//                    for (int j = 0; j < s.Members.Count; j++)
//                    {
//                        var member = s.Members[j];
//                        if (member != null)
//                        {
//                            box = BoundingBox.CreateMerged(box, member.Owner.CollisionInformation.BoundingBox);
//                        }
//                    }



//                    Color colorToUse = s.IsActive ? Color.DarkGoldenrod : Color.DarkGray;
//                    Vector3[] boundingBoxCorners = box.GetCorners();
//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], colorToUse));
//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], colorToUse));

//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], colorToUse));
//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], colorToUse));

//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], colorToUse));
//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], colorToUse));

//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], colorToUse));
//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], colorToUse));

//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], colorToUse));
//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], colorToUse));

//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], colorToUse));
//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], colorToUse));

//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], colorToUse));
//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], colorToUse));

//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], colorToUse));
//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], colorToUse));

//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], colorToUse));
//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], colorToUse));

//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], colorToUse));
//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], colorToUse));

//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], colorToUse));
//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], colorToUse));

//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], colorToUse));
//                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], colorToUse));
//                }

//                if (space.DeactivationManager.SimulationIslands.Count > 0)
//                    foreach (var pass in effect.CurrentTechnique.Passes)
//                    {
//                        pass.Apply();
//                        game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, boundingBoxLines.Elements, 0, space.DeactivationManager.SimulationIslands.Count * 12);
//                    }
//                boundingBoxLines.Clear();
//            }
//        }
//    }
//}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.DataStructures;
using BEPUphysics;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using Microsoft.Xna.Framework;
using BEPUphysics.Entities;
using BEPUphysics.DeactivationManagement;

namespace BEPUphysicsDrawer.Lines
{
    /// <summary>
    /// Renders bounding boxes of simulation islands.
    /// </summary>
    public class SimulationIslandDrawer
    {
        Game game;
        public SimulationIslandDrawer(Game game)
        {
            this.game = game;
        }

        RawList<VertexPositionColor> boundingBoxLines = new RawList<VertexPositionColor>();
        Dictionary<SimulationIsland, BoundingBox> islandBoundingBoxes = new Dictionary<SimulationIsland, BoundingBox>();

        public void Draw(Effect effect, Space space)
        {
            if (space.Entities.Count > 0)
            {
                BoundingBox box;
                foreach (var entity in space.Entities)
                {
                    var island = entity.ActivityInformation.SimulationIsland;
                    if (island != null)
                    {
                        if (islandBoundingBoxes.TryGetValue(island, out box))
                        {
                            box = BoundingBox.CreateMerged(entity.CollisionInformation.BoundingBox, box);
                            islandBoundingBoxes[island] = box;
                        }
                        else
                        {
                            islandBoundingBoxes.Add(island, entity.CollisionInformation.BoundingBox);
                        }
                    }
                }
                foreach (var islandBoundingBox in islandBoundingBoxes)
                {
                    Color colorToUse = islandBoundingBox.Key.IsActive ? Color.DarkGoldenrod : Color.DarkGray;
                    Vector3[] boundingBoxCorners = islandBoundingBox.Value.GetCorners();
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], colorToUse));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], colorToUse));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], colorToUse));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], colorToUse));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[0], colorToUse));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], colorToUse));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], colorToUse));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], colorToUse));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[1], colorToUse));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], colorToUse));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], colorToUse));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], colorToUse));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[2], colorToUse));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], colorToUse));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[3], colorToUse));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], colorToUse));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], colorToUse));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], colorToUse));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[4], colorToUse));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], colorToUse));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[5], colorToUse));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], colorToUse));

                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[6], colorToUse));
                    boundingBoxLines.Add(new VertexPositionColor(boundingBoxCorners[7], colorToUse));

                }

                if (space.DeactivationManager.SimulationIslands.Count > 0)
                    foreach (var pass in effect.CurrentTechnique.Passes)
                    {
                        pass.Apply();
                        game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, boundingBoxLines.Elements, 0, islandBoundingBoxes.Count * 12);
                    }
                islandBoundingBoxes.Clear();
                boundingBoxLines.Clear();
            }
        }
    }
}
