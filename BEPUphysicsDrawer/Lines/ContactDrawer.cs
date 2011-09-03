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
    /// Renders contact points.
    /// </summary>
    public class ContactDrawer
    {
        Game game;
        public ContactDrawer(Game game)
        {
            this.game = game;
        }
        
        RawList<VertexPositionColor> contactLines = new RawList<VertexPositionColor>();

        public void Draw(Effect effect, Space space)
        {
            contactLines.Clear();
            int contactCount = 0;
            foreach (var pair in space.NarrowPhase.Pairs)
            {
                var pairHandler = pair as CollidablePairHandler;
                if (pairHandler != null)
                {
                    foreach (ContactInformation information in pairHandler.Contacts)
                    {
                        contactCount++;
                        contactLines.Add(new VertexPositionColor(MathConverter.Convert(information.Contact.Position), Color.White));
                        contactLines.Add(new VertexPositionColor(MathConverter.Convert(information.Contact.Position + information.Contact.Normal * information.Contact.PenetrationDepth), Color.Red));
                        contactLines.Add(new VertexPositionColor(MathConverter.Convert(information.Contact.Position + information.Contact.Normal * information.Contact.PenetrationDepth), Color.White));
                        contactLines.Add(new VertexPositionColor(MathConverter.Convert(information.Contact.Position + information.Contact.Normal * (information.Contact.PenetrationDepth + .3f)), Color.White));
                    }
                }
            }

            if (contactCount > 0)
            {
                foreach (var pass in effect.CurrentTechnique.Passes)
                {
                    pass.Apply();

                    game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, contactLines.Elements, 0, contactLines.Count / 2);
                }
            }

        }
    }
}
