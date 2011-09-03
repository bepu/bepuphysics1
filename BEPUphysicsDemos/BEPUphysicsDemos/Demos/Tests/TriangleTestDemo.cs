using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUphysics.CollisionShapes.ConvexShapes;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.DataStructures;
using BEPUphysics.CollisionTests;
using Microsoft.Xna.Framework.Input;
using BEPUphysics.CollisionTests.CollisionAlgorithms.Testing;

namespace BEPUphysicsDemos.Demos.Tests
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class TriangleTestDemo : StandardDemo
    {
        TriangleShape triangleA;
        TriangleShape triangleB;
        TriangleTrianglePairTester2 pairTester;

        VertexPositionColor[] triangleLines = new VertexPositionColor[12];
        RawList<VertexPositionColor> contactLines = new RawList<VertexPositionColor>();
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public TriangleTestDemo(DemosGame game)
            : base(game)
        {
            Vector3 center;
            triangleB = new TriangleShape(new Vector3(-1, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 0, 1));
            triangleA = new TriangleShape() { VertexA = new Vector3(0, 1, 0), VertexB = new Vector3(0, 3, 0), VertexC = new Vector3(0, 2, 1) };
            triangleB.Sidedness = TriangleSidedness.Counterclockwise;
            triangleA.Sidedness = TriangleSidedness.Counterclockwise;
            pairTester = new TriangleTrianglePairTester2();
            pairTester.Initialize(triangleB, triangleA);

            game.Camera.Position = new Vector3(0, 6, 15);
        }

        public override void Update(float dt)
        {

            base.Update(dt);
            float speed = dt * (Game.KeyboardInput.IsKeyDown(Keys.LeftShift) ? 2f : .3f);
            if (Game.KeyboardInput.IsKeyDown(Keys.NumPad8))
            {
                Vector3 offset = new Vector3(0, 0, -speed);
                triangleA.VertexA += offset;
                triangleA.VertexB += offset;
                triangleA.VertexC += offset;
            }
            if (Game.KeyboardInput.IsKeyDown(Keys.NumPad5))
            {
                Vector3 offset = new Vector3(0, 0, speed);
                triangleA.VertexA += offset;
                triangleA.VertexB += offset;
                triangleA.VertexC += offset;
            }
            if (Game.KeyboardInput.IsKeyDown(Keys.NumPad1))
            {
                Vector3 offset = new Vector3(0, speed, 0);
                triangleA.VertexA += offset;
                triangleA.VertexB += offset;
                triangleA.VertexC += offset;
            }
            if (Game.KeyboardInput.IsKeyDown(Keys.NumPad0))
            {
                Vector3 offset = new Vector3(0, -speed, 0);
                triangleA.VertexA += offset;
                triangleA.VertexB += offset;
                triangleA.VertexC += offset;
            }
            if (Game.KeyboardInput.IsKeyDown(Keys.NumPad4))
            {
                Vector3 offset = new Vector3(-speed, 0, 0);
                triangleA.VertexA += offset;
                triangleA.VertexB += offset;
                triangleA.VertexC += offset;
            }
            if (Game.KeyboardInput.IsKeyDown(Keys.NumPad6))
            {
                Vector3 offset = new Vector3(speed, 0, 0);
                triangleA.VertexA += offset;
                triangleA.VertexB += offset;
                triangleA.VertexC += offset;
            }
        }

        public override void Draw()
        {
            base.Draw();
            triangleLines[0] = new VertexPositionColor(triangleA.VertexA, Color.Yellow);
            triangleLines[1] = new VertexPositionColor(triangleA.VertexB, Color.Yellow);
            triangleLines[2] = new VertexPositionColor(triangleA.VertexB, Color.Yellow);
            triangleLines[3] = new VertexPositionColor(triangleA.VertexC, Color.Yellow);
            triangleLines[4] = new VertexPositionColor(triangleA.VertexC, Color.Yellow);
            triangleLines[5] = new VertexPositionColor(triangleA.VertexA, Color.Yellow);

            triangleLines[6] = new VertexPositionColor(triangleB.VertexA, Color.Orange);
            triangleLines[7] = new VertexPositionColor(triangleB.VertexB, Color.Orange);
            triangleLines[8] = new VertexPositionColor(triangleB.VertexB, Color.Orange);
            triangleLines[9] = new VertexPositionColor(triangleB.VertexC, Color.Orange);
            triangleLines[10] = new VertexPositionColor(triangleB.VertexC, Color.Orange);
            triangleLines[11] = new VertexPositionColor(triangleB.VertexA, Color.Orange);

            TinyStructList<ContactData> contacts;
            pairTester.GenerateContactCandidate(out contacts);
            contactLines.Clear();
            for (int i = 0; i < contacts.Count; i++)
            {
                ContactData contact;
                contacts.Get(i, out contact);

                contactLines.Add(new VertexPositionColor(contact.Position, Color.White));
                contactLines.Add(new VertexPositionColor(contact.Position + contact.Normal * contact.PenetrationDepth, Color.Red));
                contactLines.Add(new VertexPositionColor(contact.Position + contact.Normal * contact.PenetrationDepth, Color.White));
                contactLines.Add(new VertexPositionColor(contact.Position + contact.Normal * (contact.PenetrationDepth + .3f), Color.White));


            }
            foreach (var pass in Game.LineDrawer.CurrentTechnique.Passes)
            {
                pass.Apply();
                Game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, triangleLines, 0, 6);
                if (contactLines.Count > 0)
                    Game.GraphicsDevice.DrawUserPrimitives(PrimitiveType.LineList, contactLines.Elements, 0, contactLines.Count / 2);

            }
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Wall"; }
        }
    }
}