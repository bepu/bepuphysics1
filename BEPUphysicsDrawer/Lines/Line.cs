using Microsoft.Xna.Framework;

namespace BEPUphysicsDrawer.Lines
{
    /// <summary>
    /// Represents a graphical line in the ConstraintDrawer.
    /// </summary>
    public class Line
    {
        private readonly LineDrawer drawer;

        /// <summary>
        /// First index of the line in the constraint drawer vertex list.
        /// </summary>
        public readonly int IndexA;

        /// <summary>
        /// Second index of the line in the constraint drawer vertex list.
        /// </summary>
        public readonly int IndexB;

        /// <summary>
        /// Constructs a new line.
        /// </summary>
        /// <param name="positionA">Initial position of the first vertex of the line.</param>
        /// <param name="positionB">Initial position of the second vertex of the line.</param>
        /// <param name="colorA">Initial color of the first vertex of the line.</param>
        /// <param name="colorB">Initial color of the second vertex of the line.</param>
        /// <param name="drawer">System responsible for drawing this line.</param>
        public Line(Vector3 positionA, Vector3 positionB, Color colorA, Color colorB, LineDrawer drawer)
        {
            this.drawer = drawer;
            drawer.GetNewLineIndex(out IndexA);
            IndexB = IndexA + 1;
            drawer.vertices[IndexA].Position = positionA;
            drawer.vertices[IndexA].Color = colorA;
            drawer.vertices[IndexB].Position = positionB;
            drawer.vertices[IndexB].Color = colorB;
        }

        /// <summary>
        /// Constructs a new line.
        /// </summary>
        /// <param name="colorA">Initial color of the first vertex of the line.</param>
        /// <param name="colorB">Initial color of the second vertex of the line.</param>
        /// <param name="drawer">System responsible for drawing this line.</param>
        public Line(Color colorA, Color colorB, LineDrawer drawer)
        {
            this.drawer = drawer;
            drawer.GetNewLineIndex(out IndexA);
            IndexB = IndexA + 1;
            drawer.vertices[IndexA].Position = Vector3.Zero;
            drawer.vertices[IndexA].Color = colorA;
            drawer.vertices[IndexB].Position = Vector3.Zero;
            drawer.vertices[IndexB].Color = colorB;
        }

        /// <summary>
        /// Gets and sets the position of the first vertex of the line.
        /// </summary>
        public Vector3 PositionA
        {
            get { return drawer.vertices[IndexA].Position; }
            set { drawer.vertices[IndexA].Position = value; }
        }

        /// <summary>
        /// Gets and sets the position of the second vertex of the line.
        /// </summary>
        public Vector3 PositionB
        {
            get { return drawer.vertices[IndexB].Position; }
            set { drawer.vertices[IndexB].Position = value; }
        }

        /// <summary>
        /// Gets and sets the color of the first vertex of the line.
        /// </summary>
        public Color ColorA
        {
            get { return drawer.vertices[IndexA].Color; }
            set { drawer.vertices[IndexA].Color = value; }
        }

        /// <summary>
        /// Gets and sets the color of the second vertex of the line.
        /// </summary>
        public Color ColorB
        {
            get { return drawer.vertices[IndexB].Color; }
            set { drawer.vertices[IndexB].Color = value; }
        }
    }
}