using BEPUphysics.Constraints.TwoEntity.JointLimits;
using Microsoft.Xna.Framework;
using ConversionHelper;

namespace BEPUphysicsDrawer.Lines
{
    /// <summary>
    /// Graphical representation of a ball socket joint.
    /// </summary>
    public class DisplayRevoluteLimit : SolverDisplayObject<RevoluteLimit>
    {
        private readonly Line bottom;
        private readonly Line bottomLeft;
        private readonly Line bottomRight;
        private readonly Line middle;
        private readonly Line testAxis;
        private readonly Line top;
        private readonly Line topLeft;
        private readonly Line topRight;

        public DisplayRevoluteLimit(RevoluteLimit constraint, LineDrawer drawer)
            : base(drawer, constraint)
        {
            topRight = new Line(Color.DarkBlue, Color.DarkBlue, drawer);
            top = new Line(Color.DarkBlue, Color.DarkBlue, drawer);
            topLeft = new Line(Color.DarkBlue, Color.DarkBlue, drawer);
            bottomRight = new Line(Color.DarkBlue, Color.DarkBlue, drawer);
            bottom = new Line(Color.DarkBlue, Color.DarkBlue, drawer);
            bottomLeft = new Line(Color.DarkBlue, Color.DarkBlue, drawer);
            middle = new Line(Color.DarkBlue, Color.DarkBlue, drawer);
            testAxis = new Line(Color.DarkRed, Color.DarkRed, drawer);

            myLines.Add(topRight);
            myLines.Add(top);
            myLines.Add(topLeft);
            myLines.Add(bottomRight);
            myLines.Add(bottom);
            myLines.Add(bottomLeft);
            myLines.Add(middle);
            myLines.Add(testAxis);
        }


        /// <summary>
        /// Moves the constraint lines to the proper location relative to the entities involved.
        /// </summary>
        public override void Update()
        {
            //Move lines around
            Vector3 left = MathConverter.Convert(LineObject.ConnectionB.Position - LineObject.Basis.PrimaryAxis);
            Vector3 right = MathConverter.Convert(LineObject.ConnectionB.Position + LineObject.Basis.PrimaryAxis);

            Vector3 upwardsOffset = Vector3.TransformNormal(MathConverter.Convert(LineObject.Basis.XAxis), Matrix.CreateFromAxisAngle(MathConverter.Convert(LineObject.Basis.PrimaryAxis), LineObject.MaximumAngle));
            Vector3 topRightPosition = right + upwardsOffset;
            Vector3 topLeftPosition = left + upwardsOffset;

            Vector3 downwardsOffset = Vector3.TransformNormal(MathConverter.Convert(LineObject.Basis.XAxis), Matrix.CreateFromAxisAngle(MathConverter.Convert(LineObject.Basis.PrimaryAxis), LineObject.MinimumAngle));
            Vector3 bottomRightPosition = right + downwardsOffset;
            Vector3 bottomLeftPosition = left + downwardsOffset;

            middle.PositionA = left;
            middle.PositionB = right;

            topLeft.PositionA = left;
            topLeft.PositionB = topLeftPosition;

            topRight.PositionA = right;
            topRight.PositionB = topRightPosition;

            top.PositionA = topLeftPosition;
            top.PositionB = topRightPosition;

            bottomLeft.PositionA = left;
            bottomLeft.PositionB = bottomLeftPosition;

            bottomRight.PositionA = right;
            bottomRight.PositionB = bottomRightPosition;

            bottom.PositionA = bottomLeftPosition;
            bottom.PositionB = bottomRightPosition;

            testAxis.PositionA = MathConverter.Convert(LineObject.ConnectionB.Position);
            testAxis.PositionB = MathConverter.Convert(LineObject.ConnectionB.Position + LineObject.TestAxis);
        }
    }
}