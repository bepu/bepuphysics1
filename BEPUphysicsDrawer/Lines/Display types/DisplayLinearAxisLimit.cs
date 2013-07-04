using BEPUphysics.Constraints.TwoEntity.JointLimits;
using Microsoft.Xna.Framework;
using ConversionHelper;

namespace BEPUphysicsDrawer.Lines
{
    /// <summary>
    /// Graphical representation of a LinearLimit.
    /// </summary>
    public class DisplayLinearAxisLimit : SolverDisplayObject<LinearAxisLimit>
    {
        private readonly Line aToConnection;
        private readonly Line axis;
        private readonly Line bToConnection;
        private readonly Line error;

        public DisplayLinearAxisLimit(LinearAxisLimit constraint, LineDrawer drawer)
            : base(drawer, constraint)
        {
            aToConnection = new Line(Color.DarkBlue, Color.DarkBlue, drawer);
            bToConnection = new Line(Color.DarkBlue, Color.DarkBlue, drawer);
            error = new Line(Color.Red, Color.Red, drawer);

            //Corners
            axis = new Line(Color.DarkBlue, Color.DarkBlue, drawer);

            myLines.Add(aToConnection);
            myLines.Add(bToConnection);
            myLines.Add(error);
            myLines.Add(axis);
        }


        /// <summary>
        /// Moves the constraint lines to the proper location relative to the entities involved.
        /// </summary>
        public override void Update()
        {
            //Move lines around
            aToConnection.PositionA = MathConverter.Convert(LineObject.ConnectionA.Position);
            aToConnection.PositionB = MathConverter.Convert(LineObject.AnchorA);

            bToConnection.PositionA = MathConverter.Convert(LineObject.ConnectionB.Position);
            bToConnection.PositionB = MathConverter.Convert(LineObject.AnchorB);

            error.PositionA = aToConnection.PositionB;
            error.PositionB = bToConnection.PositionB;


            axis.PositionA = MathConverter.Convert(LineObject.AnchorA + LineObject.Axis * LineObject.Minimum);
            axis.PositionB = MathConverter.Convert(LineObject.AnchorA + LineObject.Axis * LineObject.Maximum);
        }
    }
}