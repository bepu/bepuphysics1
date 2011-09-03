using BEPUphysics.Constraints.TwoEntity.Joints;
using Microsoft.Xna.Framework;
using ConversionHelper;

namespace BEPUphysicsDrawer.Lines
{
    /// <summary>
    /// Graphical representation of a PointOnPlaneConstraint
    /// </summary>
    public class DisplayPointOnPlaneJoint : SolverDisplayObject<PointOnPlaneJoint>
    {
        private readonly Line aToConnection;
        private readonly Line bToConnection;
        private readonly Line error;
        private readonly Line gridColumn1;
        private readonly Line gridColumn2;
        private readonly Line gridColumn3;
        private readonly Line gridRow1;
        private readonly Line gridRow2;
        private readonly Line gridRow3;

        public DisplayPointOnPlaneJoint(PointOnPlaneJoint constraint, LineDrawer drawer)
            : base(drawer, constraint)
        {
            gridRow1 = new Line(Color.Gray, Color.Gray, drawer);
            gridRow2 = new Line(Color.Gray, Color.Gray, drawer);
            gridRow3 = new Line(Color.Gray, Color.Gray, drawer);

            gridColumn1 = new Line(Color.Gray, Color.Gray, drawer);
            gridColumn2 = new Line(Color.Gray, Color.Gray, drawer);
            gridColumn3 = new Line(Color.Gray, Color.Gray, drawer);

            aToConnection = new Line(Color.DarkBlue, Color.DarkBlue, drawer);
            bToConnection = new Line(Color.DarkBlue, Color.DarkBlue, drawer);
            error = new Line(Color.Red, Color.Red, drawer);

            myLines.Add(gridRow1);
            myLines.Add(gridRow2);
            myLines.Add(gridRow3);
            myLines.Add(gridColumn1);
            myLines.Add(gridColumn2);
            myLines.Add(gridColumn3);
            myLines.Add(aToConnection);
            myLines.Add(bToConnection);
            myLines.Add(error);
        }


        /// <summary>
        /// Moves the constraint lines to the proper location relative to the entities involved.
        /// </summary>
        public override void Update()
        {
            //Move lines around
            PointOnPlaneJoint constraint = LineObject;
            Vector3 planeAnchor = MathConverter.Convert(constraint.PlaneAnchor);
            Vector3 y = MathConverter.Convert(BEPUphysics.MathExtensions.Vector3.Cross(constraint.ConnectionA.OrientationMatrix.Up, constraint.PlaneNormal));
            if (y.LengthSquared() < .001f)
            {
                y = MathConverter.Convert(BEPUphysics.MathExtensions.Vector3.Cross(constraint.ConnectionA.OrientationMatrix.Right, constraint.PlaneNormal));
            }
            Vector3 x = Vector3.Cross(MathConverter.Convert(constraint.PlaneNormal), y);

            //Grid
            gridRow1.PositionA = planeAnchor - 1.5f * x + y;
            gridRow1.PositionB = planeAnchor + 1.5f * x + y;

            gridRow2.PositionA = planeAnchor - 1.5f * x;
            gridRow2.PositionB = planeAnchor + 1.5f * x;

            gridRow3.PositionA = planeAnchor - 1.5f * x - y;
            gridRow3.PositionB = planeAnchor + 1.5f * x - y;

            gridColumn1.PositionA = planeAnchor + x - 1.5f * y;
            gridColumn1.PositionB = planeAnchor + x + 1.5f * y;

            gridColumn2.PositionA = planeAnchor - 1.5f * y;
            gridColumn2.PositionB = planeAnchor + 1.5f * y;

            gridColumn3.PositionA = planeAnchor - x - 1.5f * y;
            gridColumn3.PositionB = planeAnchor - x + 1.5f * y;

            //Connection and error
            aToConnection.PositionA = MathConverter.Convert(constraint.ConnectionA.Position);
            aToConnection.PositionB = MathConverter.Convert(constraint.PlaneAnchor);

            bToConnection.PositionA = MathConverter.Convert(constraint.ConnectionB.Position);
            bToConnection.PositionB = MathConverter.Convert(constraint.PointAnchor);

            error.PositionA = aToConnection.PositionB;
            error.PositionB = bToConnection.PositionB;
        }
    }
}