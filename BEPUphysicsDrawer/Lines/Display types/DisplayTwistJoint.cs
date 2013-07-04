using BEPUphysics.Constraints.TwoEntity.Joints;
using Microsoft.Xna.Framework;
using ConversionHelper;

namespace BEPUphysicsDrawer.Lines
{
    /// <summary>
    /// Graphical representation of a twist joint
    /// </summary>
    public class DisplayTwistJoint : SolverDisplayObject<TwistJoint>
    {
        private readonly Line axisA;
        private readonly Line axisB;


        public DisplayTwistJoint(TwistJoint constraint, LineDrawer drawer)
            : base(drawer, constraint)
        {
            axisA = new Line(Color.DarkRed, Color.DarkRed, drawer);
            axisB = new Line(Color.DarkRed, Color.DarkRed, drawer);
            myLines.Add(axisA);
            myLines.Add(axisB);
        }


        /// <summary>
        /// Moves the constraint lines to the proper location relative to the entities involved.
        /// </summary>
        public override void Update()
        {
            //Move lines around
            axisA.PositionA = MathConverter.Convert(LineObject.ConnectionA.Position);
            axisA.PositionB = MathConverter.Convert(LineObject.ConnectionA.Position + LineObject.WorldAxisA);

            axisB.PositionA = MathConverter.Convert(LineObject.ConnectionB.Position);
            axisB.PositionB = MathConverter.Convert(LineObject.ConnectionB.Position + LineObject.WorldAxisB);
        }
    }
}