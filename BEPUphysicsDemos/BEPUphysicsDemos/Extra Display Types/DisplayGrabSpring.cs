using BEPUphysicsDemos.SampleCode;
using BEPUphysicsDrawer.Lines;
using Microsoft.Xna.Framework;
using ConversionHelper;

namespace BEPUphysicsDemos
{
    /// <summary>
    /// Graphical representation of a GrabSpring.
    /// </summary>
    public class DisplayGrabSpring : LineDisplayObject<GrabSpring>
    {
        private readonly Line error;

        public DisplayGrabSpring(GrabSpring constraint, LineDrawer drawer)
            : base(drawer, constraint)
        {
            error = new Line(Color.Red, Color.Red, drawer);
            myLines.Add(error);
        }

        public override bool IsActive
        {
            get { return LineObject.IsUpdating; }
        }


        /// <summary>
        /// Moves the constraint lines to the proper location relative to the entities involved.
        /// </summary>
        public override void Update()
        {
            //Move lines around


            error.PositionA = MathConverter.Convert(LineObject.GoalPosition);
            error.PositionB = MathConverter.Convert(LineObject.GrabbedPosition);
        }
    }
}