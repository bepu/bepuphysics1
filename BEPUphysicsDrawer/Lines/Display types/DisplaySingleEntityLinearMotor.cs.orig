using BEPUphysics.Constraints.SingleEntity;
using BEPUphysics.Constraints.TwoEntity.Motors;
using Microsoft.Xna.Framework;
using ConversionHelper;

namespace BEPUphysicsDrawer.Lines
{
    /// <summary>
    /// Graphical representation of a single entity linear motor.
    /// </summary>
    public class DisplaySingleEntityLinearMotor : SolverDisplayObject<SingleEntityLinearMotor>
    {
        private readonly Line error;
        private readonly Line toPoint;

        public DisplaySingleEntityLinearMotor(SingleEntityLinearMotor constraint, LineDrawer drawer)
            : base(drawer, constraint)
        {
            toPoint = new Line(Color.DarkBlue, Color.DarkBlue, drawer);
            error = new Line(Color.Red, Color.Red, drawer);
            myLines.Add(toPoint);
            myLines.Add(error);
        }


        /// <summary>
        /// Moves the constraint lines to the proper location relative to the entities involved.
        /// </summary>
        public override void Update()
        {
            //Move lines around
            if (LineObject.IsActiveInSolver)
            {
                toPoint.PositionA = MathConverter.Convert(LineObject.Entity.Position);
                toPoint.PositionB = MathConverter.Convert(LineObject.Point);

                if (LineObject.Settings.Mode == MotorMode.Servomechanism)
                {
                    error.PositionA = toPoint.PositionB;
                    error.PositionB = MathConverter.Convert(LineObject.Settings.Servo.Goal);
                }
                else
                {
                    error.PositionA = toPoint.PositionB;
                    error.PositionB = toPoint.PositionB;
                }
            }
            else
            {
                error.PositionA = toPoint.PositionB;
                error.PositionB = toPoint.PositionB;
            }
        }
    }
}