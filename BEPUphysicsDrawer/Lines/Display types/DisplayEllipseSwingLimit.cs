using System;
using BEPUphysics.Constraints.TwoEntity.JointLimits;
using Microsoft.Xna.Framework;
using ConversionHelper;

namespace BEPUphysicsDrawer.Lines
{
    /// <summary>
    /// Graphical representation of a twist joint
    /// </summary>
    public class DisplayEllipseSwingLimit : SolverDisplayObject<EllipseSwingLimit>
    {
        /// <summary>
        /// Number of facets to use when representing the swing limit boundary.
        /// </summary>
        private static int limitFacetCount = 32;

        private readonly Line axis;
        private readonly Line[] limitLines;


        public DisplayEllipseSwingLimit(EllipseSwingLimit constraint, LineDrawer drawer)
            : base(drawer, constraint)
        {
            axis = new Line(Color.Red, Color.Red, drawer);
            myLines.Add(axis);
            //Create the lines that represent the outline of the limit.
            limitLines = new Line[limitFacetCount * 2];
            for (int i = 0; i < limitFacetCount; i++)
            {
                limitLines[i * 2] = new Line(Color.DarkRed, Color.DarkRed, drawer);
                limitLines[i * 2 + 1] = new Line(Color.DarkRed, Color.DarkRed, drawer);
            }
            myLines.AddRange(limitLines);
        }


        /// <summary>
        /// Moves the constraint lines to the proper location relative to the entities involved.
        /// </summary>
        public override void Update()
        {
            //Move lines around
            axis.PositionA = MathConverter.Convert(LineObject.ConnectionB.Position);
            axis.PositionB = MathConverter.Convert(LineObject.ConnectionB.Position + LineObject.TwistAxisB * 1.5f);


            float angleIncrement = 4 * MathHelper.Pi / limitLines.Length; //Each loop iteration moves this many radians forward.
            for (int i = 0; i < limitLines.Length / 2; i++)
            {
                Line pointToPreviousPoint = limitLines[2 * i];
                Line centerToPoint = limitLines[2 * i + 1];

                float currentAngle = i * angleIncrement;

                //Using the parametric equation for an ellipse, compute the axis of rotation and angle.
                Vector3 rotationAxis = MathConverter.Convert(LineObject.Basis.XAxis * LineObject.MaximumAngleX * (float) Math.Cos(currentAngle) +
                                                             LineObject.Basis.YAxis * LineObject.MaximumAngleY * (float) Math.Sin(currentAngle));
                float angle = rotationAxis.Length();
                rotationAxis /= angle;

                pointToPreviousPoint.PositionA = MathConverter.Convert(LineObject.ConnectionB.Position) +
                                                 //Rotate the primary axis to the ellipse boundary...
                                                 Vector3.TransformNormal(MathConverter.Convert(LineObject.Basis.PrimaryAxis), Matrix.CreateFromAxisAngle(rotationAxis, angle));

                centerToPoint.PositionA = pointToPreviousPoint.PositionA;
                centerToPoint.PositionB = MathConverter.Convert(LineObject.ConnectionB.Position);
            }
            for (int i = 0; i < limitLines.Length / 2; i++)
            {
                //Connect all the pointToPreviousPoint lines to the previous points.
                limitLines[2 * i].PositionB = limitLines[2 * ((i + 1) % (limitLines.Length / 2))].PositionA;
            }
        }
    }
}