using System;
using BEPUutilities;

namespace BEPUik
{
    /// <summary>
    /// Tries to keep the anchor points on two bones within an allowed range of distances.
    /// </summary>
    public class IKDistanceLimit : IKLimit
    {
        /// <summary>
        /// Gets or sets the offset in connection A's local space from the center of mass to the anchor point.
        /// </summary>
        public Vector3 LocalAnchorA;
        /// <summary>
        /// Gets or sets the offset in connection B's local space from the center of mass to the anchor point.
        /// </summary>
        public Vector3 LocalAnchorB;

        /// <summary>
        /// Gets or sets the offset in world space from the center of mass of connection A to the anchor point.
        /// </summary>
        public Vector3 AnchorA
        {
            get { return ConnectionA.Position + Quaternion.Transform(LocalAnchorA, ConnectionA.Orientation); }
            set { LocalAnchorA = Quaternion.Transform(value - ConnectionA.Position, Quaternion.Conjugate(ConnectionA.Orientation)); }
        }

        /// <summary>
        /// Gets or sets the offset in world space from the center of mass of connection B to the anchor point.
        /// </summary>
        public Vector3 AnchorB
        {
            get { return ConnectionB.Position + Quaternion.Transform(LocalAnchorB, ConnectionB.Orientation); }
            set { LocalAnchorB = Quaternion.Transform(value - ConnectionB.Position, Quaternion.Conjugate(ConnectionB.Orientation)); }
        }

        private float minimumDistance;
        /// <summary>
        /// Gets or sets the minimum distance that the joint connections should be kept from each other.
        /// </summary>
        public float MinimumDistance
        {
            get { return minimumDistance; }
            set { minimumDistance = Math.Max(0, value); }
        }

         private float maximumDistance;
        /// <summary>
        /// Gets or sets the maximum distance that the joint connections should be kept from each other.
        /// </summary>
        public float MaximumDistance
        {
            get { return maximumDistance; }
            set { maximumDistance = Math.Max(0, value); }
        }

        /// <summary>
        /// Constructs a new distance joint.
        /// </summary>
        /// <param name="connectionA">First bone connected by the joint.</param>
        /// <param name="connectionB">Second bone connected by the joint.</param>
        /// <param name="anchorA">Anchor point on the first bone in world space.</param>
        /// <param name="anchorB">Anchor point on the second bone in world space.</param>
        /// <param name="minimumDistance">Minimum distance that the joint connections should be kept from each other.</param>
        /// <param name="maximumDistance">Maximum distance that the joint connections should be kept from each other.</param>
        public IKDistanceLimit(Bone connectionA, Bone connectionB, Vector3 anchorA, Vector3 anchorB, float minimumDistance, float maximumDistance)
            : base(connectionA, connectionB)
        {
            AnchorA = anchorA;
            AnchorB = anchorB;
            MinimumDistance = minimumDistance;
            MaximumDistance = maximumDistance;
        }

        protected internal override void UpdateJacobiansAndVelocityBias()
        {
            //Transform the anchors and offsets into world space.
            Vector3 offsetA, offsetB;
            Quaternion.Transform(ref LocalAnchorA, ref ConnectionA.Orientation, out offsetA);
            Quaternion.Transform(ref LocalAnchorB, ref ConnectionB.Orientation, out offsetB);
            Vector3 anchorA, anchorB;
            Vector3.Add(ref ConnectionA.Position, ref offsetA, out anchorA);
            Vector3.Add(ref ConnectionB.Position, ref offsetB, out anchorB);

            //Compute the distance.
            Vector3 separation;
            Vector3.Subtract(ref anchorB, ref anchorA, out separation);
            float currentDistance = separation.Length();

            //Compute jacobians
            Vector3 linearA;
#if !WINDOWS
            linearA = new Vector3();
#endif
            if (currentDistance > Toolbox.Epsilon)
            {
                linearA.X = separation.X / currentDistance;
                linearA.Y = separation.Y / currentDistance;
                linearA.Z = separation.Z / currentDistance;

                if (currentDistance > maximumDistance)
                {
                    //We are exceeding the maximum limit.
                    velocityBias = new Vector3(errorCorrectionFactor * (currentDistance - maximumDistance), 0, 0);
                }
                else if (currentDistance < minimumDistance)
                {
                    //We are exceeding the minimum limit.
                    velocityBias = new Vector3(errorCorrectionFactor * (minimumDistance - currentDistance), 0, 0);
                    //The limit can only push in one direction. Flip the jacobian!
                    Vector3.Negate(ref linearA, out linearA);
                }
                else if (currentDistance - minimumDistance > (maximumDistance - minimumDistance) * 0.5f)
                {
                    //The objects are closer to hitting the maximum limit.
                    velocityBias = new Vector3(currentDistance - maximumDistance, 0, 0);
                }
                else
                {
                    //The objects are closer to hitting the minimum limit.
                    velocityBias = new Vector3(minimumDistance - currentDistance, 0, 0);
                    //The limit can only push in one direction. Flip the jacobian!
                    Vector3.Negate(ref linearA, out linearA);
                }
            }
            else
            {
                velocityBias = new Vector3();
                linearA = new Vector3();
            }

            Vector3 angularA, angularB;
            Vector3.Cross(ref offsetA, ref linearA, out angularA);
            //linearB = -linearA, so just swap the cross product order.
            Vector3.Cross(ref linearA, ref offsetB, out angularB);

            //Put all the 1x3 jacobians into a 3x3 matrix representation.
            linearJacobianA = new Matrix3x3 { M11 = linearA.X, M12 = linearA.Y, M13 = linearA.Z };
            linearJacobianB = new Matrix3x3 { M11 = -linearA.X, M12 = -linearA.Y, M13 = -linearA.Z };
            angularJacobianA = new Matrix3x3 { M11 = angularA.X, M12 = angularA.Y, M13 = angularA.Z };
            angularJacobianB = new Matrix3x3 { M11 = angularB.X, M12 = angularB.Y, M13 = angularB.Z };

        }
    }
}
