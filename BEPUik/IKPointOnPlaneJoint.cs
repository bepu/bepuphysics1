using BEPUutilities;

namespace BEPUik
{
    /// <summary>
    /// Keeps an anchor point on one bone on a plane defined by another bone.
    /// </summary>
    public class IKPointOnPlaneJoint : IKJoint
    {
        /// <summary>
        /// Gets or sets the offset in connection A's local space from the center of mass to the anchor point of the line.
        /// </summary>
        public Vector3 LocalPlaneAnchor;

        /// <summary>
        /// Gets or sets the direction of the line in connection A's local space.
        /// Must be unit length.
        /// </summary>
        public Vector3 LocalPlaneNormal;

        /// <summary>
        /// Gets or sets the offset in connection B's local space from the center of mass to the anchor point which will be kept on the plane.
        /// </summary>
        public Vector3 LocalAnchorB;

        /// <summary>
        /// Gets or sets the world space location of the line anchor attached to connection A.
        /// </summary>
        public Vector3 PlaneAnchor
        {
            get { return ConnectionA.Position + Quaternion.Transform(LocalPlaneAnchor, ConnectionA.Orientation); }
            set { LocalPlaneAnchor = Quaternion.Transform(value - ConnectionA.Position, Quaternion.Conjugate(ConnectionA.Orientation)); }
        }

        /// <summary>
        /// Gets or sets the world space normal of the plane attached to connection A.
        /// Must be unit length.
        /// </summary>
        public Vector3 PlaneNormal
        {
            get { return Quaternion.Transform(LocalPlaneNormal, ConnectionA.Orientation); }
            set { LocalPlaneNormal = Quaternion.Transform(value, Quaternion.Conjugate(ConnectionA.Orientation)); }
        }

        /// <summary>
        /// Gets or sets the offset in world space from the center of mass of connection B to the anchor point.
        /// </summary>
        public Vector3 AnchorB
        {
            get { return ConnectionB.Position + Quaternion.Transform(LocalAnchorB, ConnectionB.Orientation); }
            set { LocalAnchorB = Quaternion.Transform(value - ConnectionB.Position, Quaternion.Conjugate(ConnectionB.Orientation)); }
        }


        /// <summary>
        /// Constructs a new point on plane joint.
        /// </summary>
        /// <param name="connectionA">First bone connected by the joint.</param>
        /// <param name="connectionB">Second bone connected by the joint.</param>
        /// <param name="planeAnchor">Anchor point of the plane attached to the first bone in world space.</param>
        /// <param name="planeNormal">Normal of the plane attached to the first bone in world space. Must be unit length.</param>
        /// <param name="anchorB">Anchor point on the second bone in world space which is measured against the other connection's anchor.</param>
        public IKPointOnPlaneJoint(Bone connectionA, Bone connectionB, Vector3 planeAnchor, Vector3 planeNormal, Vector3 anchorB)
            : base(connectionA, connectionB)
        {
            PlaneAnchor = planeAnchor;
            PlaneNormal = planeNormal;
            AnchorB = anchorB;
        }

        protected internal override void UpdateJacobiansAndVelocityBias()
        {
            //Transform the anchors and offsets into world space.
            Vector3 offsetA, offsetB, lineDirection;
            Quaternion.Transform(ref LocalPlaneAnchor, ref ConnectionA.Orientation, out offsetA);
            Quaternion.Transform(ref LocalPlaneNormal, ref ConnectionA.Orientation, out lineDirection);
            Quaternion.Transform(ref LocalAnchorB, ref ConnectionB.Orientation, out offsetB);
            Vector3 anchorA, anchorB;
            Vector3.Add(ref ConnectionA.Position, ref offsetA, out anchorA);
            Vector3.Add(ref ConnectionB.Position, ref offsetB, out anchorB);

            //Compute the distance.
            Vector3 separation;
            Vector3.Subtract(ref anchorB, ref anchorA, out separation);
            //This entire constraint is very similar to the IKDistanceLimit, except the current distance is along an axis.
            float currentDistance;
            Vector3.Dot(ref separation, ref lineDirection, out currentDistance);
            velocityBias = new Vector3(errorCorrectionFactor * currentDistance, 0, 0);

            //Compute jacobians
            Vector3 angularA, angularB;
            //We can't just use the offset to anchor for A's jacobian- the 'collision' location is way out there at anchorB!
            Vector3 rA;
            Vector3.Subtract(ref anchorB, ref ConnectionA.Position, out rA);
            Vector3.Cross(ref rA, ref lineDirection, out angularA);
            //linearB = -linearA, so just swap the cross product order.
            Vector3.Cross(ref lineDirection, ref offsetB, out angularB);

            //Put all the 1x3 jacobians into a 3x3 matrix representation.
            linearJacobianA = new Matrix3x3 { M11 = lineDirection.X, M12 = lineDirection.Y, M13 = lineDirection.Z };
            linearJacobianB = new Matrix3x3 { M11 = -lineDirection.X, M12 = -lineDirection.Y, M13 = -lineDirection.Z };
            angularJacobianA = new Matrix3x3 { M11 = angularA.X, M12 = angularA.Y, M13 = angularA.Z };
            angularJacobianB = new Matrix3x3 { M11 = angularB.X, M12 = angularB.Y, M13 = angularB.Z };

        }
    }
}
