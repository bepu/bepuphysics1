using BEPUphysics.Constraints.TwoEntity;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Entities;
using BEPUutilities;

namespace BEPUphysics.Constraints.SolverGroups
{
    /// <summary>
    /// Restricts the linear and angular motion between two entities.
    /// </summary>
    public class WeldJoint : SolverGroup
    {
        /// <summary>
        /// Constructs a new constraint which restricts the linear and angular motion between two entities.
        /// This constructs the internal constraints, but does not configure them.  Before using a constraint constructed in this manner,
        /// ensure that its active constituent constraints are properly configured.  The entire group as well as all internal constraints are initially inactive (IsActive = false).
        /// </summary>
        public WeldJoint()
        {
            IsActive = false;
            BallSocketJoint = new BallSocketJoint();
            NoRotationJoint = new NoRotationJoint();
            Add(BallSocketJoint);
            Add(NoRotationJoint);
        }

        private static Vector3 GetAnchorGuess(Entity connectionA, Entity connectionB)
        {
            var anchor = new Vector3();
            if (connectionA != null)
                anchor += connectionA.position;
            if (connectionB != null)
                anchor += connectionB.position;
            if (connectionA != null && connectionB != null)
                anchor *= 0.5f;
            return anchor;
        }

        /// <summary>
        /// Constructs a new constraint which restricts the linear and angular motion between two entities.
        /// Uses the average of the two entity positions for the anchor.
        /// </summary>
        /// <param name="connectionA">First entity of the constraint pair.</param>
        /// <param name="connectionB">Second entity of the constraint pair.</param>
        public WeldJoint(Entity connectionA, Entity connectionB)
            : this(connectionA, connectionB, GetAnchorGuess(connectionA, connectionB))
        {
        }

        /// <summary>
        /// Constructs a new constraint which restricts the linear and angular motion between two entities.
        /// </summary>
        /// <param name="connectionA">First entity of the constraint pair.</param>
        /// <param name="connectionB">Second entity of the constraint pair.</param>
        /// <param name="anchor">The location of the weld.</param>
        public WeldJoint(Entity connectionA, Entity connectionB, Vector3 anchor)
        {
            if (connectionA == null)
                connectionA = TwoEntityConstraint.WorldEntity;
            if (connectionB == null)
                connectionB = TwoEntityConstraint.WorldEntity;
            BallSocketJoint = new BallSocketJoint(connectionA, connectionB, anchor);
            NoRotationJoint = new NoRotationJoint(connectionA, connectionB);
            Add(BallSocketJoint);
            Add(NoRotationJoint);
        }

        /// <summary>
        /// Gets the ball socket joint that restricts linear degrees of freedom.
        /// </summary>
        public BallSocketJoint BallSocketJoint { get; private set; }

        /// <summary>
        /// Gets the no rotation joint that prevents angular motion.
        /// </summary>
        public NoRotationJoint NoRotationJoint { get; private set; }

        
    }
}