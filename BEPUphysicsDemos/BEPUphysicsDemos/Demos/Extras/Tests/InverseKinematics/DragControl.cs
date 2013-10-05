namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    /// <summary>
    /// Constrains an individual bone in an attempt to reach some position goal.
    /// </summary>
    public class DragControl : Control
    {
        /// <summary>
        /// Gets or sets the controlled bone.
        /// </summary>
        public override Bone TargetBone
        {
            get { return LinearMotor.TargetBone; }
            set
            {
                LinearMotor.TargetBone = value;
            }
        }

        /// <summary>
        /// Gets or sets the linear motor used by the control.
        /// </summary>
        public SingleBoneLinearMotor LinearMotor
        {
            get;
            private set;
        }

        public DragControl()
        {
            LinearMotor = new SingleBoneLinearMotor();
            LinearMotor.DampingConstant = 1;
            LinearMotor.StiffnessConstant = 0.25f;
        }

        protected internal override void Preupdate(float dt)
        {
            LinearMotor.Preupdate(dt);
        }

        protected internal override void UpdateJacobiansAndVelocityBias()
        {
            LinearMotor.UpdateJacobiansAndVelocityBias();
        }

        protected internal override void ComputeEffectiveMass()
        {
            LinearMotor.ComputeEffectiveMass();
        }

        protected internal override void WarmStart()
        {
            LinearMotor.WarmStart();
        }

        protected internal override void SolveVelocityIteration()
        {
            LinearMotor.SolveVelocityIteration();
        }

        protected internal override void ClearAccumulatedImpulses()
        {
            LinearMotor.ClearAccumulatedImpulses();
        }

        protected internal override float MaximumForce
        {
            get { return LinearMotor.MaximumForce; }
            set
            {
                LinearMotor.MaximumForce = value;
            }
        }
    }
}