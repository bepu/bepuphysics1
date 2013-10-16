namespace BEPUik
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
            LinearMotor.Rigidity = 1;
        }

        protected internal override void Preupdate(float dt, float updateRate)
        {
            LinearMotor.Preupdate(dt, updateRate);
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

        public override float MaximumForce
        {
            get { return LinearMotor.MaximumForce; }
            set
            {
                LinearMotor.MaximumForce = value;
            }
        }
    }
}