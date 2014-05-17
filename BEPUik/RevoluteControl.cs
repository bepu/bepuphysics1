namespace BEPUik
{
    /// <summary>
    /// Constrains an individual bone in an attempt to keep a bone-attached axis aligned with a specified world axis.
    /// </summary>
    public class RevoluteControl : Control
    {
        /// <summary>
        /// Gets or sets the controlled bone.
        /// </summary>
        public override Bone TargetBone
        {
            get { return AngularMotor.TargetBone; }
            set
            {
                AngularMotor.TargetBone = value;
            }
        }

        /// <summary>
        /// Gets or sets the linear motor used by the control.
        /// </summary>
        public SingleBoneRevoluteConstraint AngularMotor
        {
            get;
            private set;
        }

        public RevoluteControl()
        {
            AngularMotor = new SingleBoneRevoluteConstraint();
            AngularMotor.Rigidity = 1;
        }

        protected internal override void Preupdate(float dt, float updateRate)
        {
            AngularMotor.Preupdate(dt, updateRate);
        }

        protected internal override void UpdateJacobiansAndVelocityBias()
        {
            AngularMotor.UpdateJacobiansAndVelocityBias();
        }

        protected internal override void ComputeEffectiveMass()
        {
            AngularMotor.ComputeEffectiveMass();
        }

        protected internal override void WarmStart()
        {
            AngularMotor.WarmStart();
        }

        protected internal override void SolveVelocityIteration()
        {
            AngularMotor.SolveVelocityIteration();
        }

        protected internal override void ClearAccumulatedImpulses()
        {
            AngularMotor.ClearAccumulatedImpulses();
        }

        public override float MaximumForce
        {
            get { return AngularMotor.MaximumForce; }
            set
            {
                AngularMotor.MaximumForce = value;
            }
        }
    }
}