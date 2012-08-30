using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    /// <summary>
    /// Connects two bones together.
    /// </summary>
    public abstract class IKJoint : IKConstraint
    {
        /// <summary>
        /// Gets the first bone connected by this joint.
        /// </summary>
        public IKBone ConnectionA { get; private set; }
        /// <summary>
        /// Gets the second bone connected by this joint.
        /// </summary>
        public IKBone ConnectionB { get; private set; }

        bool enabled;
        /// <summary>
        /// Gets or sets whether or not this joint is enabled. If set to true, this joint will be a part of
        /// the joint graph and will undergo solving. If set to false, this joint will be removed from 
        /// </summary>
        public bool Enabled
        {
            get { return enabled; }
            set 
            {
                //The bones must know which joints they are associated with so that the bone-joint graph can be traversed.
                if (enabled && !value)
                {
                    ConnectionA.joints.Remove(this);
                    ConnectionB.joints.Remove(this);
                }
                else if (!enabled && value)
                {
                    ConnectionA.joints.Add(this);
                    ConnectionB.joints.Add(this);
                }
                enabled = value;
            }
        }

        protected IKJoint(IKBone connectionA, IKBone connectionB)
        {
            ConnectionA = connectionA;
            ConnectionB = connectionB;
            Enabled = true;
        }
    }
}
