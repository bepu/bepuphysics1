using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.SolverSystems;

namespace BEPUphysics.NarrowPhaseSystems
{
    /// <summary>
    /// Contains information about a narrow phase pair's constraint.
    /// </summary>
    public class NarrowPhasePairConstraint
    {
        /// <summary>
        /// Constructs a narrow phase pair constraint.
        /// </summary>
        /// <param name="constraint">Constraint owned by the pair.</param>
        public NarrowPhasePairConstraint(SolverUpdateable constraint)
        {
            this.SolverUpdateable = constraint;
        }

        /// <summary>
        /// Gets or sets whether or not this constraint should be removed in the next narrow phase removal opportunity.
        /// </summary>
        public bool ShouldRemove { get; set; }

        /// <summary>
        /// Gets the constraint associated with a narrow phase pair.
        /// </summary>
        public SolverUpdateable SolverUpdateable
        {
            get;
            private set;
        }

    }
}
