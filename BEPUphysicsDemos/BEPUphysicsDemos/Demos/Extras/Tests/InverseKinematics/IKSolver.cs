using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    /// <summary>
    /// <para>
    /// This is a little experimental project designed to iteratively converge to a decent solution
    /// to full body inverse kinematics subject to a variety of constraints.
    /// </para>
    /// <para>
    /// It's currently separated from the BEPUphysics library internals because the immediate goal is to test out 
    /// features to be potentially integrated into a blender content pipeline. BEPUphysics interactions with this system
    /// will have to go through the interfaces like everything else for now.
    /// </para>
    /// </summary>
    public class IKSolver
    {
        /// <summary>
        /// This stores the constraints visited by the BFS thus far in the current solve operation.
        /// </summary>
        HashSet<IKConstraint> foundConstraints = new HashSet<IKConstraint>();
        public void Solve()
        {
            //TODO: Generate active graph.
        }

        public void Add(Bone bone)
        {
        }

        public void Add(IKConstraint constraint)
        {

        }
    }
}
