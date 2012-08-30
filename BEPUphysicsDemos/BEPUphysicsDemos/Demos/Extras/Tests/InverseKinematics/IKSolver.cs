using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.DataStructures;

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
        List<IKControl> controls = new List<IKControl>();
        /// <summary>
        /// Gets the list of controls used by the solver.
        /// </summary>
        public ReadOnlyList<IKControl> Controls
        {
            get { return new ReadOnlyList<IKControl>(controls); }
        }

        /// <summary>
        /// Gets the active joint set associated with the solver.
        /// </summary>
        public ActiveJointSet ActiveJointSet { get; private set; }

        /// <summary>
        /// Gets or sets the number of solver iterations to perform in an attempt to reach specified goals.
        /// </summary>
        public int ControlIterations { get; set; }
        
        /// <summary>
        /// Gets or sets the number of solter iterations to perform after the control iterations in an attempt to minimize
        /// errors introduced by unreachable goals.
        /// </summary>
        public int FixerIterations { get; set; }

        /// <summary>
        /// Constructs a new IKSolver.
        /// </summary>
        public IKSolver()
        {
            ActiveJointSet = new ActiveJointSet();
        }

        /// <summary>
        /// Updates the positions of bones acted upon by the controls given to this solver.
        /// </summary>
        public void Solve()
        {
            //Update the list of active joints.
            ActiveJointSet.UpdateActiveSet(controls);

            //Go through the set of controls and active joints, updating the state of bones.
            for (int i = 0; i < ControlIterations; i++)
            {
                //Controls are updated first, and the active joint set is sorted from closest-to-control constraints to furthest-from-control constraints.
                //This order allows the effect of controls to propagate through the graph quickly.
                //In addition, the last constraints which update get the last word in the state of bones for a given iteration,
                //so solving far constraints last means those constraints connected to the pin endpoint will always succeed in keeping a bone nearby.
                for (int j = 0; j < controls.Count; j++)
                {

                }
                for (int j = 0; j < ActiveJointSet.Joints.Count; j++)
                {

                }
            }

            //The previous loop may still have significant errors in the active joints due to 
            //unreachable targets. Run a secondary pass without the influence of the controls to
            //fix the errors without interference from impossible goals
            //This can potentially cause the bones to move away from the control targets, but with a sufficient
            //number of control iterations, the result is generally a good approximation.
            for (int i = 0; i < FixerIterations; i++)
            {
                for (int j = 0; j < ActiveJointSet.Joints.Count; j++)
                {

                }
            }

        }


        /// <summary>
        /// Adds a control constraint to the solver.
        /// </summary>
        /// <param name="control">Control to add.</param>
        public void Add(IKControl control)
        {
            if (control.solverIndex != -1)
                throw new Exception("Cannot add the control; it already belongs to a solver.");
            control.solverIndex = controls.Count;
            controls.Add(control);

        }

        /// <summary>
        /// Removes a control from the solver.
        /// </summary>
        /// <param name="control">Control to remove.</param>
        public void Remove(IKControl control)
        {
            if (controls[control.solverIndex] != control)
                throw new Exception("Cannot remove the control; it does not belong to this solver.");

            var lastControl = controls[controls.Count - 1];
            controls.RemoveAt(controls.Count - 1);
            controls[control.solverIndex] = lastControl;
            lastControl.solverIndex = control.solverIndex;
            control.solverIndex = -1;
        }
    }
}
