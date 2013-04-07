using System;
using System.Collections;
using System.Collections.Generic;
using BEPUutilities.DataStructures;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    /// <summary>
    /// <para>
    /// This is a little experimental project designed to iteratively converge to a decent solution
    /// to full body inverse kinematics subject to a variety of constraints.
    /// </para>
    /// <para>
    /// It's currently separated from the rest of BEPUphysics library internals because the immediate goal is to test out 
    /// features to be potentially integrated into a blender content pipeline. BEPUphysics interactions with this system
    /// will have to go through the interfaces like everything else for now.
    /// </para>
    /// </summary>
    public class IKSolver
    {

        /// <summary>
        /// Gets the active joint set associated with the solver.
        /// </summary>
        public ActiveSet ActiveSet { get; private set; }

        /// <summary>
        /// Gets or sets the number of solver iterations to perform in an attempt to reach specified goals.
        /// </summary>
        public int ControlIterationCount { get; set; }

        /// <summary>
        /// Gets or sets the number of solter iterations to perform after the control iterations in an attempt to minimize
        /// errors introduced by unreachable goals.
        /// </summary>
        public int FixerIterationCount { get; set; }

        /// <summary>
        /// Gets or sets the number of velocity iterations to perform per control or fixer iteration.
        /// </summary>
        public int VelocitySubiterationCount { get; set; }

        /// <summary>
        /// Gets or sets whether or not to scale control impulses such that they fit well with the mass of objects and the number of iterations.
        /// </summary>
        public bool AutoscaleControlImpulses { get; set; }

        /// <summary>
        /// Gets or sets the maximum impulse the controls will try to push bones with when AutoscaleControlImpulses is enabled.
        /// </summary>
        public float AutoscaleControlMaximumForce { get; set; }

        /// <summary>
        /// Constructs a new IKSolver.
        /// </summary>
        public IKSolver()
        {
            ActiveSet = new ActiveSet();
            ControlIterationCount = 50;
            FixerIterationCount = 70;
            VelocitySubiterationCount = 2;
        }

        /// <summary>
        /// Updates the positions of bones acted upon by the joints given to this solver.
        /// This variant of the solver can be used when there are no goal-driven controls in the simulation.
        /// This amounts to running just the 'fixer iterations' of a normal control-driven solve.
        /// </summary>
        public void Solve(List<IKJoint> joints)
        {
            ActiveSet.UpdateActiveSet(joints);

            for (int i = 0; i < FixerIterationCount; i++)
            {
                //Update the world inertia tensors of objects for the latest position.
                foreach (Bone bone in ActiveSet.bones)
                {
                    bone.UpdateInertiaTensor();
                }

                //Update the per-constraint jacobians and effective mass for the current bone orientations and positions.
                foreach (IKJoint joint in ActiveSet.joints)
                {
                    joint.UpdateJacobiansAndVelocityBias();
                    joint.ComputeEffectiveMass();
                    joint.WarmStart();
                }

                for (int j = 0; j < VelocitySubiterationCount; j++)
                {
                    //Controls are updated first, and the active joint set is sorted from closest-to-control constraints to furthest-from-control constraints.
                    //In addition, the last constraints which update get the last word in the state of bones for a given iteration,
                    //so solving far constraints last means those constraints connected to pin endpoints will always succeed in keeping a bone nearby.
                    foreach (IKJoint joint in ActiveSet.joints)
                    {
                        joint.SolveVelocityIteration();
                    }
                }

                //Integrate the positions of the bones forward.
                foreach (Bone bone in ActiveSet.bones)
                {
                    bone.UpdatePosition();
                }
            }

            //Clear out accumulated impulses; they should not persist through to another solving round because the state could be arbitrarily different.
            for (int j = 0; j < ActiveSet.joints.Count; j++)
            {
                ActiveSet.joints[j].ClearAccumulatedImpulses();
            }
        }

        /// <summary>
        /// Updates the positions of bones acted upon by the controls given to this solver.
        /// </summary>
        /// <param name="controls">List of currently active controls.</param>
        public void Solve(List<Control> controls)
        {
            //Update the list of active joints.
            ActiveSet.UpdateActiveSet(controls);

            if (AutoscaleControlImpulses)
            {
                //Update the control strengths to match the mass of the target bones and the desired maximum force.
                foreach (var control in controls)
                {
                    control.MaximumImpulse = control.TargetBone.Mass * AutoscaleControlMaximumForce / ControlIterationCount;
                }
            }


            //Go through the set of controls and active joints, updating the state of bones.
            for (int i = 0; i < ControlIterationCount; i++)
            {
                //Update the world inertia tensors of objects for the latest position.
                foreach (Bone bone in ActiveSet.bones)
                {
                    bone.UpdateInertiaTensor();
                }

                //Update the per-constraint jacobians and effective mass for the current bone orientations and positions.
                foreach (IKJoint joint in ActiveSet.joints)
                {
                    joint.UpdateJacobiansAndVelocityBias();
                    joint.ComputeEffectiveMass();
                    joint.WarmStart();
                }

                foreach (var control in controls)
                {
                    if (control.TargetBone.Pinned)
                        throw new InvalidOperationException("Pinned objects cannot be moved by controls.");
                    control.UpdateJacobiansAndVelocityBias();
                    control.ComputeEffectiveMass();
                    control.WarmStart();
                }

                for (int j = 0; j < VelocitySubiterationCount; j++)
                {
                    //Controls are updated first, and the active joint set is sorted from closest-to-control constraints to furthest-from-control constraints.
                    //This order allows the effect of controls to propagate through the graph quickly.
                    //In addition, the last constraints which update get the last word in the state of bones for a given iteration,
                    //so solving far constraints last means those constraints connected to pin endpoints will always succeed in keeping a bone nearby.
                    foreach (Control control in controls)
                    {
                        control.SolveVelocityIteration();
                    }
                    foreach (IKJoint joint in ActiveSet.joints)
                    {
                        joint.SolveVelocityIteration();
                    }
                }

                //Integrate the positions of the bones forward.
                foreach (Bone bone in ActiveSet.bones)
                {
                    bone.UpdatePosition();
                }
            }

            //The previous loop may still have significant errors in the active joints due to 
            //unreachable targets. Run a secondary pass without the influence of the controls to
            //fix the errors without interference from impossible goals
            //This can potentially cause the bones to move away from the control targets, but with a sufficient
            //number of control iterations, the result is generally a good approximation.
            for (int i = 0; i < FixerIterationCount; i++)
            {
                //Update the world inertia tensors of objects for the latest position.
                foreach (Bone bone in ActiveSet.bones)
                {
                    bone.UpdateInertiaTensor();
                }

                //Update the per-constraint jacobians and effective mass for the current bone orientations and positions.
                foreach (IKJoint joint in ActiveSet.joints)
                {
                    joint.UpdateJacobiansAndVelocityBias();
                    joint.ComputeEffectiveMass();
                    joint.WarmStart();
                }

                for (int j = 0; j < VelocitySubiterationCount; j++)
                {
                    //Controls are updated first, and the active joint set is sorted from closest-to-control constraints to furthest-from-control constraints.
                    //In addition, the last constraints which update get the last word in the state of bones for a given iteration,
                    //so solving far constraints last means those constraints connected to pin endpoints will always succeed in keeping a bone nearby.
                    foreach (IKJoint joint in ActiveSet.joints)
                    {
                        joint.SolveVelocityIteration();
                    }
                }

                //Integrate the positions of the bones forward.
                foreach (Bone bone in ActiveSet.bones)
                {
                    bone.UpdatePosition();
                }
            }

            //Clear out accumulated impulses; they should not persist through to another solving round because the state could be arbitrarily different.
            for (int j = 0; j < ActiveSet.joints.Count; j++)
            {
                ActiveSet.joints[j].ClearAccumulatedImpulses();
            }

            foreach (Control control in controls)
            {
                control.ClearAccumulatedImpulses();
            }
        }



        
    }
}
