using System;
using System.Collections.Generic;
using BEPUutilities.DataStructures;

namespace BEPUik
{
    /// <summary>
    /// Manages the subset of joints which potentially need solving.
    /// The active joint set contains connected components in the joint-bone graph which interact with control constraints.
    /// These connected components can be bounded by pinned bones which do not transfer any motion.
    /// </summary>
    public class ActiveSet : IDisposable
    {
        internal List<IKJoint> joints = new List<IKJoint>();
        /// <summary>
        /// Gets the most recently computed set of active joints sorted by their traversal distance from control constraints.
        /// </summary>
        public ReadOnlyList<IKJoint> Joints
        {
            get { return new ReadOnlyList<IKJoint>(joints); }
        }

        internal List<Bone> bones = new List<Bone>();
        /// <summary>
        /// Gets the most recently computed set of active bones sorted by their traversal distance from control constraints.
        /// </summary>
        public ReadOnlyList<Bone> Bones
        {
            get { return new ReadOnlyList<Bone>(bones); }
        }

        /// <summary>
        /// Gets or sets whether or not to automatically configure the masses of bones in the active set based upon their dependencies.
        /// Enabling this makes the solver more responsive and avoids some potential instability.
        /// This will overwrite any existing mass settings.
        /// </summary>
        public bool UseAutomass { get; set; }

        private float automassUnstressedFalloff = 0.9f;
        /// <summary>
        /// Gets or sets the multiplier applied to the mass of a bone before distributing it to the child bones.
        /// Used only when UseAutomass is set to true.
        /// </summary>
        public float AutomassUnstressedFalloff
        {
            get { return automassUnstressedFalloff; }
            set
            {
                automassUnstressedFalloff = Math.Max(value, 0);
            }
        }

        private float automassTarget = 1;
        /// <summary>
        /// Gets or sets the mass that the heaviest bones will have when automass is enabled.
        /// </summary>
        public float AutomassTarget
        {
            get { return automassTarget; }
            set
            {
                if (value <= 0)
                    throw new ArgumentException("Mass must be positive.");
                automassTarget = value;
            }
        }

        //Stores data about an in-process BFS.
        Queue<Bone> bonesToVisit = new Queue<Bone>();


        void FindStressedPaths(List<Control> controls)
        {


            //Start a depth first search from each controlled bone to find any pinned bones.
            //All paths from the controlled bone to the pinned bones are 'stressed.'
            //Stressed bones are given greater mass later on.
            foreach (var control in controls)
            {
                //Paths connecting controls should be considered stressed just in case someone tries to pull things apart.
                //Mark bones affected by controls so we can find them in the traversal.
                foreach (var otherControl in controls)
                {
                    if (otherControl != control) //Don't include the current control; that could cause false positives for stress cycles.
                        otherControl.TargetBone.targetedByOtherControl = true;
                }

                //The control.TargetBone.Parent is null; that's one of the terminating condition for the 'upwards' post-traversal
                //that happens after a pin or stressed path is found.
                FindStressedPaths(control.TargetBone);

                //We've analyzed the whole graph for this control. Clean up the bits we used.
                foreach (var bone in bones)
                {
                    bone.traversed = false;
                    bone.IsActive = false;
                    bone.predecessors.Clear();
                }
                bones.Clear();

                //Get rid of the targetedByOtherControl markings.
                foreach (var otherControl in controls)
                {
                    otherControl.TargetBone.targetedByOtherControl = false;
                }
            }

            //All bones in the active set now have their appropriate StressCount values.



        }

        void NotifyPredecessorsOfStress(Bone bone)
        {
            //We don't need to tell already-stressed bones about the fact that they are stressed.
            //Their predecessors are already stressed either by previous notifications like this or
            //through the predecessors being added on after the fact and seeing that the path was stressed.
            if (!bone.traversed)
            {
                bone.traversed = true;
                bone.stressCount++;
                foreach (var predecessor in bone.predecessors)
                {

                    NotifyPredecessorsOfStress(predecessor);
                }
            }
        }
        void FindStressedPaths(Bone bone)
        {
            bone.IsActive = true; //We must keep track of which bones have been visited
            bones.Add(bone);
            foreach (var joint in bone.joints)
            {
                Bone boneToAnalyze = joint.ConnectionA == bone ? joint.ConnectionB : joint.ConnectionA;
                if (bone.predecessors.Contains(boneToAnalyze) ||  //boneToAnalyze is a parent of bone. Don't revisit them, that's where we came from!
                    boneToAnalyze.predecessors.Contains(bone)) //This bone already explored the next bone; don't do it again.
                    continue;

                if (!boneToAnalyze.Pinned)
                {
                    //The boneToAnalyze is reached by following a path from bone. We record this regardless of whether or not we traverse further.
                    //There is one exception: DO NOT create paths to pinned bones!
                    boneToAnalyze.predecessors.Add(bone);
                }

                if (boneToAnalyze.Pinned || boneToAnalyze.traversed)
                {
                    //This bone is connected to a pinned bone (or a bone which is directly or indirectly connected to a pinned bone)!
                    //This bone and all of its predecessors are a part of a 'stressed path.'
                    //This backwards notification is necessary because this depth first search could attempt a deep branch which winds 
                    //its way back up to a part of the graph which SHOULD be marked as stressed, but is not yet marked because this path
                    //has not popped its way all the way up the stack yet! Left untreated, this would lead to missed stressed paths.
                    NotifyPredecessorsOfStress(bone);
                    continue;
                }

                if (boneToAnalyze.targetedByOtherControl)
                {
                    //We will consider other controls to be sources of stress. This prevents mass ratio issues from allowing multiple controls to tear a structure apart.
                    //We do not, however, stop the traversal here. Allow it to continue.         
                    NotifyPredecessorsOfStress(bone);
                }
                if (boneToAnalyze.IsActive)
                {
                    //The bone has already been visited. We should not proceed.
                    //Any bone which is visited but not stressed is either A: not fully explored yet or B: fully explored.
                    //Given that we followed an unexplored path to the bone, it must be not fully explored.
                    //However, we do not attempt to perform exploration on the bone: any not-yet-fully-explored bones
                    //must belong to one of our parents in the DFS! They will take care of it.
                    continue;
                }

                //The search hasn't yet found a stressed path or pinned bone yet.
                //Keep on movin' on!
                FindStressedPaths(boneToAnalyze);
                //If a child finds a pin, we will be notified of that fact by the above while loop which traverses the parent pointers.
            }


        }

        void NotifyPredecessorsOfCycle(Bone bone)
        {
            //Rather than attempting to only mark cycles, this will simply mark all of the cycle elements and any cycle predecessors up to the unstressed root.
            if (!bone.unstressedCycle && bone.stressCount == 0)
            {
                bone.unstressedCycle = true;
                foreach (var predecessor in bone.predecessors)
                {
                    NotifyPredecessorsOfCycle(predecessor);
                }
            }
        }

        void FindCycles(Bone bone)
        {
            //The current bone is known to not be stressed.
            foreach (var joint in bone.joints)
            {
                Bone boneToAnalyze = joint.ConnectionA == bone ? joint.ConnectionB : joint.ConnectionA;

                if (bone.predecessors.Contains(boneToAnalyze) || //Do not attempt to traverse a path which leads to this bone.
                    boneToAnalyze.predecessors.Contains(bone)) //Do not attempt to traverse a path which was already traversed *from this bone.*
                    continue;
                //We found this bone. Regardless of what happens after, make sure that the bone knows about this path.
                boneToAnalyze.predecessors.Add(bone);

                if (boneToAnalyze.IsActive)
                {
                    //This bone is butting up against a node which was previously visited.
                    //Based on the previous stress path computation, there is only one entry point into an unstressed part of the graph.
                    //by the previous condition, we know it's not our immediate parent. We hit an unstressed part of the graph.

                    //In other words, this is an unstressed cycle.

                    //Rather than attempting to only mark cycles, this will simply mark all of the cycle elements and any cycle predecessors up to the unstressed root.
                    NotifyPredecessorsOfCycle(bone);
                    continue;
                }
                //Note that no testing for pinned bones is necessary; based on the previous stressed path searches,
                //any unstressed bone is known to not be a path to any pinned bones.

                //The root bone is already added to the active set by the parent breadth-first search.
                //Children are added to the active set.
                boneToAnalyze.IsActive = true;
                bones.Add(boneToAnalyze);
                FindCycles(boneToAnalyze);
            }
        }

        private List<Bone> uniqueChildren = new List<Bone>();
        void DistributeMass(Bone bone)
        {
            //Accumulate the number of child joints which we are going to distribute mass to.
            foreach (var joint in bone.joints)
            {
                Bone boneToAnalyze = joint.ConnectionA == bone ? joint.ConnectionB : joint.ConnectionA;

                if (boneToAnalyze.traversed || boneToAnalyze.unstressedCycle ||
                    uniqueChildren.Contains(boneToAnalyze)) //There could exist multiple joints involved with the same pair of bones; don't continually double count.
                {
                    //The bone was already visited or was a member of the stressed path we branched from. Do not proceed.
                    continue;
                }
                uniqueChildren.Add(boneToAnalyze);
            }
            //We distribute a portion of the current bone's total mass to the child bones.
            //By applying a multiplier automassUnstressedFalloff, we guarantee that a chain has a certain maximum weight (excluding cycles).
            //This is thanks to the convergent geometric series sum(automassUnstressedFalloff^n, 1, infinity).
            float massPerChild = uniqueChildren.Count > 0 ? automassUnstressedFalloff * bone.Mass / uniqueChildren.Count : 0;

            uniqueChildren.Clear();
            //(If the number of children is 0, then the only bones which can exist are either bones which were already traversed and will be skipped
            //or bones which are members of unstressed cycles and will inherit the full parent weight. Don't have to worry about the 0 mass.)

            //The current bone is known to not be stressed.
            foreach (var joint in bone.joints)
            {
                Bone boneToAnalyze = joint.ConnectionA == bone ? joint.ConnectionB : joint.ConnectionA;
                //Note that no testing for pinned bones is necessary; based on the previous stressed path searches,
                //any unstressed bone is known to not be a path to any pinned bones.
                if (boneToAnalyze.traversed)// || bone.unstressedCycle)//bone.predecessors.Contains(boneToAnalyze))
                {
                    //The bone was already visited or was a member of the stressed path we branched from. Do not proceed.
                    continue;
                }

                if (boneToAnalyze.unstressedCycle)
                {
                    //This bone is part of a cycle! We cannot give it less mass; that would add in a potential instability.
                    //Just give it the current node's full mass.
                    boneToAnalyze.Mass = bone.Mass;
                }
                else
                {
                    //This bone is not a part of a cycle; give it the allotted mass.
                    boneToAnalyze.Mass = massPerChild;
                }
                //The root bone is already added to the traversal set; add the children.
                boneToAnalyze.traversed = true;
                //Note that we do not need to add anything to the bones list here; the previous FindCycles DFS on this unstressed part of the graph did it for us.
                DistributeMass(boneToAnalyze);

            }

        }

        void DistributeMass(List<Control> controls)
        {
            //We assume that all stressed paths have already been marked with nonzero StressCounts.
            //Perform a multi-origin breadth-first search starting at every control. Look for any bones
            //which still have a StressCount of zero.

            //These zero-StressCount bones are the beginnings of isolated 'limbs' in the graph; there is only
            //one bone-to-bone connection (potentially made of multiple constraints, of course, but that does not affect graph connectivity)
            //between the stressed component of the graph and the isolated limb.
            //That means any traversal starting at that first bone and moving out away from the stressed graph will never return to the stressed graph
            //(no bone can be revisited).

            //Because these unstressed limbs are not critical weight-carrying paths, they do not need to be as heavy as the stressed paths.
            //In addition, to make the IK more responsive, unstressed bones further from the stressed component of the graph can be made less massive.

            //Care must be taken in determining the masses, though; if the root is light and its children, while individually lighter, are cumulatively much heavier,
            //there could be mass-ratio related instability.

            //To address this, cycles are found and given equal mass and each noncycle branch splits the current object's mass between all noncycle children.


            //Perform a breadth-first search through the graph starting at the bones targeted by each control.
            foreach (var control in controls)
            {
                bonesToVisit.Enqueue(control.TargetBone);
                //Note that a bone is added to the visited bone set before it is actually processed.
                //This prevents a bone from being put in the queue redundantly.
                control.TargetBone.IsActive = true;
                //A second traversal flag is required for the mass distribution phase on each unstressed part to work efficiently.
                control.TargetBone.traversed = true;
                bones.Add(control.TargetBone);
            }

            //Note that it's technically possible for multiple controls to affect the same bone.
            //The containment tests will stop it from adding in any redundant constraints as a result.
            while (bonesToVisit.Count > 0)
            {
                var bone = bonesToVisit.Dequeue();
                if (bone.stressCount == 0)
                {
                    bone.Mass = automassUnstressedFalloff;
                    //This is an unstressed bone. We should start a DFS to identify any cycles in the unstressed graph.
                    FindCycles(bone);
                    //Once the cycles are marked, we can proceed through the unstressed graph component and give child bones mass.
                    DistributeMass(bone);
                    //Do not continue the breadth-first search into the unstressed part of the graph.
                    continue;
                }
                else
                {
                    //The mass of stressed bones is a multiplier on the number of stressed paths overlapping the bone.
                    bone.Mass = bone.stressCount;
                }
                //This bone is not an unstressed branch root. Continue the breadth first search!
                foreach (var joint in bone.joints)
                {
                    Bone boneToAdd = joint.ConnectionA == bone ? joint.ConnectionB : joint.ConnectionA;
                    if (!boneToAdd.Pinned && //Pinned bones act as dead ends! Don't try to traverse them.
                        !boneToAdd.IsActive) //Don't try to add a bone if it's already active.
                    {
                        boneToAdd.IsActive = true;
                        //A second traversal flag is required for the mass distribution phase on each unstressed part to work efficiently.
                        boneToAdd.traversed = true;
                        boneToAdd.predecessors.Add(bone);
                        //The bone was not already present in the active set. We should visit it!
                        //Note that a bone is added to the visited bone set before it is actually processed.
                        //This prevents a bone from being put in the queue redundantly.
                        bonesToVisit.Enqueue(boneToAdd);
                        bones.Add(boneToAdd);
                    }
                }
            }

            //Normalize the masses of objects so that the heaviest bones have AutomassTarget mass.
            float lowestInverseMass = float.MaxValue;
            foreach (var bone in bones)
            {
                if (bone.inverseMass < lowestInverseMass)
                    lowestInverseMass = bone.inverseMass;
            }

            float inverseMassScale = 1 / (AutomassTarget * lowestInverseMass);

            foreach (var bone in bones)
            {
                //Normalize the mass to the AutomassTarget.
                bone.inverseMass *= inverseMassScale;

                //Also clear the traversal flags while we're at it.
                bone.IsActive = false;
                bone.traversed = false;
                bone.stressCount = 0;
                bone.unstressedCycle = false;
                bone.predecessors.Clear();
            }

            bones.Clear();
        }

        /// <summary>
        /// Clears out the bone and joint listings and unsets all flags.
        /// </summary>
        public void Clear()
        {
            for (int i = 0; i < bones.Count; i++)
            {
                bones[i].IsActive = false;
                bones[i].stressCount = 0;
                bones[i].predecessors.Clear();
                bones[i].Mass = .01f;
            }
            for (int i = 0; i < joints.Count; i++)
            {
                joints[i].IsActive = false;
            }
            bones.Clear();
            joints.Clear();
        }

        internal void UpdateActiveSet(List<IKJoint> joints)
        {
            //Clear out the previous active set to make way for the new active set.   
            //Note that the below flag clearing and usage creates a requirement.
            //Two IKSolvers cannot operate on the same graph; the active set flags could be corrupted.
            Clear();

            for (int i = 0; i < joints.Count; ++i)
            {
                if (joints[i].Enabled)
                {
                    if (!joints[i].ConnectionA.IsActive)
                    {
                        joints[i].ConnectionA.IsActive = true;
                        bones.Add(joints[i].ConnectionA);
                    }

                    if (!joints[i].ConnectionB.IsActive)
                    {
                        joints[i].ConnectionB.IsActive = true;
                        bones.Add(joints[i].ConnectionB);
                    }

                    this.joints.Add(joints[i]);
                }
            }

            //Use an arbitrary mass for the bones.
            //This could conceivably encounter issues with pathological cases, but we don't have controls to easily guide a better choice.
            if (UseAutomass)
            {
                for (int i = 0; i < bones.Count; ++i)
                {
                    bones[i].Mass = automassTarget;
                }
            }




        }

        /// <summary>
        /// Updates the ordered set of active joints.
        /// Joints are ordered according to their graph traversal distance from control constraints.
        /// Joints close to the control constraints are closer to index 0 than joints that are far from control constraints.
        /// The relative ordering of joints from different connected components is irrelevant; the only guarantee is that
        /// constraints further from the source in a particular connected component are later in the list than those that are close.
        /// </summary>
        /// <param name="controls">Currently active control constraints.</param>
        internal void UpdateActiveSet(List<Control> controls)
        {
            //Clear out the previous active set to make way for the new active set.
            //Note that the below flag clearing and usage creates a requirement.
            //Two IKSolvers cannot operate on the same graph; the active set flags could be corrupted.
            Clear();

            if (UseAutomass)
            {
                //Identify the stressed bones.
                FindStressedPaths(controls);

                //Compute the dependency graph for all the unstressed bones and assign masses.
                DistributeMass(controls);
            }

            //While we have traversed the whole active set in the previous stressed/unstressed searches, we do not yet have a proper breadth-first constraint ordering available.

            //Perform a breadth-first search through the graph starting at the bones targeted by each control.
            foreach (var control in controls)
            {
                bonesToVisit.Enqueue(control.TargetBone);
                //Note that a bone is added to the visited bone set before it is actually processed.
                //This prevents a bone from being put in the queue redundantly.
                control.TargetBone.IsActive = true;
                bones.Add(control.TargetBone);
            }

            //Note that it's technically possible for multiple controls to affect the same bone.
            //The containment tests will stop it from adding in any redundant constraints as a result.
            while (bonesToVisit.Count > 0)
            {
                var bone = bonesToVisit.Dequeue();
                foreach (var joint in bone.joints)
                {
                    if (!joint.IsActive)
                    {
                        joint.IsActive = true;
                        //This is the first time the joint has been visited, so plop it into the list.
                        joints.Add(joint);
                    }
                    Bone boneToAdd = joint.ConnectionA == bone ? joint.ConnectionB : joint.ConnectionA;
                    if (!boneToAdd.Pinned && //Pinned bones act as dead ends! Don't try to traverse them.
                        !boneToAdd.IsActive) //Don't try to add a bone if it's already active.
                    {
                        boneToAdd.IsActive = true;
                        //The bone was not already present in the active set. We should visit it!
                        //Note that a bone is added to the visited bone set before it is actually processed.
                        //This prevents a bone from being put in the queue redundantly.
                        bonesToVisit.Enqueue(boneToAdd);
                        bones.Add(boneToAdd);
                    }
                }
            }

        }

        ~ActiveSet()
        {
            Dispose();
        }

        public void Dispose()
        {
            Clear();
        }
    }
}
