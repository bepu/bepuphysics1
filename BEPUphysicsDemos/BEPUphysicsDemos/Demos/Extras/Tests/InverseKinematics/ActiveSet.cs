using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.DataStructures;

namespace BEPUphysicsDemos.Demos.Extras.Tests.InverseKinematics
{
    /// <summary>
    /// Manages the subset of joints which potentially need solving.
    /// The active joint set contains connected components in the joint-bone graph which interact with control constraints.
    /// These connected components can be bounded by pinned bones which do not transfer any motion.
    /// </summary>
    public class ActiveSet
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

        private float massPerStressPath = 1;
        /// <summary>
        /// Gets or sets the multiplier to apply to a bone's stress path count to find the mass.
        /// Used only when UseAutomass is set to true.
        /// </summary>
        public float MassPerStressPath
        {
            get { return massPerStressPath; }
            set { massPerStressPath = Math.Max(value, 0); }
        }

        //Stores data about an in-process BFS.
        //HashSet<IKJoint> visitedJoints = new HashSet<IKJoint>();
        //HashSet<IKBone> visitedBones = new HashSet<IKBone>();
        Queue<Bone> bonesToVisit = new Queue<Bone>();


        void FindStressedPaths(List<Control> controls)
        {
            //Start a depth first search from each controlled bone to find any pinned bones.
            //All paths from the controlled bone to the pinned bones are 'stressed.'
            //Stressed bones are given greater mass later on.
            foreach (var control in controls)
            {
                //The control.TargetBone.Parent is null; that's one of the terminating condition for the 'upwards' post-traversal
                //that happens after a pin or stressed path is found.
                FindStressedPaths(control.TargetBone);

                //The stressed traversal state needs to be reset for each control; it will be reused by the next control.
                //(cannot reuse the old stressed states without clearing them; each control 'layer' is added to the last
                //to figure out the points with the greatest degree of dependency (StressCount).)
                foreach (var bone in bones)
                {
                    bone.traversed = false;
                    bone.IsActive = false;
                }
                //Don't forget to clear out the accumulated bones list!
                bones.Clear();
            }

            //All bones in the active set now have their appropriate StressCount values.

        }

        void FindStressedPaths(Bone bone)
        {
            bone.IsActive = true; //We must keep track of which bones have been visited
            bones.Add(bone);
            foreach (var joint in bone.joints)
            {
                Bone boneToAnalyze = joint.ConnectionA == bone ? joint.ConnectionB : joint.ConnectionA;
                if (boneToAnalyze.IsActive)
                {
                    //The bone has already been visited. We should not proceed.
                    continue;
                }
                if (boneToAnalyze.Pinned || boneToAnalyze.traversed)
                {
                    //This bone is connected to a pinned bone (or a bone which is directly or indirectly connected to a pinned bone)!
                    //This bone and all of its parents returning to the root are a part of a 'stressed path.'
                    //This backwards notification is necessary because this depth first search could attempt a deep branch which winds 
                    //its way back up to a part of the graph which SHOULD be marked as stressed, but is not yet marked because this path
                    //has not popped its way all the way up the stack yet! Left untreated, this would lead to missed stressed paths.
                    Bone traversalBone = bone;
                    while (true)
                    {
                        traversalBone.stressCount++;
                        traversalBone.traversed = true;
                        if (traversalBone.parent == null ||
                            traversalBone.parent.traversed)//If the parent is already stressed, it got the message from another path. We don't need to/shouldn't proceed.
                            break;
                        traversalBone = traversalBone.parent;
                    }
                    continue;
                }

                //The search hasn't yet found a stressed path or pinned bone yet.
                //Keep on movin' on!
                //Keep the parent pointer handy so that we can work our way back up when we find a pin or stressed path.
                boneToAnalyze.parent = bone;
                FindStressedPaths(boneToAnalyze);
                //If a child finds a pin, we will be notified of that fact by the above while loop which traverses the parent pointers.
            }
        }

        void FindCycles(Bone bone)
        {
            //The current bone is known to not be stressed.
            foreach (var joint in bone.joints)
            {
                Bone boneToAnalyze = joint.ConnectionA == bone ? joint.ConnectionB : joint.ConnectionA;
                if (boneToAnalyze == bone.parent)
                    continue; //Don't consider the parent. We can't only test for activity to avoid it; that is how we find cycles.
                if (boneToAnalyze.IsActive)
                {
                    //This bone is butting up against a node which was previously visited.
                    //Based on the previous stress path computation, there is only one entry point into an unstressed part of the graph.
                    //by the previous condition, we know it's not our immediate parent. We hit an unstressed part of the graph.
                    //The unstressed part of the graph is explored by this DFS, taking the deepest branches first.
                    //If we've ran into an already-visited bone which isn't our *direct* parent,
                    //it must in the chain of our parents up to the root of the unstressed part of the graph or
                    //it would have already been explored by the DFS.

                    //In other words, this is an unstressed cycle.

                    //Because we know the hit node is one of our (potentially distant great-*) grandparents,
                    //the parent path can be traversed until the bone is found. All bones along the path are
                    //marked as cyclic.
                    Bone traversalBone = bone;
                    while (true)
                    {
                        traversalBone.unstressedCycle = true;
                        if (traversalBone == boneToAnalyze)
                            break;
                        //Note: It is not possible for the parent to be null before boneToAnalyze is found; there is no need to test for it.
                        traversalBone = traversalBone.parent;
                    }
                    //Note: We do not stop the DFS here or anything like that. It just continues regularly, looking for more paths.
                }
                //Note that no testing for pinned bones is necessary; based on the previous stressed path searches,
                //any unstressed bone is known to not be a path to any pinned bones.

                //The root bone is already added to the active set by the parent breadth-first search.
                //Children are added to the active set.
                boneToAnalyze.IsActive = true;
                bones.Add(boneToAnalyze);
                FindCycles(boneToAnalyze);
                //Don't have to assign parent pointers; the stress path calculation already did that for us.
            }
        }

        void DistributeMass(Bone bone)
        {
            //Accumulate the number of child joints which we are going to distribute mass to.
            int numberOfChildren = 0;
            foreach (var joint in bone.joints)
            {
                Bone boneToAnalyze = joint.ConnectionA == bone ? joint.ConnectionB : joint.ConnectionA;

                if (boneToAnalyze.traversed)
                {
                    //The bone was already visited or was a member of the stressed path we branched from. Do not proceed.
                    continue;
                }
                numberOfChildren++;
            }
            if (numberOfChildren > 0)
            {
                //We distribute a portion of the current bone's total mass to the child bones.
                //By applying a multiplier automassUnstressedFalloff, we guarantee that a chain has a certain maximum weight (excluding cycles).
                //This is thanks to the convergent geometric series sum(automassUnstressedFalloff^n, 1, infinity).
                float massPerChild = automassUnstressedFalloff * bone.Mass / numberOfChildren;
                //The current bone is known to not be stressed.
                foreach (var joint in bone.joints)
                {
                    Bone boneToAnalyze = joint.ConnectionA == bone ? joint.ConnectionB : joint.ConnectionA;
                    //Note that no testing for pinned bones is necessary; based on the previous stressed path searches,
                    //any unstressed bone is known to not be a path to any pinned bones.
                    if (boneToAnalyze.traversed)
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

            //To address this, a search through this isolated limb's graph is performed and the number of bones dependent on each bone is computed.
            //(This will use the convention that a leaf node has one dependency- itself- so a tree of height one with 3 nodes will have two leaves of dependency count 1 and a root of dependency count 3.)

            //Those dependency counts are used to assign stable masses to the unstressed portions of the graph. A bone with more dependents is heavier, but still less than or equal to a stressed bone.


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
                    bone.Mass = massPerStressPath * automassUnstressedFalloff;
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
                    bone.Mass = massPerStressPath * bone.stressCount;
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
                        //The bone was not already present in the active set. We should visit it!
                        //Note that a bone is added to the visited bone set before it is actually processed.
                        //This prevents a bone from being put in the queue redundantly.
                        bonesToVisit.Enqueue(boneToAdd);
                        bones.Add(boneToAdd);
                    }
                }
            }

            //Clean the bones up!
            for (int i = 0; i < bones.Count; i++)
            {
                bones[i].IsActive = false;
                bones[i].traversed = false;
                bones[i].stressCount = 0;
                bones[i].parent = null;
            }
            bones.Clear();
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
            for (int i = 0; i < bones.Count; i++)
            {
                bones[i].IsActive = false;
                bones[i].stressCount = 0;
                bones[i].parent = null;
                bones[i].Mass = 1;
            }
            for (int i = 0; i < joints.Count; i++)
            {
                joints[i].IsActive = false;
            }
            bones.Clear();
            joints.Clear();

            if (UseAutomass)
            {
                //Identify the stressed bones.
                FindStressedPaths(controls);

                //Compute the dependency graph for all the unstressed bones and assign masses.
                DistributeMass(controls);

                ////All the stress counts and dependency counts are now available on each bone.

                //foreach (var bone in bones)
                //{
                //    bone.IsActive = false;
                //    if (bone.stressCount > 0)
                //        bone.Mass = bone.stressCount;
                //    else
                //        bone.Mass = .5f; //For now, just arbitrarily choose a low mass for all unstressed paths (a fraction of the weight of a one-path stressed bone).
                //    //Keep in mind here that the parents are still not null for unstressed things!
                //}

                ////Clear out all the touched bones in preparation for the upcoming traversal.
                //bones.Clear();
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


    }
}
