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
    public class ActiveJointSet
    {
        internal List<IKJoint> joints = new List<IKJoint>();
        /// <summary>
        /// Gets the most recently computed set of active joints sorted by their traversal distance from control constraints.
        /// </summary>
        public ReadOnlyList<IKJoint> Joints
        {
            get { return new ReadOnlyList<IKJoint>(joints); }
        }


        //Stores data about an in-process BFS.
        HashSet<IKJoint> visitedJoints = new HashSet<IKJoint>();
        HashSet<IKBone> visitedBones = new HashSet<IKBone>();
        Queue<IKBone> bonesToVisit = new Queue<IKBone>();


        /// <summary>
        /// Updates the ordered set of active joints.
        /// Joints are ordered according to their graph traversal distance from control constraints.
        /// Joints close to the control constraints are closer to index 0 than joints that are far from control constraints.
        /// The relative ordering of joints from different connected components is irrelevant; the only guarantee is that
        /// constraints further from the source in a particular connected component are later in the list than those that are close.
        /// </summary>
        /// <param name="controls">Currently active control constraints.</param>
        internal void UpdateActiveSet(List<IKControl> controls)
        {
            //Clear out the previous list to make way for the new active set.
            joints.Clear();

            //Perform a breadth-first search through the graph starting at the bones targeted by each control.
            foreach (var control in controls)
            {
                bonesToVisit.Enqueue(control.TargetBone);
                //Note that a bone is added to the visited bone set before it is actually processed.
                //This prevents a bone from being put in the queue redundantly.
                visitedBones.Add(control.TargetBone);
            }

            //Note that it's technically possible for multiple controls to affect the same bone.
            //The containment tests will stop it from adding in any redundant constraints as a result.
            while (bonesToVisit.Count > 0)
            {
                var bone = bonesToVisit.Dequeue();
                foreach (var joint in bone.joints)
                {
                    if (!visitedJoints.Contains(joint))
                    {
                        visitedJoints.Add(joint);
                        //This is the first time the joint has been visited, so plop it into the list.
                        joints.Add(joint);
                    }
                    IKBone boneToAdd = joint.ConnectionA == bone ? joint.ConnectionB : joint.ConnectionA;
                    if (!boneToAdd.Pinned && //Pinned bones act as dead ends! Don't try to traverse them.
                        visitedBones.Add(boneToAdd)) //Try to add the bone to the visited bones set.
                    {
                        //The bone was not already present. We should visit it!
                        //Note that a bone is added to the visited bone set before it is actually processed.
                        //This prevents a bone from being put in the queue redundantly.
                        bonesToVisit.Enqueue(boneToAdd);
                    }
                }
            }

            //The visited sets aren't used anywhere else; they don't need to persist.
            visitedJoints.Clear();
            visitedBones.Clear();
        }
    }
}
