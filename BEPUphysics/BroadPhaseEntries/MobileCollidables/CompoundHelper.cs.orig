using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BEPUphysics.CollisionShapes;
 
using BEPUphysics.MathExtensions;
using BEPUphysics.Entities;
using BEPUphysics.CollisionShapes.ConvexShapes;

namespace BEPUphysics.Collidables.MobileCollidables
{
    /// <summary>
    /// Contains methods to help with splitting compound objects into multiple pieces.
    /// </summary>
    public static class CompoundHelper
    {
        /// <summary>
        /// Splits a single compound collidable into two separate compound collidables and computes information needed by the simulation.
        /// </summary>
        /// <param name="splitPredicate">Delegate which determines if a child in the original compound should be moved to the new compound.</param>
        /// <param name="a">Original compound to be split.  Children in this compound will be removed and added to the other compound.</param>
        /// <param name="b">Compound to receive children removed from the original compound.</param>
        /// <returns>Whether or not the predicate returned true for any element in the original compound and split the compound.</returns>
        public static bool SplitCompound(Func<CompoundChild, bool> splitPredicate,
            Entity<CompoundCollidable> a, out Entity<CompoundCollidable> b)
        {
            var childContributions = a.CollisionInformation.Shape.ComputeChildContributions();
            return SplitCompound(childContributions, splitPredicate, a, out b);
        }
        /// <summary>
        /// Splits a single compound collidable into two separate compound collidables and computes information needed by the simulation.
        /// </summary>
        /// <param name="childContributions">List of distribution information associated with each child shape of the whole compound shape used by the compound being split.</param>
        /// <param name="splitPredicate">Delegate which determines if a child in the original compound should be moved to the new compound.</param>
        /// <param name="a">Original compound to be split.  Children in this compound will be removed and added to the other compound.</param>
        /// <param name="b">Compound to receive children removed from the original compound.</param>
        /// <returns>Whether or not the predicate returned true for any element in the original compound and split the compound.</returns>
        public static bool SplitCompound(IList<ShapeDistributionInformation> childContributions, Func<CompoundChild, bool> splitPredicate,
            Entity<CompoundCollidable> a, out Entity<CompoundCollidable> b)
        {

            ShapeDistributionInformation distributionInfoA, distributionInfoB;
            if (SplitCompound(childContributions, splitPredicate, a, out b, out distributionInfoA, out distributionInfoB))
            {
                return true;
            }
            else
                return false;
        }

        /// <summary>
        /// Splits a single compound collidable into two separate compound collidables and computes information needed by the simulation.
        /// </summary>
        /// <param name="childContributions">List of distribution information associated with each child shape of the whole compound shape used by the compound being split.</param>
        /// <param name="splitPredicate">Delegate which determines if a child in the original compound should be moved to the new compound.</param>
        /// <param name="a">Original compound to be split.  Children in this compound will be removed and added to the other compound.</param>
        /// <param name="b">Compound to receive children removed from the original compound.</param>
        /// <param name="distributionInfoA">Volume, volume distribution, and center information about the new form of the original compound collidable.</param>
        /// <param name="distributionInfoB">Volume, volume distribution, and center information about the new compound collidable.</param>
        /// <returns>Whether or not the predicate returned true for any element in the original compound and split the compound.</returns>
        public static bool SplitCompound(IList<ShapeDistributionInformation> childContributions, Func<CompoundChild, bool> splitPredicate,
                        Entity<CompoundCollidable> a, out Entity<CompoundCollidable> b,
                        out ShapeDistributionInformation distributionInfoA, out ShapeDistributionInformation distributionInfoB)
        {
            CompoundCollidable bCollidable = new CompoundCollidable();
            bCollidable.Shape = a.CollisionInformation.Shape;
            b = new Entity<CompoundCollidable>();


            float weightA, weightB;
            if (SplitCompound(childContributions, splitPredicate, a.CollisionInformation, bCollidable, out distributionInfoA, out distributionInfoB, out weightA, out weightB))
            {
                //Reconfigure the entities using the data computed in the split.
                float originalMass = a.mass;
                float newMassA = (weightA / (weightA + weightB)) * originalMass;
                Matrix3X3.Multiply(ref distributionInfoA.VolumeDistribution, newMassA * InertiaHelper.InertiaTensorScale, out distributionInfoA.VolumeDistribution);
                a.Initialize(a.CollisionInformation, newMassA, distributionInfoA.VolumeDistribution, distributionInfoA.Volume);

                float newMassB = (weightB / (weightA + weightB)) * originalMass;
                Matrix3X3.Multiply(ref distributionInfoB.VolumeDistribution, newMassB * InertiaHelper.InertiaTensorScale, out distributionInfoB.VolumeDistribution);
                b.Initialize(bCollidable, newMassB, distributionInfoB.VolumeDistribution, distributionInfoB.Volume);


                Reposition(a, b, ref distributionInfoA, ref distributionInfoB, weightA, weightB);
                return true;
            }
            else
                return false;
        }

        static void Reposition(Entity a, Entity b, ref ShapeDistributionInformation distributionInfoA, ref ShapeDistributionInformation distributionInfoB, float weightA, float weightB)
        {
            //The compounds are not aligned with the original's position yet.
            //In order to align them, first look at the centers the split method computed.
            //They are offsets from the center of the original shape in local space.
            //These can be used to reposition the objects in world space.
            Vector3 weightedA, weightedB;
            Vector3.Multiply(ref distributionInfoA.Center, weightA, out weightedA);
            Vector3.Multiply(ref distributionInfoB.Center, weightB, out weightedB);
            Vector3 newLocalCenter;
            Vector3.Add(ref weightedA, ref weightedB, out newLocalCenter);
            Vector3.Divide(ref newLocalCenter, weightA + weightB, out newLocalCenter);

            Vector3 localOffsetA;
            Vector3 localOffsetB;
            Vector3.Subtract(ref distributionInfoA.Center, ref newLocalCenter, out localOffsetA);
            Vector3.Subtract(ref distributionInfoB.Center, ref newLocalCenter, out localOffsetB);

            Vector3 originalPosition = a.position;

            b.Orientation = a.Orientation;
            //Imagine a split that does not occur right down the middle, but along a far side: (oo - (o - o))
            //Both distributions will be positive.  Only the first split works properly because there the position
            //was aligned with the local space origin.
            Vector3 offsetA = Vector3.Transform(localOffsetA, a.Orientation);
            Vector3 offsetB = Vector3.Transform(localOffsetB, a.Orientation);
            a.Position = originalPosition + offsetA;
            b.Position = originalPosition + offsetB;

            Vector3 originalLinearVelocity = a.linearVelocity;
            Vector3 originalAngularVelocity = a.angularVelocity;
            a.AngularVelocity = originalAngularVelocity;
            b.AngularVelocity = originalAngularVelocity;
            a.LinearVelocity = originalLinearVelocity + Vector3.Cross(originalAngularVelocity, offsetA);
            b.LinearVelocity = originalLinearVelocity + Vector3.Cross(originalAngularVelocity, offsetB);
        }


        /// <summary>
        /// Splits a single compound collidable into two separate compound collidables and computes information needed by the simulation.
        /// </summary>
        /// <param name="splitPredicate">Delegate which determines if a child in the original compound should be moved to the new compound.</param>
        /// <param name="a">Original compound to be split.  Children in this compound will be removed and added to the other compound.</param>
        /// <param name="b">Compound to receive children removed from the original compound.</param>
        /// <returns>Whether or not the predicate returned true for any element in the original compound and split the compound.</returns>
        public static bool SplitCompound(Func<CompoundChild, bool> splitPredicate,
            Entity<CompoundCollidable> a, Entity<CompoundCollidable> b)
        {
            var childContributions = a.CollisionInformation.Shape.ComputeChildContributions();
            if (SplitCompound(childContributions, splitPredicate, a, b))
            {
                return true;
            }
            else
                return false;
        }


        /// <summary>
        /// Splits a single compound collidable into two separate compound collidables and computes information needed by the simulation.
        /// </summary>
        /// <param name="childContributions">List of distribution information associated with each child shape of the whole compound shape used by the compound being split.</param>
        /// <param name="splitPredicate">Delegate which determines if a child in the original compound should be moved to the new compound.</param>
        /// <param name="a">Original compound to be split.  Children in this compound will be removed and added to the other compound.</param>
        /// <param name="b">Compound to receive children removed from the original compound.</param>
        /// <returns>Whether or not the predicate returned true for any element in the original compound and split the compound.</returns>
        public static bool SplitCompound(IList<ShapeDistributionInformation> childContributions, Func<CompoundChild, bool> splitPredicate,
            Entity<CompoundCollidable> a, Entity<CompoundCollidable> b)
        {
            ShapeDistributionInformation distributionInfoA, distributionInfoB;
            if (SplitCompound(childContributions, splitPredicate, a, b, out distributionInfoA, out distributionInfoB))
            {
                return true;
            }
            else
                return false;
        }

        /// <summary>
        /// Splits a single compound collidable into two separate compound collidables and computes information needed by the simulation.
        /// </summary>
        /// <param name="childContributions">List of distribution information associated with each child shape of the whole compound shape used by the compound being split.</param>
        /// <param name="splitPredicate">Delegate which determines if a child in the original compound should be moved to the new compound.</param>
        /// <param name="distributionInfoA">Volume, volume distribution, and center information about the new form of the original compound collidable.</param>
        /// <param name="distributionInfoB">Volume, volume distribution, and center information about the new compound collidable.</param>
        /// <param name="a">Original compound to be split.  Children in this compound will be removed and added to the other compound.</param>
        /// <param name="b">Compound to receive children removed from the original compound.</param>
        /// <returns>Whether or not the predicate returned true for any element in the original compound and split the compound.</returns>
        public static bool SplitCompound(IList<ShapeDistributionInformation> childContributions, Func<CompoundChild, bool> splitPredicate,
                        Entity<CompoundCollidable> a, Entity<CompoundCollidable> b,
                        out ShapeDistributionInformation distributionInfoA, out ShapeDistributionInformation distributionInfoB)
        {
            float weightA, weightB;
            if (SplitCompound(childContributions, splitPredicate, a.CollisionInformation, b.CollisionInformation, out distributionInfoA, out distributionInfoB, out weightA, out weightB))
            {
                //Reconfigure the entities using the data computed in the split.
                float originalMass = a.mass;
                float newMassA = (weightA / (weightA + weightB)) * originalMass;
                Matrix3X3.Multiply(ref distributionInfoA.VolumeDistribution, newMassA * InertiaHelper.InertiaTensorScale, out distributionInfoA.VolumeDistribution);
                a.Initialize(a.CollisionInformation, newMassA, distributionInfoA.VolumeDistribution, distributionInfoA.Volume);

                float newMassB = (weightB / (weightA + weightB)) * originalMass;
                Matrix3X3.Multiply(ref distributionInfoB.VolumeDistribution, newMassB * InertiaHelper.InertiaTensorScale, out distributionInfoB.VolumeDistribution);
                b.Initialize(b.CollisionInformation, newMassB, distributionInfoB.VolumeDistribution, distributionInfoB.Volume);


                Reposition(a, b, ref distributionInfoA, ref distributionInfoB, weightA, weightB);

                return true;
            }
            else
                return false;
        }

        /// <summary>
        /// Splits a single compound collidable into two separate compound collidables and computes information needed by the simulation.
        /// </summary>
        /// <param name="childContributions">List of distribution information associated with each child shape of the whole compound shape used by the compound being split.</param>
        /// <param name="splitPredicate">Delegate which determines if a child in the original compound should be moved to the new compound.</param>
        /// <param name="a">Original compound to be split.  Children in this compound will be removed and added to the other compound.</param>
        /// <param name="b">Compound to receive children removed from the original compound.</param>
        /// <param name="distributionInfoA">Volume, volume distribution, and center information about the new form of the original compound collidable.</param>
        /// <param name="distributionInfoB">Volume, volume distribution, and center information about the new compound collidable.</param>
        /// <param name="weightA">Total weight associated with the new form of the original compound collidable.</param>
        /// <param name="weightB">Total weight associated with the new compound collidable.</param>
        /// <returns>Whether or not the predicate returned true for any element in the original compound and split the compound.</returns>
        public static bool SplitCompound(IList<ShapeDistributionInformation> childContributions, Func<CompoundChild, bool> splitPredicate,
            CompoundCollidable a, CompoundCollidable b,
            out ShapeDistributionInformation distributionInfoA, out ShapeDistributionInformation distributionInfoB,
            out float weightA, out float weightB)
        {
            bool splitOccurred = false;
            for (int i = a.children.count - 1; i >= 0; i--)
            {
                //The shape doesn't change during this process.  The entity could, though.
                //All of the other collidable information, like the Tag, CollisionRules, Events, etc. all stay the same.
                var child = a.children.Elements[i];
                if (splitPredicate(child))
                {
                    splitOccurred = true;
                    a.children.FastRemoveAt(i);
                    b.children.Add(child);
                }
            }

            if (!splitOccurred)
            {
                //No split occurred, so we cannot proceed.
                distributionInfoA = new ShapeDistributionInformation();
                distributionInfoB = new ShapeDistributionInformation();
                weightA = 0;
                weightB = 0;
                return false;
            }

            //Compute the contributions from the original shape to the new form of the original collidable.
            distributionInfoA = new ShapeDistributionInformation();
            weightA = 0;
            distributionInfoB = new ShapeDistributionInformation();
            weightB = 0;
            for (int i = a.children.count - 1; i >= 0; i--)
            {
                var child = a.children.Elements[i];
                var entry = child.Entry;
                var contribution = childContributions[child.shapeIndex];
                Vector3.Add(ref contribution.Center, ref entry.LocalTransform.Position, out contribution.Center);
                Vector3.Multiply(ref contribution.Center, child.Entry.Weight, out contribution.Center);
                Vector3.Add(ref contribution.Center, ref distributionInfoA.Center, out distributionInfoA.Center);
                distributionInfoA.Volume += contribution.Volume;
                weightA += entry.Weight;
            }
            for (int i = b.children.count - 1; i >= 0; i--)
            {
                var child = b.children.Elements[i];
                var entry = child.Entry;
                var contribution = childContributions[child.shapeIndex];
                Vector3.Add(ref contribution.Center, ref entry.LocalTransform.Position, out contribution.Center);
                Vector3.Multiply(ref contribution.Center, child.Entry.Weight, out contribution.Center);
                Vector3.Add(ref contribution.Center, ref distributionInfoB.Center, out distributionInfoB.Center);
                distributionInfoB.Volume += contribution.Volume;
                weightB += entry.Weight;
            }

            //Average the center out.
            Vector3.Divide(ref distributionInfoA.Center, weightA, out distributionInfoA.Center);
            Vector3.Divide(ref distributionInfoB.Center, weightB, out distributionInfoB.Center);

            //Note that the 'entry' is from the Shape, and so the translations are local to the shape's center.
            //That is not technically the center of the new collidable- distributionInfoA.Center is.
            //Offset the child collidables by -distributionInfoA.Center using their local offset.
            Vector3 offsetA;
            Vector3.Negate(ref distributionInfoA.Center, out offsetA);
            Vector3 offsetB;
            Vector3.Negate(ref distributionInfoB.Center, out offsetB);

            //Compute the unscaled inertia tensor.
            for (int i = a.children.count - 1; i >= 0; i--)
            {
                var child = a.children.Elements[i];
                child.CollisionInformation.localPosition = offsetA;
                var entry = child.Entry;
                var contribution = childContributions[child.shapeIndex];
                CompoundShape.TransformContribution(ref entry.LocalTransform, ref distributionInfoA.Center, ref contribution.VolumeDistribution, entry.Weight, out contribution.VolumeDistribution);
                Vector3.Add(ref entry.LocalTransform.Position, ref offsetA, out entry.LocalTransform.Position);
                Matrix3X3.Add(ref contribution.VolumeDistribution, ref distributionInfoA.VolumeDistribution, out distributionInfoA.VolumeDistribution);
            }
            for (int i = b.children.count - 1; i >= 0; i--)
            {
                var child = b.children.Elements[i];
                child.CollisionInformation.localPosition = offsetB;
                var entry = child.Entry;
                var contribution = childContributions[child.shapeIndex];
                CompoundShape.TransformContribution(ref entry.LocalTransform, ref distributionInfoB.Center, ref contribution.VolumeDistribution, entry.Weight, out contribution.VolumeDistribution);
                Vector3.Add(ref entry.LocalTransform.Position, ref offsetB, out entry.LocalTransform.Position);
                Matrix3X3.Add(ref contribution.VolumeDistribution, ref distributionInfoB.VolumeDistribution, out distributionInfoB.VolumeDistribution);
            }

            //Normalize the volume distribution.
            Matrix3X3.Multiply(ref distributionInfoA.VolumeDistribution, 1 / weightA, out distributionInfoA.VolumeDistribution);
            Matrix3X3.Multiply(ref distributionInfoB.VolumeDistribution, 1 / weightB, out distributionInfoB.VolumeDistribution);

            //Update the hierarchies of the compounds.
            //TODO: Create a new method that does this quickly without garbage.  Requires a new Reconstruct method which takes a pool which stores the appropriate node types.
            a.hierarchy.Tree.Reconstruct(a.children);
            b.hierarchy.Tree.Reconstruct(b.children);

            return true;
        }

    }
}
