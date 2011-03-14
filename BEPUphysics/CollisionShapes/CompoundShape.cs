using System;
using System.Collections.Generic;
using BEPUphysics.Collidables.MobileCollidables;
using Microsoft.Xna.Framework;
using BEPUphysics.DataStructures;
using BEPUphysics.MathExtensions;
using BEPUphysics.ResourceManagement;

namespace BEPUphysics.CollisionShapes
{
    ///<summary>
    /// Contains a shape and its local transform relative to its owning compound shape.
    /// This is used to construct compound shapes.
    ///</summary>
    public struct CompoundShapeEntry
    {
        ///<summary>
        /// Local transform of the shape relative to its owning compound shape.
        ///</summary>
        public RigidTransform LocalTransform;
        /// <summary>
        /// Shape used by the compound.
        /// </summary>
        public EntityShape Shape;

        ///<summary>
        /// Constructs a new compound shape entry.
        ///</summary>
        ///<param name="shape">Shape to use.</param>
        ///<param name="localTransform">Local transform of the shape.</param>
        public CompoundShapeEntry(EntityShape shape, RigidTransform localTransform)
        {
            LocalTransform = localTransform;
            Shape = shape;
        }

        ///<summary>
        /// Constructs a new compound shape entry.
        ///</summary>
        ///<param name="shape">Shape to use.</param>
        ///<param name="position">Local position of the shape.</param>
        public CompoundShapeEntry(EntityShape shape, Vector3 position)
        {
            LocalTransform = new RigidTransform(position);
            Shape = shape;
        }

        ///<summary>
        /// Constructs a new compound shape entry.
        ///</summary>
        ///<param name="shape">Shape to use.</param>
        ///<param name="orientation">Local orientation of the shape.</param>
        public CompoundShapeEntry(EntityShape shape, Quaternion orientation)
        {
            LocalTransform = new RigidTransform(orientation);
            Shape = shape;
        }
        ///<summary>
        /// Constructs a new compound shape entry.
        ///</summary>
        ///<param name="shape">Shape to use.</param>
        public CompoundShapeEntry(EntityShape shape)
        {
            LocalTransform = RigidTransform.Identity;
            Shape = shape;
        }
    }

    ///<summary>
    /// Contains a shape, its local transform, and mass.
    /// Used to create a compound shape.
    ///</summary>
    public struct DynamicCompoundEntry
    {
        ///<summary>
        /// The shape and local transform information of the entry.
        ///</summary>
        public CompoundShapeEntry Entry;
        ///<summary>
        /// Mass of the entry, used to compute the center of mass and other physical properties.
        ///</summary>
        public float Mass;
        ///<summary>
        /// Constructs a new dynamic compound entry.
        ///</summary>
        ///<param name="entry">Entry to use.</param>
        ///<param name="mass">Mass of the entry.</param>
        public DynamicCompoundEntry(CompoundShapeEntry entry, float mass)
        {
            Entry = entry;
            Mass = mass;
        }
        ///<summary>
        /// Constructs a new dynamic compound entry.
        ///</summary>
        ///<param name="shape">Shape of the entry to use.</param>
        ///<param name="transform">Local transform of the entry.</param>
        ///<param name="mass">Mass of the entry.</param>
        public DynamicCompoundEntry(EntityShape shape, RigidTransform transform, float mass)
        {
            Entry = new CompoundShapeEntry(shape, transform);
            Mass = mass;
        }
        ///<summary>
        /// Constructs a new dynamic compound entry.
        ///</summary>
        ///<param name="shape">Shape of the entry to use.</param>
        ///<param name="position">Local position of the entry.</param>
        ///<param name="mass">Mass of the entry.</param>
        public DynamicCompoundEntry(EntityShape shape, Vector3 position, float mass)
        {
            Entry = new CompoundShapeEntry(shape, new RigidTransform(position));
            Mass = mass;
        }
        ///<summary>
        /// Constructs a new dynamic compound entry.
        ///</summary>
        ///<param name="shape">Shape of the entry to use.</param>
        ///<param name="orientation">Local orientation of the entry.</param>
        ///<param name="mass">Mass of the entry.</param>
        public DynamicCompoundEntry(EntityShape shape, Quaternion orientation, float mass)
        {
            Entry = new CompoundShapeEntry(shape, new RigidTransform(orientation));
            Mass = mass;
        }
        ///<summary>
        /// Constructs a new dynamic compound entry.
        ///</summary>
        ///<param name="shape">Shape of the entry to use.</param>
        ///<param name="mass">Mass of the entry.</param>
        public DynamicCompoundEntry(EntityShape shape, float mass)
        {
            Entry = new CompoundShapeEntry(shape);
            Mass = mass;
        }
    }





    ///<summary>
    /// Shape composed of multiple other shapes.
    ///</summary>
    public class CompoundShape : EntityShape
    {
        ObservableList<CompoundShapeEntry> shapes = new ObservableList<CompoundShapeEntry>();
        ///<summary>
        /// Gets the list of shapes in the compound shape.
        ///</summary>
        ///<exception cref="Exception">Thrown if the set shapes list has 0 shapes in it.</exception>
        public ObservableList<CompoundShapeEntry> Shapes
        {
            get
            {
                return shapes;
            }
            set
            {
                if (value.Count > 0)
                {
                    shapes = value;
                    OnShapeChanged();
                }
                else
                {
                    throw new Exception("Compound shape must have at least 1 subshape.");
                }
            }
        }

        ///<summary>
        /// Constructs a compound shape.
        ///</summary>
        ///<param name="shapes">Shape entries used to create the compound.</param>
        public CompoundShape(IList<CompoundShapeEntry> shapes)
            :this(shapes, Vector3.Zero)
        {
        }
        ///<summary>
        /// Constructs a compound shape.
        ///</summary>
        ///<param name="shapes">Shape entries used to create the compound.</param>
        /// <param name="center">Center to use when creating the compound.</param>
        public CompoundShape(IList<CompoundShapeEntry> shapes, Vector3 center)
        {
            if (shapes.Count > 0)
            {
                for (int i = 0; i < shapes.Count; i++)
                {
                    var shape = shapes[i];
                    shape.LocalTransform.Position -= center;
                    this.shapes.Add(shape);
                }
            }
            else
            {
                throw new Exception("Compound shape must have at least 1 subshape.");
            }
        }

        #region EntityShape members and support

        ///<summary>
        /// Computes the minimum radius of the shape.
        /// This is often smaller than the actual minimum radius;
        /// it is simply an approximation that avoids overestimating.
        ///</summary>
        ///<returns>Minimum radius of the shape.</returns>
        public override float ComputeMinimumRadius()
        {
            float minRadius = 0;
            for (int i = 0; i < shapes.list.count; i++)
            {
                float radius = shapes.list.Elements[i].Shape.ComputeMinimumRadius();
                if (radius < minRadius)
                    minRadius = radius;
            }
            return minRadius;
        }

        /// <summary>
        /// Computes the maximum radius of the shape.
        /// This is often larger than the actual maximum radius;
        /// it is simply an approximation that avoids underestimating.
        /// </summary>
        /// <returns>Maximum radius of the shape.</returns>
        public override float ComputeMaximumRadius()
        {
            float maxRadius = 0;
            for (int i = 0; i < shapes.list.count; i++)
            {
                float radius = shapes.list.Elements[i].LocalTransform.Position.Length() + shapes.list.Elements[i].Shape.ComputeMaximumRadius();
                if (radius > maxRadius)
                    maxRadius = radius;
            }
            return maxRadius;
        }

        /// <summary>
        /// Computes the center of the shape.  This can be considered its 
        /// center of mass.  This calculation is often associated with the 
        /// volume calculation, which is given by this method as well.
        /// </summary>
        /// <param name="volume">Volume of the shape.</param>
        /// <returns>Center of the shape.</returns>
        public override Vector3 ComputeCenter(out float volume)
        {
            volume = 0;
            var center = new Vector3();
            for (int i = 0; i < shapes.list.count; i++)
            {
                float volumeContribution = shapes.list.Elements[i].Shape.ComputeVolume();
                volume += volumeContribution;
                Vector3 centerContribution;
                //TODO: This does not factor in the computed center of the subshape.  If inertia tensor is ACTUALLY local, that's fine. 
                //It's a bit weird if the inertia tensor is computed as local, but isn't ACTUALLY local.
                Vector3.Multiply(ref shapes.list.Elements[i].LocalTransform.Position, volumeContribution, out centerContribution);
                Vector3.Add(ref center, ref centerContribution, out center);

            }
            Vector3.Multiply(ref center, 1 / volume, out center);
            return center;
        }

        ///<summary>
        /// Computes the center, volume, and volume-based weights of the entries of the compound shape.
        ///</summary>
        ///<param name="volume">Total volume of the compound shape.</param>
        ///<param name="outputWeights">Volume-based weights of each entry in the compound shape.</param>
        ///<returns></returns>
        public Vector3 ComputeCenter(out float volume, IList<float> outputWeights)
        {
            volume = 0;
            var center = new Vector3();
            for (int i = 0; i < shapes.list.count; i++)
            {
                float volumeContribution = shapes.list.Elements[i].Shape.ComputeVolume();
                outputWeights.Add(volumeContribution);
                volume += volumeContribution;
                Vector3 centerContribution;
                Vector3.Multiply(ref shapes.list.Elements[i].LocalTransform.Position, volumeContribution, out centerContribution);
                Vector3.Add(ref center, ref centerContribution, out center);

            }
            Vector3.Multiply(ref center, 1 / volume, out center);
            return center;
        }

        ///<summary>
        /// Computes the center of a compound using its child data.
        /// Children are weighted using their volumes for contribution to the center of 'mass.'
        ///</summary>
        ///<param name="childData">Child data to use to compute the center.</param>
        ///<returns>Center of the children.</returns>
        public static Vector3 ComputeCenter(IList<CompoundChildData> childData)
        {
            var center = new Vector3();
            float volume = 0;
            for (int i = 0; i < childData.Count; i++)
            {
                float volumeContribution = childData[i].Entry.Shape.ComputeVolume();
                volume += volumeContribution;
                center += childData[i].Entry.LocalTransform.Position * volumeContribution;
            }
            Vector3.Divide(ref center, volume, out center);
            return center;

        }

        ///<summary>
        /// Computes the center of a compound using its child data.
        /// Children are weighted using their volumes for contribution to the center of 'mass.'
        ///</summary>
        ///<param name="childData">Child data to use to compute the center.</param>
        ///<returns>Center of the children.</returns>
        public static Vector3 ComputeCenter(IList<CompoundShapeEntry> childData)
        {
            var center = new Vector3();
            float volume = 0;
            for (int i = 0; i < childData.Count; i++)
            {
                float volumeContribution = childData[i].Shape.ComputeVolume();
                volume += volumeContribution;
                center += childData[i].LocalTransform.Position * volumeContribution;
            }
            Vector3.Divide(ref center, volume, out center);
            return center;

        }

        /// <summary>
        /// Computes the volume of the shape.
        /// </summary>
        /// <returns>Volume of the shape.</returns>
        public override float ComputeVolume()
        {
            float volume = 0;
            for (int i = 0; i < shapes.list.count; i++)
            {
                volume += shapes.list.Elements[i].Shape.ComputeVolume();
            }
            return volume;
        }

        /// <summary>
        /// Computes the volume distribution of the shape as well as its volume.
        /// The volume distribution can be used to compute inertia tensors when
        /// paired with mass and other tuning factors.
        /// </summary>
        /// <param name="volume">Volume of the shape.</param>
        /// <returns>Volume distribution of the shape.</returns>
        public override Matrix3X3 ComputeVolumeDistribution(out float volume)
        {
            var weights = Resources.GetFloatList();
            Vector3 center = ComputeCenter(out volume, weights);
            Matrix3X3 volumeDistribution = ComputeVolumeDistribution(center, weights, volume);
            Resources.GiveBack(weights);
            Matrix3X3.Multiply(ref volumeDistribution, 1 / volume, out volumeDistribution);
            return volumeDistribution;
        }

        /// <summary>
        /// Computes the volume distribution, center, volume-based weights, and total weight (volume) of the shape.
        /// </summary>
        /// <param name="center">Center of the compound.</param>
        /// <param name="weights">Per-shape volume-based weights of the compound.</param>
        /// <param name="totalWeight">Total volume of the shape.</param>
        /// <returns>Volume distribution of the shape.</returns>
        public Matrix3X3 ComputeVolumeDistribution(Vector3 center, List<float> weights, float totalWeight)
        {
            var volumeDistribution = new Matrix3X3();
            for (int i = 0; i < shapes.list.count; i++)
            {
                Matrix3X3 contribution = GetContribution(shapes.list.Elements[i].Shape, ref shapes.list.Elements[i].LocalTransform, ref center, weights[i]);
                Matrix3X3.Add(ref contribution, ref volumeDistribution, out volumeDistribution);

            }
            Matrix3X3.Multiply(ref volumeDistribution, 1 / totalWeight, out volumeDistribution);
            return volumeDistribution;
        }


        /// <summary>
        /// Computes the volume distribution and center of the shape.
        /// </summary>
        /// <param name="entries">Mass-weighted entries of the compound.</param>
        /// <param name="center">Center of the compound.</param>
        /// <returns>Volume distribution of the shape.</returns>
        public static Matrix3X3 ComputeVolumeDistribution(IList<DynamicCompoundEntry> entries, out Vector3 center)
        {
            center = new Vector3();
            float totalWeight = 0;
            for (int i = 0; i < entries.Count; i++)
            {
                center += entries[i].Entry.LocalTransform.Position * entries[i].Mass;
                totalWeight += entries[i].Mass;
            }
            center /= totalWeight;
            var volumeDistribution = new Matrix3X3();
            for (int i = 0; i < entries.Count; i++)
            {
                RigidTransform transform = entries[i].Entry.LocalTransform;
                Matrix3X3 contribution = GetContribution(entries[i].Entry.Shape, ref transform, ref center, entries[i].Mass);
                Matrix3X3.Add(ref volumeDistribution, ref contribution, out volumeDistribution);
            }
            return volumeDistribution;
        }

        ///<summary>
        /// Gets the volume distribution and weight contributed by a single shape.
        ///</summary>
        ///<param name="shape">Shape to use to compute a contribution.</param>
        ///<param name="transform">Transform of the shape.</param>
        ///<param name="center">Center to use when computing the distribution.</param>
        ///<param name="weight">Weight associated with the contribution.</param>
        ///<returns>Volume distribution of the contribution.</returns>
        public static Matrix3X3 GetContribution(EntityShape shape, ref RigidTransform transform, ref Vector3 center, out float weight)
        {
            Matrix3X3 baseContribution = shape.ComputeVolumeDistribution(out weight);
            return TransformContribution(ref transform, ref center, ref baseContribution, weight);
        }

        ///<summary>
        /// Gets the volume distribution contributed by a single shape.
        ///</summary>
        ///<param name="shape">Shape to use to compute a contribution.</param>
        ///<param name="transform">Transform of the shape.</param>
        ///<param name="center">Center to use when computing the distribution.</param>
        ///<param name="weight">Weighting to apply to the contribution.</param>
        ///<returns>Volume distribution of the contribution.</returns>
        public static Matrix3X3 GetContribution(EntityShape shape, ref RigidTransform transform, ref Vector3 center, float weight)
        {
            Matrix3X3 baseContribution = shape.ComputeVolumeDistribution();
            return TransformContribution(ref transform, ref center, ref baseContribution, weight);
        }

        /// <summary>
        /// Modifies a contribution using a transform, position, and weight.
        /// </summary>
        /// <param name="transform">Transform to use to modify the contribution.</param>
        /// <param name="center">Center to use to modify the contribution.</param>
        /// <param name="baseContribution">Original unmodified contribution.</param>
        /// <param name="weight">Weight of the contribution.</param>
        /// <returns>Transformed contribution.</returns>
        public static Matrix3X3 TransformContribution(ref RigidTransform transform, ref Vector3 center, ref Matrix3X3 baseContribution, float weight)
        {
            Matrix3X3 rotation;
            Matrix3X3.CreateFromQuaternion(ref transform.Orientation, out rotation);
            Matrix3X3 inverseRotation;
            Matrix3X3.Transpose(ref rotation, out inverseRotation);

            Matrix3X3 contribution;

            //TODO: Verify contribution

            //Do angular transformed contribution first...
            Matrix3X3.Multiply(ref inverseRotation, ref baseContribution, out contribution);
            Matrix3X3.Multiply(ref contribution, ref rotation, out contribution);

            Matrix3X3 volumeDistribution = contribution;

            //Now add in the offset from the origin.
            Vector3 offset;
            Vector3.Subtract(ref transform.Position, ref center, out offset);
            Matrix3X3 innerProduct;
            Matrix3X3.CreateScale(offset.LengthSquared(), out innerProduct);
            Matrix3X3 outerProduct;
            Matrix3X3.CreateOuterProduct(ref offset, ref offset, out outerProduct);

            Matrix3X3.Subtract(ref innerProduct, ref outerProduct, out contribution);

            Matrix3X3.Add(ref volumeDistribution, ref contribution, out volumeDistribution);
            Matrix3X3.Multiply(ref volumeDistribution, weight, out volumeDistribution);

            return volumeDistribution;
        }


        /// <summary>
        /// Retrieves an instance of an EntityCollidable that uses this EntityShape.  Mainly used by compound bodies.
        /// </summary>
        /// <returns>EntityCollidable that uses this shape.</returns>
        public override EntityCollidable GetMobileInstance()
        {
            return new CompoundCollidable(this);
        }


        /// <summary>
        /// Computes the volume distribution of the shape.
        /// The volume distribution can be used to compute inertia tensors when
        /// paired with mass and other tuning factors.
        /// </summary>
        /// <returns>Volume distribution of the shape.</returns>
        public override Matrix3X3 ComputeVolumeDistribution()
        {
            float volume;
            return ComputeVolumeDistribution(out volume);
        }

        /// <summary>
        /// Computes the center of the shape.  This can be considered its 
        /// center of mass.
        /// </summary>
        /// <returns>Center of the shape.</returns>
        public override Vector3 ComputeCenter()
        {
            float volume;
            return ComputeCenter(out volume);
        }

        /// <summary>
        /// Computes a variety of shape information all at once.
        /// </summary>
        /// <param name="shapeInfo">Properties of the shape.</param>
        public override void ComputeDistributionInformation(out ShapeDistributionInformation shapeInfo)
        {
            shapeInfo.VolumeDistribution = ComputeVolumeDistribution(out shapeInfo.Volume);
            shapeInfo.Center = ComputeCenter();
        }

        #endregion
    }


}
