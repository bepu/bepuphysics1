using System;
using System.Collections.Generic;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUutilities;
using BEPUutilities.DataStructures;

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
        /// <summary>
        /// Weight of the entry.  This defines how much the entry contributes to its owner
        /// for the purposes of center of rotation computation.
        /// </summary>
        public float Weight;

        ///<summary>
        /// Constructs a new compound shape entry using the volume of the shape as a weight.
        ///</summary>
        ///<param name="shape">Shape to use.</param>
        ///<param name="localTransform">Local transform of the shape.</param>
        ///<param name="weight">Weight of the entry.  This defines how much the entry contributes to its owner
        /// for the purposes of center of rotation computation.</param>
        public CompoundShapeEntry(EntityShape shape, RigidTransform localTransform, float weight)
        {
            localTransform.Validate();
            LocalTransform = localTransform;
            Shape = shape;
            Weight = weight;
        }

        ///<summary>
        /// Constructs a new compound shape entry using the volume of the shape as a weight.
        ///</summary>
        ///<param name="shape">Shape to use.</param>
        ///<param name="position">Local position of the shape.</param>
        ///<param name="weight">Weight of the entry.  This defines how much the entry contributes to its owner
        /// for the purposes of center of mass and inertia computation.</param>
        public CompoundShapeEntry(EntityShape shape, Vector3 position, float weight)
        {
            position.Validate();
            LocalTransform = new RigidTransform(position);
            Shape = shape;
            Weight = weight;
        }

        ///<summary>
        /// Constructs a new compound shape entry using the volume of the shape as a weight.
        ///</summary>
        ///<param name="shape">Shape to use.</param>
        ///<param name="orientation">Local orientation of the shape.</param>
        ///<param name="weight">Weight of the entry.  This defines how much the entry contributes to its owner
        /// for the purposes of center of rotation computation.</param>
        public CompoundShapeEntry(EntityShape shape, Quaternion orientation, float weight)
        {
            orientation.Validate();
            LocalTransform = new RigidTransform(orientation);
            Shape = shape;
            Weight = weight;
        }
        ///<summary>
        /// Constructs a new compound shape entry using the volume of the shape as a weight.
        ///</summary>
        ///<param name="shape">Shape to use.</param>
        ///<param name="weight">Weight of the entry.  This defines how much the entry contributes to its owner
        /// for the purposes of center of rotation computation.</param>
        public CompoundShapeEntry(EntityShape shape, float weight)
        {
            LocalTransform = RigidTransform.Identity;
            Shape = shape;
            Weight = weight;
        }

        ///<summary>
        /// Constructs a new compound shape entry using the volume of the shape as a weight.
        ///</summary>
        ///<param name="shape">Shape to use.</param>
        ///<param name="localTransform">Local transform of the shape.</param>
        public CompoundShapeEntry(EntityShape shape, RigidTransform localTransform)
        {
            localTransform.Validate();
            LocalTransform = localTransform;
            Shape = shape;
            Weight = shape.Volume;
        }

        ///<summary>
        /// Constructs a new compound shape entry using the volume of the shape as a weight.
        ///</summary>
        ///<param name="shape">Shape to use.</param>
        ///<param name="position">Local position of the shape.</param>
        public CompoundShapeEntry(EntityShape shape, Vector3 position)
        {
            position.Validate();
            LocalTransform = new RigidTransform(position);
            Shape = shape;
            Weight = shape.Volume;
        }

        ///<summary>
        /// Constructs a new compound shape entry using the volume of the shape as a weight.
        ///</summary>
        ///<param name="shape">Shape to use.</param>
        ///<param name="orientation">Local orientation of the shape.</param>
        public CompoundShapeEntry(EntityShape shape, Quaternion orientation)
        {
            orientation.Validate();
            LocalTransform = new RigidTransform(orientation);
            Shape = shape;
            Weight = shape.Volume;
        }
        ///<summary>
        /// Constructs a new compound shape entry using the volume of the shape as a weight.
        ///</summary>
        ///<param name="shape">Shape to use.</param>
        public CompoundShapeEntry(EntityShape shape)
        {
            LocalTransform = RigidTransform.Identity;
            Shape = shape;
            Weight = shape.Volume;
        }
    }




    ///<summary>
    /// Shape composed of multiple other shapes.
    ///</summary>
    public class CompoundShape : EntityShape
    {
        internal RawList<CompoundShapeEntry> shapes;
        ///<summary>
        /// Gets the list of shapes in the compound shape.
        ///</summary>
        public ReadOnlyList<CompoundShapeEntry> Shapes
        {
            get
            {
                return new ReadOnlyList<CompoundShapeEntry>(shapes);
            }
        }



        ///<summary>
        /// Constructs a compound shape.
        ///</summary>
        ///<param name="shapes">Shape entries used to create the compound.</param>
        /// <param name="center">Computed center of the compound shape, using the entry weights.</param>
        public CompoundShape(IList<CompoundShapeEntry> shapes, out Vector3 center)
        {
            if (shapes.Count > 0)
            {
                float volume;
                ComputeVolumeDistribution(shapes, out volume, out volumeDistribution, out center);
                Volume = volume;

                this.shapes = new RawList<CompoundShapeEntry>(shapes);
                //Recenter the shapes.
                for (int i = 0; i < this.shapes.Count; i++)
                {
                    this.shapes.Elements[i].LocalTransform.Position -= center;
                }
            }
            else
            {
                throw new ArgumentException("Compound shape must have at least 1 subshape.");
            }
        }

        ///<summary>
        /// Constructs a compound shape.
        ///</summary>
        ///<param name="shapes">Shape entries used to create the compound.</param>
        public CompoundShape(IList<CompoundShapeEntry> shapes)
        {
            if (shapes.Count > 0)
            {
                float volume;
                Vector3 center;
                ComputeVolumeDistribution(shapes, out volume, out volumeDistribution, out center);
                Volume = volume;

                this.shapes = new RawList<CompoundShapeEntry>(shapes);
                //Recenter the shapes.
                for (int i = 0; i < this.shapes.Count; i++)
                {
                    this.shapes.Elements[i].LocalTransform.Position -= center;
                }
            }
            else
            {
                throw new ArgumentException("Compound shape must have at least 1 subshape.");
            }
        }
        ///<summary>
        /// Constructs a new compound shape from cached data.
        ///</summary>
        ///<param name="shapes">Shape entries used to create the compound. Assumed to be centered properly.</param>
        /// <param name="volumeDescription">Description of the volume and its distribution in the shape. Assumed to be correct; no processing or validation is performed.</param>
        public CompoundShape(IList<CompoundShapeEntry> shapes, EntityShapeVolumeDescription volumeDescription)
        {
            this.shapes = new RawList<CompoundShapeEntry>(shapes);
            UpdateEntityShapeVolume(volumeDescription);
        }
        

        /// <summary>
        /// Computes the volume distribution and center of the shape.
        /// </summary>
        /// <param name="entries">Mass-weighted entries of the compound.</param>
        /// <param name="volume">Summed volume of the constituent shapes. Intersecting volumes get double counted.</param>
        /// <param name="volumeDistribution">Volume distribution of the shape.</param>
        /// <param name="center">Center of the compound.</param>
        public static void ComputeVolumeDistribution(IList<CompoundShapeEntry> entries, out float volume, out Matrix3x3 volumeDistribution, out Vector3 center)
        {
            center = new Vector3();
            float totalWeight = 0;
            volume = 0;
            for (int i = 0; i < entries.Count; i++)
            {
                center += entries[i].LocalTransform.Position * entries[i].Weight;
                volume += entries[i].Shape.Volume;
                totalWeight += entries[i].Weight;
            }
            if (totalWeight <= 0)
                throw new NotFiniteNumberException("Cannot compute distribution; the total weight of a compound shape must be positive.");
            float totalWeightInverse = 1 / totalWeight;
            totalWeightInverse.Validate();
            center *= totalWeightInverse;

            volumeDistribution = new Matrix3x3();
            for (int i = 0; i < entries.Count; i++)
            {
                RigidTransform transform = entries[i].LocalTransform;
                Matrix3x3 contribution;
                TransformContribution(ref transform, ref center, ref entries[i].Shape.volumeDistribution, entries[i].Weight, out contribution);
                Matrix3x3.Add(ref volumeDistribution, ref contribution, out volumeDistribution);
            }
            Matrix3x3.Multiply(ref volumeDistribution, totalWeightInverse, out volumeDistribution);
            volumeDistribution.Validate();
        }


        /// <summary>
        /// Modifies a contribution using a transform, position, and weight.
        /// </summary>
        /// <param name="transform">Transform to use to modify the contribution.</param>
        /// <param name="center">Center to use to modify the contribution.</param>
        /// <param name="baseContribution">Original unmodified contribution.</param>
        /// <param name="weight">Weight of the contribution.</param>
        /// <param name="contribution">Transformed contribution.</param>
        public static void TransformContribution(ref RigidTransform transform, ref Vector3 center, ref Matrix3x3 baseContribution, float weight, out Matrix3x3 contribution)
        {
            Matrix3x3 rotation;
            Matrix3x3.CreateFromQuaternion(ref transform.Orientation, out rotation);
            Matrix3x3 temp;
            
            //Do angular transformed contribution first...
            Matrix3x3.MultiplyTransposed(ref rotation, ref baseContribution, out temp);
            Matrix3x3.Multiply(ref temp, ref rotation, out temp);

            contribution = temp;

            //Now add in the offset from the origin.
            Vector3 offset;
            Vector3.Subtract(ref transform.Position, ref center, out offset);
            Matrix3x3 innerProduct;
            Matrix3x3.CreateScale(offset.LengthSquared(), out innerProduct);
            Matrix3x3 outerProduct;
            Matrix3x3.CreateOuterProduct(ref offset, ref offset, out outerProduct);

            Matrix3x3.Subtract(ref innerProduct, ref outerProduct, out temp);

            Matrix3x3.Add(ref contribution, ref temp, out contribution);
            Matrix3x3.Multiply(ref contribution, weight, out contribution);

        }


        /// <summary>
        /// Retrieves an instance of an EntityCollidable that uses this EntityShape.  Mainly used by compound bodies.
        /// </summary>
        /// <returns>EntityCollidable that uses this shape.</returns>
        public override EntityCollidable GetCollidableInstance()
        {
            return new CompoundCollidable(this);
        }


     

        /// <summary>
        /// Computes a bounding box for the shape given the specified transform.
        /// </summary>
        /// <param name="transform">Transform to apply to the shape to compute the bounding box.</param>
        /// <param name="boundingBox">Bounding box for the shape given the transform.</param>
        public override void GetBoundingBox(ref RigidTransform transform, out BoundingBox boundingBox)
        {
            RigidTransform combinedTransform;
            RigidTransform.Multiply(ref shapes.Elements[0].LocalTransform, ref transform, out combinedTransform);
            shapes.Elements[0].Shape.GetBoundingBox(ref combinedTransform, out boundingBox);

            for (int i = 0; i < shapes.Count; i++)
            {
                RigidTransform.Multiply(ref shapes.Elements[i].LocalTransform, ref transform, out combinedTransform);
                BoundingBox childBoundingBox;
                shapes.Elements[i].Shape.GetBoundingBox(ref combinedTransform, out childBoundingBox);
                BoundingBox.CreateMerged(ref boundingBox, ref childBoundingBox, out boundingBox);
            }
        }
    }


}
