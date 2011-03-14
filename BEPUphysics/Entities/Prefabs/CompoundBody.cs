using System;
using System.Collections.Generic;
using BEPUphysics.Collidables.MobileCollidables;
using Microsoft.Xna.Framework;
using BEPUphysics.DataStructures;
using BEPUphysics.CollisionShapes;
using BEPUphysics.MathExtensions;

namespace BEPUphysics.Entities.Prefabs
{
    /// <summary>
    /// Acts as a grouping of multiple other objects.  Can be used to form physically simulated concave shapes.
    /// </summary>
    public class CompoundBody : Entity<CompoundCollidable>
    {
        ///<summary>
        /// Gets or sets the list of shapes in the compound.
        ///</summary>
        public ObservableList<CompoundShapeEntry> Shapes
        {
            get
            {
                return CollisionInformation.Shape.Shapes;
            }
            set
            {
                CollisionInformation.Shape.Shapes = value;
            }
        }


        /// <summary>
        /// Creates a new CompoundBody with the given subbodies.
        /// If all of the bodies are dynamic, the compound body is dynamic; if all of the bodies are kinematic, the compound body is kinematic.
        /// </summary>
        /// <param name="bodies">List of entities to use as subbodies of the compound body.</param>
        /// <exception cref="InvalidOperationException">Thrown when the bodies list is empty or there is a mix of kinematic and dynamic entities in the body list.</exception>
        public CompoundBody(IList<CompoundShapeEntry> bodies)
        {
            //Recenter the data based on volume since the per shape masses aren't available.
            Vector3 center = CompoundShape.ComputeCenter(bodies);
            Initialize(new CompoundCollidable(new CompoundShape(bodies, center)));
            Position = center;
        }


        /// <summary>
        /// Creates a new CompoundBody with the given subbodies.
        /// If all of the bodies are dynamic, the compound body is dynamic; if all of the bodies are kinematic, the compound body is kinematic.
        /// </summary>
        /// <param name="bodies">List of entities to use as subbodies of the compound body.</param>
        /// <param name="mass">Mass of the compound.</param>
        /// <exception cref="InvalidOperationException">Thrown when the bodies list is empty or there is a mix of kinematic and dynamic entities in the body list.</exception>
        public CompoundBody(IList<CompoundShapeEntry> bodies, float mass)
        {
            //Recenter the data based on volume since the per shape masses aren't available.
            Vector3 center = CompoundShape.ComputeCenter(bodies);
            Initialize(new CompoundCollidable(new CompoundShape(bodies, center)), mass);
            Position = center;
        }

        ///<summary>
        /// Constructs a compound body from a list of dynamic entries.
        ///</summary>
        ///<param name="bodies">Entries to construct the compound from.</param>
        public CompoundBody(IList<DynamicCompoundEntry> bodies)
        {
            var shapeEntries = new RawList<CompoundShapeEntry>(bodies.Count);

            float mass = 0;
            for (int i = 0; i < bodies.Count; i++)
            {
                shapeEntries.Add(bodies[i].Entry);
                mass += bodies[i].Mass;

            }

            //Recenter the data based on the data from the dynamic entries.
            Vector3 center;
            Matrix3X3 inertia = CompoundShape.ComputeVolumeDistribution(bodies, out center);
            Initialize(new CompoundCollidable(new CompoundShape(shapeEntries, center)), mass, inertia);
            Position = center;
        }


        ///<summary>
        /// Constructs a kinematic compound body from the children data.
        ///</summary>
        ///<param name="children">Children data to construct the compound from.</param>
        public CompoundBody(IList<CompoundChildData> children)
        {
            //Recenter the data based on volume since the per shape masses aren't available.
            Position = CompoundShape.ComputeCenter(children);
            Initialize(new CompoundCollidable(children, Position));
        }

        ///<summary>
        /// Constructs a dynamic compound body from the children data.
        ///</summary>
        ///<param name="children">Children data to construct the compound from.</param>
        ///<param name="mass">Mass of the compound body.</param>
        public CompoundBody(IList<CompoundChildData> children, float mass)
        {
            //Recenter the data based on volume sine the per shape masses aren't available.
            Position = CompoundShape.ComputeCenter(children);
            Initialize(new CompoundCollidable(children, Position), mass);
        }

        ///<summary>
        /// Constructs a dynamic compound object from dynamic entries.
        ///</summary>
        ///<param name="children">Dynamic entries to construct the compound from.</param>
        public CompoundBody(IList<DynamicCompoundChildData> children)
        {
            var childrenData = new RawList<CompoundChildData>();
            var shapeEntries = new RawList<DynamicCompoundEntry>();
            float mass = 0;
            for (int i = 0; i < children.Count; i++)
            {
                childrenData.Add(children[i].ChildData);
                shapeEntries.Add(new DynamicCompoundEntry(children[i].ChildData.Entry, children[i].Mass));
                mass += children[i].Mass;
            }

            //Recenter the data based on mass weights.

            Vector3 center;
            Matrix3X3 inertia = CompoundShape.ComputeVolumeDistribution(shapeEntries, out center);
            Initialize(new CompoundCollidable(childrenData, center), mass, inertia);
            Position = center;
        }


    }


}