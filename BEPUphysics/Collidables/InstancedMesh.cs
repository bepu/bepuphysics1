using System;
using BEPUphysics.Collidables.Events;
using BEPUphysics.CollisionShapes;
using Microsoft.Xna.Framework;
using BEPUphysics.Materials;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.MathExtensions;

namespace BEPUphysics.Collidables
{
    ///<summary>
    /// Collidable mesh which can be created from a reusable InstancedMeshShape.
    /// Very little data is needed for each individual InstancedMesh object, allowing
    /// a complicated mesh to be repeated many times.  Since the hierarchy used to accelerate
    /// collisions is purely local, it may be marginally slower than an individual StaticMesh.
    ///</summary>
    public class InstancedMesh : Collidable, ISpaceObject, IMaterialOwner
    {

        internal AffineTransform worldTransform;
        ///<summary>
        /// Gets or sets the world transform of the mesh.
        ///</summary>
        public AffineTransform WorldTransform
        {
            get
            {
                return worldTransform;
            }
            set
            {
                worldTransform = value;
                Shape.ComputeBoundingBox(ref value, out boundingBox);
            }
        }


        protected override void OnShapeChanged(CollisionShape collisionShape)
        {
            Shape.ComputeBoundingBox(ref worldTransform, out boundingBox);
        }

        ///<summary>
        /// Constructs a new InstancedMesh.
        ///</summary>
        ///<param name="meshShape">Shape to use for the instance.</param>
        public InstancedMesh(InstancedMeshShape meshShape)
            : this(meshShape, AffineTransform.Identity)
        {
        }

        ///<summary>
        /// Constructs a new InstancedMesh.
        ///</summary>
        ///<param name="meshShape">Shape to use for the instance.</param>
        ///<param name="worldTransform">Transform to use for the instance.</param>
        public InstancedMesh(InstancedMeshShape meshShape, AffineTransform worldTransform)
        {
            this.worldTransform = worldTransform;
            base.Shape = meshShape;
            collisionRules.group = CollisionRules.DefaultKinematicCollisionGroup;
            events = new ContactEventManager<InstancedMesh>(this);

            material = new Material();
            materialChangedDelegate = OnMaterialChanged;
            material.MaterialChanged += materialChangedDelegate;

        }

        ///<summary>
        /// Gets the shape used by the instanced mesh.
        ///</summary>
        public new InstancedMeshShape Shape
        {
            get
            {
                return base.Shape as InstancedMeshShape;
            }
        }

        internal TriangleSidedness sidedness = TriangleSidedness.DoubleSided;
        ///<summary>
        /// Gets or sets the sidedness of the mesh.  This can be used to ignore collisions and rays coming from a direction relative to the winding of the triangle.
        ///</summary>
        public TriangleSidedness Sidedness
        {
            get
            {
                return sidedness;
            }
            set
            {
                sidedness = value;
            }
        }

        internal bool improveBoundaryBehavior = true;
        /// <summary>
        /// Gets or sets whether or not the collision system should attempt to improve contact behavior at the boundaries between triangles.
        /// This has a slight performance cost, but prevents objects sliding across a triangle boundary from 'bumping,' and otherwise improves
        /// the robustness of contacts at edges and vertices.
        /// </summary>
        public bool ImproveBoundaryBehavior
        {
            get
            {
                return improveBoundaryBehavior;
            }
            set
            {
                improveBoundaryBehavior = value;
            }
        }


        protected internal ContactEventManager<InstancedMesh> events;
        ///<summary>
        /// Gets the event manager of the mesh.
        ///</summary>
        public ContactEventManager<InstancedMesh> Events
        {
            get
            {
                return events;
            }
        }

        internal Material material;
        //NOT thread safe due to material change pair update.
        ///<summary>
        /// Gets or sets the material of the mesh.
        ///</summary>
        public Material Material
        {
            get
            {
                return material;
            }
            set
            {
                if (material != null)
                    material.MaterialChanged -= materialChangedDelegate;
                material = value;
                if (material != null)
                    material.MaterialChanged += materialChangedDelegate;
                OnMaterialChanged(material);
            }
        }

        readonly Action<Material> materialChangedDelegate;
        void OnMaterialChanged(Material newMaterial)
        {
            for (int i = 0; i < pairs.Count; i++)
            {
                pairs[i].UpdateMaterialProperties();
            }
        }

        protected internal override bool IsActive
        {
            get { return false; }
        }


        /// <summary>
        /// Tests a ray against the entry.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length, in units of the ray's direction's length, to test.</param>
        /// <param name="rayHit">Hit location of the ray on the entry, if any.</param>
        /// <returns>Whether or not the ray hit the entry.</returns>
        public override bool RayCast(Ray ray, float maximumLength, out RayHit rayHit)
        {
            return RayCast(ray, maximumLength, sidedness, out rayHit);
        }

        ///<summary>
        /// Tests a ray against the instance.
        ///</summary>
        ///<param name="ray">Ray to test.</param>
        ///<param name="maximumLength">Maximum length of the ray to test; in units of the ray's direction's length.</param>
        ///<param name="sidedness">Sidedness to use during the ray cast.  This does not have to be the same as the mesh's sidedness.</param>
        ///<param name="rayHit">The hit location of the ray on the mesh, if any.</param>
        ///<returns>Whether or not the ray hit the mesh.</returns>
        public bool RayCast(Ray ray, float maximumLength, TriangleSidedness sidedness, out RayHit rayHit)
        {
            //Put the ray into local space.
            Ray localRay;
            AffineTransform inverse;
            AffineTransform.Invert(ref worldTransform, out inverse);
            Matrix3X3.Transform(ref ray.Direction, ref inverse.LinearTransform, out localRay.Direction);
            AffineTransform.Transform(ref ray.Position, ref inverse, out localRay.Position);

            if (Shape.TriangleMesh.RayCast(localRay, maximumLength, sidedness, out rayHit))
            {
                //Transform the hit into world space.
                Vector3.Multiply(ref ray.Direction, rayHit.T, out rayHit.Location);
                Vector3.Add(ref rayHit.Location, ref ray.Position, out rayHit.Location);
                Matrix3X3.TransformTranspose(ref rayHit.Normal, ref inverse.LinearTransform, out rayHit.Normal);
                return true;
            }
            rayHit = new RayHit();
            return false;
        }

        ISpace space;
        ISpace ISpaceObject.Space
        {
            get
            {
                return space;
            }
            set
            {
                space = value;
            }
        }
        ///<summary>
        /// Gets the Space to which the instance belongs.
        ///</summary>
        public ISpace Space
        {
            get
            {
                return space;
            }
        }

        void ISpaceObject.OnAdditionToSpace(ISpace newSpace)
        {
        }

        void ISpaceObject.OnRemovalFromSpace(ISpace oldSpace)
        {
        }
    }
}
