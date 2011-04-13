using System;
using BEPUphysics.Collidables.Events;
using BEPUphysics.CollisionShapes;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;
using BEPUphysics.DataStructures;
using BEPUphysics.Materials;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.CollisionShapes.ConvexShapes;

namespace BEPUphysics.Collidables
{
    ///<summary>
    /// Unmoving, collidable triangle mesh.
    ///</summary>
    ///<remarks>
    /// The acceleration structure for the mesh is created individually for each
    /// StaticMesh; if you want to create many meshes of the same model, consider using the
    /// InstancedMesh.
    /// </remarks>
    public class StaticMesh : Collidable, ISpaceObject, IMaterialOwner
    {

        TriangleMesh mesh;
        ///<summary>
        /// Gets the TriangleMesh acceleration structure used by the StaticMesh.
        ///</summary>
        public TriangleMesh Mesh
        {
            get
            {
                return mesh;
            }
        }

        ///<summary>
        /// Gets or sets the world transform of the mesh.
        ///</summary>
        public AffineTransform WorldTransform
        {
            get
            {
                return (mesh.Data as TransformableMeshData).worldTransform;
            }
            set
            {
                (mesh.Data as TransformableMeshData).WorldTransform = value;
                mesh.Tree.Refit();
                boundingBox = mesh.Tree.BoundingBox;
            }
        }

        ///<summary>
        /// Constructs a new static mesh.
        ///</summary>
        ///<param name="vertices">Vertex positions of the mesh.</param>
        ///<param name="indices">Index list of the mesh.</param>
        public StaticMesh(Vector3[] vertices, int[] indices)
        {
            base.Shape = new StaticMeshShape(vertices, indices);
            mesh = new TriangleMesh(Shape.TriangleMeshData);
            boundingBox = mesh.Tree.BoundingBox;
            collisionRules.group = CollisionRules.DefaultKinematicCollisionGroup;
            events = new ContactEventManager<StaticMesh>(this);

            material = new Material();
            materialChangedDelegate = OnMaterialChanged;
            material.MaterialChanged += materialChangedDelegate;
        }

        ///<summary>
        /// Constructs a new static mesh.
        ///</summary>
        ///<param name="vertices">Vertex positions of the mesh.</param>
        ///<param name="indices">Index list of the mesh.</param>
        /// <param name="worldTransform">Transform to use to create the mesh initially.</param>
        public StaticMesh(Vector3[] vertices, int[] indices, AffineTransform worldTransform)
        {
            base.Shape = new StaticMeshShape(vertices, indices, worldTransform);

            collisionRules.group = CollisionRules.DefaultKinematicCollisionGroup;
            events = new ContactEventManager<StaticMesh>(this);



            material = new Material();
            materialChangedDelegate = OnMaterialChanged;
            material.MaterialChanged += materialChangedDelegate;
        }

        ///<summary>
        /// Gets the shape used by the mesh.
        ///</summary>
        public new StaticMeshShape Shape
        {
            get
            {
                return base.Shape as StaticMeshShape;
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


        protected internal ContactEventManager<StaticMesh> events;

        ///<summary>
        /// Gets the event manager used by the mesh.
        ///</summary>
        public ContactEventManager<StaticMesh> Events
        {
            get
            {
                return events;
            }
        }
        protected internal override IContactEventTriggerer EventTriggerer
        {
            get { return events; }
        }

        internal Material material;
        //NOT thread safe due to material change pair update.
        ///<summary>
        /// Gets or sets the material used by the mesh.
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

        protected override void OnShapeChanged(CollisionShape collisionShape)
        {
            mesh = new TriangleMesh(Shape.TriangleMeshData);
            boundingBox = mesh.Tree.BoundingBox;
        }

        Action<Material> materialChangedDelegate;
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
            return mesh.RayCast(ray, maximumLength, sidedness, out rayHit);
        }

        ///<summary>
        /// Tests a ray against the mesh.
        ///</summary>
        ///<param name="ray">Ray to test.</param>
        ///<param name="maximumLength">Maximum length to test in units of the ray direction's length.</param>
        ///<param name="sidedness">Sidedness to use when raycasting.  Doesn't have to be the same as the mesh's own sidedness.</param>
        ///<param name="rayHit">Data about the ray's intersection with the mesh, if any.</param>
        ///<returns>Whether or not the ray hit the mesh.</returns>
        public bool RayCast(Ray ray, float maximumLength, TriangleSidedness sidedness, out RayHit rayHit)
        {
            return mesh.RayCast(ray, maximumLength, sidedness, out rayHit);
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
        /// Gets the space that owns the mesh.
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
