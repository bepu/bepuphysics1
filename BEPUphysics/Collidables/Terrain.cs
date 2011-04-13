using System;
using BEPUphysics.Collidables.Events;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionShapes;
using BEPUphysics.Materials;
using BEPUphysics.CollisionRuleManagement;

namespace BEPUphysics.Collidables
{
    ///<summary>
    /// Heightfield-based, unmovable collidable object.
    ///</summary>
    public class Terrain : Collidable, ISpaceObject, IMaterialOwner
    {
        ///<summary>
        /// Gets the shape of this collidable.
        ///</summary>
        public new TerrainShape Shape
        {
            get
            {
                return base.Shape as TerrainShape;
            }
            set
            {
                base.Shape = value;
            }
        }


        internal AffineTransform worldTransform;
        ///<summary>
        /// Gets or sets the affine transform of the terrain.
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

        protected internal ContactEventManager<Terrain> events;
        ///<summary>
        /// Gets the event manager used by the Terrain.
        ///</summary>
        public ContactEventManager<Terrain> Events
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
        /// Gets or sets the material of the terrain.
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

        ///<summary>
        /// Constructs a new Terrain.
        ///</summary>
        ///<param name="shape">Shape to use for the terrain.</param>
        ///<param name="worldTransform">Transform to use for the terrain.</param>
        public Terrain(TerrainShape shape, AffineTransform worldTransform)
        {
            this.worldTransform = worldTransform;
            Shape = shape;
            collisionRules.group = CollisionRules.DefaultKinematicCollisionGroup;

            material = new Material();
            materialChangedDelegate = OnMaterialChanged;
            material.MaterialChanged += materialChangedDelegate;

            events = new ContactEventManager<Terrain>(this);
        }


        ///<summary>
        /// Constructs a new Terrain.
        ///</summary>
        ///<param name="heights">Height data to use to create the TerrainShape.</param>
        ///<param name="worldTransform">Transform to use for the terrain.</param>
        public Terrain(float[,] heights, AffineTransform worldTransform)
            : this(new TerrainShape(heights), worldTransform)
        {
        }

        protected override void OnShapeChanged(CollisionShape collisionShape)
        {
            UpdateBoundingBox();
        }

        ///<summary>
        /// Updates the bounding box of the terrain.
        ///</summary>
        public void UpdateBoundingBox()
        {
            Shape.GetBoundingBox(ref worldTransform, out boundingBox);
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
            return Shape.RayCast(ref ray, maximumLength, ref worldTransform, out rayHit);
        }

        ///<summary>
        /// Gets the normal of a vertex at the given indices.
        ///</summary>
        ///<param name="i">First dimension index into the heightmap array.</param>
        ///<param name="j">Second dimension index into the heightmap array.</param>
        ///<param name="normal">Normal at the given indices.</param>
        public void GetNormal(int i, int j, out Vector3 normal)
        {
            Shape.GetNormal(i, j, ref worldTransform, out normal);
        }

        ///<summary>
        /// Gets the position of a vertex at the given indices.
        ///</summary>
        ///<param name="i">First dimension index into the heightmap array.</param>
        ///<param name="j">Second dimension index into the heightmap array.</param>
        ///<param name="position">Position at the given indices.</param>
        public void GetPosition(int i, int j, out Vector3 position)
        {
            Shape.GetPosition(i, j, ref worldTransform, out position);
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
        /// <summary>
        /// Gets the space that owns this terrain.
        /// </summary>
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
