using System;
using BEPUphysics.Collidables.Events;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;
using BEPUphysics.CollisionShapes;
using BEPUphysics.Materials;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.ResourceManagement;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionTests.CollisionAlgorithms;

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
                return (TerrainShape)shape;
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


        internal float thickness;
        /// <summary>
        /// Gets or sets the thickness of the terrain.  The thickness 
        /// </summary>
        public float Thickness
        {
            get
            {
                return thickness;
            }
            set
            {
                if (value < 0)
                    throw new Exception("Cannot use a negative thickness value.");

                //Modify the bounding box to include the new thickness.
                Vector3 down = Vector3.Normalize(worldTransform.LinearTransform.Down);
                Vector3 thicknessOffset = down * (value - thickness);
                //Use the down direction rather than the thicknessOffset to determine which
                //component of the bounding box to subtract, since the down direction contains all
                //previous extra thickness.
                if (down.X < 0)
                    boundingBox.Min.X += thicknessOffset.X;
                else
                    boundingBox.Max.X += thicknessOffset.X;
                if (down.Y < 0)
                    boundingBox.Min.Y += thicknessOffset.Y;
                else
                    boundingBox.Max.Y += thicknessOffset.Y;
                if (down.Z < 0)
                    boundingBox.Min.Z += thicknessOffset.Z;
                else
                    boundingBox.Max.Z += thicknessOffset.Z;

                thickness = value;
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
        public override void UpdateBoundingBox()
        {
            Shape.GetBoundingBox(ref worldTransform, out boundingBox);
            //Include the thickness of the terrain.
            Vector3 thicknessOffset = Vector3.Normalize(worldTransform.LinearTransform.Down) * thickness;
            if (thicknessOffset.X < 0)
                boundingBox.Min.X += thicknessOffset.X;
            else
                boundingBox.Max.X += thicknessOffset.X;
            if (thicknessOffset.Y < 0)
                boundingBox.Min.Y += thicknessOffset.Y;
            else
                boundingBox.Max.Y += thicknessOffset.Y;
            if (thicknessOffset.Z < 0)
                boundingBox.Min.Z += thicknessOffset.Z;
            else
                boundingBox.Max.Z += thicknessOffset.Z;
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

        /// <summary>
        /// Casts a convex shape against the collidable.
        /// </summary>
        /// <param name="castShape">Shape to cast.</param>
        /// <param name="startingTransform">Initial transform of the shape.</param>
        /// <param name="sweep">Sweep to apply to the shape.</param>
        /// <param name="hit">Hit data, if any.</param>
        /// <returns>Whether or not the cast hit anything.</returns>
        public override bool ConvexCast(CollisionShapes.ConvexShapes.ConvexShape castShape, ref RigidTransform startingTransform, ref Vector3 sweep, out RayHit hit)
        {
            hit = new RayHit();
            BoundingBox boundingBox;
            castShape.GetSweptLocalBoundingBox(ref startingTransform, ref worldTransform, ref sweep, out boundingBox);
            var tri = Resources.GetTriangle();
            var hitElements = Resources.GetTriangleIndicesList();
            if (Shape.GetOverlaps(boundingBox, hitElements))
            {
                hit.T = float.MaxValue;
                for (int i = 0; i < hitElements.count; i++)
                {
                    Shape.GetTriangle(ref hitElements.Elements[i], ref worldTransform, out tri.vA, out tri.vB, out tri.vC);
                    Vector3 center;
                    Vector3.Add(ref tri.vA, ref tri.vB, out center);
                    Vector3.Add(ref center, ref tri.vC, out center);
                    Vector3.Multiply(ref center, 1f / 3f, out center);
                    Vector3.Subtract(ref tri.vA, ref center, out tri.vA);
                    Vector3.Subtract(ref tri.vB, ref center, out tri.vB);
                    Vector3.Subtract(ref tri.vC, ref center, out tri.vC);
                    tri.maximumRadius = tri.vA.LengthSquared();
                    float radius = tri.vB.LengthSquared();
                    if (tri.maximumRadius < radius)
                        tri.maximumRadius = radius;
                    radius = tri.vC.LengthSquared();
                    if (tri.maximumRadius < radius)
                        tri.maximumRadius = radius;
                    tri.maximumRadius = (float)Math.Sqrt(tri.maximumRadius);
                    tri.collisionMargin = 0;
                    var triangleTransform = new RigidTransform();
                    triangleTransform.Orientation = Quaternion.Identity;
                    triangleTransform.Position = center;
                    RayHit tempHit;
                    if (MPRToolbox.Sweep(castShape, tri, ref sweep, ref Toolbox.ZeroVector, ref startingTransform, ref triangleTransform, out tempHit) && tempHit.T < hit.T)
                    {
                        hit = tempHit;
                    }
                }
                tri.maximumRadius = 0;
                Resources.GiveBack(tri);
                Resources.GiveBack(hitElements);
                return hit.T != float.MaxValue;
            }
            Resources.GiveBack(tri);
            Resources.GiveBack(hitElements);
            return false;
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
