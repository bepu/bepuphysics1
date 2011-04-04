using System.Collections.Generic;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Collidables.Events;
using BEPUphysics.CollisionShapes;
using BEPUphysics.MathExtensions;
using Microsoft.Xna.Framework;
using BEPUphysics.ResourceManagement;
using BEPUphysics.DataStructures;
using BEPUphysics.Materials;
using System.Collections.ObjectModel;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.CollisionShapes.ConvexShapes;

namespace BEPUphysics.Collidables.MobileCollidables
{
    ///<summary>
    /// Collidable used by compound shapes.
    ///</summary>
    public class MobileMeshCollidable : EntityCollidable
    {
        ///<summary>
        /// Gets the shape of the collidable.
        ///</summary>
        public new MobileMeshShape Shape
        {
            get
            {
                return base.Shape as MobileMeshShape;
            }
        }

        /// <summary>
        /// Constructs a new mobile mesh collidable.
        /// </summary>
        /// <param name="shape">Shape to use in the collidable.</param>
        public MobileMeshCollidable(MobileMeshShape shape)
            : base(shape)
        {
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

        protected internal override void UpdateBoundingBoxInternal(float dt)
        {
            Shape.GetBoundingBox(ref worldTransform, out boundingBox);

            //This DOES NOT EXPAND the local hierarchy.
            //The bounding boxes of queries against the local hierarchy
            //should be expanded using the relative velocity.
            ExpandBoundingBox(ref boundingBox, dt);
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
            //Put the ray into local space.
            Ray localRay;
            Matrix3X3 orientation;
            Matrix3X3.CreateFromQuaternion(ref worldTransform.Orientation, out orientation);
            Matrix3X3.TransformTranspose(ref ray.Direction, ref orientation, out localRay.Direction);
            Vector3.Subtract(ref ray.Position, ref worldTransform.Position, out localRay.Position);
            Matrix3X3.TransformTranspose(ref localRay.Position, ref orientation, out localRay.Position);


            if (Shape.solidity == MobileMeshSolidity.Solid)
            {
                var hits = Resources.GetRayHitList();
                //Find all hits.  Use the count to determine the ray started inside or outside.
                //If it starts inside and we're in 'solid' mode, then return the ray start.
                //The raycast must be of infinite length at first.  This allows it to determine
                //if it is inside or outside.
                if (Shape.TriangleMesh.RayCast(localRay, float.MaxValue, hits))
                {
                    if (hits.count % 2 == 0 || Shape.solidity == MobileMeshSolidity.Solid)
                    {
                        //Even number of hits; the ray started on the outside.
                        //Find the earliest hit.

                        rayHit = hits.Elements[0];
                        for (int i = 1; i < hits.count; i++)
                        {
                            if (hits.Elements[i].T < rayHit.T)
                                rayHit = hits.Elements[i];
                        }

                        if (rayHit.T < maximumLength)
                        {
                            //Transform the hit into world space.
                            Vector3.Multiply(ref ray.Direction, rayHit.T, out rayHit.Location);
                            Vector3.Add(ref rayHit.Location, ref ray.Position, out rayHit.Location);
                            Matrix3X3.Transform(ref rayHit.Normal, ref orientation, out rayHit.Normal);
                        }
                        else
                        {
                            //The hit was too far away.
                            Resources.GiveBack(hits);
                            return false;
                        }

                    }
                    else
                    {
                        //Odd number of hits; the ray started on the inside.
                        rayHit = new RayHit() { Location = ray.Position, Normal = Vector3.Zero, T = 0 };
                    }
                    Resources.GiveBack(hits);
                    return true;
                }
            }
            else
            {
                //Just do a normal raycast since the object isn't solid.
                TriangleSidedness sidedness;
                switch (Shape.solidity)
                {
                    case MobileMeshSolidity.Clockwise:
                        sidedness = TriangleSidedness.Clockwise;
                        break;
                    case MobileMeshSolidity.Counterclockwise:
                        sidedness = TriangleSidedness.Counterclockwise;
                        break;
                    case MobileMeshSolidity.DoubleSided:
                    default:
                        sidedness = TriangleSidedness.DoubleSided;
                        break;
                }
                if (Shape.TriangleMesh.RayCast(localRay, maximumLength, sidedness, out rayHit))
                {
                    //Transform the hit into world space.
                    Vector3.Multiply(ref ray.Direction, rayHit.T, out rayHit.Location);
                    Vector3.Add(ref rayHit.Location, ref ray.Position, out rayHit.Location);
                    Matrix3X3.Transform(ref rayHit.Normal, ref orientation, out rayHit.Normal);
                    return true;
                }
            }
            rayHit = new RayHit();
            return false;
        }

        ///<summary>
        /// Tests a ray against the surface of the mesh.  This does not take into account solidity.
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
            Matrix3X3 orientation;
            Matrix3X3.CreateFromQuaternion(ref worldTransform.Orientation, out orientation);
            Matrix3X3.TransformTranspose(ref ray.Direction, ref orientation, out localRay.Direction);
            Vector3.Subtract(ref ray.Position, ref worldTransform.Position, out localRay.Position);
            Matrix3X3.TransformTranspose(ref localRay.Position, ref orientation, out localRay.Position);

            if (Shape.TriangleMesh.RayCast(localRay, maximumLength, sidedness, out rayHit))
            {
                //Transform the hit into world space.
                Vector3.Multiply(ref ray.Direction, rayHit.T, out rayHit.Location);
                Vector3.Add(ref rayHit.Location, ref ray.Position, out rayHit.Location);
                Matrix3X3.Transform(ref rayHit.Normal, ref orientation, out rayHit.Normal);
                return true;
            }
            rayHit = new RayHit();
            return false;
        }

    }



}
