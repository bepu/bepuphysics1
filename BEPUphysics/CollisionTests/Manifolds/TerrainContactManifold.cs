using System;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUutilities;
using BEPUutilities.DataStructures;

namespace BEPUphysics.CollisionTests.Manifolds
{
    ///<summary>
    /// Manages persistent contacts between a Terrain and a convex.
    ///</summary>
    public abstract class TerrainContactManifold : TriangleMeshConvexContactManifold
    {
        protected Terrain terrain;

        internal RawList<int> overlappedTriangles = new RawList<int>(4);

        ///<summary>
        /// Gets the terrain associated with this pair.
        ///</summary>
        public Terrain Terrain
        {
            get
            {
                return terrain;
            }
        }

        protected internal override int FindOverlappingTriangles(float dt)
        {
            BoundingBox boundingBox;
            convex.Shape.GetLocalBoundingBox(ref convex.worldTransform, ref terrain.worldTransform, out boundingBox);


            if (convex.entity != null)
            {
                Vector3 transformedVelocity;
                Matrix3x3 inverse;
                Matrix3x3.Invert(ref terrain.worldTransform.LinearTransform, out inverse);
                Matrix3x3.Transform(ref convex.entity.linearVelocity, ref inverse, out transformedVelocity);
                Vector3.Multiply(ref transformedVelocity, dt, out transformedVelocity);


                if (transformedVelocity.X > 0)
                    boundingBox.Max.X += transformedVelocity.X;
                else
                    boundingBox.Min.X += transformedVelocity.X;

                if (transformedVelocity.Y > 0)
                    boundingBox.Max.Y += transformedVelocity.Y;
                else
                    boundingBox.Min.Y += transformedVelocity.Y;

                if (transformedVelocity.Z > 0)
                    boundingBox.Max.Z += transformedVelocity.Z;
                else
                    boundingBox.Min.Z += transformedVelocity.Z;
            }


            terrain.Shape.GetOverlaps(boundingBox, ref overlappedTriangles);
            return overlappedTriangles.Count;
        }

        /// <summary>
        /// Precomputes the transform to bring triangles from their native local space to the local space of the convex.
        /// </summary>
        /// <param name="convexInverseWorldTransform">Inverse of the world transform of the convex shape.</param>
        /// <param name="fromMeshLocalToConvexLocal">Transform to apply to native local triangles to bring them into the local space of the convex.</param>
        protected override void PrecomputeTriangleTransform(ref AffineTransform convexInverseWorldTransform, out AffineTransform fromMeshLocalToConvexLocal)
        {
            AffineTransform.Multiply(ref terrain.worldTransform, ref convexInverseWorldTransform, out fromMeshLocalToConvexLocal);
        }

        protected override bool ConfigureLocalTriangle(int i, TriangleShape localTriangleShape, out TriangleIndices indices)
        {
            TerrainVertexIndices a, b, c;
            terrain.Shape.GetLocalIndices(overlappedTriangles[i], out a, out b, out c);
            int terrainWidth = terrain.Shape.Heights.GetLength(0);
            indices.A = a.ToSequentialIndex(terrainWidth);
            indices.B = b.ToSequentialIndex(terrainWidth);
            indices.C = c.ToSequentialIndex(terrainWidth);
            terrain.Shape.GetLocalPosition(a.ColumnIndex, a.RowIndex, out localTriangleShape.vA);
            terrain.Shape.GetLocalPosition(b.ColumnIndex, b.RowIndex, out localTriangleShape.vB);
            terrain.Shape.GetLocalPosition(c.ColumnIndex, c.RowIndex, out localTriangleShape.vC);
            localTriangleShape.collisionMargin = 0;

            localTriangleShape.sidedness = terrain.sidedness;
 
            //Unlike other 'instanced' geometries, terrains are almost always axis aligned in some way and/or have low triangle density relative to what they are colliding with.
            //Instead of performing additional tests, just assume that it's a fairly regular situation.
            return true;
        }

        protected internal override void CleanUpOverlappingTriangles()
        {
            overlappedTriangles.Clear();
        }


        protected override void ProcessCandidates(ref QuickList<ContactData> candidates)
        {
            //If the candidates list is empty, then let's see if the convex is in the 'thickness' of the terrain.
            if (candidates.Count == 0 & terrain.thickness > 0)
            {
                RayHit rayHit;
                Ray ray = new Ray { Position = convex.worldTransform.Position, Direction = terrain.worldTransform.LinearTransform.Up };
                ray.Direction.Normalize();
                //The raycast has to use doublesidedness, since we're casting from the bottom up.
                if (terrain.Shape.RayCast(ref ray, terrain.thickness, ref terrain.worldTransform, TriangleSidedness.DoubleSided, out rayHit))
                {
                    //Found a hit!
                    rayHit.Normal.Normalize();
                    float dot;
                    Vector3.Dot(ref ray.Direction, ref rayHit.Normal, out dot);

                    var newContact = new ContactData
                    {
                        Normal = rayHit.Normal,
                        Position = convex.worldTransform.Position,
                        Id = 2,
                        PenetrationDepth = -rayHit.T * dot + convex.Shape.MinimumRadius
                    };
                    newContact.Validate();
                    bool found = false;
                    for (int i = 0; i < contacts.Count; i++)
                    {
                        if (contacts.Elements[i].Id == 2)
                        {
                            //As set above, an id of 2 corresponds to a contact created from this raycast process.
                            contacts.Elements[i].Normal = newContact.Normal;
                            contacts.Elements[i].Position = newContact.Position;
                            contacts.Elements[i].PenetrationDepth = newContact.PenetrationDepth;
                            supplementData.Elements[i].BasePenetrationDepth = newContact.PenetrationDepth;
                            supplementData.Elements[i].LocalOffsetA = new Vector3();
                            supplementData.Elements[i].LocalOffsetB = ray.Position; //convex local position in mesh.
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                        candidates.Add(ref newContact);
                }
            }
        }

        protected override bool UseImprovedBoundaryHandling
        {
            get { return terrain.improveBoundaryBehavior; }
        }


        ///<summary>
        /// Cleans up the manifold.
        ///</summary>
        public override void CleanUp()
        {
            terrain = null;
            convex = null;
            base.CleanUp();
        }

        ///<summary>
        /// Initializes the manifold.
        ///</summary>
        ///<param name="newCollidableA">First collidable.</param>
        ///<param name="newCollidableB">Second collidable.</param>
        public override void Initialize(Collidable newCollidableA, Collidable newCollidableB)
        {
            convex = newCollidableA as ConvexCollidable;
            terrain = newCollidableB as Terrain;


            if (convex == null || terrain == null)
            {
                convex = newCollidableB as ConvexCollidable;
                terrain = newCollidableA as Terrain;
                if (convex == null || terrain == null)
                    throw new ArgumentException("Inappropriate types used to initialize contact manifold.");
            }

        }


    }
}
