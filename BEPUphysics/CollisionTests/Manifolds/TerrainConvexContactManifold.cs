using System;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using Microsoft.Xna.Framework;
using BEPUphysics.DataStructures;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.MathExtensions;

namespace BEPUphysics.CollisionTests.Manifolds
{
    ///<summary>
    /// Manages persistent contacts between a Terrain and a convex.
    ///</summary>
    public class TerrainConvexContactManifold : TriangleMeshConvexContactManifold
    {
        protected Terrain terrain;

        internal RawList<TriangleIndices> overlappedTriangles = new RawList<TriangleIndices>(4);

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
            Vector3 transformedVelocity;
            Matrix3X3 inverse;
            Matrix3X3.Invert(ref terrain.worldTransform.LinearTransform, out inverse);

            Matrix3X3.Transform(ref convex.entity.linearVelocity, ref inverse, out transformedVelocity);
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


            terrain.Shape.GetOverlaps(boundingBox, overlappedTriangles);
            return overlappedTriangles.count;
        }

        protected override void ConfigureTriangle(int i, out TriangleIndices indices)
        {
            indices = overlappedTriangles.Elements[i];
            terrain.Shape.GetTriangle(ref indices, ref terrain.worldTransform, out localTriangleShape.vA, out localTriangleShape.vB, out localTriangleShape.vC);
            localTriangleShape.collisionMargin = 0;
            
            //Calibrate the sidedness of the triangle such that it's always facing up.
            //TODO: There's quite a bit of redundancy in here with other systems.

            Vector3 AB, AC, normal;
            Vector3.Subtract(ref localTriangleShape.vB, ref localTriangleShape.vA, out AB);
            Vector3.Subtract(ref localTriangleShape.vC, ref localTriangleShape.vA, out AC);
            Vector3.Cross(ref AB, ref AC, out normal);

            Vector3 terrainUp = new Vector3(terrain.worldTransform.LinearTransform.M21, terrain.worldTransform.LinearTransform.M22, terrain.worldTransform.LinearTransform.M23);
            float dot;
            Vector3.Dot(ref terrainUp, ref normal, out dot);
            if (dot > 0)
            {
                localTriangleShape.sidedness = TriangleSidedness.Clockwise;
            }
            else
            {
                localTriangleShape.sidedness = TriangleSidedness.Counterclockwise;
            }
        }

        protected internal override void CleanUpOverlappingTriangles()
        {
            overlappedTriangles.Clear();
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
                    throw new Exception("Inappropriate types used to initialize contact manifold.");
            }

        }


    }
}
