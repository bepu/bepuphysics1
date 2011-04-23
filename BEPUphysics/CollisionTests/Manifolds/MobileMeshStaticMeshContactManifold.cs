using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.MathExtensions;

namespace BEPUphysics.CollisionTests.Manifolds
{
    public class MobileMeshStaticMeshContactManifold : MobileMeshMeshContactManifold
    {
        StaticMesh staticMesh;
        protected override void UpdatePairs()
        {
            //Go through the pair testers and update their vertices, world transform, etc. for the next run.
            //If we're colliding against a static mesh, the collidable needs a correct bounding box too.
            //Since this is a static mesh, just assume the mobile mesh is smaller.
            for (int i = 0; i < activePairTesters.count; i++)
            {
                var shape = activePairTesters.Elements[i].shape;
                mobileMesh.Shape.TriangleMesh.Data.GetTriangle(i * 3, out shape.vA, out shape.vB, out shape.vC);
                //If it's a mesh that has additional transforms, then apply them here to bring the shape into world space.
                Matrix3X3 orientation;
                Matrix3X3.CreateFromQuaternion(ref mobileMesh.worldTransform.Orientation, out orientation);
                Matrix3X3.Transform(ref shape.vA, ref orientation, out shape.vA);
                Matrix3X3.Transform(ref shape.vB, ref orientation, out shape.vB);
                Matrix3X3.Transform(ref shape.vC, ref orientation, out shape.vC);
                Vector3.Add(ref shape.vA, ref mobileMesh.worldTransform.Position, out shape.vA);
                Vector3.Add(ref shape.vB, ref mobileMesh.worldTransform.Position, out shape.vB);
                Vector3.Add(ref shape.vC, ref mobileMesh.worldTransform.Position, out shape.vC); 
                Vector3 center;
                Vector3.Add(ref shape.vA, ref shape.vB, out center);
                Vector3.Add(ref center, ref shape.vC, out center);
                Vector3.Divide(ref center, 3, out center);
                var triangleCollidable = activePairTesters.Elements[i].triangle;
                triangleCollidable.worldTransform.Position = center;
                //Recenter the vertex positions.
                Vector3.Subtract(ref shape.vA, ref center, out shape.vA);
                Vector3.Subtract(ref shape.vB, ref center, out shape.vB);
                Vector3.Subtract(ref shape.vC, ref center, out shape.vC);

                //For static meshes, would need to set the triangle collidable's bounding box.
                Vector3.Max(ref shape.vA, ref shape.vB, out triangleCollidable.boundingBox.Max);
                Vector3.Max(ref shape.vC, ref triangleCollidable.boundingBox.Max, out triangleCollidable.boundingBox.Max);

                Vector3.Min(ref shape.vA, ref shape.vB, out triangleCollidable.boundingBox.Min);
                Vector3.Min(ref shape.vC, ref triangleCollidable.boundingBox.Min, out triangleCollidable.boundingBox.Min);



            }
        }



        public override void Initialize(Collidables.Collidable newCollidableA, Collidables.Collidable newCollidableB)
        {
            staticMesh = newCollidableA as StaticMesh;
            mobileMesh = newCollidableB as MobileMeshCollidable;
            if (staticMesh == null || mobileMesh == null)
            {
                staticMesh = newCollidableB as StaticMesh;
                mobileMesh = newCollidableA as MobileMeshCollidable;
                if (staticMesh == null || mobileMesh == null)
                {
                    throw new Exception("Invalid types used to initialize contact manifold.");
                }
            }
        }
    }
}
