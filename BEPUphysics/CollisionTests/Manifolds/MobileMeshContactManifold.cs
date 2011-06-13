using System;
using BEPUphysics.Collidables;
using BEPUphysics.Collidables.MobileCollidables;
using Microsoft.Xna.Framework;
using BEPUphysics.DataStructures;
using BEPUphysics.MathExtensions;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionShapes;
using BEPUphysics.ResourceManagement;
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using System.Diagnostics;

namespace BEPUphysics.CollisionTests.Manifolds
{
    ///<summary>
    /// Manages persistent contacts between a convex and an instanced mesh.
    ///</summary>
    public abstract class MobileMeshContactManifold : TriangleMeshConvexContactManifold
    {
        protected MobileMeshCollidable mesh;

        internal RawList<int> overlappedTriangles = new RawList<int>(4);

        ///<summary>
        /// Gets the mesh of the pair.
        ///</summary>
        public MobileMeshCollidable Mesh
        {
            get
            {
                return mesh;
            }
        }

        protected override RigidTransform MeshTransform
        {
            get { return mesh.worldTransform; }
        }

        protected internal override int FindOverlappingTriangles(float dt)
        {
            BoundingBox boundingBox;
            AffineTransform transform = new AffineTransform(mesh.worldTransform.Orientation, mesh.worldTransform.Position);
            convex.Shape.GetLocalBoundingBox(ref convex.worldTransform, ref transform, out boundingBox);
            Vector3 transformedVelocity;
            //Compute the relative velocity with respect to the mesh.  The mesh's bounding tree is NOT expanded with velocity,
            //so whatever motion there is between the two objects needs to be included in the convex's bounding box.
            if (convex.entity != null)
                Vector3.Subtract(ref convex.entity.linearVelocity, ref mesh.entity.linearVelocity, out transformedVelocity);
            else
                Vector3.Negate(ref mesh.entity.linearVelocity, out transformedVelocity);
            //The linear transform is known to be orientation only, so using the transpose is allowed.
            Matrix3X3.TransformTranspose(ref transformedVelocity, ref transform.LinearTransform, out transformedVelocity);
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

            mesh.Shape.TriangleMesh.Tree.GetOverlaps(boundingBox, overlappedTriangles);
            return overlappedTriangles.count;
        }

        protected override bool ConfigureTriangle(int i, out TriangleIndices indices)
        {
            MeshBoundingBoxTreeData data = mesh.Shape.TriangleMesh.Data;
            int triangleIndex = overlappedTriangles.Elements[i];
            data.GetTriangle(triangleIndex, out localTriangleShape.vA, out localTriangleShape.vB, out localTriangleShape.vC);
            AffineTransform transform;
            AffineTransform.CreateFromRigidTransform(ref mesh.worldTransform, out transform);
            AffineTransform.Transform(ref localTriangleShape.vA, ref transform, out localTriangleShape.vA);
            AffineTransform.Transform(ref localTriangleShape.vB, ref transform, out localTriangleShape.vB);
            AffineTransform.Transform(ref localTriangleShape.vC, ref transform, out localTriangleShape.vC);
            //In instanced meshes, the bounding box we found in local space could collect more triangles than strictly necessary.
            //By doing a second pass, we should be able to prune out quite a few of them.
            BoundingBox triangleAABB;
            Toolbox.GetTriangleBoundingBox(ref localTriangleShape.vA, ref localTriangleShape.vB, ref localTriangleShape.vC, out triangleAABB);
            bool toReturn;
            triangleAABB.Intersects(ref convex.boundingBox, out toReturn);
            if (!toReturn)
            {
                indices = new TriangleIndices();
                return false;
            }

            TriangleSidedness sidedness;
            switch (mesh.Shape.solidity)
            {
                case MobileMeshSolidity.Clockwise:
                    sidedness = TriangleSidedness.Clockwise;
                    break;
                case MobileMeshSolidity.Counterclockwise:
                    sidedness = TriangleSidedness.Counterclockwise;
                    break;
                case MobileMeshSolidity.DoubleSided:
                    sidedness = TriangleSidedness.DoubleSided;
                    break;
                default:
                    sidedness = mesh.Shape.solidSidedness;
                    break;
            }
            localTriangleShape.sidedness = sidedness;
            localTriangleShape.collisionMargin = 0;
            indices = new TriangleIndices()
            {
                A = data.indices[triangleIndex],
                B = data.indices[triangleIndex + 1],
                C = data.indices[triangleIndex + 2]
            };
            return true;

        }

        protected internal override void CleanUpOverlappingTriangles()
        {
            overlappedTriangles.Clear();
        }

        protected override bool UseImprovedBoundaryHandling
        {
            get { return mesh.improveBoundaryBehavior; }
        }

        /// <summary>
        /// If an object is detected to be within the mobile mesh with a penetration depth greater than this limit,
        /// it will try a different direction to verify that it wasn't a false positive.
        /// </summary>
        public static float DepthDoubleCheckLimit = 1;
        static int numberOfContainmentChecks = 3;
        /// <summary>
        /// This is how many tests are required before an object is accepted as actually inside the solid portion of a mesh.
        /// The reason for the additional tests is robustness; one test may fail, but two is extremely unlikely, and three more so.
        /// More tests may also introduce false negatives, though they resolve quickly and are generally not as much of a problem as
        /// false positives.
        /// Valid values are 1, 2, or 3.  Defaults to 3.
        /// </summary>
        public static int NumberOfContainmentChecks
        {
            get
            {
                return numberOfContainmentChecks;
            }
            set
            {
                if (value == 1 || value == 2 || value == 3)
                    numberOfContainmentChecks = value;
                else
                    throw new Exception("May only use 1, 2, or 3 containment checks.");
            }
        }
        Vector3 lastValidConvexPosition;
        protected override void ProcessCandidates(RawValueList<ContactData> candidates)
        {
            if (candidates.count == 0 && Mesh.Shape.solidity == MobileMeshSolidity.Solid)
            {

                //If there's no new contacts on the mesh and it's supposed to be a solid,
                //then we must check the convex for containment within the shell.
                //We already know that it's not on the shell, meaning that the shape is either
                //far enough away outside the shell that there's no contact (and we're done), 
                //or it's far enough inside the shell that the triangles cannot create contacts.

                //To find out which it is, raycast against the shell.

                Matrix3X3 orientation;
                Matrix3X3.CreateFromQuaternion(ref mesh.worldTransform.Orientation, out orientation);

                Ray ray;
                Vector3.Subtract(ref convex.worldTransform.Position, ref mesh.worldTransform.Position, out ray.Position);
                Matrix3X3.TransformTranspose(ref ray.Position, ref orientation, out ray.Position);

                //Cast from the current position back to the previous position.
                Vector3.Subtract(ref lastValidConvexPosition, ref ray.Position, out ray.Direction);
                float rayDirectionLength = ray.Direction.LengthSquared();
                if (rayDirectionLength < Toolbox.Epsilon)
                {
                    //The object may not have moved enough to normalize properly.  If so, choose something arbitrary.
                    //Try the direction from the center of the object to the convex's position.
                    ray.Direction = ray.Position;
                    rayDirectionLength = ray.Direction.LengthSquared();
                    if (rayDirectionLength < Toolbox.Epsilon)
                    {
                        //This is unlikely; just pick something completely arbitrary then.
                        ray.Direction = Vector3.Up;
                        rayDirectionLength = 1;
                    }
                }
                Vector3.Divide(ref ray.Direction, (float)Math.Sqrt(rayDirectionLength), out ray.Direction);

                RayHit hit;
                if (mesh.Shape.IsRayOriginInMesh(ref ray, out hit))
                {
                    #region Double Checking
                    if (numberOfContainmentChecks > 1 && hit.T > DepthDoubleCheckLimit)
                    {
                        //It's an extremely deep penetration... Suspicious.
                        //Send the ray in the other direction to corroborate the result.
                        //A false positive can sometimes occur
                        Ray alternateRay;
                        Vector3.Negate(ref ray.Direction, out alternateRay.Direction);
                        alternateRay.Position = ray.Position;
                        RayHit alternateHit;
                        if (!mesh.Shape.IsRayOriginInMesh(ref alternateRay, out alternateHit))
                        {
                            //The double check didn't hit anything.  It's very unlikely that the object is actually inside the shape.
                            return;
                        }
                        else if (numberOfContainmentChecks > 2)
                        {
                            //It found a hit in the other direction.  Triple checking is enabled though, so let's try a third direction!
                            //This one will be perpendicular to the current ray direction.
                            Vector3.Cross(ref ray.Direction, ref Toolbox.UpVector, out alternateRay.Direction);
                            float lengthSquared = alternateRay.Direction.LengthSquared();
                            if (lengthSquared < Toolbox.Epsilon)
                            {
                                //The ray direction was already pointing nearly up.  Pick a different direction.
                                Vector3.Cross(ref ray.Direction, ref Toolbox.RightVector, out alternateRay.Direction);
                                lengthSquared = alternateRay.Direction.LengthSquared();
                            }
                            Vector3.Divide(ref alternateRay.Direction, (float)Math.Sqrt(lengthSquared), out alternateRay.Direction);
                            alternateRay.Position = ray.Position;
                            if (!mesh.Shape.IsRayOriginInMesh(ref alternateRay, out alternateHit))
                            {
                                //No hit was found on the third test!  The fact that we had two false positives is extremely improbable,
                                //but it's better to assume that it is not intersecting.
                                return;
                            }
                        }

                    }
                    #endregion
                    ContactData newContact = new ContactData();
                    newContact.Id = 2; //Give it a special id so that we know that it came from the inside.
                    Matrix3X3.Transform(ref ray.Position, ref orientation, out newContact.Position);
                    Vector3.Add(ref newContact.Position, ref mesh.worldTransform.Position, out newContact.Position);

                    newContact.Normal = hit.Normal;
                    newContact.Normal.Normalize();

                    float factor;
                    Vector3.Dot(ref ray.Direction, ref newContact.Normal, out factor);
                    newContact.PenetrationDepth = -factor * hit.T + convex.Shape.minimumRadius;

                    Matrix3X3.Transform(ref newContact.Normal, ref orientation, out newContact.Normal);

                    //Do not yet create a new contact.  Check to see if an 'inner contact' with id == 2 already exists.
                    bool addContact = true;
                    for (int i = 0; i < contacts.count; i++)
                    {
                        if (contacts.Elements[i].Id == 2)
                        {
                            contacts.Elements[i].Position = newContact.Position;
                            contacts.Elements[i].Normal = newContact.Normal;
                            contacts.Elements[i].PenetrationDepth = newContact.PenetrationDepth;
                            supplementData.Elements[i].BasePenetrationDepth = newContact.PenetrationDepth;
                            supplementData.Elements[i].LocalOffsetA = new Vector3();
                            supplementData.Elements[i].LocalOffsetB = ray.Position; //convex local position in mesh.
                            addContact = false;
                            break;
                        }
                    }
                    if (addContact && contacts.count == 0)
                        Add(ref newContact);

                }
                else
                {
                    //We're not touching the mesh.
                    lastValidConvexPosition = ray.Position;

                }
            }
        }

        ///<summary>
        /// Cleans up the manifold.
        ///</summary>
        public override void CleanUp()
        {
            mesh = null;
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
            mesh = newCollidableB as MobileMeshCollidable;


            if (convex == null || mesh == null)
            {
                convex = newCollidableB as ConvexCollidable;
                mesh = newCollidableA as MobileMeshCollidable;
                if (convex == null || mesh == null)
                    throw new Exception("Inappropriate types used to initialize contact manifold.");
            }

        }

        UnsafeResourcePool<TriangleConvexPairTester> testerPool = new UnsafeResourcePool<TriangleConvexPairTester>();
        protected override void GiveBackTester(CollisionAlgorithms.TrianglePairTester tester)
        {
            testerPool.GiveBack((TriangleConvexPairTester)tester);
        }

        protected override CollisionAlgorithms.TrianglePairTester GetTester()
        {
            return testerPool.Take();
        }

    }
}
