using System;
using System.Collections.Generic;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities.ResourceManagement;
using BEPUphysics.Settings;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUutilities;
using BEPUutilities.DataStructures;

namespace BEPUphysics.CollisionTests.Manifolds
{
    ///<summary>
    /// Manages persistent contact data between a triangle mesh and a convex.
    ///</summary>
    public abstract class TriangleMeshConvexContactManifold : ContactManifold
    {
        protected RawValueList<ContactSupplementData> supplementData = new RawValueList<ContactSupplementData>(4);
        private QuickDictionary<TriangleIndices, TrianglePairTester> activePairTesters;


        protected abstract TrianglePairTester GetTester();

        protected abstract void GiveBackTester(TrianglePairTester tester);

        private struct BoundarySets
        {
            public QuickSet<int> BlockedVertexRegions;
            public QuickSet<Edge> BlockedEdgeRegions;
            public QuickList<EdgeContact> EdgeContacts;
            public QuickList<VertexContact> VertexContacts;

            public BoundarySets(int sizePower)
            {
                BlockedVertexRegions = new QuickSet<int>(BufferPools<int>.Thread, BufferPools<int>.Thread, sizePower);
                BlockedEdgeRegions = new QuickSet<Edge>(BufferPools<Edge>.Thread, BufferPools<int>.Thread, sizePower);
                EdgeContacts = new QuickList<EdgeContact>(BufferPools<EdgeContact>.Thread, sizePower);
                VertexContacts = new QuickList<VertexContact>(BufferPools<VertexContact>.Thread, sizePower);
            }

            internal void Dispose()
            {
                BlockedEdgeRegions.Dispose();
                BlockedVertexRegions.Dispose();
                VertexContacts.Dispose();
                EdgeContacts.Dispose();
            }
        }


        protected ConvexCollidable convex;

        ///<summary>
        /// Gets the convex collidable associated with this pair.
        ///</summary>
        public ConvexCollidable ConvexCollidable
        {
            get
            {
                return convex;
            }
        }

        ///<summary>
        /// Constructs a new contact manifold.
        ///</summary>
        protected TriangleMeshConvexContactManifold()
        {
            contacts = new RawList<Contact>(4);
            unusedContacts = new UnsafeResourcePool<Contact>(4);
            contactIndicesToRemove = new RawList<int>(4);
            activePairTesters = new QuickDictionary<TriangleIndices, TrianglePairTester>(BufferPools<TriangleIndices>.Locking, BufferPools<TrianglePairTester>.Locking, BufferPools<int>.Locking, 3);
        }

        protected virtual RigidTransform MeshTransform
        {
            get
            {
                return RigidTransform.Identity;
            }
        }


        protected abstract bool UseImprovedBoundaryHandling { get; }
        protected internal abstract int FindOverlappingTriangles(float dt);

        /// <summary>
        /// Precomputes the transform to bring triangles from their native local space to the local space of the convex.
        /// </summary>
        /// <param name="convexInverseWorldTransform">Inverse of the world transform of the convex shape.</param>
        /// <param name="fromMeshLocalToConvexLocal">Transform to apply to native local triangles to bring them into the local space of the convex.</param>
        protected abstract void PrecomputeTriangleTransform(ref AffineTransform convexInverseWorldTransform, out AffineTransform fromMeshLocalToConvexLocal);
        protected abstract bool ConfigureLocalTriangle(int i, TriangleShape localTriangleShape, out TriangleIndices indices);
        protected internal abstract void CleanUpOverlappingTriangles();

        ///<summary>
        /// Updates the manifold.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        public override void Update(float dt)
        {
            //First, refresh all existing contacts.  This is an incremental manifold.
            var transform = MeshTransform;
            ContactRefresher.ContactRefresh(contacts, supplementData, ref convex.worldTransform, ref transform, contactIndicesToRemove);

            RemoveQueuedContacts();


            CleanUpOverlappingTriangles();
            //Get all the overlapped triangle indices.
            //Note that the collection of triangles is left up to the child implementation.
            //We're basically treating the child class like an indexable collection.
            //A little gross to have it organized this way instead of an explicit collection to separate the logic up. Would be nice to improve someday!
            int triangleCount = FindOverlappingTriangles(dt);

            //Just use 32 elements for all the lists and sets in this system.
            const int bufferPoolSizePower = 5;
            BoundarySets boundarySets;
            if (UseImprovedBoundaryHandling)
                boundarySets = new BoundarySets(bufferPoolSizePower);
            else
                boundarySets = new BoundarySets();

            var candidatesToAdd = new QuickList<ContactData>(BufferPools<ContactData>.Thread, bufferPoolSizePower);

            //A single triangle shape will be reused for all operations. It's pulled from a thread local pool to avoid holding a TriangleShape around for every single contact manifold or pair tester.
            var localTriangleShape = PhysicsThreadResources.GetTriangle();

            //Precompute the transform to take triangles from their native local space to the convex's local space.
            RigidTransform inverseConvexWorldTransform;
            RigidTransform.Invert(ref convex.worldTransform, out inverseConvexWorldTransform);
            AffineTransform convexInverseWorldTransform;
            AffineTransform.CreateFromRigidTransform(ref inverseConvexWorldTransform, out convexInverseWorldTransform);
            AffineTransform fromMeshLocalToConvexLocal;
            PrecomputeTriangleTransform(ref convexInverseWorldTransform, out fromMeshLocalToConvexLocal);

            //Grab the convex's local space bounding box up front. This will be used for a secondary pruning step.
            BoundingBox convexLocalBoundingBox;
            convex.Shape.GetBoundingBox(ref Toolbox.RigidIdentity, out convexLocalBoundingBox);

            Matrix3x3 orientation;
            Matrix3x3.CreateFromQuaternion(ref convex.worldTransform.Orientation, out orientation);
            var guaranteedContacts = 0;
            for (int i = 0; i < triangleCount; i++)
            {
                //Initialize the local triangle.
                TriangleIndices indices;
                if (ConfigureLocalTriangle(i, localTriangleShape, out indices))
                {
                    //Put the triangle into the local space of the convex.
                    AffineTransform.Transform(ref localTriangleShape.vA, ref fromMeshLocalToConvexLocal, out localTriangleShape.vA);
                    AffineTransform.Transform(ref localTriangleShape.vB, ref fromMeshLocalToConvexLocal, out localTriangleShape.vB);
                    AffineTransform.Transform(ref localTriangleShape.vC, ref fromMeshLocalToConvexLocal, out localTriangleShape.vC);

                    //Do one last AABB test between the convex and triangle in the convex's local space.
                    //This can prune out a lot of triangles when dealing with larger objects, and it's pretty cheap to do.
                    BoundingBox triangleBoundingBox;
                    Toolbox.GetTriangleBoundingBox(ref localTriangleShape.vA, ref localTriangleShape.vB, ref localTriangleShape.vC, out triangleBoundingBox);

                    bool intersecting;
                    triangleBoundingBox.Intersects(ref convexLocalBoundingBox, out intersecting);
                    if (!intersecting)
                        continue;

                    //Find a pairtester for the triangle.
                    TrianglePairTester pairTester;
                    if (!activePairTesters.TryGetValue(indices, out pairTester))
                    {
                        pairTester = GetTester();
                        pairTester.Initialize(convex.Shape);
                        activePairTesters.Add(indices, pairTester);
                    }
                    pairTester.Updated = true;


                    //Now, generate a contact between the two shapes.
                    TinyStructList<ContactData> contactList;
                    if (pairTester.GenerateContactCandidates(localTriangleShape, out contactList))
                    {
                        for (int j = 0; j < contactList.Count; j++)
                        {
                            ContactData contact;
                            contactList.Get(j, out contact);


                            if (UseImprovedBoundaryHandling)
                            {
                                if (AnalyzeCandidate(ref indices, localTriangleShape, pairTester, ref contact, ref boundarySets))
                                {
                                    //This is let through if there's a face contact. Face contacts cannot be blocked.
                                    guaranteedContacts++;
                                    AddLocalContact(ref contact, ref orientation, ref candidatesToAdd);
                                }
                            }
                            else
                            {
                                AddLocalContact(ref contact, ref orientation, ref candidatesToAdd);
                            }

                        }
                    }

                    //Get the voronoi region from the contact candidate generation.  Possibly just recalculate, since most of the systems don't calculate it.
                    //Depending on which voronoi region it is in (Switch on enumeration), identify the indices composing that region.  For face contacts, don't bother- just add it if unique.
                    //For AB, AC, or BC, add an Edge to the blockedEdgeRegions set with the corresponding indices.
                    //For A, B, or C, add the index of the vertex to the blockedVertexRegions set.
                    //If the edge/vertex is already present in the set, then DO NOT add the contact.
                    //When adding a contact, add ALL other voronoi regions to the blocked sets. 
                }

            }



            if (UseImprovedBoundaryHandling)
            {

                //If there were no face contacts that absolutely must be included, we may get into a very rare situation
                //where absolutely no contacts get created.  For example, a sphere falling directly on top of a vertex in a flat terrain.
                //It will generally get locked out of usage by belonging only to restricted regions (numerical issues make it visible by both edges and vertices).
                //In some cases, the contacts will be ignored instead of corrected (e.g. spheres).
                //To prevent objects from just falling through the ground in such a situation, force-correct the contacts regardless of the pair tester's desires.
                //Sure, it might not be necessary under normal circumstances, but it's a better option than having no contacts.
                //TODO: There is another option: Changing restricted regions so that a vertex only restricts the other two vertices and the far edge,
                //and an edge only restricts the far vertex and other two edges.  This introduces an occasional bump though...

                //It's possible, in very specific instances, for an object to wedge itself between two adjacent triangles.
                //For this state to continue beyond a brief instant generally requires the object be orientation locked and slender.
                //However, some characters fit this description, so it can't be ignored!

                //Conceptually, this issue can occur at either a vertex junction or a shared edge (usually on extremely flat surfaces only).
                //However, an object stuck between multiple triangles is not in a stable state.  In the edge case, the object gets shoved to one side
                //as one contact 'wins' the solver war.  That's not enough to escape, unfortunately.
                //The vertex case, on the other hand, is degenerate and decays into an edge case rapidly thanks to this lack of stability.
                //So, we don't have to explicitly handle the somewhat more annoying and computationally expensive vertex unstucking case, because the edge case handles both! :)

                //This isn't a completely free operation, but it's guarded behind pretty rare conditions.
                //Essentially, we will check to see if there's just edge contacts fighting against each other.
                //If they are, then we will correct any stuck-contributing normals to the triangle normal.
                if (boundarySets.VertexContacts.Count == 0 && guaranteedContacts == 0 && boundarySets.EdgeContacts.Count > 1)
                {
                    //There are only edge contacts, check to see if:
                    //all normals are coplanar, and
                    //at least one normal faces against the other normals (meaning it's probably stuck, as opposed to just colliding on a corner).

                    bool allNormalsInSamePlane = true;
                    bool atLeastOneNormalAgainst = false;

                    var firstNormal = boundarySets.EdgeContacts.Elements[0].ContactData.Normal;
                    boundarySets.EdgeContacts.Elements[0].CorrectedNormal.Normalize();
                    float dot;
                    Vector3.Dot(ref firstNormal, ref boundarySets.EdgeContacts.Elements[0].CorrectedNormal, out dot);
                    if (Math.Abs(dot) > .01f)
                    {
                        //Go ahead and test the first contact separately, since we're using its contact normal to determine coplanarity.
                        allNormalsInSamePlane = false;
                    }
                    else
                    {
                        //TODO: Note that we're only checking the new edge contacts, not the existing contacts.
                        //It's possible that some existing contacts could interfere and cause issues, but for the sake of simplicity and due to rarity
                        //we'll ignore that possibility for now.
                        for (int i = 1; i < boundarySets.EdgeContacts.Count; i++)
                        {
                            Vector3.Dot(ref boundarySets.EdgeContacts.Elements[i].ContactData.Normal, ref firstNormal, out dot);
                            if (dot < 0)
                            {
                                atLeastOneNormalAgainst = true;
                            }
                            //Check to see if the normal is outside the plane.
                            Vector3.Dot(ref boundarySets.EdgeContacts.Elements[i].ContactData.Normal, ref boundarySets.EdgeContacts.Elements[0].CorrectedNormal, out dot);

                            if (Math.Abs(dot) > .01f)
                            {

                                //We are not stuck!
                                allNormalsInSamePlane = false;
                                break;
                            }
                        }
                    }

                    if (allNormalsInSamePlane && atLeastOneNormalAgainst)
                    {
                        //Uh oh! all the normals are parallel... The object is probably in a weird situation.
                        //Let's correct the normals!

                        //Already normalized the first contact above.
                        //We don't need to perform the perpendicularity test here- we did that before! We know it's perpendicular already.
                        boundarySets.EdgeContacts.Elements[0].ContactData.Normal = boundarySets.EdgeContacts.Elements[0].CorrectedNormal;
                        boundarySets.EdgeContacts.Elements[0].ShouldCorrect = true;

                        for (int i = 1; i < boundarySets.EdgeContacts.Count; i++)
                        {
                            //Must normalize the corrected normal before using it.
                            boundarySets.EdgeContacts.Elements[i].CorrectedNormal.Normalize();
                            Vector3.Dot(ref boundarySets.EdgeContacts.Elements[i].CorrectedNormal, ref boundarySets.EdgeContacts.Elements[i].ContactData.Normal, out dot);
                            if (dot < .01)
                            {
                                //Only bother doing the correction if the normal appears to be pointing nearly horizontally- implying that it's a contributor to the stuckness!
                                //If it's blocked, the next section will use the corrected normal- if it's not blocked, the next section will use the direct normal.
                                //Make them the same thing :)
                                boundarySets.EdgeContacts.Elements[i].ContactData.Normal = boundarySets.EdgeContacts.Elements[i].CorrectedNormal;
                                boundarySets.EdgeContacts.Elements[i].ShouldCorrect = true;
                                //Note that the penetration depth is NOT corrected.  The contact's depth no longer represents the true depth.
                                //However, we only need to have some penetration depth to get the object to escape the rut.
                                //Furthermore, the depth computed from the horizontal opposing contacts is known to be less than the depth in the perpendicular direction.
                                //If the current depth was NOT less than the true depth along the corrected normal, then the collision detection system 
                                //would have picked a different depth, as it finds a reasonable approximation of the minimum penetration!
                                //As a consequence, this contact will not be active beyond the object's destuckification, because its contact depth will be negative (or very close to it).

                            }
                        }
                    }
                }





                for (int i = 0; i < boundarySets.EdgeContacts.Count; i++)
                {
                    //Only correct if it's allowed AND it's blocked.
                    //If it's not blocked, the contact being created is necessary!
                    //The normal generated by the triangle-convex tester is already known not to
                    //violate the triangle sidedness.
                    if (!boundarySets.BlockedEdgeRegions.Contains(boundarySets.EdgeContacts.Elements[i].Edge))
                    {
                        //If it's not blocked, use the contact as-is without correcting it.
                        AddLocalContact(ref boundarySets.EdgeContacts.Elements[i].ContactData, ref orientation, ref candidatesToAdd);

                    }
                    else if (boundarySets.EdgeContacts.Elements[i].ShouldCorrect || guaranteedContacts == 0)
                    {
                        //If it is blocked, we can still make use of the contact.  But first, we need to change the contact normal to ensure that
                        //it will not interfere (and cause a bump or something).
                        float dot;
                        boundarySets.EdgeContacts.Elements[i].CorrectedNormal.Normalize();
                        Vector3.Dot(ref boundarySets.EdgeContacts.Elements[i].CorrectedNormal, ref boundarySets.EdgeContacts.Elements[i].ContactData.Normal, out dot);
                        boundarySets.EdgeContacts.Elements[i].ContactData.Normal = boundarySets.EdgeContacts.Elements[i].CorrectedNormal;
                        boundarySets.EdgeContacts.Elements[i].ContactData.PenetrationDepth *= MathHelper.Max(0, dot); //Never cause a negative penetration depth.
                        AddLocalContact(ref boundarySets.EdgeContacts.Elements[i].ContactData, ref orientation, ref candidatesToAdd);
                    }
                    //If it's blocked AND it doesn't allow correction, ignore its existence.



                }




                for (int i = 0; i < boundarySets.VertexContacts.Count; i++)
                {

                    if (!boundarySets.BlockedVertexRegions.Contains(boundarySets.VertexContacts.Elements[i].Vertex))
                    {
                        //If it's not blocked, use the contact as-is without correcting it.
                        AddLocalContact(ref boundarySets.VertexContacts.Elements[i].ContactData, ref orientation, ref candidatesToAdd);
                    }
                    else if (boundarySets.VertexContacts.Elements[i].ShouldCorrect || guaranteedContacts == 0)
                    {
                        //If it is blocked, we can still make use of the contact.  But first, we need to change the contact normal to ensure that
                        //it will not interfere (and cause a bump or something).
                        float dot;
                        boundarySets.VertexContacts.Elements[i].CorrectedNormal.Normalize();
                        Vector3.Dot(ref boundarySets.VertexContacts.Elements[i].CorrectedNormal, ref boundarySets.VertexContacts.Elements[i].ContactData.Normal, out dot);
                        boundarySets.VertexContacts.Elements[i].ContactData.Normal = boundarySets.VertexContacts.Elements[i].CorrectedNormal;
                        boundarySets.VertexContacts.Elements[i].ContactData.PenetrationDepth *= MathHelper.Max(0, dot); //Never cause a negative penetration depth.
                        AddLocalContact(ref boundarySets.VertexContacts.Elements[i].ContactData, ref orientation, ref candidatesToAdd);
                    }
                    //If it's blocked AND it doesn't allow correction, ignore its existence.


                }


                boundarySets.Dispose();


            }



            //Remove stale pair testers.
            for (int i = activePairTesters.Count - 1; i >= 0; --i)
            {
                var tester = activePairTesters.Values[i];
                if (!tester.Updated)
                {
                    tester.CleanUp();
                    GiveBackTester(tester);
                    activePairTesters.FastRemove(activePairTesters.Keys[i]);
                }
                else
                {
                    tester.Updated = false;
                }

            }

            //Some child types will want to do some extra post processing on the manifold.        
            ProcessCandidates(ref candidatesToAdd);


            //Check if adding the new contacts would overflow the manifold.
            if (contacts.Count + candidatesToAdd.Count > 4)
            {
                //Adding all the contacts would overflow the manifold.  Reduce to the best subset.
                var reducedCandidates = new QuickList<ContactData>(BufferPools<ContactData>.Thread, bufferPoolSizePower);
                ContactReducer.ReduceContacts(contacts, ref candidatesToAdd, contactIndicesToRemove, ref reducedCandidates);
                RemoveQueuedContacts();
                for (int i = reducedCandidates.Count - 1; i >= 0; i--)
                {
                    Add(ref reducedCandidates.Elements[i]);
                    reducedCandidates.RemoveAt(i);
                }
                reducedCandidates.Dispose();
            }
            else if (candidatesToAdd.Count > 0)
            {
                //Won't overflow the manifold, so just toss it in PROVIDED that it isn't too close to something else.
                for (int i = 0; i < candidatesToAdd.Count; i++)
                {
                    Add(ref candidatesToAdd.Elements[i]);
                }
            }

            
            PhysicsThreadResources.GiveBack(localTriangleShape);
            candidatesToAdd.Dispose();

        }

        void AddLocalContact(ref ContactData contact, ref Matrix3x3 orientation, ref QuickList<ContactData> candidatesToAdd)
        {
            //Put the contact into world space.
            Matrix3x3.Transform(ref contact.Position, ref orientation, out contact.Position);
            Vector3.Add(ref contact.Position, ref convex.worldTransform.Position, out contact.Position);
            Matrix3x3.Transform(ref contact.Normal, ref orientation, out contact.Normal);
            //Check to see if the contact is unique before proceeding.
            if (IsContactUnique(ref contact, ref candidatesToAdd))
            {
                candidatesToAdd.Add(ref contact);
            }
        }


        protected void GetNormal(ref Vector3 uncorrectedNormal, TriangleShape localTriangleShape, out Vector3 normal)
        {
            //Compute the normal of the triangle in the current convex's local space.
            //Note its reliance on the local triangle shape.  It must be initialized to the correct values before this is called.
            Vector3 AB, AC;
            Vector3.Subtract(ref localTriangleShape.vB, ref localTriangleShape.vA, out AB);
            Vector3.Subtract(ref localTriangleShape.vC, ref localTriangleShape.vA, out AC);
            //Compute the normal based on the sidedness.
            switch (localTriangleShape.sidedness)
            {
                case TriangleSidedness.DoubleSided:
                    //If it's double sided, then pick the triangle normal which points in the same direction
                    //as the contact normal that's going to be corrected.
                    float dot;
                    Vector3.Cross(ref AB, ref AC, out normal);
                    Vector3.Dot(ref normal, ref uncorrectedNormal, out dot);
                    if (dot < 0)
                        Vector3.Negate(ref normal, out normal);
                    break;
                case TriangleSidedness.Clockwise:
                    //If it's clockwise, always use ACxAB.
                    Vector3.Cross(ref AC, ref AB, out normal);
                    break;
                default:
                    //If it's counterclockwise, always use ABxAC.
                    Vector3.Cross(ref AB, ref AC, out normal);
                    break;
            }
            //If the normal is degenerate, just use the uncorrected normal.
            if (normal.LengthSquared() < Toolbox.Epsilon)
                normal = uncorrectedNormal;
        }

        bool AnalyzeCandidate(ref TriangleIndices indices, TriangleShape triangle, TrianglePairTester pairTester, ref ContactData contact, ref BoundarySets sets)
        {
            switch (pairTester.GetRegion(triangle, ref contact))
            {
                case VoronoiRegion.A:
                    //Add the contact.
                    VertexContact vertexContact;
                    GetNormal(ref contact.Normal, triangle, out vertexContact.CorrectedNormal);
                    vertexContact.ContactData = contact;
                    vertexContact.Vertex = indices.A;
                    vertexContact.ShouldCorrect = pairTester.ShouldCorrectContactNormal;
                    sets.VertexContacts.Add(ref vertexContact);

                    //Block all of the other voronoi regions.
                    sets.BlockedEdgeRegions.Add(new Edge(indices.A, indices.B));
                    sets.BlockedEdgeRegions.Add(new Edge(indices.B, indices.C));
                    sets.BlockedEdgeRegions.Add(new Edge(indices.A, indices.C));
                    sets.BlockedVertexRegions.Add(indices.B);
                    sets.BlockedVertexRegions.Add(indices.C);

                    break;
                case VoronoiRegion.B:
                    //Add the contact.
                    GetNormal(ref contact.Normal, triangle, out vertexContact.CorrectedNormal);
                    vertexContact.ContactData = contact;
                    vertexContact.Vertex = indices.B;
                    vertexContact.ShouldCorrect = pairTester.ShouldCorrectContactNormal;
                    sets.VertexContacts.Add(ref vertexContact);

                    //Block all of the other voronoi regions.
                    sets.BlockedEdgeRegions.Add(new Edge(indices.A, indices.B));
                    sets.BlockedEdgeRegions.Add(new Edge(indices.B, indices.C));
                    sets.BlockedEdgeRegions.Add(new Edge(indices.A, indices.C));
                    sets.BlockedVertexRegions.Add(indices.A);
                    sets.BlockedVertexRegions.Add(indices.C);

                    break;
                case VoronoiRegion.C:
                    //Add the contact.
                    GetNormal(ref contact.Normal, triangle, out vertexContact.CorrectedNormal);
                    vertexContact.ContactData = contact;
                    vertexContact.Vertex = indices.C;
                    vertexContact.ShouldCorrect = pairTester.ShouldCorrectContactNormal;
                    sets.VertexContacts.Add(ref vertexContact);

                    //Block all of the other voronoi regions.
                    sets.BlockedEdgeRegions.Add(new Edge(indices.A, indices.B));
                    sets.BlockedEdgeRegions.Add(new Edge(indices.B, indices.C));
                    sets.BlockedEdgeRegions.Add(new Edge(indices.A, indices.C));
                    sets.BlockedVertexRegions.Add(indices.A);
                    sets.BlockedVertexRegions.Add(indices.B);

                    break;
                case VoronoiRegion.AB:
                    //Add the contact.
                    EdgeContact edgeContact;
                    GetNormal(ref contact.Normal, triangle, out edgeContact.CorrectedNormal);
                    edgeContact.Edge = new Edge(indices.A, indices.B);
                    edgeContact.ContactData = contact;
                    edgeContact.ShouldCorrect = pairTester.ShouldCorrectContactNormal;
                    sets.EdgeContacts.Add(ref edgeContact);

                    //Block all of the other voronoi regions.
                    sets.BlockedEdgeRegions.Add(new Edge(indices.B, indices.C));
                    sets.BlockedEdgeRegions.Add(new Edge(indices.A, indices.C));
                    sets.BlockedVertexRegions.Add(indices.A);
                    sets.BlockedVertexRegions.Add(indices.B);
                    sets.BlockedVertexRegions.Add(indices.C);
                    break;
                case VoronoiRegion.AC:
                    //Add the contact.
                    GetNormal(ref contact.Normal, triangle, out edgeContact.CorrectedNormal);
                    edgeContact.Edge = new Edge(indices.A, indices.C);
                    edgeContact.ContactData = contact;
                    edgeContact.ShouldCorrect = pairTester.ShouldCorrectContactNormal;
                    sets.EdgeContacts.Add(ref edgeContact);

                    //Block all of the other voronoi regions.
                    sets.BlockedEdgeRegions.Add(new Edge(indices.A, indices.B));
                    sets.BlockedEdgeRegions.Add(new Edge(indices.B, indices.C));
                    sets.BlockedVertexRegions.Add(indices.A);
                    sets.BlockedVertexRegions.Add(indices.B);
                    sets.BlockedVertexRegions.Add(indices.C);
                    break;
                case VoronoiRegion.BC:
                    //Add the contact.
                    GetNormal(ref contact.Normal, triangle, out edgeContact.CorrectedNormal);
                    edgeContact.Edge = new Edge(indices.B, indices.C);
                    edgeContact.ContactData = contact;
                    edgeContact.ShouldCorrect = pairTester.ShouldCorrectContactNormal;
                    sets.EdgeContacts.Add(ref edgeContact);

                    //Block all of the other voronoi regions.
                    sets.BlockedEdgeRegions.Add(new Edge(indices.A, indices.B));
                    sets.BlockedEdgeRegions.Add(new Edge(indices.A, indices.C));
                    sets.BlockedVertexRegions.Add(indices.A);
                    sets.BlockedVertexRegions.Add(indices.B);
                    sets.BlockedVertexRegions.Add(indices.C);
                    break;
                default:
                    //Block all of the other voronoi regions.
                    sets.BlockedEdgeRegions.Add(new Edge(indices.A, indices.B));
                    sets.BlockedEdgeRegions.Add(new Edge(indices.B, indices.C));
                    sets.BlockedEdgeRegions.Add(new Edge(indices.A, indices.C));
                    sets.BlockedVertexRegions.Add(indices.A);
                    sets.BlockedVertexRegions.Add(indices.B);
                    sets.BlockedVertexRegions.Add(indices.C);
                    //Should add the contact.
                    return true;
            }


            return false;
        }

        protected override void Add(ref ContactData contactCandidate)
        {
            ContactSupplementData supplement;
            supplement.BasePenetrationDepth = contactCandidate.PenetrationDepth;
            //The closest point method computes the local space versions before transforming to world... consider cutting out the middle man
            RigidTransform.TransformByInverse(ref contactCandidate.Position, ref convex.worldTransform, out supplement.LocalOffsetA);
            RigidTransform transform = MeshTransform;
            RigidTransform.TransformByInverse(ref contactCandidate.Position, ref transform, out supplement.LocalOffsetB);
            supplementData.Add(ref supplement);
            base.Add(ref contactCandidate);
        }

        protected override void Remove(int contactIndex)
        {
            supplementData.RemoveAt(contactIndex);
            base.Remove(contactIndex);
        }


        private bool IsContactUnique(ref ContactData contactCandidate, ref QuickList<ContactData> candidatesToAdd)
        {
            contactCandidate.Validate();
            float distanceSquared;
            RigidTransform meshTransform = MeshTransform;
            for (int i = 0; i < contacts.Count; i++)
            {
                Vector3.DistanceSquared(ref contacts.Elements[i].Position, ref contactCandidate.Position, out distanceSquared);
                if (distanceSquared < CollisionDetectionSettings.ContactMinimumSeparationDistanceSquared)
                {
                    //This is a nonconvex manifold.  There will be times where a an object will be shoved into a corner such that
                    //a single position will have two reasonable normals.  If the normals aren't mostly aligned, they should NOT be considered equivalent.
                    Vector3.Dot(ref contacts.Elements[i].Normal, ref contactCandidate.Normal, out distanceSquared);
                    if (Math.Abs(distanceSquared) >= CollisionDetectionSettings.nonconvexNormalDotMinimum)
                    {
                        //Update the existing 'redundant' contact with the new information.
                        //This works out because the new contact is the deepest contact according to the previous collision detection iteration.
                        contacts.Elements[i].Normal = contactCandidate.Normal;
                        contacts.Elements[i].Position = contactCandidate.Position;
                        contacts.Elements[i].PenetrationDepth = contactCandidate.PenetrationDepth;
                        supplementData.Elements[i].BasePenetrationDepth = contactCandidate.PenetrationDepth;
                        RigidTransform.TransformByInverse(ref contactCandidate.Position, ref convex.worldTransform, out supplementData.Elements[i].LocalOffsetA);
                        RigidTransform.TransformByInverse(ref contactCandidate.Position, ref meshTransform, out supplementData.Elements[i].LocalOffsetB);
                        return false;
                    }
                }
            }
            for (int i = 0; i < candidatesToAdd.Count; i++)
            {
                Vector3.DistanceSquared(ref candidatesToAdd.Elements[i].Position, ref contactCandidate.Position, out distanceSquared);
                if (distanceSquared < CollisionDetectionSettings.ContactMinimumSeparationDistanceSquared)
                {
                    //This is a nonconvex manifold.  There will be times where a an object will be shoved into a corner such that
                    //a single position will have two reasonable normals.  If the normals aren't mostly aligned, they should NOT be considered equivalent.
                    Vector3.Dot(ref candidatesToAdd.Elements[i].Normal, ref contactCandidate.Normal, out distanceSquared);
                    if (Math.Abs(distanceSquared) >= CollisionDetectionSettings.nonconvexNormalDotMinimum)
                        return false;
                }
            }
            //for (int i = 0; i < edgeContacts.count; i++)
            //{
            //    Vector3.DistanceSquared(ref edgeContacts.Elements[i].ContactData.Position, ref contactCandidate.Position, out distanceSquared);
            //    if (distanceSquared < CollisionDetectionSettings.ContactMinimumSeparationDistanceSquared)
            //    {
            //        return false;
            //    }
            //}
            //for (int i = 0; i < vertexContacts.count; i++)
            //{
            //    Vector3.DistanceSquared(ref vertexContacts.Elements[i].ContactData.Position, ref contactCandidate.Position, out distanceSquared);
            //    if (distanceSquared < CollisionDetectionSettings.ContactMinimumSeparationDistanceSquared)
            //    {
            //        return false;
            //    }
            //}
            return true;

        }

        protected virtual void ProcessCandidates(ref QuickList<ContactData> candidates)
        {

        }


        ///<summary>
        /// Cleans up the manifold.
        ///</summary>
        public override void CleanUp()
        {
            supplementData.Clear();
            convex = null;
            for (int i = activePairTesters.Count - 1; i >= 0; --i)
            {
                activePairTesters.Values[i].CleanUp();
                GiveBackTester(activePairTesters.Values[i]);
            }
            activePairTesters.Dispose();
            activePairTesters = new QuickDictionary<TriangleIndices, TrianglePairTester>(BufferPools<TriangleIndices>.Locking, BufferPools<TrianglePairTester>.Locking, BufferPools<int>.Locking, 3);
            CleanUpOverlappingTriangles();
            base.CleanUp();
        }

        /// <summary>
        /// Edge of a triangle in a mesh in terms of vertex indices.
        /// </summary>
        public struct Edge : IEquatable<Edge>
        {
            private int A;
            private int B;

            public Edge(int a, int b)
            {
                A = a;
                B = b;
            }

            public override int GetHashCode()
            {
                return A + B;
            }

            public bool Equals(Edge edge)
            {
                return (edge.A == A && edge.B == B) || (edge.A == B && edge.B == A);
            }

            public override string ToString()
            {
                return "{" + A + ", " + B + "}";
            }
        }

        ///<summary>
        /// Stores indices of a triangle.
        ///</summary>
        public struct TriangleIndices : IEquatable<TriangleIndices>
        {
            ///<summary>
            /// First index in the triangle.
            ///</summary>
            public int A;
            ///<summary>
            /// Second index in the triangle.
            ///</summary>
            public int B;
            ///<summary>
            /// Third index in the triangle.
            ///</summary>
            public int C;

            /// <summary>
            /// Returns the hash code for this instance.
            /// </summary>
            /// <returns>
            /// A 32-bit signed integer that is the hash code for this instance.
            /// </returns>
            /// <filterpriority>2</filterpriority>
            public override int GetHashCode()
            {
                return A + B + C;
            }

            /// <summary>
            /// Indicates whether the current object is equal to another object of the same type.
            /// </summary>
            /// <returns>
            /// true if the current object is equal to the <paramref name="other"/> parameter; otherwise, false.
            /// </returns>
            /// <param name="other">An object to compare with this object.</param>
            public bool Equals(TriangleIndices other)
            {
                return A == other.A && B == other.B && C == other.C;
            }
        }

        struct EdgeContact
        {
            public bool ShouldCorrect;
            public Vector3 CorrectedNormal;
            public Edge Edge;
            public ContactData ContactData;
        }

        struct VertexContact
        {
            public bool ShouldCorrect;
            public Vector3 CorrectedNormal;
            public int Vertex;
            public ContactData ContactData;
        }
    }
}
