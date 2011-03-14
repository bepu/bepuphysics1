using System;
using System.Collections.Generic;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using Microsoft.Xna.Framework;
using BEPUphysics.DataStructures;
using BEPUphysics.Settings;
using BEPUphysics.ResourceManagement;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.MathExtensions;

namespace BEPUphysics.CollisionTests.Manifolds
{
    ///<summary>
    /// Manages persistent contact data between a triangle mesh and a convex.
    ///</summary>
    public abstract class TriangleMeshConvexContactManifold : ContactManifold
    {
        RawValueList<ContactSupplementData> supplementData = new RawValueList<ContactSupplementData>(4);
        Dictionary<TriangleIndices, TriangleConvexPairTester> activePairTesters = new Dictionary<TriangleIndices, TriangleConvexPairTester>(4);
        RawValueList<ContactData> candidatesToAdd;
        RawValueList<ContactData> reducedCandidates = new RawValueList<ContactData>();
        protected TriangleShape localTriangleShape = new TriangleShape();

        UnsafeResourcePool<TriangleConvexPairTester> pairTestersPool = new UnsafeResourcePool<TriangleConvexPairTester>(4);

        HashSet<int> blockedVertexRegions = new HashSet<int>();
        HashSet<Edge> blockedEdgeRegions = new HashSet<Edge>();
        RawValueList<EdgeContact> edgeContacts = new RawValueList<EdgeContact>();
        RawValueList<VertexContact> vertexContacts = new RawValueList<VertexContact>();

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
            unusedContacts = new UnsafeResourcePool<Contact>(4);
            contactIndicesToRemove = new RawList<int>(4);
            candidatesToAdd = new RawValueList<ContactData>(1);
        }


        protected abstract bool UseImprovedBoundaryHandling { get; }
        protected abstract int FindOverlappingTriangles(float dt);
        protected abstract void ConfigureTriangle(int i, out TriangleIndices indices);
        protected abstract void CleanUpOverlappingTriangles();

        ///<summary>
        /// Updates the manifold.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        public override void Update(float dt)
        {
            //First, refresh all existing contacts.  This is an incremental manifold.
            ContactRefresher.ContactRefresh(contacts, supplementData, ref convex.worldTransform, ref Toolbox.RigidIdentity, contactIndicesToRemove);
            RemoveQueuedContacts();

            CleanUpOverlappingTriangles();
            //Get all the overlapped triangle indices.
            int triangleCount = FindOverlappingTriangles(dt);

            Matrix3X3 orientation;
            Matrix3X3.CreateFromQuaternion(ref convex.worldTransform.Orientation, out orientation);
            for (int i = 0; i < triangleCount; i++)
            {
                //Initialize the local triangle.
                TriangleIndices indices;
                ConfigureTriangle(i, out indices);

                //Find a pairtester for the triangle.
                TriangleConvexPairTester pairTester;
                if (!activePairTesters.TryGetValue(indices, out pairTester))
                {
                    pairTester = pairTestersPool.Take();
                    pairTester.Initialize(convex.Shape, localTriangleShape);
                    activePairTesters.Add(indices, pairTester);
                }
                pairTester.Updated = true;


                //Put the triangle into the local space of the convex.
                Vector3.Subtract(ref localTriangleShape.vA, ref convex.worldTransform.Position, out localTriangleShape.vA);
                Vector3.Subtract(ref localTriangleShape.vB, ref convex.worldTransform.Position, out localTriangleShape.vB);
                Vector3.Subtract(ref localTriangleShape.vC, ref convex.worldTransform.Position, out localTriangleShape.vC);
                Matrix3X3.TransformTranspose(ref localTriangleShape.vA, ref orientation, out localTriangleShape.vA);
                Matrix3X3.TransformTranspose(ref localTriangleShape.vB, ref orientation, out localTriangleShape.vB);
                Matrix3X3.TransformTranspose(ref localTriangleShape.vC, ref orientation, out localTriangleShape.vC);


                //Now, generate a contact between the two shapes.
                ContactData contact;
                if (pairTester.GenerateContactCandidate(out contact))
                {
                    if (UseImprovedBoundaryHandling)
                    {
                        //if (Keyboard.GetState().IsKeyDown(Keys.O))
                        //    Debug.WriteLine("break.");
                        if (AnalyzeCandidate(ref indices, pairTester, ref contact))
                        {
                            AddLocalContact(ref contact, ref orientation);
                        }
                    }
                    else
                    {
                        AddLocalContact(ref contact, ref orientation);
                    }
                }

                //Get the voronoi region from the contact candidate generation.  Possibly just recalculate, since most of the systems don't calculate it.
                //Depending on which voronoi region it is in (Switch on enumeration), identify the indices composing that region.  For face contacts, don't bother- just add it if unique.
                //For AB, AC, or BC, add an Edge to the blockedEdgeRegions set with the corresponding indices.
                //For A, B, or C, add the index of the vertex to the blockedVertexRegions set.
                //If the edge/vertex is already present in the set, then DO NOT add the contact.
                //When adding a contact, add ALL other voronoi regions to the blocked sets. 

            }


            if (UseImprovedBoundaryHandling)
            {
                for (int i = 0; i < edgeContacts.count; i++)
                {
                    if (edgeContacts.Elements[i].ShouldCorrect)
                    {
                        float dot;
                        edgeContacts.Elements[i].CorrectedNormal.Normalize();
                        Vector3.Dot(ref edgeContacts.Elements[i].CorrectedNormal, ref edgeContacts.Elements[i].ContactData.Normal, out dot);
                        edgeContacts.Elements[i].ContactData.Normal = edgeContacts.Elements[i].CorrectedNormal;
                        edgeContacts.Elements[i].ContactData.PenetrationDepth *= MathHelper.Max(0, dot); //Never cause a negative penetration depth.
                        AddLocalContact(ref edgeContacts.Elements[i].ContactData, ref orientation);

                    }
                    else if (!blockedEdgeRegions.Contains(edgeContacts.Elements[i].Edge))
                    {
                        AddLocalContact(ref edgeContacts.Elements[i].ContactData, ref orientation);
                    }
                }
                for (int i = 0; i < vertexContacts.count; i++)
                {
                    if (vertexContacts.Elements[i].ShouldCorrect)
                    {
                        float dot;
                        vertexContacts.Elements[i].CorrectedNormal.Normalize();
                        Vector3.Dot(ref vertexContacts.Elements[i].CorrectedNormal, ref vertexContacts.Elements[i].ContactData.Normal, out dot);
                        vertexContacts.Elements[i].ContactData.Normal = vertexContacts.Elements[i].CorrectedNormal;
                        vertexContacts.Elements[i].ContactData.PenetrationDepth *= MathHelper.Max(0, dot); //Never cause a negative penetration depth.
                        AddLocalContact(ref vertexContacts.Elements[i].ContactData, ref orientation);
                    }
                    else if (!blockedVertexRegions.Contains(vertexContacts.Elements[i].Vertex))
                    {
                        AddLocalContact(ref vertexContacts.Elements[i].ContactData, ref orientation);
                    }
                }
                blockedEdgeRegions.Clear();
                blockedVertexRegions.Clear();
                vertexContacts.Clear();
                edgeContacts.Clear();
            }



            //Remove stale pair testers.
            //This will only remove 8 stale ones per frame, but it doesn't really matter.
            //VERY rarely will there be more than 8 in a single frame, and they will be immediately taken care of in the subsequent frame.
            var toRemove = new TinyList<TriangleIndices>();
            foreach (KeyValuePair<TriangleIndices, TriangleConvexPairTester> pair in activePairTesters)
            {
                if (!pair.Value.Updated)
                {
                    if (!toRemove.Add(pair.Key))
                        break;
                }
                else
                    pair.Value.Updated = false;
            }



            for (int i = toRemove.count - 1; i >= 0; i--)
            {
                var pairTester = activePairTesters[toRemove[i]];
                pairTester.CleanUp();
                pairTestersPool.GiveBack(pairTester);
                activePairTesters.Remove(toRemove[i]);
            }

            //Check if adding the new contacts would overflow the manifold.
            if (contacts.count + candidatesToAdd.count > 4)
            {
                //Adding all the contacts would overflow the manifold.  Reduce to the best subset.
                ContactReducer.ReduceContacts(contacts, candidatesToAdd, contactIndicesToRemove, reducedCandidates);
                RemoveQueuedContacts();
                for (int i = reducedCandidates.count - 1; i >= 0; i--)
                {
                    Add(ref reducedCandidates.Elements[i]);
                    reducedCandidates.RemoveAt(i);
                }
            }
            else if (candidatesToAdd.count > 0)
            {
                //Won't overflow the manifold, so just toss it in PROVIDED that it isn't too close to something else.
                for (int i = 0; i < candidatesToAdd.count; i++)
                {
                    Add(ref candidatesToAdd.Elements[i]);
                }
            }




            candidatesToAdd.Clear();

        }

        void AddLocalContact(ref ContactData contact, ref Matrix3X3 orientation)
        {
            //Put the contact into world space.
            Matrix3X3.Transform(ref contact.Position, ref orientation, out contact.Position);
            Vector3.Add(ref contact.Position, ref convex.worldTransform.Position, out contact.Position);
            Matrix3X3.Transform(ref contact.Normal, ref orientation, out contact.Normal);
            //Check to see if the contact is unique before proceeding.
            if (IsContactUnique(ref contact))
                candidatesToAdd.Add(ref contact);
        }


        TriangleConvexPairTester.VoronoiRegion GetRegion(TriangleConvexPairTester pairTester, ref ContactData contact)
        {

            switch (pairTester.state)
            {
                case TriangleConvexPairTester.CollisionState.Deep:
                case TriangleConvexPairTester.CollisionState.ExternalNear:
                    //Deep contact can produce non-triangle normals while still being within the triangle.
                    //To solve this problem, find the voronoi region to which the contact belongs using its normal.
                    //The voronoi region will be either the most extreme vertex, or the edge that includes
                    //the first and second most extreme vertices.
                    //If the normal dotted with an extreme edge direction is near 0, then it belongs to the edge.
                    //Otherwise, it belongs to the vertex.
                    //MPR tends to produce 'approximate' normals, though.
                    //Use a fairly forgiving epsilon.
                    float dotA, dotB, dotC;
                    Vector3.Dot(ref localTriangleShape.vA, ref contact.Normal, out dotA);
                    Vector3.Dot(ref localTriangleShape.vB, ref contact.Normal, out dotB);
                    Vector3.Dot(ref localTriangleShape.vC, ref contact.Normal, out dotC);

                    //Since normal points from convex to triangle always, reverse dot signs.
                    dotA = -dotA;
                    dotB = -dotB;
                    dotC = -dotC;


                    float faceEpsilon = .01f;
                    const float edgeEpsilon = .01f;

                    float edgeDot;
                    Vector3 edgeDirection;
                    if (dotA > dotB && dotA > dotC)
                    {
                        //A is extreme.
                        if (dotB > dotC)
                        {
                            //B is second most extreme.
                            if (Math.Abs(dotA - dotC) < faceEpsilon)
                            {
                                //The normal is basically a face normal.  This can happen at the edges occasionally.
                                return TriangleConvexPairTester.VoronoiRegion.ABC;
                            }
                            else
                            {
                                Vector3.Subtract(ref localTriangleShape.vB, ref localTriangleShape.vA, out edgeDirection);
                                Vector3.Dot(ref edgeDirection, ref contact.Normal, out edgeDot);
                                if (edgeDot * edgeDot < edgeDirection.LengthSquared() * edgeEpsilon)
                                    return TriangleConvexPairTester.VoronoiRegion.AB;
                                else
                                    return TriangleConvexPairTester.VoronoiRegion.A;
                            }
                        }
                        else
                        {
                            //C is second most extreme.
                            if (Math.Abs(dotA - dotB) < faceEpsilon)
                            {
                                //The normal is basically a face normal.  This can happen at the edges occasionally.
                                return TriangleConvexPairTester.VoronoiRegion.ABC;
                            }
                            else
                            {
                                Vector3.Subtract(ref localTriangleShape.vC, ref localTriangleShape.vA, out edgeDirection);
                                Vector3.Dot(ref edgeDirection, ref contact.Normal, out edgeDot);
                                if (edgeDot * edgeDot < edgeDirection.LengthSquared() * edgeEpsilon)
                                    return TriangleConvexPairTester.VoronoiRegion.AC;
                                else
                                    return TriangleConvexPairTester.VoronoiRegion.A;
                            }
                        }
                    }
                    else if (dotB > dotC)
                    {
                        //B is extreme.
                        if (dotC > dotA)
                        {
                            //C is second most extreme.
                            if (Math.Abs(dotB - dotA) < faceEpsilon)
                            {
                                //The normal is basically a face normal.  This can happen at the edges occasionally.
                                return TriangleConvexPairTester.VoronoiRegion.ABC;
                            }
                            else
                            {
                                Vector3.Subtract(ref localTriangleShape.vC, ref localTriangleShape.vB, out edgeDirection);
                                Vector3.Dot(ref edgeDirection, ref contact.Normal, out edgeDot);
                                if (edgeDot * edgeDot < edgeDirection.LengthSquared() * edgeEpsilon)
                                    return TriangleConvexPairTester.VoronoiRegion.BC;
                                else
                                    return TriangleConvexPairTester.VoronoiRegion.B;
                            }
                        }
                        else
                        {
                            //A is second most extreme.
                            if (Math.Abs(dotB - dotC) < faceEpsilon)
                            {
                                //The normal is basically a face normal.  This can happen at the edges occasionally.
                                return TriangleConvexPairTester.VoronoiRegion.ABC;
                            }
                            else
                            {
                                Vector3.Subtract(ref localTriangleShape.vA, ref localTriangleShape.vB, out edgeDirection);
                                Vector3.Dot(ref edgeDirection, ref contact.Normal, out edgeDot);
                                if (edgeDot * edgeDot < edgeDirection.LengthSquared() * edgeEpsilon)
                                    return TriangleConvexPairTester.VoronoiRegion.AB;
                                else
                                    return TriangleConvexPairTester.VoronoiRegion.B;
                            }
                        }
                    }
                    else
                    {
                        //C is extreme.
                        if (dotA > dotB)
                        {
                            //A is second most extreme.
                            if (Math.Abs(dotC - dotB) < faceEpsilon)
                            {
                                //The normal is basically a face normal.  This can happen at the edges occasionally.
                                return TriangleConvexPairTester.VoronoiRegion.ABC;
                            }
                            else
                            {
                                Vector3.Subtract(ref localTriangleShape.vA, ref localTriangleShape.vC, out edgeDirection);
                                Vector3.Dot(ref edgeDirection, ref contact.Normal, out edgeDot);
                                if (edgeDot * edgeDot < edgeDirection.LengthSquared() * edgeEpsilon)
                                    return TriangleConvexPairTester.VoronoiRegion.AC;
                                else
                                    return TriangleConvexPairTester.VoronoiRegion.C;
                            }
                        }
                        else
                        {
                            //B is second most extreme.
                            if (Math.Abs(dotC - dotA) < faceEpsilon)
                            {
                                //The normal is basically a face normal.  This can happen at the edges occasionally.
                                return TriangleConvexPairTester.VoronoiRegion.ABC;
                            }
                            else
                            {
                                Vector3.Subtract(ref localTriangleShape.vB, ref localTriangleShape.vC, out edgeDirection);
                                Vector3.Dot(ref edgeDirection, ref contact.Normal, out edgeDot);
                                if (edgeDot * edgeDot < edgeDirection.LengthSquared() * edgeEpsilon)
                                    return TriangleConvexPairTester.VoronoiRegion.BC;
                                else
                                    return TriangleConvexPairTester.VoronoiRegion.C;
                            }
                        }
                    }
                case TriangleConvexPairTester.CollisionState.Plane:
                    //This can only happen if it's actually from the plane (and thus a face contact)
                    //OR it was an escape attempt, which will only happen with a contact when 
                    //the contact is found to be on the face.
                    return TriangleConvexPairTester.VoronoiRegion.ABC;

                default:
                    return pairTester.GetVoronoiRegion(ref contact.Position);
            }




        }

        protected void GetNormal(out Vector3 normal)
        {
            //Compute the normal of the triangle in the current convex's local space.
            //Note its reliance on the local triangle shape.  It must be initialized to the correct values before this is called.
            Vector3 AB, AC;
            Vector3.Subtract(ref localTriangleShape.vB, ref localTriangleShape.vA, out AB);
            Vector3.Subtract(ref localTriangleShape.vC, ref localTriangleShape.vA, out AC);
            Vector3.Cross(ref AB, ref AC, out normal);
            float dot;
            Vector3.Dot(ref normal, ref localTriangleShape.vA, out dot);
            //Calibrate the normal so that it points toward the convex.
            if (dot < 0)
                Vector3.Negate(ref normal, out normal);
        }

        bool AnalyzeCandidate(ref TriangleIndices indices, TriangleConvexPairTester pairTester, ref ContactData contact)
        {
            switch (GetRegion(pairTester, ref contact))
            {
                case TriangleConvexPairTester.VoronoiRegion.A:
                    //Add the contact.
                    VertexContact vertexContact;
                    vertexContact.ContactData = contact;
                    vertexContact.Vertex = indices.A;
                    vertexContact.ShouldCorrect = pairTester.state == TriangleConvexPairTester.CollisionState.Deep;
                    if (vertexContact.ShouldCorrect)
                        GetNormal(out vertexContact.CorrectedNormal);
                    else
                        vertexContact.CorrectedNormal = new Vector3();
                    vertexContacts.Add(ref vertexContact);

                    //Block all of the other voronoi regions.
                    blockedEdgeRegions.Add(new Edge(indices.A, indices.B));
                    blockedEdgeRegions.Add(new Edge(indices.B, indices.C));
                    blockedEdgeRegions.Add(new Edge(indices.A, indices.C));
                    blockedVertexRegions.Add(indices.B);
                    blockedVertexRegions.Add(indices.C);
                    break;
                case TriangleConvexPairTester.VoronoiRegion.B:
                    //Add the contact.
                    vertexContact.ContactData = contact;
                    vertexContact.Vertex = indices.B;
                    vertexContact.ShouldCorrect = pairTester.state == TriangleConvexPairTester.CollisionState.Deep;
                    if (vertexContact.ShouldCorrect)
                        GetNormal(out vertexContact.CorrectedNormal);
                    else
                        vertexContact.CorrectedNormal = new Vector3();
                    vertexContacts.Add(ref vertexContact);

                    //Block all of the other voronoi regions.
                    blockedEdgeRegions.Add(new Edge(indices.A, indices.B));
                    blockedEdgeRegions.Add(new Edge(indices.B, indices.C));
                    blockedEdgeRegions.Add(new Edge(indices.A, indices.C));
                    blockedVertexRegions.Add(indices.A);
                    blockedVertexRegions.Add(indices.C);
                    break;
                case TriangleConvexPairTester.VoronoiRegion.C:
                    //Add the contact.
                    vertexContact.ContactData = contact;
                    vertexContact.Vertex = indices.C;
                    vertexContact.ShouldCorrect = pairTester.state == TriangleConvexPairTester.CollisionState.Deep;
                    if (vertexContact.ShouldCorrect)
                        GetNormal(out vertexContact.CorrectedNormal);
                    else
                        vertexContact.CorrectedNormal = new Vector3();
                    vertexContacts.Add(ref vertexContact);

                    //Block all of the other voronoi regions.
                    blockedEdgeRegions.Add(new Edge(indices.A, indices.B));
                    blockedEdgeRegions.Add(new Edge(indices.B, indices.C));
                    blockedEdgeRegions.Add(new Edge(indices.A, indices.C));
                    blockedVertexRegions.Add(indices.A);
                    blockedVertexRegions.Add(indices.C);
                    break;
                case TriangleConvexPairTester.VoronoiRegion.AB:
                    //Add the contact.
                    EdgeContact edgeContact;
                    edgeContact.Edge = new Edge(indices.A, indices.B);
                    edgeContact.ContactData = contact;
                    edgeContact.ShouldCorrect = pairTester.state == TriangleConvexPairTester.CollisionState.Deep;
                    if (edgeContact.ShouldCorrect)
                        GetNormal(out edgeContact.CorrectedNormal);
                    else
                        edgeContact.CorrectedNormal = new Vector3();
                    edgeContacts.Add(ref edgeContact);

                    //Block all of the other voronoi regions.
                    blockedEdgeRegions.Add(new Edge(indices.B, indices.C));
                    blockedEdgeRegions.Add(new Edge(indices.A, indices.C));
                    blockedVertexRegions.Add(indices.A);
                    blockedVertexRegions.Add(indices.B);
                    blockedVertexRegions.Add(indices.C);
                    break;
                case TriangleConvexPairTester.VoronoiRegion.AC:
                    //Add the contact.
                    edgeContact.Edge = new Edge(indices.A, indices.C);
                    edgeContact.ContactData = contact;
                    edgeContact.ShouldCorrect = pairTester.state == TriangleConvexPairTester.CollisionState.Deep;
                    if (edgeContact.ShouldCorrect)
                        GetNormal(out edgeContact.CorrectedNormal);
                    else
                        edgeContact.CorrectedNormal = new Vector3();
                    edgeContacts.Add(ref edgeContact);

                    //Block all of the other voronoi regions.
                    blockedEdgeRegions.Add(new Edge(indices.A, indices.B));
                    blockedEdgeRegions.Add(new Edge(indices.B, indices.C));
                    blockedVertexRegions.Add(indices.A);
                    blockedVertexRegions.Add(indices.B);
                    blockedVertexRegions.Add(indices.C);
                    break;
                case TriangleConvexPairTester.VoronoiRegion.BC:
                    //Add the contact.
                    edgeContact.Edge = new Edge(indices.B, indices.C);
                    edgeContact.ContactData = contact;
                    edgeContact.ShouldCorrect = pairTester.state == TriangleConvexPairTester.CollisionState.Deep;
                    if (edgeContact.ShouldCorrect)
                        GetNormal(out edgeContact.CorrectedNormal);
                    else
                        edgeContact.CorrectedNormal = new Vector3();
                    edgeContacts.Add(ref edgeContact);

                    //Block all of the other voronoi regions.
                    blockedEdgeRegions.Add(new Edge(indices.A, indices.B));
                    blockedEdgeRegions.Add(new Edge(indices.A, indices.C));
                    blockedVertexRegions.Add(indices.A);
                    blockedVertexRegions.Add(indices.B);
                    blockedVertexRegions.Add(indices.C);
                    break;
                default:
                    //Block all of the other voronoi regions.
                    blockedEdgeRegions.Add(new Edge(indices.A, indices.B));
                    blockedEdgeRegions.Add(new Edge(indices.B, indices.C));
                    blockedEdgeRegions.Add(new Edge(indices.A, indices.C));
                    blockedVertexRegions.Add(indices.A);
                    blockedVertexRegions.Add(indices.B);
                    blockedVertexRegions.Add(indices.C);
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
            supplement.LocalOffsetB = contactCandidate.Position;
            //RigidTransform.TransformByInverse(ref contactCandidate.Position, ref mesh.worldTransform, out supplement.LocalOffsetB);
            supplementData.Add(ref supplement);
            base.Add(ref contactCandidate);
        }

        protected override void Remove(int contactIndex)
        {
            supplementData.RemoveAt(contactIndex);
            base.Remove(contactIndex);
        }


        private bool IsContactUnique(ref ContactData contactCandidate)
        {

            float distanceSquared;
            for (int i = 0; i < contacts.count; i++)
            {
                Vector3.DistanceSquared(ref contacts.Elements[i].Position, ref contactCandidate.Position, out distanceSquared);
                if (distanceSquared < CollisionDetectionSettings.ContactMinimumSeparationDistanceSquared)
                {
                    //This is a nonconvex manifold.  There will be times where a an object will be shoved into a corner such that
                    //a single position will have two reasonable normals.  If the normals aren't mostly aligned, they should NOT be considered equivalent.
                    Vector3.Dot(ref contacts.Elements[i].Normal, ref contactCandidate.Normal, out distanceSquared);
                    if (Math.Abs(distanceSquared) >= .9)
                    {
                        //Update the existing 'redundant' contact with the new information.
                        //This works out because the new contact is the deepest contact according to the previous collision detection iteration.
                        contacts.Elements[i].Normal = contactCandidate.Normal;
                        contacts.Elements[i].Position = contactCandidate.Position;
                        contacts.Elements[i].PenetrationDepth = contactCandidate.PenetrationDepth;
                        supplementData.Elements[i].BasePenetrationDepth = contactCandidate.PenetrationDepth;
                        RigidTransform.TransformByInverse(ref contactCandidate.Position, ref convex.worldTransform, out supplementData.Elements[i].LocalOffsetA);
                        supplementData.Elements[i].LocalOffsetB = contactCandidate.Position;
                        //RigidTransform.TransformByInverse(ref contactCandidate.Position, ref mesh.worldTransform, out supplementData.Elements[i].LocalOffsetB);
                        return false;
                    }
                }
            }
            for (int i = 0; i < candidatesToAdd.count; i++)
            {
                Vector3.DistanceSquared(ref candidatesToAdd.Elements[i].Position, ref contactCandidate.Position, out distanceSquared);
                if (distanceSquared < CollisionDetectionSettings.ContactMinimumSeparationDistanceSquared)
                {
                    //This is a nonconvex manifold.  There will be times where a an object will be shoved into a corner such that
                    //a single position will have two reasonable normals.  If the normals aren't mostly aligned, they should NOT be considered equivalent.
                    Vector3.Dot(ref candidatesToAdd.Elements[i].Normal, ref contactCandidate.Normal, out distanceSquared);
                    if (Math.Abs(distanceSquared) >= .9)
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


        ///<summary>
        /// Cleans up the manifold.
        ///</summary>
        public override void CleanUp()
        {
            supplementData.Clear();
            contacts.Clear();
            convex = null;
            foreach (KeyValuePair<TriangleIndices, TriangleConvexPairTester> pair in activePairTesters)
            {
                pair.Value.CleanUp();
                pairTestersPool.GiveBack(pair.Value);
            }
            activePairTesters.Clear();
            CleanUpOverlappingTriangles();
            base.CleanUp();
        }

        struct Edge : IEquatable<Edge>
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
