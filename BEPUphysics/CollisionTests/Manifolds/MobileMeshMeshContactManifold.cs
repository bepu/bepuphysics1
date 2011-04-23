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
using System.Diagnostics;
using BEPUphysics.Entities;

namespace BEPUphysics.CollisionTests.Manifolds
{


    ///<summary>
    /// Manages persistent contact data between a triangle mesh and a convex.
    ///</summary>
    public abstract class MobileMeshMeshContactManifold : ContactManifold
    {

        protected class MeshConvexPair
        {
            internal TriangleMeshConvexPairTester tester;
            internal TriangleCollidable triangle = new TriangleCollidable();
            internal TriangleShape shape = new TriangleShape();

            public MeshConvexPair()
            {
                triangle.shape = shape;
                triangle.worldTransform.Orientation = Quaternion.Identity;
            }

            internal void Initialize(MobileMeshCollidable mesh, TriangleShape shape, Entity shapeOwner)
            {
                triangle.entity = shapeOwner;
                //tester.Initialize(mesh, triangle);
            }

            internal void CleanUp()
            {
                triangle.entity = null;
                tester.CleanUp();
            }

          

        }


        protected abstract void UpdatePairs();

        protected RawValueList<ContactSupplementData> supplementData = new RawValueList<ContactSupplementData>(4);
        protected RawList<MeshConvexPair> activePairTesters = new RawList<MeshConvexPair>(20);
        RawValueList<ContactData> candidatesToAdd;
        RawValueList<ContactData> reducedCandidates = new RawValueList<ContactData>();

        UnsafeResourcePool<MeshConvexPair> pairTestersPool = new UnsafeResourcePool<MeshConvexPair>(4);

        protected MobileMeshCollidable mobileMesh;


        ///<summary>
        /// Constructs a new contact manifold.
        ///</summary>
        protected MobileMeshMeshContactManifold()
        {
            contacts = new RawList<Contact>(4);
            unusedContacts = new UnsafeResourcePool<Contact>(4);
            contactIndicesToRemove = new RawList<int>(4);
            candidatesToAdd = new RawValueList<ContactData>(1);
        }

        protected virtual RigidTransform MeshTransform
        {
            get
            {
                return RigidTransform.Identity;
            }
        }



        ///<summary>
        /// Updates the manifold.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        public override void Update(float dt)
        {
            ////First, refresh all existing contacts.  This is an incremental manifold.
            //var transform = MeshTransform;
            //ContactRefresher.ContactRefresh(contacts, supplementData, ref mobileMesh.worldTransform, ref transform, contactIndicesToRemove);
            //RemoveQueuedContacts();

            //CleanUpOverlappingTriangles();
            ////Get all the overlapped triangle indices.
            //int triangleCount = FindOverlappingTriangles(dt);

            //Matrix3X3 orientation;
            //Matrix3X3.CreateFromQuaternion(ref mobileMesh.worldTransform.Orientation, out orientation);
            //for (int i = 0; i < triangleCount; i++)
            //{
            //    //Initialize the local triangle.
            //    TriangleIndices indices;
            //    ConfigureTriangle(i, out indices);

            //    //Find a pairtester for the triangle.
            //    TriangleConvexPairTester pairTester;
            //    if (!activePairTesters.TryGetValue(indices, out pairTester))
            //    {
            //        pairTester = pairTestersPool.Take();
            //        pairTester.Initialize(mobileMesh.Shape, localTriangleShape);
            //        activePairTesters.Add(indices, pairTester);
            //    }
            //    pairTester.Updated = true;


            //    //Put the triangle into the local space of the convex.
            //    Vector3.Subtract(ref localTriangleShape.vA, ref mobileMesh.worldTransform.Position, out localTriangleShape.vA);
            //    Vector3.Subtract(ref localTriangleShape.vB, ref mobileMesh.worldTransform.Position, out localTriangleShape.vB);
            //    Vector3.Subtract(ref localTriangleShape.vC, ref mobileMesh.worldTransform.Position, out localTriangleShape.vC);
            //    Matrix3X3.TransformTranspose(ref localTriangleShape.vA, ref orientation, out localTriangleShape.vA);
            //    Matrix3X3.TransformTranspose(ref localTriangleShape.vB, ref orientation, out localTriangleShape.vB);
            //    Matrix3X3.TransformTranspose(ref localTriangleShape.vC, ref orientation, out localTriangleShape.vC);


            //    //Now, generate a contact between the two shapes.
            //    ContactData contact;
            //    if (pairTester.GenerateContactCandidate(out contact))
            //    {
            //        if (UseImprovedBoundaryHandling)
            //        {
            //            if (AnalyzeCandidate(ref indices, pairTester, ref contact))
            //            {
            //                AddLocalContact(ref contact, ref orientation);
            //            }
            //        }
            //        else
            //        {
            //            AddLocalContact(ref contact, ref orientation);
            //        }
            //    }

            //    //Get the voronoi region from the contact candidate generation.  Possibly just recalculate, since most of the systems don't calculate it.
            //    //Depending on which voronoi region it is in (Switch on enumeration), identify the indices composing that region.  For face contacts, don't bother- just add it if unique.
            //    //For AB, AC, or BC, add an Edge to the blockedEdgeRegions set with the corresponding indices.
            //    //For A, B, or C, add the index of the vertex to the blockedVertexRegions set.
            //    //If the edge/vertex is already present in the set, then DO NOT add the contact.
            //    //When adding a contact, add ALL other voronoi regions to the blocked sets. 

            //}





            ////Remove stale pair testers.
            ////This will only remove 8 stale ones per frame, but it doesn't really matter.
            ////VERY rarely will there be more than 8 in a single frame, and they will be immediately taken care of in the subsequent frame.
            //var toRemove = new TinyList<TriangleIndices>();
            //foreach (KeyValuePair<TriangleIndices, TriangleConvexPairTester> pair in activePairTesters)
            //{
            //    if (!pair.Value.Updated)
            //    {
            //        if (!toRemove.Add(pair.Key))
            //            break;
            //    }
            //    else
            //        pair.Value.Updated = false;
            //}



            //for (int i = toRemove.count - 1; i >= 0; i--)
            //{
            //    var pairTester = activePairTesters[toRemove[i]];
            //    pairTester.CleanUp();
            //    pairTestersPool.GiveBack(pairTester);
            //    activePairTesters.Remove(toRemove[i]);
            //}


            ////Some child types will want to do some extra post processing on the manifold.        
            //ProcessCandidates(candidatesToAdd);

            ////Check if adding the new contacts would overflow the manifold.
            //if (contacts.count + candidatesToAdd.count > 4)
            //{
            //    //Adding all the contacts would overflow the manifold.  Reduce to the best subset.
            //    ContactReducer.ReduceContacts(contacts, candidatesToAdd, contactIndicesToRemove, reducedCandidates);
            //    RemoveQueuedContacts();
            //    for (int i = reducedCandidates.count - 1; i >= 0; i--)
            //    {
            //        Add(ref reducedCandidates.Elements[i]);
            //        reducedCandidates.RemoveAt(i);
            //    }
            //}
            //else if (candidatesToAdd.count > 0)
            //{
            //    //Won't overflow the manifold, so just toss it in PROVIDED that it isn't too close to something else.
            //    for (int i = 0; i < candidatesToAdd.count; i++)
            //    {
            //        Add(ref candidatesToAdd.Elements[i]);
            //    }
            //}




            //candidatesToAdd.Clear();

        }

        void AddLocalContact(ref ContactData contact, ref Matrix3X3 orientation)
        {
            //Put the contact into world space.
            Matrix3X3.Transform(ref contact.Position, ref orientation, out contact.Position);
            Vector3.Add(ref contact.Position, ref mobileMesh.worldTransform.Position, out contact.Position);
            Matrix3X3.Transform(ref contact.Normal, ref orientation, out contact.Normal);
            //Check to see if the contact is unique before proceeding.
            if (IsContactUnique(ref contact))
            {
                candidatesToAdd.Add(ref contact);
            }
        }


        protected override void Add(ref ContactData contactCandidate)
        {
            ContactSupplementData supplement;
            supplement.BasePenetrationDepth = contactCandidate.PenetrationDepth;
            //The closest point method computes the local space versions before transforming to world... consider cutting out the middle man
            RigidTransform.TransformByInverse(ref contactCandidate.Position, ref mobileMesh.worldTransform, out supplement.LocalOffsetA);
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


        private bool IsContactUnique(ref ContactData contactCandidate)
        {

            float distanceSquared;
            RigidTransform meshTransform = MeshTransform;
            for (int i = 0; i < contacts.count; i++)
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
                        RigidTransform.TransformByInverse(ref contactCandidate.Position, ref mobileMesh.worldTransform, out supplementData.Elements[i].LocalOffsetA);
                        RigidTransform.TransformByInverse(ref contactCandidate.Position, ref meshTransform, out supplementData.Elements[i].LocalOffsetB);
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

        protected virtual void ProcessCandidates(RawValueList<ContactData> candidates)
        {

        }


        ///<summary>
        /// Cleans up the manifold.
        ///</summary>
        public override void CleanUp()
        {
            supplementData.Clear();
            contacts.Clear();
            mobileMesh = null;
            //foreach (KeyValuePair<TriangleIndices, TriangleConvexPairTester> pair in activePairTesters)
            //{
            //    pair.Value.CleanUp();
            //    pairTestersPool.GiveBack(pair.Value);
            //}
            //activePairTesters.Clear();
            //CleanUpOverlappingTriangles();
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
