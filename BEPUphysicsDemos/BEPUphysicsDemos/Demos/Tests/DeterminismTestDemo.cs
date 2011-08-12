using BEPUphysics.Collidables;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUphysics.PositionUpdating;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;
using System.Diagnostics;
using System.Collections.Generic;
using System;
using BEPUphysics.MathExtensions;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.DataStructures;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.Collidables.MobileCollidables;
using BEPUphysics.CollisionShapes;
using BEPUphysics.Settings;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.BroadPhaseSystems;
using BEPUphysics.Constraints;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using BEPUphysics.Constraints.SolverGroups;
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUphysics.CollisionTests;
using BEPUphysics;
using BEPUphysics.EntityStateManagement;
using BEPUphysics.ResourceManagement;

namespace BEPUphysicsDemos.Demos.Tests
{
    /// <summary>
    /// Demo showing a wall of blocks stacked up.
    /// </summary>
    public class DeterminismTestDemo : StandardDemo
    {
        List<Entity> entities = new List<Entity>();
        int numFrames;

        static int numRuns = 0;
        static List<MotionState> motionState = new List<MotionState>();

        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public DeterminismTestDemo(DemosGame game)
            : base(game)
        {

            //while (Space.ThreadManager.ThreadCount > 0)
            //    Space.ThreadManager.RemoveThread();
            //Resources.ResetPools();

            //Space.BroadPhase.AllowMultithreading = false;
            //Space.Solver.AllowMultithreading = false;
            ////Space.NarrowPhase.AllowMultithreading = false;

            ////Space.BoundingBoxUpdater.AllowMultithreading = false;
            ////Space.DeactivationManager.AllowMultithreading = false;
            ////Space.ForceUpdater.AllowMultithreading = false;
            ////Space.PositionUpdater.AllowMultithreading = false;

            //Space.BeforeNarrowPhaseUpdateables.Enabled = false;
            //Space.DuringForcesUpdateables.Enabled = false;
            //Space.EndOfFrameUpdateables.Enabled = false;
            //Space.EndOfTimeStepUpdateables.Enabled = false;

            //Space.BroadPhase = new BruteForce();
            //Space.NarrowPhase.BroadPhaseOverlaps = Space.BroadPhase.Overlaps;

            //SolverSettings.DefaultMinimumImpulse = 0;
            //SolverSettings.DefaultMinimumIterations = 10;

            Entity ground = new MorphableEntity(new BoxShape(50, 1, 50));
            Space.Add(ground);

            Vector3[] vertices;
            int[] indices;
            TriangleMesh.GetVerticesAndIndicesFromModel(Game.Content.Load<Model>("playground"), out vertices, out indices);
            var mesh = new StaticMesh(vertices, indices, new AffineTransform(new Vector3(50, -20, 0)));
            Space.Add(mesh);
            game.ModelDrawer.Add(mesh);

            for (int i = 0; i < 100; i++)
            {
                Entity e = new Box(new Vector3(.1f * i, 1 * i + 1, 0), 1, 1, 1, 1);
                //Entity e = new Capsule(new Vector3(.1f * i, 1 * i + 1, 0), .5f, .5f, 1);
                //Entity e = new Sphere(new Vector3(.1f * i, 1 * i + 1, 0), .5f, 1);
                e.ActivityInformation.IsAlwaysActive = true;
                entities.Add(e);
                e.CollisionInformation.Tag = i;
                Space.Add(e);
            }
            for (int i = 0; i < 200; i++)
            {
                //Entity e = new Box(new Vector3(.1f * i, 1 * i + 1, 2), 1, 1, 1, 1);
                Entity e = new Capsule(new Vector3(.1f * i, 1 * i + 1, 2), .5f, .5f, 1);
                //Entity e = new Sphere(new Vector3(.1f * i, 1 * i + 1, 2), .5f, 1);
                e.ActivityInformation.IsAlwaysActive = true;
                entities.Add(e);
                e.CollisionInformation.Tag = i;
                Space.Add(e);
            }
            for (int i = 0; i < 300; i++)
            {
                //Entity e = new Box(new Vector3(.1f * i, 1 * i + 1, 4), 1, 1, 1, 1);
                //Entity e = new Capsule(new Vector3(.1f * i, 1 * i + 1, 4), .5f, .5f, 1);
                Entity e = new Sphere(new Vector3(.1f * i, 1 * i + 1, 4), .5f, 1);
                e.ActivityInformation.IsAlwaysActive = true;
                entities.Add(e);
                e.CollisionInformation.Tag = i;
                Space.Add(e);
            }

        }

        bool allowUpdates = true;

        public override void Update(float dt)
        {
            if (allowUpdates)
            {

                base.Update(dt);
                numFrames++;


                if (numFrames > 300)
                {
                    numRuns++;
                    if (numRuns > 1)
                    {
                        for (int i = 0; i < entities.Count; i++)
                        {
                            if (!motionState[i].Equals(entities[i].MotionState))
                            {
                                //Determinism failed.
                                Debug.WriteLine("Break");
                            }
                        }

                        //Determinism success! 
                    }
                    motionState.Clear();
                    foreach (var e in entities)
                        motionState.Add(e.MotionState);
                    allowUpdates = false;



                }
                else
                {

                    if (numRuns == 0)
                    {
                        if (numFrames > frameOverlaps.Count)
                        {
                            frameOverlaps.Add(new List<BroadPhaseOverlap>(Space.BroadPhase.Overlaps));
                            //var entriesList = ((BruteForce)Space.BroadPhase).entries;
                            //entries.Add(new List<BroadPhaseEntry>(entriesList));
                            //var aabblist = new List<BoundingBox>();
                            //for (int i = 0; i < entriesList.Count; i++)
                            //{
                            //    aabblist.Add(entriesList[i].BoundingBox);
                            //}
                            //boundingBoxes.Add(aabblist);
                            var motionStatesList = new List<MotionState>();
                            for (int i = 0; i < Space.Entities.Count; i++)
                            {
                                motionStatesList.Add(Space.Entities[i].MotionState);
                            }
                            motionStates.Add(motionStatesList);

                            var contactDataList = new List<ContactInfo>();
                            for (int i = 0; i < Space.NarrowPhase.Pairs.Count; i++)
                            {
                                var pair = Space.NarrowPhase.Pairs[i] as CollidablePairHandler;
                                if (pair != null)
                                {
                                    foreach (var contact in pair.Contacts)
                                    {
                                        var c = new ContactInfo();
                                        c.data.Id = contact.Contact.Id;
                                        c.data.Normal = contact.Contact.Normal;
                                        c.data.PenetrationDepth = contact.Contact.PenetrationDepth;
                                        c.data.Position = contact.Contact.Position;
                                        c.frictionForce = contact.FrictionForce;
                                        c.normalForce = contact.NormalForce;
                                        c.velocity = contact.RelativeVelocity;
                                        contactDataList.Add(c);
                                    }
                                }
                            }
                            contacts.Add(contactDataList);

                        }
                        else
                        {
                            //frameOverlaps[numFrames - 1] = new List<BroadPhaseOverlap>(Space.BroadPhase.Overlaps);
                            //var entriesList = ((BruteForce)Space.BroadPhase).entries;
                            //entries[numFrames - 1] = new List<BroadPhaseEntry>(entriesList);
                            //for (int i = 0; i < boundingBoxes[numFrames - 1].Count; i++)
                            //{
                            //    boundingBoxes[numFrames - 1][i] = entriesList[i].BoundingBox;
                            //}
                            //for (int i = 0; i < Space.Entities.Count; i++)
                            //{
                            //    motionStates[numFrames - 1][i] = Space.Entities[i].MotionState;
                            //}
                        }
                    }
                    else
                    {
                        //Test determinism of entries.
                        //var currentEntries = (Space.BroadPhase as BruteForce).entries;
                        //if (entries[numFrames - 1].Count != currentEntries.Count)
                        //    Debug.WriteLine("Determinism failed.");
                        //for (int i = 0; i < currentEntries.Count; i++)
                        //{
                        //    if (!(currentEntries[i].Tag == null && entries[numFrames - 1][i].Tag == null) && (int)currentEntries[i].Tag != (int)entries[numFrames - 1][i].Tag)
                        //        Debug.WriteLine("Determinism failed.");
                        //}
                        //Test determinism of bounding boxes.
                        //for (int i = 0; i < currentEntries.Count; i++)
                        //{
                        //    if (boundingBoxes[numFrames - 1][i] != currentEntries[i].BoundingBox)
                        //        Debug.WriteLine("Determinism failed.");
                        //}
                        //Test determinism of broadphase.
                        if (frameOverlaps[numFrames - 1].Count != Space.BroadPhase.Overlaps.Count)
                            Debug.WriteLine("Determinism failed.");
                        for (int i = 0; i < frameOverlaps[numFrames - 1].Count; i++)
                        {
                            if (frameOverlaps[numFrames - 1][i].EntryA.Tag != null && frameOverlaps[numFrames - 1][i].EntryB.Tag != null)
                            {
                                if (!(((int)frameOverlaps[numFrames - 1][i].EntryA.Tag == (int)Space.BroadPhase.Overlaps[i].EntryA.Tag &&
                                   (int)frameOverlaps[numFrames - 1][i].EntryB.Tag == (int)Space.BroadPhase.Overlaps[i].EntryB.Tag) ||
                                   ((int)frameOverlaps[numFrames - 1][i].EntryB.Tag == (int)Space.BroadPhase.Overlaps[i].EntryA.Tag &&
                                   (int)frameOverlaps[numFrames - 1][i].EntryA.Tag == (int)Space.BroadPhase.Overlaps[i].EntryB.Tag)))
                                {
                                    Debug.WriteLine("Determinism failed.");
                                }
                            }
                        }
                        //Test determinism of narrowphase.
                        var contactDataList = new List<ContactInfo>();
                        for (int i = 0; i < Space.NarrowPhase.Pairs.Count; i++)
                        {
                            var pair = Space.NarrowPhase.Pairs[i] as CollidablePairHandler;
                            if (pair != null)
                            {
                                foreach (var contact in pair.Contacts)
                                {
                                    var c = new ContactInfo();
                                    c.data.Id = contact.Contact.Id;
                                    c.data.Normal = contact.Contact.Normal;
                                    c.data.PenetrationDepth = contact.Contact.PenetrationDepth;
                                    c.data.Position = contact.Contact.Position;
                                    c.frictionForce = contact.FrictionForce;
                                    c.normalForce = contact.NormalForce;
                                    c.velocity = contact.RelativeVelocity;
                                    contactDataList.Add(c);
                                }
                            }
                        }
                        if (contactDataList.Count != contacts[numFrames - 1].Count)
                            Debug.WriteLine("Determinism failed.");
                        //for (int i = 0; i < contactDataList.Count; i++)
                        //{
                        //    if (!contacts[numFrames - 1][i].Equals(contactDataList[i]))
                        //        Debug.WriteLine("Determinism failed.");
                        //}
                        //Test determinism of motion states.
                        for (int i = 0; i < Space.Entities.Count; i++)
                        {
                            if (!motionStates[numFrames - 1][i].Equals(Space.Entities[i].MotionState))
                                Debug.WriteLine("Determinism failed.");
                        }



                    }
                }
            }
        }

        static List<List<ContactInfo>> contacts = new List<List<ContactInfo>>();
        static List<List<MotionState>> motionStates = new List<List<MotionState>>();
        static List<List<BoundingBox>> boundingBoxes = new List<List<BoundingBox>>();
        static List<List<BroadPhaseEntry>> entries = new List<List<BroadPhaseEntry>>();
        static List<List<BroadPhaseOverlap>> frameOverlaps = new List<List<BroadPhaseOverlap>>();


        struct ContactInfo : IEquatable<ContactInfo>
        {
            internal ContactData data;
            internal float normalForce;
            internal float frictionForce;
            internal Vector3 velocity;



            public bool Equals(ContactInfo other)
            {
                return data.Equals(other.data) &&
                    other.frictionForce == frictionForce &&
                    other.normalForce == normalForce &&
                    other.velocity == velocity;

            }
        }



        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Test3"; }
        }

    }
}