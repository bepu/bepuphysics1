using BEPUphysics.Entities.Prefabs;
using BEPUutilities;
using BEPUphysics.CollisionShapes;
using BEPUutilities.DataStructures;
using Microsoft.Xna.Framework.Graphics;
using BEPUphysics.BroadPhaseEntries;
using System;
using BEPUphysics.CollisionShapes.ConvexShapes;
using System.Collections.Generic;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;

namespace BEPUphysicsDemos.Demos.Extras.Tests
{
    /// <summary>
    /// Demo showing a whole bunch of efficient static geometry that gets shuffled in and out of the StaticGroup at load time.
    /// </summary>
    public class MutableStaticGroupTestDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public MutableStaticGroupTestDemo(DemosGame game)
            : base(game)
        {


            //Creating a bunch of separate StaticMeshes or kinematic Entity objects for an environment can pollute the broad phase.
            //This is because the broad phase implementation doesn't have guarantees about what elements can collide, so it has to
            //traverse the acceleration structure all the way down to pairs to figure it out.  That can get expensive!

            //Individual objects, like StaticMeshes, can have very complicated geometry without hurting the broad phase because the broad phase
            //has no knowledge of the thousands of triangles in the mesh.  The StaticMesh itself knows that the triangles within the mesh
            //never need to collide, so it never needs to test them against each other.

            //Similarly, the StaticGroup can be given a bunch of separate collidables.  The broad phase doesn't directly know about these child collidables-
            //it only sees the StaticGroup.  The StaticGroup knows that the things inside it can't ever collide with each other, so no tests are needed.
            //This avoids the performance problem!

            //To demonstrate, we'll be creating a set of static objects and giving them to a group to manage.
            var collidables = new List<Collidable>();

            //Start with a whole bunch of boxes.  These are entity collidables, but without entities!
            float xSpacing = 6;
            float ySpacing = 6;
            float zSpacing = 6;


            //NOTE: You might notice this demo takes a while to start, especially on the Xbox360.  Do not fear!  That's due to the creation of the graphics data, not the physics.
            //The physics can handle over 100,000 static objects pretty easily.  The graphics, not so much :)
            //Try disabling the game.ModelDrawer.Add() lines and increasing the number of static objects.  
            int xCount = 15;
            int yCount = 7;
            int zCount = 15;


            var random = new Random(5);
            for (int i = 0; i < xCount; i++)
            {
                for (int j = 0; j < yCount; j++)
                {
                    for (int k = 0; k < zCount; k++)
                    {
                        //Create a transform and the instance of the mesh.
                        var collidable = new ConvexCollidable<BoxShape>(new BoxShape((float)random.NextDouble() * 6 + .5f, (float)random.NextDouble() * 6 + .5f, (float)random.NextDouble() * 6 + .5f));

                        //This EntityCollidable isn't associated with an entity, so we must manually tell it where to sit by setting the WorldTransform.
                        //This also updates its bounding box.
                        collidable.WorldTransform = new RigidTransform(
                            new Vector3(i * xSpacing - xCount * xSpacing * .5f, j * ySpacing + 3, k * zSpacing - zCount * zSpacing * .5f),
                            Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble())), (float)random.NextDouble() * 100));

                        collidables.Add(collidable);
                    }
                }
            }


            //Now create a bunch of instanced meshes too.
            xSpacing = 6;
            ySpacing = 6;
            zSpacing = 6;

            xCount = 10;
            yCount = 2;
            zCount = 10;

            Vector3[] vertices;
            int[] indices;
            ModelDataExtractor.GetVerticesAndIndicesFromModel(game.Content.Load<Model>("fish"), out vertices, out indices);
            var meshShape = new InstancedMeshShape(vertices, indices);

            for (int i = 0; i < xCount; i++)
            {
                for (int j = 0; j < yCount; j++)
                {
                    for (int k = 0; k < zCount; k++)
                    {
                        //Create a transform and the instance of the mesh.
                        var transform = new AffineTransform(
                            new Vector3((float)random.NextDouble() * 6 + .5f, (float)random.NextDouble() * 6 + .5f, (float)random.NextDouble() * 6 + .5f),
                             Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble())), (float)random.NextDouble() * 100),
                            new Vector3(i * xSpacing - xCount * xSpacing * .5f, j * ySpacing + 50, k * zSpacing - zCount * zSpacing * .5f));
                        var mesh = new InstancedMesh(meshShape, transform);
                        //Making the triangles one-sided makes collision detection a bit more robust, since the backsides of triangles won't try to collide with things
                        //and 'pull' them back into the mesh.
                        mesh.Sidedness = TriangleSidedness.Counterclockwise;
                        collidables.Add(mesh);
                    }
                }
            }

            var ground = new ConvexCollidable<BoxShape>(new BoxShape(200, 1, 200));
            ground.WorldTransform = new RigidTransform(new Vector3(0, -3, 0), Quaternion.Identity);
            collidables.Add(ground);

            var group = new StaticGroup(collidables);
            var removed = new RawList<Collidable>();
            var contained = new RawList<Collidable>();
            group.Shape.CollidableTree.CollectLeaves(contained);
            for (int i = 0; i < 100000; ++i)
            {
                for (int collidableIndex = contained.Count - 1; collidableIndex >= 0; --collidableIndex)
                {
                    if (random.NextDouble() < 0.6)
                    {
                        group.Shape.Remove(contained[collidableIndex]);
                        removed.Add(contained[collidableIndex]);
                        contained.FastRemoveAt(collidableIndex);
                    }
                }

                for (int collidableIndex = removed.Count - 1; collidableIndex >= 0; --collidableIndex)
                {
                    if (random.NextDouble() < 0.4)
                    {
                        group.Shape.Add(removed[collidableIndex]);
                        contained.Add(removed[collidableIndex]);
                        removed.FastRemoveAt(collidableIndex);
                    }
                }
            }

            for (int i = 0; i < contained.Count; ++i)
            {
                game.ModelDrawer.Add(contained[i]);
            }
            Space.Add(group);




            //Create a bunch of dynamic boxes to drop on the staticswarm.
            xCount = 8;
            yCount = 3;
            zCount = 8;
            xSpacing = 3f;
            ySpacing = 5f;
            zSpacing = 3f;
            for (int i = 0; i < xCount; i++)
                for (int j = 0; j < zCount; j++)
                    for (int k = 0; k < yCount; k++)
                    {
                        Space.Add(new Box(new Vector3(
                                                 xSpacing * i - (xCount - 1) * xSpacing / 2f,
                                                 100 + k * (ySpacing),
                                                 2 + zSpacing * j - (zCount - 1) * zSpacing / 2f),
                                             1, 1, 1, 10));
                    }




            game.Camera.Position = new Vector3(0, 60, 90);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Static Group"; }
        }
    }
}