using BEPUphysics.Entities.Prefabs;
using Microsoft.Xna.Framework;
using BEPUphysics.Entities;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Collidables.MobileCollidables;
using System.Collections.Generic;
using BEPUphysics.CollisionShapes;
using BEPUphysics.MathExtensions;
using BEPUphysics.CollisionRuleManagement;
using BEPUphysics.Materials;
using BEPUphysics.Settings;

namespace BEPUphysicsDemos.Demos
{
    /// <summary>
    /// Demo showing a variety of alternate ways to construct entities.
    /// </summary>
    public class EntityConstructionDemo : StandardDemo
    {
        /// <summary>
        /// Constructs a new demo.
        /// </summary>
        /// <param name="game">Game owning this demo.</param>
        public EntityConstructionDemo(DemosGame game)
            : base(game)
        {
            //An entity is the basic physical simulation object in BEPUphysics.  It represents a rigid body which can be dynamic or kinematic.
            //Dynamic objects can bounce around and respond to collisions using forces.  Kinematic entities can move using velocity,
            //but effectively have infinite inertia and do not respond to collisions or forces.

            //For each of the below constructors, there are multiple overloads.  Half have a mass parameter, half do not.  If a mass parameter is specified,
            //the object is created as a dynamic object.  If a mass parameter is not specified, it is created as a kinematic object.  This state can be changed later
            //by using the BecomeDynamic/BecomeKinematic methods.


            //************************
            //There are a variety of ways to construct entities depending on your needs.
            //The most common approach is to use the 'prefab' types, like the Box, Sphere, ConvexHull, Cylinder, and so on (BEPUphysics.Entities.Prefabs namespace).
            //These provide a short cut to constructing the objects and generally provide convenience accessors to shape properties.

            //We'll start with such an entity for the kinematic ground.  Notice how it allows the definition of position and shape data all in the constructor.
            //It has other overloads that allow you to specify a mass (for a dynamic object) or a full MotionState instead of just a position.
            Box ground = new Box(new Vector3(0, -.5f, 0), 50, 1, 50);
            Space.Add(ground);

            //If you examine the ground's CollisionInformation property, you'll find that it is a generic method which returns a ConvexCollidable<BoxShape>.
            //So, in addition to having convenience properties, it also lets you go deeper into the object's data by knowing what the types are.


            //************************
            //Let's go one level up the hierarchy to Entity<T>.  This is the superclass of all prefab types.
            //The generic parameter is the type of the Collidable that the entity uses as its collision proxy.
            //It is also what allows prefab types' CollisionInformation property to return that type rather than EntityCollidable.

            Space.Add(new Entity<ConvexCollidable<SphereShape>>(new ConvexCollidable<SphereShape>(new SphereShape(1)), 1)
                          {
                              Position = new Vector3(-10, 1, 0)
                          });

            //That's a bit unwieldy, but it works.  Directly constructing entities in this manner isn't common or generally necessary.
            //If you want to have a CollisionInformation property that returns a specific type but don't want to use a specific prefab type, this is the way to go.

            //Note that the constructor itself does not include any position/motion state parameters nor does it include any direct shape data.  The position and other auxiliary
            //properties can be set after it's been constructed.  In this case, object initializer syntax is used to set the Position.
            //The shape data is included within the first EntityCollidable parameter which defines the entity's collision proxy.
            //The prefab types discussed previously all create such EntityCollidables internally using the data passed into the constructor.


            //************************
            //So let's move onto something a little more convenient.  Entity<T> has a non-generic parent Entity.  It still has a CollisionInformation property,
            //but it returns an EntityCollidable rather than a specific type.

            Space.Add(new Entity(new BoxShape(2, 2, 2), 1)
                {
                    Position = new Vector3(10, 1, 0)
                });

            //Much better! No more ugly generic type syntax pollution.  Notice that there are quite a few overloads in the Entity.
            //In addition to overloads that accept EntityCollidable instances (as used in the generic entity above), Entity provides constructors that
            //take shapes directly.  Internally, the entity can create its own EntityCollidable instance based on the shape.  This further simplifies the construction process.

            //An object initializer is once again used to set the position, since the constructors do not have all the convenience constructor frills of the prefab types.


            //************************
            //All of the above entity types have read-only CollisionInformation properties.  Once an entity of those types is created, it cannot change its fundamental shape.
            //A sphere may get bigger or smaller, but a sphere entity couldn't become a box entity.

            //That's where the MorphableEntity comes in.  

            var morphable = new MorphableEntity(new CylinderShape(2, 1), 1)
            {
                Position = new Vector3(10, 3, 0)
            };
            Space.Add(morphable);

            //It is constructed identically to its parent Entity class, but after being created, the EntityCollidable it uses can change!
            morphable.CollisionInformation = new ConvexCollidable<ConeShape>(new ConeShape(2, 1));

            //That's neat, but that collidable constructor with generic type is a bit ugly.  We don't care about doing any type-specific configuration
            //on the EntityCollidable itself, so we can just tell the shape to make us an EntityCollidable instance.  This is what the Entity/MorphableEntity constructors
            //do internally when given an EntityShape instead of an EntityCollidable (which in turn contains an EntityShape).
            morphable.CollisionInformation = new ConeShape(2, 1).GetCollidableInstance();

            //While this is arguably a bit prettier, its major benefit comes from the fact that you don't need to know the type of the shape to call GetCollidableInstance.
            //If it's an EntityShape, it can create an EntityCollidable for itself.



            //************************
            //That's it for the different types of entities.  Another very useful technique is to share shapes and initialization between entities.
            //Re-using convex hulls is a common use case.  They have a fairly expensive initialization, and they tend to be some of the larger shapes memory-wise.
            //Rather than having a thousand redundant copies of identical geometry, one shape can be made, one entity can be initialized using it, and the same data can be reused
            //for subsequent entities.

            //First, create the pointset that composes the convex hull.
            Vector3[] vertices = new Vector3[] 
            {
                new Vector3(-1,0,-1),
                new Vector3(-1,0,1),
                new Vector3(1,0,-1),
                new Vector3(1,0,1),
                new Vector3(0,2,0)             
            };
            //Construct the shape itself.
            var convexHullShape = new ConvexHullShape(vertices);

            //Create an entity using the shape.
            var convexHull = new Entity(convexHullShape, 2) { Position = new Vector3(0, 1, 0) };
            Space.Add(convexHull);
            //Create a bunch of other shapes using that first shape's properties.  Passing in the inertia and volume allows the new entities to avoid almost
            //all of the initialization costs.  Instead of using a first entity's data, this could also be pulled in from some external source (deserialized save data or something).
            for (int i = 1; i <= 10; i++)
            {
                Space.Add(new Entity(convexHullShape, convexHull.Mass, convexHull.LocalInertiaTensor, convexHull.Volume) { Position = new Vector3(0, 1 + i * 3, 0) });
            }

            //************************
            //All convex shapes are centered on their local origin.  If you create a box entity and assign its position to (0,1,0), the box shape's center will end up at (0,1,0).
            //For simple shapes like the box, that's always the case without doing any work just based on the definition of the shape.

            //More complicated shapes, like the ConvexHullShape, can be constructed from data which is not initially centered around the origin.  For example, consider these vertices:
            vertices = new Vector3[] 
            {
                new Vector3(-5,15,-1),
                new Vector3(-5,15,1),
                new Vector3(-3,15,-1),
                new Vector3(-3,15,1),
                new Vector3(-4,17,0),    
                new Vector3(-4,13,0)             
            };
            //The center of those points is obviously going to be very far from the origin.
            //When we construct a ConvexHullShape, the points get 'recentered.'
            convexHullShape = new ConvexHullShape(vertices);

            //If you look at the convexHullShape.Vertices list, you'll see that the points are relative to a local origin now.
            //So now it's centered on the local origin, as was needed.  But there's a problem- what if you wanted your convex hull entity to end up at the original vertex positions?
            //To get it there, we need to know the offset from the original vertices to the local vertices.
            //Fortunately, there is a constructor overload which provides it!
            Vector3 center;
            convexHullShape = new ConvexHullShape(vertices, out center);

            //Now, when the entity is created, we can position it at the original point's center.  The shape still contains local data, but the entity's location ends up putting the shape in the right spot.
            convexHull = new Entity(convexHullShape, 2) { Position = center };
            Space.Add(convexHull);


            //************************
            //Compound bodies are unique in that they allow you to specify groups of shapes to create a concave shape.
            //Here's the common simple approach to making a compound body, using a prefab type for now.
            CompoundBody body = new CompoundBody(new List<CompoundShapeEntry>()
            {
                new CompoundShapeEntry(new BoxShape(1, 1, 1), new Vector3(-7, 3, 8), 1),
                new CompoundShapeEntry(new BoxShape(1, 3, 1), new Vector3(-8, 2, 8), 5),
                new CompoundShapeEntry(new BoxShape(1, 1, 1), new Vector3(-9, 1, 8), 1)
            }, 10);
            Space.Add(body);
            //Each entry has a shape, a rigid transform (orientation and/or translation), and a weight.
            //The rigid transform defines the position of the shape in world space.  They will be recentered into local space within the CompoundShape itself (see previous convex hull example).
            //The 'weight' parameter does not correspond directly to mass, but rather to the contribution of that shape to the whole relative to other shapes.
            //It used to compute the center position of the shape.  When used by a dynamic entity, it is also used to compute a proper inertia tensor.  The resulting inertia tensor is scaled by the entity's actual mass.


            //************************
            //Just like shapes can be shared between entities, you can re-use a shape for multiple entries within a compound body.
            var compoundShape = new CompoundShape(new List<CompoundShapeEntry>()
            {
                new CompoundShapeEntry(convexHullShape, new Vector3(7, 3, 8), 1),
                new CompoundShapeEntry(convexHullShape, new RigidTransform(new Vector3(8, 2, 8), Quaternion.CreateFromAxisAngle(Vector3.Forward, MathHelper.PiOver2)), 5),
                new CompoundShapeEntry(convexHullShape, new Vector3(9, 1, 8), 1)
            });

            //And just like normal convex shapes can be shared, so can compound shapes!
            for (int i = 0; i < 3; i++)
                Space.Add(new Entity(compoundShape, 10) { Position = new Vector3(8, 10 + i * 4, 8) });

            //************************
            //You can also use compound shapes as subshapes, creating nested compounds.
            Space.Add(new Entity(new CompoundShape(new List<CompoundShapeEntry>()
            {
                new CompoundShapeEntry(compoundShape, new Vector3(7, 5, 8), 1),
                new CompoundShapeEntry(compoundShape, new Vector3(9, 1, 8), 1)
            }), 10) { Position = new Vector3(8, 25, 8) });


            //************************
            //Sometimes, it's nice to be able to change how subshapes behave.
            //This example will use a prefab CompoundBody type, but note that there exists a CompoundCollidable that can be constructed independently as well.
            //Just like a list of CompoundShapeEntry objects can be ued to construct a compound object, a list of CompoundChildData objects can too.
            //CompoundChildData objects contain CompoundShapeEntry objects as well as other data to be used by a Collidable instance.
            //That extra data includes material, events, and collision rules.

            var compoundBody = new CompoundBody(new List<CompoundChildData>()
            {
                new CompoundChildData() { Entry = new CompoundShapeEntry(new CylinderShape(1, 1), new Vector3(0, 2, 8)), CollisionRules = new CollisionRules() { Personal = CollisionRule.NoBroadPhase } },
                new CompoundChildData() { Entry = new CompoundShapeEntry(new BoxShape(3, 1, 3), new Vector3(0, 1, 8)), Material = new Material(3, 3, 0) }
            }, 10);
            Space.Add(compoundBody);

            //In this example, one of the two blocks doesn't collide with anything.  The other does collide, and has a very high friction material.

            Game.Camera.Position = new Vector3(0, 3, 25);
        }

        /// <summary>
        /// Gets the name of the simulation.
        /// </summary>
        public override string Name
        {
            get { return "Entity Construction"; }
        }
    }
}