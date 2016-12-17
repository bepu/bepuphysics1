# **Getting Started**

A basic tutorial on setting up a simulation. The following assumes that you have installed, and are reasonably comfortable with, Visual Studio 2015.

## 1 | Getting the libraries

A periodically packaged binary form of the libraries can be downloaded on [github](https://github.com/RossNordby/bepuphysics1/releases). There's also a [nuget package](https://www.nuget.org/packages/BEPUphysics/).

Be advised, historical evidence suggests I'll be pretty lazy about setting up packaged releases. There may be significantly newer versions available in the [source](../../../).

The following will use monogame for simple visualization, but the details of the graphical setup are not covered. Check out the [GettingStartedDemo](Isolated Demos/GettingStartedDemo) for the end result of this walkthrough, and the [BasicSetupDemo](Isolated Demos/BasicSetupDemo) for a super-barebones simulation to fiddle with.

Don't forget about all the samples in the [BEPUphysicsDemos](../BEPUphysicsDemos/Demos), too!

To begin, create a monogame project and reference the BEPUphysics.dll and BEPUutilities.dll.

## 2 | Setting Up a Simulation

Before a simulation can start, a place for the simulation to occur must be created. This is handled by the Space class of BEPUphysics. Other simulation objects can be added to and removed from a Space instance. Creating a space is very simple; first, set up a field in your game for the space, and then construct it in the game's LoadContent method:
```cs
space = new Space();
```
Now your simulation objects have a place to live.

BEPUphysics needs to be told that time is passing to allow objects to move. In your game's Update method, add in the following line:
```cs
space.Update();
```
When you run the game at this point, BEPUphysics is invisibly working in the background, updating the world every frame.

### 2.A | Adding Entities

The world is pretty boring without anything in it. The Entity class represents common physical objects populating the space. You can create a variety of different Entity types, including Box, Cylinder, Sphere, Capsules, and others which can be found in the BEPUphysics.Entities namespace. If during the following process you encounter a 'type not found' error similar to the above with Space, you can right click and use the Resolve functionality as before.

All entities can either be dynamic or kinematic. Dynamic entities fall, get knocked around, bounce, and slide as expected. Kinematic entities are like dynamic entities, but can be thought of as having infinite mass. They will not change their velocities as the result of any collision or interaction unless explicitly told to do so. A kinematic entity moving towards a dynamic entity will simply push the dynamic entity around. If a kinematic entity encounters another kinematic entity, they pass through each other.

The only difference between constructing a kinematic entity and a dynamic entity is the last parameter of the constructor. Dynamic constructors have a “mass” parameter, while kinematic constructors do not. You can change between dynamic and kinematic later by calling the entity's BecomeKinematic and BecomeDynamic methods.

Kinematic entities are a good choice for static and structural shapes, like the ground. You can make a kinematic box representing the ground by putting the following in the game's LoadContent method:
```cs
Box ground = new Box(Vector3.Zero, 30, 1, 30);
```
The first parameter represents the position of the box and the following three parameters are its width, height, and length. Now add the ground to the space:
```cs
space.Add(ground);
```
Throw some extra dynamic cubes at the ground too:
```cs
space.Add(new Box(new Vector3(0, 4, 0), 1, 1, 1, 1)); 
space.Add(new Box(new Vector3(0, 8, 0), 1, 1, 1, 1)); 
space.Add(new Box(new Vector3(0, 12, 0), 1, 1, 1, 1));
```
The last parameter specified is the mass as explained earlier, making these entities dynamic.

The simulation's gravity acceleration vector defaults to (0, 0, 0), so to make things move, set the gravity to a more earth-like value by changing the space’s gravity settings:
```cs
space.ForceUpdater.Gravity = new Vector3(0, -9.81f, 0);
```
## 3 | Basic Interaction with BEPUphysics

If you run the game now, BEPUphyics is running but nothing is visible. To remedy this, attach some graphics to the entities. A basic implementation of a DrawableGameComponent that follows an Entity and a Camera which manages the viewpoint of the user are available in the [GettingStartedDemo](Isolated Demos/GettingStartedDemo). This document will not go in depth on how to set up these systems, but will describe how these pieces interact with the physics engine.

### 3.A | Getting Entity Position for Rendering

The Draw method of the EntityModel class has the following line of code at the beginning:
```cs
Matrix worldMatrix = Transform * entity.WorldTransform;
```
This line defines what transformation to use for rendering the model. The transform variable is just a local space transformation that can be used to adjust the model in case it needs to be scaled, moved, or rotated. A [common problem](ShapeRecentering.md) is a model being loaded in that wasn't centered on the origin in the modeling application. This extra transform would allow you to re-center the model without going back into the application.

The line of code grabs the entity's current world transformation. This matrix represents a rigid transformation; that is, it only includes orientation and translation. It is a convenient representation of the entity's state for rendering, though you can also directly access the entity's Position, OrientationMatrix, and a variety of other properties. Look around in the entity's property list to get an idea of what is available.

### 3.B | Firing a Box

In addition to the basic camera manipulation controls in the Camera class, there is a section in the game's Update method which creates a box when the left mouse button is clicked. This looks like the box creation code shown previously:
```cs
Box toAdd = new Box(camera.Position, 1, 1, 1, 1);
```
However, the velocity of the box is set as well. This is done by accessing another one of the entity's properties- the LinearVelocity:
```cs
toAdd.LinearVelocity = camera.WorldMatrix.Forward * 10;
```
The box will fly off in the direction that the camera is facing. Try changing the speed and changing other properties to see the result.

## 4 | Adding an Environment

In the same way that entities can be added to the space, a variety of other types can be added as well. One common type is known as the StaticMesh. This object represents a triangle mesh that can collide with entities. It is well suited for creating a physical environment for your game.

To do this, find a model that you want to use and load it into your game in the LoadContent method. The information stored in the model needs to be put into a form that BEPUphysics can understand. A ModelDataExtractor is included in the GettingStartedDemo project and BEPUphysicsDemos
which offers a helper method for extracting this information:
```cs
Vector3[] vertices; 
int[] indices; 
ModelDataExtractor.GetVerticesAndIndicesFromModel(model, out vertices, out indices); 
var mesh = new StaticMesh(vertices, indices, new AffineTransform(new Vector3(0, -40, 0))); 
space.Add(mesh);
```
Since the StaticMesh starts a bit high relative to the rest of the simulation, the AffineTransform parameter includes a translation that pushes it back down. Also, note that the indices list is used to smooth out the boundaries of connected triangles so that things sliding along the ground don't hit bumps on the edge of every triangle. A pure triangle soup with no connectivity information will have bumps!

Try setting up a renderer for the triangle mesh. You can get an example StaticModel game component from the GettingStartedDemo. Once it's added, you can run the game and see the mesh below the boxes we set up previously. Try shooting some boxes at it and watching them collide.

## 5 | Handling an Event

One effective way of binding your game's logic to the physics engine is by using events. There are a variety of events that can trigger related to collisions; a listing of them can be found in the BEPUphysics.Events namespace.

To set up an event, use an entity.CollisionInformation’s Events property. Here, we will use the InitialCollisionDetected event which fires when the first contact point in a collision pair is created. When the event is triggered, the entity's event manager will call the event handling method that was passed in. Adding the method to the event looks like this:
```cs
deleterBox.EventManager.InitialCollisionDetected += HandleCollision;
```
The deleterBox is another kinematic entity floating near the large 'ground' box. The HandleCollision method is:
```cs
void HandleCollision(EntityCollidable sender, Collidable other, CollidablePairHandler pair) 
{ 
    var otherEntityInformation = other as EntityCollidable; 
    if (otherEntityInformation != null) 
    { 
        space.Remove(otherEntityInformation.Entity);&nbsp; 
        Components.Remove((EntityModel)otherEntityInformation.Entity.Tag); 
    } 
}
```
The method signature matches the required signature of the InitialCollisionDetectedEventHandler. The "sender" parameter is the entity collidable that has the event hook, in this case the deleterBox.CollisionInformation. The "other" collidable is whatever is colliding with the sender. The pair is the object which oversees the collision between the two entities. A CollidablePairHandler exists between any two collidables that are in danger of colliding and have overlapping bounding boxes.

This event handler grabs the entity of the opposing collidable and removes it from the space and graphics component list. In the GettingStartedDemo, the EntityModel rendering object for each entity is put into the entity's tag property. An entity's tag is an object that can be set by the user to store arbitrary data. This tag is referenced by the event handler to retrieve its EntityModel.

For more detailed information about events, check the Collision Events documentation.

## 6 | Going Further

To learn more about BEPUphysics, check out the other demos and documentation available in the [repository](../../../). Try changing various settings and creating your own simulations. If you ever need help, please feel free to post on the [forums](http://www.bepu-games.com/forum/). Asking questions is a great way to learn and helps make BEPUphysics better too!