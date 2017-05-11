# **Collision Events**

A way to hook game logic to the physical world.

## 1 | Setting Up Events

While it is possible to scan over the results of the engine every frame and collect information, it can be convenient to get notifications of specific occurrences involving specific collidables. BEPUphysics provides for this using a variety of event types.&nbsp; There are many non-collision events available in the engine as well, such as the Entity activation/deactivation and update events.&nbsp; This documentation is primarily concerned with events resulting from the collision detection pipeline.

Every Collidable has its own ContactEventManager, accessible in the collidable’s Events property.&nbsp; An entity’s collidable is in its CollisionInformation property, so entity events can be accessed through the entity.CollisionInformation.Events property.&nbsp; To set up an event:

1.  Create a method with a signature matching one of the event types (found in BEPUphysics.Collidables.Events)

2.  Add it to the corresponding event in the event manager.

### 1.A | Collision Event Types

Each of the following types can be found in a ContactEventManager. Methods matching the event handler's required signature can be used as event handlers.

*   **PairCreated**: Triggered when two collidable objects' bounding boxes begin to overlap.

*   **PairRemoved**: Triggered when two collidable objects' bounding boxes cease to overlap.

*   **ContactCreated**: Triggered when a collision pair’s contact list is updated with an additional contact.

*   **InitialCollisionDetected**: Triggered when a collision pair’s contact list goes from zero contacts to one or more contacts.

*   **ContactRemoved**: Triggered when a collision pair’s contact list is updated by the removal of a contact point.

*   **CollisionEnded**: Triggered when a collision pair’s contact list goes from one or more contacts to zero contacts.

*   **PairUpdated**: Triggered when a collision pair’s narrow phase update method runs.

*   **PairTouched**: Triggered when a collision pair’s narrow phase update method runs and the collision pair has one or more contacts in it.

## 2 | Immediate versus Deferred Events

All of the events listed above are called at the end of the update in which the event occurred. They are known as “deferred” events for this reason. Their end-of-update positioning allows them to change settings of a wide scope without interfering with the engine's execution. They execute sequentially, preventing a situation where a deferred event handler could interfere with other concurrently executing deferred event handlers.

For every event listed above, there also exists another event with a present-tense name (“ing” instead of “ed”). These 'immediate' events are called directly by the actions that trigger the events in the first place. Essentially, they allow arbitrary code to be inserted directly into the execution of the engine.

There are safety considerations to take into account with both immediate and deferred events. These are addressed in section 3.

### 2.A | Immediate Events in the Pipeline

Since immediate events can be used to modify information while the engine is executing, it helps to know exactly where they trigger from in the process. In the following list, “broad phase” refers to the process of collecting of collision pairs between potentially colliding objects. Its followup, the “narrow phase,” analyzes overlaps and creates CollisionPairHandlers which manage contact manifolds between objects of a collision pair.

*   **CreatingPair**: Triggered when the narrow phase adds a new collision pair handler to the space due to overlapping bounding boxes detected by the broad phase. Initialization is performed before the event is triggered so the event handler can override and change information in the pair before it is used in simulation.

*   **RemovingPair**: Triggered when the narrow phase gets rid of a stale collision pair as determined by the broad phase.

*   **CreatingContact**: Triggered when a collision pair’s contact list is updated with an additional contact during the CollisionPairHandler’s Update method. Both the CollisionPairHandler and Contact settings can be changed within this event.

*   **DetectingInitialCollision**: Triggered when a collision pair’s contact list is updated with an additional contact during the CollisionPairHandler’s Update method method when there were zero contacts previously. Both the collision pair and Contact settings can be changed within this event.

*   **RemovingContact**: Triggered when a collision pair’s contact list is updated by the removal of a contact point in the CollisionPairHandler’s Update method. The collision pair’s settings can be changed in this event.

*   **CollisionEnding**: Triggered when a collision pair’s contact list is updated by the removal of a contact point in the CollisionPairHandler’s Update method and there are no more contact points in the CollisionPair's contact list. The collision pair’s settings can be changed in this event.

*   **PairUpdating**: Triggered at the end of CollisionPairHandler’s Update method method. The collision pair’s settings and the settings of any Contacts within the collision pair can be changed from this method.

*   **PairTouching**: Triggered at the end of CollisionPairHandler’s Update method method when the collision pair’s contact list is not empty. The collision pair’s settings and the settings of any Contacts within the CollisionPair can be changed from this method.

## 3 | Safe Operations Inside Event Handlers

Depending on the context from which an event handler is called, some operations are unsafe. Unsafe operations are those which either negatively interfere with the functioning of the engine in some way or can cause unintended behavior and errors.

### 3.A | Deferred Event Safety

For all event types with past-tense names, the events handlers are called at the end of a space time step. In general, this is a fairly safe operation. Their deferred nature prevents them from directly interfering with the engine's execution.

INarrowPhasePair/CollidablePairHandler and Contact object references passed into a deferred event handler are not always guaranteed to be valid at the end of an update, as CollisionPairs and Contacts can quickly return to the resource pool. Be careful when collecting information from these object instances. References to these objects should not be kept outside of the event handler.

### 3.B | Immediate Event Safety

Immediate events are called from within the execution of the engine and the scope of safe operations is much smaller than the deferred versions. However, due to their in-engine execution, they can intercept and modify interactions. In general, operations should be performed only on the INarrowPhasePair/CollidablePairHandler or Contact passed into the event handler.

If the engine is utilizing multiple threads, the event handlers will be called from the context of worker threads. Any operations performed outside of the INarrowPhasePair/CollidablePairHandler or Contact passed into the event handler must be handled very carefully. Even reading data outside of the CollisionPair or Contact may be unreliable depending on the event.

INarrowPhasePair/CollidablePairHandler and Contact object references passed into an immediate event handler are guaranteed to be valid for the duration of the event handler, but should not be kept outside of the event handler.

## 4 | Events with Compound Bodies

Every CompoundBody has a tree of collidables. Compound bodies can have collidable children which are also compound bodies, forming a multi-level tree. However, a CompoundBody has no physical geometry of its own; its non-compound children are its geometry. Therefore, all collision-related events originate in a non-compound body child in the tree.

In a CompoundBody, it may be inconvenient to attach an event handler to every non-compound child entity. To avoid having to do this, compound bodies collect the events of their children. Attaching an event to a CompoundBody will trigger on the events of its children.

![image](images/collision%20events/compoundevents.png)

Any CompoundBody in the tree above a event-causing child will receive events from that child. This allows the root body to fire all events and the sub-compound bodies to fire events only from particular entities in the tree.