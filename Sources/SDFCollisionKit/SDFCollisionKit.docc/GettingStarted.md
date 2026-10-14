# Getting Started

Add a simulation to a scene, give entities bodies, and react to their contacts.

## Add the package

In Xcode, add `https://github.com/elkraneo/SDFCollisionKit` as a package dependency and link the
`SDFCollisionKit` library. In a package manifest:

```swift
.package(url: "https://github.com/elkraneo/SDFCollisionKit", from: "0.1.0"),
```

The package supports macOS 15, iOS 18 and visionOS 2. The engine is C++, but its interface is C,
so your targets don't need C++ interoperability.

## Create a simulation

A simulation runs for the descendants of an entity with an ``SDFSimulationComponent``, in that
entity's local frame. ``SDFSimulationSystem`` steps it every frame; it registers itself when you
create the first simulation.

```swift
RealityView { content in
    let root = Entity()
    root.components.set(SDFSimulationComponent())
    content.add(root)
}
```

The root's frame is the simulation's frame. Positions, velocities, forces and query results are in
it, and gravity points down its y axis. The default parameters suit objects about 1 unit across,
so for tabletop scale, build the scene in units of about a meter and scale the root down. Scaling
the root scales the whole simulation, gravity included.

## Add bodies

An entity becomes a body of the nearest ancestor simulation when it gets an ``SDFBodyComponent``.
Every body has a shape, its geometry in the entity's frame:

```swift
// A static floor whose top face is at y = 0
let floor = Entity()
floor.position = [0, -0.5, 0]
floor.components.set(SDFBodyComponent(shape: .box(halfExtents: [5, 0.5, 5]), mode: .static))
root.addChild(floor)

// A model, its meshes baked into a voxel distance field
let gear = try await Entity(named: "Gear", in: realityKitContentBundle)
gear.components.set(SDFBodyComponent(shape: try await .bake(from: gear)))
gear.position = [0, 2, 0]
root.addChild(gear)
```

Bodies appear and disappear with their entities and components. The simulation writes the
transforms of dynamic bodies; set the transform of a kinematic body to move it, and it pushes
dynamic bodies with the velocity of that motion. Setting the transform of a dynamic or static body
teleports it.

Configure a body like a `PhysicsBodyComponent`:

```swift
var body = SDFBodyComponent(shape: shape, massProperties: .init(mass: 2),
                            material: .init(staticFriction: 0.8, dynamicFriction: 0.6, restitution: 0.3))
body.linearDamping = 0.1
body.isRotationLocked = (x: true, y: false, z: true)
body.filter = CollisionFilter(group: pieces, mask: .all)
entity.components.set(body)
```

## Push bodies

Read and set velocities with ``SDFMotionComponent``. Apply forces and impulses through
``SDFPhysics``, which has the methods of RealityKit's `HasPhysicsBody`:

```swift
ball.sdfPhysics.applyLinearImpulse([0, 3, 0], relativeTo: nil)
wheel.sdfPhysics.addTorque([0, 0, -20], relativeTo: wheel)
```

Forces act during the next simulation update and are then cleared, so add them every frame for a
continuous push, for example from a handler of ``SDFSimulationEvents/WillSimulate``.

## React to contacts

Subscribe to ``SDFCollisionEvents`` through the simulation, for all bodies or for one entity. Keep
the returned value for as long as you want the events:

```swift
let simulation = root.components[SDFSimulationComponent.self]!.simulation
subscription = simulation.subscribe(to: SDFCollisionEvents.Began.self, on: gear) { event in
    playClick(at: event.position, strength: event.impulse)
}
```

Set ``SDFBodyComponent/isTrigger`` on a body to detect what enters it without stopping it.

## Ask whether a shape fits

``SDFSimulation/overlaps(of:at:margin:mask:)`` runs the simulation's collision detection for a
shape at a pose without adding it, for example to check a placement before making it:

```swift
let blocked = simulation.overlaps(of: piece, at: Transform(translation: slot), margin: 0.005)
if blocked.isEmpty { place(piece, at: slot) }
```

The answer is deterministic and the query doesn't change the simulation.
