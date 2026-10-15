# ``RealitySDFPhysics``

Rigid body physics for RealityKit in which every body is a signed distance field, so concave
shapes collide exactly.

## Overview

RealityKit's physics, like most game physics engines, gives moving bodies convex shapes only.
Gears, spikes, chairs and most meshes have to be approximated by boxes and capsules or cut into
convex pieces. RealitySDFPhysics replaces RealityKit's physics for the entities you choose with a
simulation in which every body is a signed distance field: the distance to its surface,
negative inside. Concave and interlocking shapes collide as they are.

The collision detection is that of Chris Giles and Sheldon Andrews,
*Real-Time Collision Handling for Signed Distance Fields via Caching and Importance Sampling*
(Computer Graphics Forum 45(7), Pacific Graphics 2026), vendored unmodified from their
[reference implementation](https://github.com/savant117/sdf-dcd-demo). It finds the contacts of
two bodies as the deepest points of their intersection, with an optimizer that is warm started
from the previous step's points and seeded with samples from high curvature regions.

The API mirrors RealityKit's physics components, so code that uses `PhysicsBodyComponent` moves
over with few changes:

```swift
let root = Entity()
root.components.set(SDFPhysicsSimulationComponent())
content.add(root)

let gear = try await Entity(named: "Gear")
gear.components.set(SDFPhysicsBodyComponent(shape: try await .bake(from: gear)))
root.addChild(gear)
```

## Topics

### Essentials

- <doc:GettingStarted>
- <doc:MigratingFromRealityKitPhysics>

### Simulations

- ``SDFPhysicsSimulationComponent``
- ``SDFSimulation``
- ``SDFSimulationSystem``
- ``SDFCollisionDetection``

### Bodies

- ``SDFPhysicsBodyComponent``
- ``SDFPhysicsMaterial``
- ``SDFPhysicsMassProperties``
- ``SDFPhysicsMotionComponent``
- ``SDFPhysics``

### Shapes

- <doc:ShapesAndBaking>
- ``SDFShape``

### Events

- ``SDFSimulationEvent``
- ``SDFCollisionEvents``
- ``SDFPhysicsSimulationEvents``

### Tuning and performance

- <doc:TuningCollisionDetection>
- <doc:Performance>
