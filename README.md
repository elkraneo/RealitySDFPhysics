# SDFCollisionKit

Rigid body physics for RealityKit in which every body is a signed distance field, built on

> Chris Giles and Sheldon Andrews. **Real-Time Collision Handling for Signed Distance Fields via
> Caching and Importance Sampling.** *Computer Graphics Forum* 45(7), Pacific Graphics 2026.
> [Project page](https://savant117.github.io/sdf-dcd-demo/) ·
> [Reference implementation](https://github.com/savant117/sdf-dcd-demo)

Concave and interlocking shapes such as gears, spikes, chairs and your own meshes collide
exactly, with no convex decomposition. RealityKit's physics only gives moving bodies convex shapes.
This package replaces it for the entities you choose, with an API that mirrors RealityKit's physics
components. The paper's collision detection is vendored unmodified behind a C interface (see
[UPSTREAM.md](UPSTREAM.md)), so your project does not need C++ interoperability.

## Usage

```swift
import RealityKit
import SDFCollisionKit

// A root entity runs the simulation for its descendants, in its local frame
let root = Entity()
root.components.set(SDFSimulationComponent())
content.add(root)

// Static ground
let ground = Entity()
ground.position = [0, -0.5, 0]
ground.components.set(SDFBodyComponent(shape: .box(halfExtents: [5, 0.5, 5]), mode: .static))
root.addChild(ground)

// Any model: bake its meshes into a voxel SDF, in the entity's own frame
let gear = try await Entity(named: "Gear")
gear.components.set(SDFBodyComponent(shape: try await .bake(from: gear),
                                     material: .init(friction: 0.6, restitution: 0.2)))
gear.position = [0, 2, 0]
root.addChild(gear)

// Forces and events, as with RealityKit's physics
gear.sdfPhysics.applyAngularImpulse([0, 2, 0], relativeTo: nil)
let simulation = root.components[SDFSimulationComponent.self]!.simulation
subscription = simulation.subscribe(to: SDFCollisionEvents.Began.self, on: gear) { event in
    print("hit \(event.entityB.name) with impulse \(event.impulse)")
}
```

| RealityKit | SDFCollisionKit |
| --- | --- |
| `PhysicsSimulationComponent` | `SDFSimulationComponent` |
| `PhysicsBodyComponent` + `CollisionComponent` | `SDFBodyComponent` (mode, mass, material, locks, damping, gravity, filter, trigger) |
| `ShapeResource` | `SDFShape`: analytical, or baked from an entity's meshes |
| `PhysicsMotionComponent` | `SDFMotionComponent` |
| `addForce`, `applyLinearImpulse`, … | `entity.sdfPhysics.addForce`, … |
| `CollisionEvents`, `PhysicsSimulationEvents` | `SDFCollisionEvents`, `SDFSimulationEvents` |
| `raycast`, `convexCast` | `raycast`, and `overlaps(of:at:)`: does a shape fit here? |

The documentation catalog covers getting started, migrating from RealityKit physics, shapes and
baking, tuning the paper's parameters, and performance. Build it with Xcode's
**Product › Build Documentation**.

## Demo

`SDFDemos` is a macOS app with the 11 scenes of the upstream demo, rebuilt on the public API in the
`SDFDemoScenes` module. `sdf-snapshot` renders them offscreen with timings, and `sdf-video` renders
a showcase video.

```sh
swift run -c release SDFDemos
swift run -c release sdf-snapshot out/ [steps] [scene index...]
swift test
```

Use release builds: debug builds of the engine are several times slower.

Apple M1, release, 240 steps at 60 Hz with 4 substeps, single threaded. Times vary by about 20%
between runs.

| Scene | Bodies | Step ms mean |
| --- | ---: | ---: |
| Box Stack | 29 | 1.8 |
| Gear Stack | 11 | 0.7 |
| Falling Boxes (plain / bend / twist / displace) | 13 | 0.9–2.3 |
| Pile of Chairs | 28 | 5.8 |
| Spiky Wheel | 12 | 0.9 |
| Cows on Spikes | 20 | 4.4 |
| Bunny Pile | 28 | 10.8 |
| Primitives | 49 | 7.0 |

## Notes

- **Deterministic.** The engine is single threaded and seeds its sampling with a fixed value, so
  the same bodies and inputs give bit-identical results.
- **Main actor.** The simulation steps on the main actor. Only baking runs in the background.
- **Not RealityKit physics.** SDF bodies and `PhysicsBodyComponent` bodies do not collide with each
  other. Give an entity one or the other.
- **Discrete.** Collisions are detected at each substep, so very fast thin bodies can tunnel.
- **Not yet available:** joints, continuous collision detection, sleeping and shape sweeps.

The reference port of the upstream demo, without the component API, is
[sdf-dcd-realitykit](https://github.com/elkraneo/sdf-dcd-realitykit).

## License

MIT for the upstream engine (see `Sources/SDFCore/upstream/LICENSE`). Please cite the paper when
using this work.
