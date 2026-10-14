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

// Objects of another size: model them at about 1 unit in a scaled root, and say how long a unit is
// let root = Entity(); root.scale = [0.01, 0.01, 0.01]
// root.components.set(SDFSimulationComponent(metersPerUnit: 0.01)) // Centimeters

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

## Samples

In `Samples/`, macOS apps built on the public API:

- `SDFDemos`: the 11 scenes of the upstream demo, rebuilt in the `SDFDemoScenes` module with the
  paper's settings on one thread, so they play out as in the upstream demo, for comparison.
- `RealityKitSamples`: Apple's RealityKit physics samples ported to SDF bodies, keeping Apple's
  structure and values so that the two can be compared. So far:
  - [Simulating physics with collisions in your visionOS app](https://developer.apple.com/documentation/realitykit/simulating-physics-with-collisions-in-your-visionos-app),
    with a switch from spheres to concave tori.
  - [Simulating dice rolls as a component for your game](https://developer.apple.com/documentation/tabletopkit/simulating-dice-rolls-as-a-component-for-your-game),
    with Apple's seven dice models (MIT, see `Samples/RealityKitSamples/Dice/Models/LICENSE.txt`). Its
    "Max score toss" runs each toss in a hidden simulation first: the engine is deterministic, so
    the real roll lands exactly as predicted.

`sdf-snapshot` renders the demo scenes offscreen with timings, and `sdf-video` renders a showcase
video.

```sh
swift run -c release SDFDemos
swift run -c release RealityKitSamples
swift run -c release sdf-snapshot out/ [steps] [scene index...]
swift test
```

Use release builds: debug builds of the engine are several times slower.

Apple M1, release, 240 steps at 60 Hz with 4 substeps. Times vary by up to about 20% between runs.

| Scene | Bodies | Step ms, multithreaded | Step ms, one thread |
| --- | ---: | ---: | ---: |
| Box Stack | 29 | 1.7 | 1.7 |
| Gear Stack | 11 | 0.5 | 0.6 |
| Falling Boxes (plain / bend / twist / displace) | 13 | 0.6–1.0 | 0.8–2.3 |
| Pile of Chairs | 28 | 1.6 | 4.5 |
| Spiky Wheel | 12 | 0.7 | 0.9 |
| Cows on Spikes | 20 | 1.4 | 4.1–5.9 |
| Bunny Pile | 28 | 2.4 | 8.5–9.8 |
| Primitives | 49 | 2.7 | 6.6 |

## Notes

- **Deterministic.** Sampling is seeded, per pair of bodies when multithreaded, so the same bodies
  and inputs give bit-identical results whatever the number of threads. Turn
  `isMultithreaded` off to reproduce the paper's engine exactly.
- **Threads.** The simulation steps on the main actor; its collision detection runs on several
  threads, and baking in the background.
- **Not RealityKit physics.** SDF bodies and `PhysicsBodyComponent` bodies do not collide with each
  other. Give an entity one or the other.
- **Discrete.** Collisions are detected at each substep, so very fast thin bodies can tunnel.
- **Not yet available:** joints, continuous collision detection, sleeping and shape sweeps.

The reference port of the upstream demo, without the component API, is
[sdf-dcd-realitykit](https://github.com/elkraneo/sdf-dcd-realitykit).

## License

MIT for the upstream engine (see `Sources/SDFCore/upstream/LICENSE`). Please cite the paper when
using this work.
