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

- `SDFDemos`: the 11 scenes of the upstream demo, rebuilt in the `SDFDemoScenes` module with
  their shapes, sizes and the paper's settings, on one thread, for comparison. They are not
  bit-identical runs of upstream's: bodies have the density of water rather than upstream's unit
  density, and random placements come from another generator. Loading or resetting a scene
  always plays out the same.
- `RealityKitSamples`: Apple's RealityKit physics samples ported to SDF bodies, keeping Apple's
  structure and values so that the two can be compared. So far:
  - [Simulating physics with collisions in your visionOS app](https://developer.apple.com/documentation/realitykit/simulating-physics-with-collisions-in-your-visionos-app),
    with a switch from spheres to concave tori.
  - [Simulating dice rolls as a component for your game](https://developer.apple.com/documentation/tabletopkit/simulating-dice-rolls-as-a-component-for-your-game),
    with Apple's seven dice models (MIT, see `Samples/RealityKitSamples/Dice/Models/LICENSE.txt`). Its
    The roll is forecast with `SDFSimulation.forecast()`, an exact copy of the simulation: the
    engine is deterministic, so the real roll lands exactly as predicted.

`sdf-snapshot` renders the demo scenes offscreen with timings, and `sdf-video` renders a showcase
video.

```sh
swift run -c release SDFDemos
swift run -c release RealityKitSamples
swift run -c release sdf-snapshot out/ [steps] [scene index...]
swift test
```

Use release builds: debug builds of the engine are several times slower.

Apple M1, release, 240 steps at 60 Hz with the paper's settings (`preset: .paper`: 4 substeps of
20 velocity and 20 position iterations). The default, `.realTime` (2 substeps of 10 and 4), costs
about half as much and still settles piles of 400 tori without overlap (`sdf-bench quality`). Times
vary by up to about 20% between runs.

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

- **Deterministic.** Sampling is seeded, per pair of bodies when multithreaded, and the solver
  takes its contacts in a fixed order, so the same bodies and inputs give bit-identical results
  whatever the number of threads. Turn `isMultithreaded` off for upstream's single random
  sequence and contact order; with `preset: .paper` our solver then takes the same steps as
  upstream's (`SolverReferenceTests`).
- **Threads.** The simulation steps on the main actor; its collision detection and its contact
  solver run on several threads, and baking in the background.
- **Not RealityKit physics.** SDF bodies and `PhysicsBodyComponent` bodies don't push each other;
  give a moving entity one or the other. `entity.addSDFColliders()` gives an entity's RealityKit
  collision shapes to the SDF simulation as static or kinematic bodies, so SDF bodies land on the
  floors and tables a scene already has.
- **Discrete.** Collisions are detected at each substep, so very fast thin bodies can tunnel.
- **Sleeping.** With the default preset, bodies at rest fall asleep and wake when touched, moved or
  pushed (`isSleepingEnabled`).
- **Not yet available:** joints, continuous collision detection and shape sweeps.

## License

MIT (see [LICENSE](LICENSE)). The vendored engine is MIT too, by Chris Giles (see
`Sources/SDFCore/upstream/LICENSE`), and the dice models of the dice sample are Apple's, under the
license in `Samples/RealityKitSamples/Dice/Models/LICENSE.txt`. Please cite the paper when using
this work.
