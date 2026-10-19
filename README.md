# RealitySDFPhysics

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

## What it's good for

Contact between shapes as they are, concave and interlocking, without cutting them into convex
pieces, which RealityKit's physics can't do for moving bodies:

- **Parts that fit into each other:** gears that mesh by their teeth, rings on a peg, hooks,
  chains on sprockets.
- **Shapes without a convex decomposition:** scanned or baked meshes, used as they are.
- **Dice:** rolls on the dice's real shape, forecast exactly before they happen.

What the other engines are good for:

- **RealityKit's physics:** built in, fast and tied into Reality Composer Pro.
- **[RealityBox3D](https://github.com/elkraneo/RealityBox3D)** (Erin Catto's Box3D): convex and
  compound bodies with control over the engine, ragdolls, vehicles, character controllers, and
  tall stacks.

Tall stacks of boxes show the difference. Towers of 1 m boxes, 10 s at 60 Hz, on an Apple M1
(October 2026, with a stacking scene outside this repository). Times are the engines' own steps,
without the synchronization of the entities:

| | Tower of 10 | Tower of 20 | Tower of 40 | Pyramid of 820 boxes |
| --- | --- | --- | --- | --- |
| RealitySDFPhysics, `.realTime` | falls | falls | falls | 710 fall |
| RealitySDFPhysics, `.paper` | stands | stands | falls | stands, 29 ms a step |
| RealityKit's physics | stands | stands | falls | stands, 0.8 ms |
| Box3D | stands | stands | stands (80 too) | stands, 0.8 ms on one thread |

The engines run side by side in one scene, each stepping its own entities, so a scene can use
each where it is best (see [Known gaps](#known-gaps) for what they don't share).

## What comes from where

- **The paper** (Giles and Andrews, and their reference implementation, vendored unmodified in
  `Sources/SDFCore/upstream/`): the collision detection, which is the core of the package: the
  signed distance shapes and the voxel baking of meshes, the contact search of Algorithm 1 with
  its cached and importance-sampled points, and the contact manifolds. Our solver also started as
  the reference's (sequential impulses, with split impulses for the overlap, and its friction),
  and with every option at its default it still takes the same steps (`SolverReferenceTests`).
- **Box2D v3 and Box3D** (Erin Catto): techniques in our solver, not code. The soft step
  that joints and the drag are solved with, the forms of the revolute and prismatic joints and of
  the motor joint the drag follows, the graph coloring that solves contacts on several threads,
  and restitution by a threshold.
- **Guendelman, Bridson and Fedkiw (2003)**: bounces taken pair by pair before the contacts are
  solved together, for a Newton's cradle that behaves as one.
- **Ours**: the RealityKit API and its synchronization, the per body options (separate static
  and dynamic friction, restitution, damping, locks, filters, triggers, forces), joints with limits, motors and springs,
  determinism on several threads and `forecast()`, sleeping, interpolation between steps, reading
  RealityKit meshes and the disk cache of baked shapes, the broad phase's sphere test, and
  RealityKit colliders as SDF bodies.

## Usage

```swift
import RealityKit
import RealitySDFPhysics

// A root entity runs the simulation for its descendants, in its local frame
let root = Entity()
root.components.set(SDFPhysicsSimulationComponent())
content.add(root)

// Static ground
let ground = Entity()
ground.position = [0, -0.5, 0]
ground.components.set(SDFPhysicsBodyComponent(shape: .box(halfExtents: [5, 0.5, 5]), mode: .static))
root.addChild(ground)

// Any model: bake its meshes into a voxel SDF, in the entity's own frame
let gear = try await Entity(named: "Gear")
gear.components.set(SDFPhysicsBodyComponent(shape: try await .bake(from: gear),
                                     material: .init(friction: 0.6, restitution: 0.2)))
gear.position = [0, 2, 0]
root.addChild(gear)

// Objects of another size: model them at about 1 unit in a scaled root, and say how long a unit is
// let root = Entity(); root.scale = [0.01, 0.01, 0.01]
// root.components.set(SDFPhysicsSimulationComponent(metersPerUnit: 0.01)) // Centimeters

// Forces and events, as with RealityKit's physics
gear.sdf.applyAngularImpulse([0, 2, 0], relativeTo: nil)
let simulation = root.components[SDFPhysicsSimulationComponent.self]!.simulation
subscription = simulation.subscribe(to: SDFCollisionEvents.Began.self, on: gear) { event in
    print("hit \(event.entityB.name) with impulse \(event.impulse)")
}
```

| RealityKit | RealitySDFPhysics |
| --- | --- |
| `PhysicsSimulationComponent` | `SDFPhysicsSimulationComponent` |
| `PhysicsBodyComponent` + `CollisionComponent` | `SDFPhysicsBodyComponent` (mode, mass, material, locks, damping, gravity, filter, trigger) |
| `ShapeResource` | `SDFShape`: analytical, or baked from an entity's meshes |
| `PhysicsMotionComponent` | `SDFPhysicsMotionComponent` |
| `addForce`, `applyLinearImpulse`, … | `entity.sdf.addForce`, … |
| `CollisionEvents`, `PhysicsSimulationEvents` | `SDFCollisionEvents`, `SDFPhysicsSimulationEvents` |
| `raycast`, `convexCast` | `raycast`, and `overlaps(of:at:)`: does a shape fit here? |

The documentation catalog covers getting started, migrating from RealityKit physics, shapes and
baking, tuning the paper's parameters, and performance. Build it with Xcode's
**Product › Build Documentation**.

## Samples

In `Samples/`, macOS apps built on the public API:

- `SDFDemos`: the 11 scenes of the paper's demo, rebuilt in the `SDFDemoScenes` module with
  their shapes, sizes and the paper's settings, on one thread, for comparison. They are not
  bit-identical runs of upstream's: bodies have the density of water rather than upstream's unit
  density, and random placements come from another generator. Loading or resetting a scene
  always plays out the same. The scenes and their builder, and the window, the `SDFDemoViewer`
  module, whose `DemoInteraction` lets a scene take the drag, are library products, for packages
  that build scenes of their own on them.
- `SDFRealityKitSamples`: Apple's RealityKit physics samples ported to SDF bodies, keeping Apple's
  structure and values so that the two can be compared. So far:
  - [Simulating physics with collisions in your visionOS app](https://developer.apple.com/documentation/realitykit/simulating-physics-with-collisions-in-your-visionos-app),
    with a switch from spheres to concave tori.
  - [Simulating dice rolls as a component for your game](https://developer.apple.com/documentation/tabletopkit/simulating-dice-rolls-as-a-component-for-your-game),
    with Apple's seven dice models (MIT, see `Samples/SDFRealityKitSamples/Dice/Models/LICENSE.txt`). Its
    The roll is forecast with `SDFSimulation.forecast()`, an exact copy of the simulation: the
    engine is deterministic, so the real roll lands exactly as predicted.
  - [Simulating physics joints in your RealityKit app](https://developer.apple.com/documentation/realitykit/simulating-physics-joints-in-your-realitykit-app),
    Apple's pendulums on revolute joints, five of which make a Newton's cradle. Here it behaves
    as one: one ball in, one ball out; two in, two out. In RealityKit one ball in sends four out.

- `Benchmarks`: the pile of tori of `sdf-bench` timed on the machine at hand, RealitySDFPhysics
  against RealityKit's physics, in a window. The scene, the timings and the view are the
  `SDFBenchmarks` module; `Samples/BenchmarksApp` wraps it in an Xcode project for iPhone and
  Vision Pro, where the same table runs on the device.

`sdf-snapshot` renders the demo scenes offscreen with timings, and `sdf-video` renders a
showcase video.

```sh
swift run -c release SDFDemos
swift run -c release SDFRealityKitSamples
swift run -c release Benchmarks
swift run -c release sdf-snapshot out/ [seconds] [scene name...]
swift run -c release sdf-bench [tori ...] [--threads 1,8] [--runs 2] [--steps N]
swift test
```

Use release builds: debug builds of the engine are several times slower.

## Performance

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

## Design

- **Deterministic.** Sampling is seeded, per pair of bodies when multithreaded, and the solver
  takes its contacts in a fixed order, so the same bodies and inputs give bit-identical results
  whatever the number of threads. Turn `isMultithreaded` off for upstream's single random
  sequence and contact order; with `preset: .paper` our solver then takes the same steps as
  upstream's (`SolverReferenceTests`).
- **Threads.** The simulation steps on the main actor; its collision detection and its contact
  solver run on several threads, and baking in the background.
- **Bounces in turn.** Bodies that approach faster than the restitution threshold bounce pair by
  pair before their contacts are solved together (Guendelman, Bridson and Fedkiw 2003), along the
  normal both fields agree on, at the point where the surfaces are closest. A bounce so runs down
  a row of touching bodies whole, as in a Newton's cradle, where solving the contacts together
  would share it out.
- **Joints.** Revolute and prismatic joints with limits, motors and springs, solved as soft
  constraints in the form of Box3D's, with the contacts first so pins hold against them.
- **A hand, not a teleport.** Dragging holds a body with a soft, force-limited pull, so a gear or
  a link pulled into another stops against it rather than passing through.
- **Smooth at any frame rate.** The simulation steps 60 times a second, and bodies show between
  their last two steps by how far a frame is into the next (`isInterpolationEnabled`), so they
  move evenly on 60, 90 and 120 Hz displays, and through frames that come early or late.
- **Sleeping.** With the default preset, bodies at rest fall asleep and wake when touched, moved or
  pushed (`isSleepingEnabled`).
- **Baked once.** Shapes baked from meshes can be cached on disk (`SDFShape.cached`), so a scene
  loads in a fraction of a second after its first bake. The cache keeps to 512 MB, dropping the
  least recently used bakes.

## Known gaps

- **Bounces don't see joints.** Bodies bounce before joints are solved, so an elastic bounce at an
  angle to what a joint lets the body do loses some speed: a ball on a slider driven into a 45°
  slope comes back at about 0.7 of its speed. Bounces in the direction a joint allows, such as a
  pendulum's at the bottom of its swing, keep it.
- **Not RealityKit physics.** SDF bodies and `PhysicsBodyComponent` bodies don't push each other;
  give a moving entity one or the other. `entity.addSDFColliders()` gives an entity's RealityKit
  collision shapes to the SDF simulation as static or kinematic bodies, so SDF bodies land on the
  floors and tables a scene already has. RealityBox3D's bodies and these don't push each other
  either.
- **Tall stacks.** Towers of more than a few boxes topple with the default preset, and towers of
  40 with the paper's (see [What it's good for](#what-its-good-for)). In traces of a buckling tower, the
  contact points between two boxes bunched on one side of the face, and we think the column buckles
  where its support is one sided. Two solver experiments made towers of 20 stand, but at a cost we didn't
  accept: soft contacts made piles of tori settle slower and overlap deeper, and shock propagation
  either kept the towers awake and drifting or made the piles overlap deeper. Use RealityBox3D for
  stacks.
- **Discrete.** Collisions are detected at each substep, so very fast thin bodies can tunnel.
- **Not yet available:** continuous collision detection, shape sweeps, spherical and fixed joints.

## Siblings

[RealityBox3D](https://github.com/elkraneo/RealityBox3D) wraps Erin Catto's Box3D with the same
conventions: types named as RealityKit's with the engine's prefix (`B3PhysicsBodyComponent`,
`SDFPhysicsBodyComponent`), one entity accessor (`entity.b3`, `entity.sdf`), the same samples and
articles. Both run side by side in one RealityKit scene, each stepping its own entities.

## License

MIT (see [LICENSE](LICENSE)). The vendored engine is MIT too, by Chris Giles (see
`Sources/SDFCore/upstream/LICENSE`), and the dice models of the dice sample are Apple's, under the
license in `Samples/SDFRealityKitSamples/Dice/Models/LICENSE.txt`. Please cite the paper when using
this work.
