# SDFCollisionKit

Rigid body physics for RealityKit where every body is a signed distance field, built on

> Chris Giles and Sheldon Andrews. **Real-Time Collision Handling for Signed Distance Fields via
> Caching and Importance Sampling.** *Computer Graphics Forum* 45(7), Pacific Graphics 2026.
> [Project page](https://savant117.github.io/sdf-dcd-demo/) ·
> [Reference implementation](https://github.com/savant117/sdf-dcd-demo)

Concave and interlocking shapes such as gears, spikes, chairs and your own meshes collide
exactly, with no convex decomposition. RealityKit's own physics only supports convex shapes for
moving bodies. This package replaces it for the entities you choose. The reference C++ engine is
vendored unmodified behind a C interface (see [UPSTREAM.md](UPSTREAM.md)), so your project does not
need C++ interoperability.

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
gear.components.set(SDFBodyComponent(shape: try await .bake(from: gear)))
gear.position = [0, 2, 0]
root.addChild(gear)
```

`SDFSimulationSystem` then steps the simulation at a fixed rate every frame and writes the poses of
dynamic bodies to their entities.

### Bodies

`SDFBodyComponent(shape:mode:mass:friction:)` makes an entity a body of its nearest ancestor with
an `SDFSimulationComponent`:

- **`.dynamic`**: moved by the simulation. Setting the entity's transform yourself teleports it.
- **`.kinematic`**: moved by you; pushes dynamic bodies with the velocity of its motion.
- **`.static`**: does not move, or teleports when you set its transform.

Bodies appear and disappear with their entities. Changing the shape, mode or the entity's scale
recreates the body; mass and friction update in place. Scales must be uniform. The default mass is
that of the shape's volume of water.

`SDFMotionComponent` exposes a dynamic body's velocities. The simulation writes them after every
step, and changes you make are applied before the next.

### Shapes

`SDFShape` is immutable and can be created on any thread and shared between bodies:

- Analytical: `.box`, `.sphere`, `.capsule`, `.cylinder`, `.cone`, `.torus`
- From the paper: `.spikePlate`, `.spikyWheel`, `.warpedBox(.bend / .twist / .displace)`
- Baked: `SDFShape.bake(from: entity)` (meshes of an entity and its descendants, baked off the main
  actor) or `SDFShape(baking: positions, indices:)`. Meshes should be closed; check
  `isWatertight`. Duplicate vertices at seams are welded.

`shape.distance(at:)` queries the field directly, and `MeshResource.generate(from: shape)` gives a
display mesh.

### Simulation

`SDFSimulation` holds the world: `parameters` (the paper's collision parameters, solver substeps,
time step, gravity), `isPaused`, `statistics`, `contacts()` with the entities involved,
`applyImpulse(_:at:to:)`, `mass(of:)` and mouse-style dragging (`beginDrag`, `drag(to:)`,
`endDrag`). `step(root:)` takes a single step, also while paused.

**Units.** The simulation runs in the root's local units, and the default contact margin and
tolerances suit objects about 1 unit across. For tabletop scale, keep objects about 1 unit in the
root and scale the root down. Gravity is in root units too, so divide it by the root's scale to
keep real time falls.

## Demo

`SDFDemos` is a macOS app with the 11 scenes of the upstream demo, rebuilt on the public API in the
`SDFDemoScenes` module. `sdf-snapshot` renders them offscreen with timings.

```sh
swift run -c release SDFDemos
swift run -c release sdf-snapshot out/ [steps] [scene index...]
swift test
```

Use release builds: debug builds of the engine are several times slower.

Apple M1, release, 240 steps at 60 Hz with 4 substeps, single threaded:

| Scene | Bodies | Load ms | Step ms mean |
| --- | ---: | ---: | ---: |
| Box Stack | 29 | 142 | 1.65 |
| Gear Stack | 11 | 211 | 0.56 |
| Falling Boxes (plain / bend / twist / displace) | 13 | 18–23 | 0.74–2.20 |
| Pile of Chairs | 28 | 223 | 4.36 |
| Spiky Wheel | 12 | 31 | 0.89 |
| Cows on Spikes | 20 | 312 | 3.97 |
| Bunny Pile | 28 | 908 | 11.16 |
| Primitives | 49 | 157 | 7.45 |

Load time includes baking a scene's mesh shapes the first time.

## Notes

- **Deterministic.** The engine is single threaded and seeds its sampling with a fixed value, so
  the same bodies and inputs give bit-identical results.
- **Main actor.** The simulation steps on the main actor. Only baking runs in the background.
- **Not RealityKit physics.** SDF bodies and `PhysicsBodyComponent` bodies do not collide with each
  other. Mark an entity with one or the other.
- **Discrete.** Collisions are detected at each substep, so very fast thin bodies can tunnel.
- **Missing compared to RealityKit physics:** joints, continuous collision detection, collision
  filtering, sleeping and collision events.

The reference port of the upstream demo, without the component API, is
[sdf-dcd-realitykit](https://github.com/elkraneo/sdf-dcd-realitykit).

## License

MIT for the upstream engine (see `Sources/SDFCore/upstream/LICENSE`). Please cite the paper when
using this work.
