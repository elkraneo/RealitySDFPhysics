# SDFCollisionKit

SDF-SDF rigid body collisions for RealityKit, built on the method of

> Chris Giles and Sheldon Andrews. **Real-Time Collision Handling for Signed Distance Fields via
> Caching and Importance Sampling.** *Computer Graphics Forum* 45(7), Pacific Graphics 2026.
> [Project page](https://savant117.github.io/sdf-dcd-demo/) ·
> [Reference implementation](https://github.com/savant117/sdf-dcd-demo)

Every body is a signed distance field (analytical, operator-distorted or a voxel grid baked from a
mesh), so concave and interlocking shapes such as gears, spikes and chairs collide exactly. No
convex decomposition is needed. The reference C++ engine is vendored unmodified (see
[UPSTREAM.md](UPSTREAM.md)) behind a C interface, so clients do not need C++ interoperability.

RealityKit's own physics is not used. `SDFSimulation` steps the engine at a fixed rate from a
RealityKit `System` and mirrors its bodies as entities.

## Layout

| Target | Contents |
| --- | --- |
| `SDFCore` | Upstream engine (`upstream/`), SDL/renderer shims (`shim/`) and the C facade (`include/SDFCore.h`) |
| `SDFCollisionKit` | `SDFWorld` (Swift wrapper), `SDFSimulation` (stepped by an internal RealityKit component and system), `SDFDemoStage` (floor, light, camera), bundled OBJ assets |
| `SDFDemos` | macOS app: the 11 upstream demo scenes in a `RealityView` |
| `sdf-bench` | Headless timing of every scene |
| `sdf-snapshot` | Offscreen `RealityRenderer` PNGs of every scene |

## Running

```sh
swift run -c release SDFDemos                 # the demo app (macOS 15+)
swift run -c release sdf-bench [steps]        # timings, default 600 steps
swift run -c release sdf-snapshot out/ [steps] # PNG per scene
swift test
```

Use release builds: debug builds of the engine are several times slower.

In the app, drag a body to move it, drag the background to orbit and pinch to zoom.
<kbd>Space</kbd> drops the selected shape, <kbd>P</kbd> pauses and <kbd>R</kbd> resets, as in the
upstream demo.

## Using it in a RealityKit scene

```swift
let simulation = SDFSimulation()   // keep a strong reference, the component holds it weakly
simulation.loadScene(1)            // Gear Stack
content.add(simulation.root)       // SDFSimulationSystem now steps it every frame
```

## Measurements

Apple M1, release build, 600 steps (10 s at 60 Hz, 4 substeps per step), single threaded:

| Scene | Bodies | Bake ms | Step ms mean / max |
| --- | ---: | ---: | ---: |
| Box Stack | 29 | 3 | 1.63 / 2.56 |
| Gear Stack | 11 | 184 | 0.57 / 0.76 |
| Falling Boxes | 13 | 1 | 0.72 / 0.98 |
| Falling Boxes: Bend | 13 | 5 | 2.17 / 2.97 |
| Falling Boxes: Twist | 13 | 5 | 1.90 / 5.85 |
| Falling Boxes: Displace | 13 | 5 | 1.98 / 14.15 |
| Pile of Chairs | 28 | 188 | 4.09 / 9.42 |
| Spiky Wheel | 12 | 14 | 0.91 / 1.52 |
| Cows on Spikes | 20 | 285 | 4.27 / 10.92 |
| Bunny Pile | 28 | 857 | 6.97 / 11.71 |
| Primitives | 49 | 8 | 6.41 / 9.00 |

Bake time is the first load of a scene's shapes in a process (voxel SDFs are cached afterwards).

## Notes

- **Deterministic.** The engine is single threaded and seeds its random sampling with a fixed
  value, so the same scene and inputs give bit-identical results (covered by a test).
- **Scale.** The scenes use the paper's units (objects about 1 m across). For tabletop scale,
  scale `simulation.root` rather than the bodies.
- **Threading.** Everything runs on the main actor. Baking voxel shapes blocks while a scene loads.
- **Shapes.** Only the built-in scenes and shape names are exposed so far. Baking arbitrary
  `MeshResource`s and USDZ assets is the next step.

## License

MIT for the upstream engine (see `Sources/SDFCore/upstream/LICENSE`). Please cite the paper when
using this work.
