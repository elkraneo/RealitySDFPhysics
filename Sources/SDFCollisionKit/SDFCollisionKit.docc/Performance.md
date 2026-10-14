# Performance

What a step costs, where it runs, and the current limits.

## Overview

The simulation steps on the main actor, single threaded, inside RealityKit's update. Only baking
runs in the background. The engine seeds its random sampling with a fixed value, so the same bodies
and inputs give bit-identical results on every run.

## Measurements

The demo scenes of the paper, rebuilt on this package, on an Apple M1 in a release build: 240 steps
at 60 steps per second with 4 substeps each. Times vary by about 20% from run to run on the same
machine; run `swift run -c release sdf-snapshot out/` to measure yours.

| Scene | Bodies | Contacts at the end | Step ms, mean |
| --- | ---: | ---: | ---: |
| Box Stack | 29 | 196 | 1.8 |
| Gear Stack | 11 | 48 | 0.7 |
| Falling Boxes (plain, bent, twisted, displaced) | 13 | 47–54 | 0.9–2.3 |
| Pile of Chairs | 28 | 115 | 5.8 |
| Spiky Wheel | 12 | 40 | 0.9 |
| Cows on Spikes | 20 | 74 | 4.4 |
| Bunny Pile | 28 | 92 | 10.8 |
| Primitives | 49 | 187 | 7.0 |

At 60 frames per second a frame has about 16 ms, so piles of a few dozen concave bodies fit in a
frame with room to spare, and the heaviest scenes take most of it on an M1.

## Keep it fast

- Use analytical shapes where they fit; they are cheaper to query than baked voxel shapes.
- Separate bodies that never touch with collision filters, so their pairs are skipped.
- Lower ``SDFSimulationComponent/substeps`` and the solver iterations for scenes that don't stack.
- Bake each mesh once and share the shape.

## Limits

- **Discrete collision detection.** Contacts are found at each substep, so a fast, thin body can
  pass through another between two substeps. Raise the substeps, or keep such bodies thicker.
- **No sleeping yet.** Bodies at rest are simulated like moving ones.
- **No joints yet.**
- **Uniform scale.** A body's entity can be scaled, but uniformly.
