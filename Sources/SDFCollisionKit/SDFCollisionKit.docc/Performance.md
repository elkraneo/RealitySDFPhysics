# Performance

What a step costs, where it runs, and the current limits.

## Overview

The simulation steps on the main actor, inside RealityKit's update. Collision detection, which
takes most of a step, runs on several threads (``SDFSimulationComponent/isMultithreaded``); the
contact solver runs on the main actor. Baking runs in the background.

The engine seeds its random sampling with fixed values, so the same bodies and inputs give
bit-identical results on every run. Multithreaded, every pair of bodies draws from its own seeded
sequence, so results don't depend on the number of threads either. They do differ from a single
thread's, which follows upstream's single random sequence.

## Measurements

The demo scenes of the paper, rebuilt on this package, on an Apple M1 (4 performance and 4
efficiency cores) in a release build: 240 steps at 60 steps per second with the paper's settings
(``SDFSimulationComponent/Preset/paper``: 4 substeps of 20 and 20 iterations). The default
``SDFSimulationComponent/Preset/realTime`` costs about half as much. Times
vary by up to about 20% from run to run on the same machine; run
`swift run -c release sdf-snapshot out/` to measure yours, and add `--single-threaded` for the
other column.

| Scene | Bodies | Step ms, multithreaded | Step ms, one thread |
| --- | ---: | ---: | ---: |
| Box Stack | 29 | 1.7 | 1.7 |
| Gear Stack | 11 | 0.5 | 0.6 |
| Falling Boxes (plain, bent, twisted, displaced) | 13 | 0.6–1.0 | 0.8–2.3 |
| Pile of Chairs | 28 | 1.6 | 4.5 |
| Spiky Wheel | 12 | 0.7 | 0.9 |
| Cows on Spikes | 20 | 1.4 | 4.1–5.9 |
| Bunny Pile | 28 | 2.4 | 8.5–9.8 |
| Primitives | 49 | 2.7 | 6.6 |

At 60 frames per second a frame has about 16 ms, so piles of a few dozen concave bodies take a
small part of a frame. Scenes of convex shapes gain little from threads, since their pairs are
cheap to begin with.

## Keep it fast

- Use analytical shapes where they fit; they are cheaper to query than baked voxel shapes.
- Separate bodies that never touch with collision filters, so their pairs are skipped.
- Lower ``SDFSimulationComponent/substeps`` and the solver iterations for scenes that don't stack.
- Bake each mesh once and share the shape.
- Keep ``SDFSimulationComponent/isSleepingEnabled`` on. In a box of tori, a pile of 400 asleep
  takes 4.4 ms a step, against 22 ms awake, most of it the broad phase, which still runs.

## Limits

- **Discrete collision detection.** Contacts are found at each substep, so a fast, thin body can
  pass through another between two substeps. Raise the substeps, or keep such bodies thicker.
- **Solver on the main actor.** The contact solver is sequential; with many contacts, it becomes
  the larger part of a multithreaded step.
- **No joints yet.**
- **Uniform scale.** A body's entity can be scaled, but uniformly.
