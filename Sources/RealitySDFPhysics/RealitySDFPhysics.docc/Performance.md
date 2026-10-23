# Performance

What a step costs, where it runs, and the current limits.

## Overview

The simulation steps on the main actor, inside RealityKit's update. With
``SDFPhysicsSimulationComponent/isMultithreaded``, collision detection, which takes most of a step, runs
on several threads, and so does the contact solver: its contacts are colored so that no two of a
color share a moving body, and each color is solved at once on the performance cores but one.
Baking runs in the background.

The engine seeds its random sampling with fixed values, so the same bodies and inputs give
bit-identical results on every run. Multithreaded, every pair of bodies draws from its own seeded
sequence, and the solver takes its contacts in the order of their colors, so results don't depend
on the number of threads either. They do differ from a single thread's, which follows upstream's
single random sequence and solves the contacts in the order of the pairs.

## Measurements

The demo scenes of the paper, rebuilt on this package, on an Apple M1 (4 performance and 4
efficiency cores) in a release build: 240 steps at 60 steps per second with the paper's settings
(``SDFPhysicsSimulationComponent/Preset/paper``: 4 substeps of 20 and 20 iterations). The default
``SDFPhysicsSimulationComponent/Preset/realTime`` costs about half as much. Times
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
- Lower ``SDFPhysicsSimulationComponent/substeps`` and the solver iterations for scenes that don't stack.
- Bake each mesh once and share the shape.
- Keep ``SDFPhysicsSimulationComponent/isSleepingEnabled`` on. In a box of tori, a pile of 400 asleep
  takes 1.7 ms a step on an M1, against about 10 ms awake. While every body is asleep or static,
  the broad phase and the collision detection are skipped.
- Keep ``SDFPhysicsSimulationComponent/isMultithreaded`` on. Besides the threads, it lets the broad phase
  test bounding spheres after the boxes, which rejects more than half of the pairs of a pile of
  tori before the narrow phase, and it skips the scan of the curvature seeds for a pair of tori
  or capsules, whose seeds cover the whole surface and add nothing over uniform samples. The
  single-threaded path is the reference's, pair for pair and sample for sample.
- Subscribe to collision events only where they are used. While nothing subscribes, the pairs
  in contact are not tracked, which at 400 tori is a third of the synchronization.

## Limits

- **Discrete collision detection.** Contacts are found at each substep, so a fast, thin body can
  pass through another between two substeps. Raise the substeps, or keep such bodies thicker.
- **Scalar solver.** The contact solver takes one contact at a time; with many contacts, it stays
  a large part of a multithreaded step. On an M1, 400 tori take about 2.5 ms a step for the solver,
  against 6 ms on one thread.
- **Tall stacks.** With the default preset, a tower of 10 boxes topples within 10 s; with
  ``SDFPhysicsSimulationComponent/Preset/paper``, a tower of 20 stands and one of 40 topples. In
  traces of a buckling tower, the contact points between two boxes bunched on one side of the
  face, which likely leaves the column a one sided support. RealityKit's physics stacks as high as the paper preset for a
  fraction of the cost, and Box3D higher.
- **Uniform scale.** A body's entity can be scaled, but uniformly.
