# Upstream

The collision engine is vendored unmodified from
[savant117/sdf-dcd-demo](https://github.com/savant117/sdf-dcd-demo) (MIT License, Chris Giles).

Commit: `1dcdf1e7fa07d16fda15826ad8c2e6697075a76e`

- `Sources/SDFCore/upstream/`: the engine sources, byte for byte. Upstream's scenes, OpenGL
  renderer, SDL window and ImGui UI (`scenes.*`, `main.cpp`, `render.*`, `gl.*`) are not vendored.
- `Sources/SDFDemoScenes/Resources/assets/`: the OBJ models, used only by the demo scenes.

Do not edit files in `upstream/`. Behavior belongs in the C facade (`SDFCore.cpp`), our solver
(`WorldStep.cpp`) or the Swift layer. To update, run `Scripts/sync-upstream.sh [commit]`.

## Our solver

Worlds are stepped by `WorldStep.cpp`, derived from upstream's `Solver::step` (`upstream/solver.cpp`).
It keeps upstream's bodies, manifolds and collision detection (`Manifold::update`, the paper's
Algorithm 1) and adds per body options: static and dynamic friction, restitution, damping, gravity,
axis locks, forces, collision filtering, triggers, and contact totals for events.

With every option at its default it takes bit-identical steps to upstream's solver, which
`SolverReferenceTests` checks through `sdf_world_step_reference`. After an upstream update, a failure
of that test means `Solver::step` changed and `WorldStep.cpp` needs the same change.
