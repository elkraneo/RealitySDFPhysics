# Upstream

The collision engine is vendored unmodified from
[savant117/sdf-dcd-demo](https://github.com/savant117/sdf-dcd-demo) (MIT License, Chris Giles).

Commit: `1dcdf1e7fa07d16fda15826ad8c2e6697075a76e`

- `Sources/SDFCore/upstream/`: the engine and scene sources, byte for byte. The OpenGL renderer,
  SDL window and ImGui UI (`main.cpp`, `render.*`, `gl.*`) are not vendored.
- `Sources/SDFCollisionKit/Resources/assets/`: the OBJ models.
- `Sources/SDFCore/shim/`: stand-ins for `SDL.h` (asset path and logging) and `render.h` (the
  orbit camera), so `scenes.cpp` compiles without SDL or OpenGL.

Do not edit files in `upstream/`. Changes in behavior go in the C facade (`SDFCore.cpp`) or the
Swift layer. To update, run `Scripts/sync-upstream.sh [commit]`.
