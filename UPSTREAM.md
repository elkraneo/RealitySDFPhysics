# Upstream

The collision engine is vendored unmodified from
[savant117/sdf-dcd-demo](https://github.com/savant117/sdf-dcd-demo) (MIT License, Chris Giles).

Commit: `1dcdf1e7fa07d16fda15826ad8c2e6697075a76e`

- `Sources/SDFCore/upstream/`: the engine sources, byte for byte. Upstream's scenes, OpenGL
  renderer, SDL window and ImGui UI (`scenes.*`, `main.cpp`, `render.*`, `gl.*`) are not vendored.
- `Sources/SDFDemoScenes/Resources/assets/`: the OBJ models, used only by the demo scenes.

Do not edit files in `upstream/`. Behavior belongs in the C facade (`SDFCore.cpp`) or the Swift
layer. To update, run `Scripts/sync-upstream.sh [commit]`.
