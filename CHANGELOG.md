# Changelog

## 0.4.0

New public API, so a minor release. Existing initializers, cache keys and baked data are unchanged.

### Added

- `SDFShape.validateMesh(positions:indices:)` and `SDFMeshValidation`: welded connectivity and
  winding checks before baking (open, nonmanifold and inconsistently oriented edges, invalid
  indices, nonfinite positions, degenerate triangles), without allocating a grid.
  See [MeshValidation.md](Documentation/MeshValidation.md).
- `SDFSimulation.sampledShapeCast(of:from:to:...)`: overlap queries sampled along a translation
  and rotation, returning the first sampled contact, a clear result for the samples taken, or an
  explicit `.incomplete` when the budget ends. Not continuous collision detection. The default
  budget (512 samples at 1 cm) covers translations of up to about 5.1 m.
  See [SampledShapeCast.md](Documentation/SampledShapeCast.md).
- `SDFShape.bakedGridInfo`: sample dimensions, cell size, sample count, grid payload bytes and
  interior sample count of a baked shape. See [BakedGridInfo.md](Documentation/BakedGridInfo.md).
- `SDFShape(baking:indices:resolution:signMethod:)` with `.nodeWinding`: an opt-in correction of
  node signs by solid-angle winding, for thin parts that the legacy bake votes outside, with
  matching `bakeKey` and `cached` overloads. It needs a grid node inside the thinnest part;
  `SDFShape.minimumResolution(forThickness:longestExtent:)` gives the resolution, and
  `bakedGridInfo.interiorSampleCount` of zero shows a field with no inside. The node pass runs in
  parallel and cannot be cancelled. See [NodeWinding.md](Documentation/NodeWinding.md).

### Changed

- Ray and overlap queries: `.nearest` and `.any` pick the nearest hit without sorting all of them,
  empty masks and worlds return at once, and native scratch buffers are not zero-filled. Results
  are unchanged. See [QueryPerformance.md](Documentation/QueryPerformance.md).

## 0.3.1 and earlier

See the [releases](https://github.com/elkraneo/RealitySDFPhysics/releases) and tags.
