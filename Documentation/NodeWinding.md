# Opt-in winding classification at actual voxel nodes

```swift
let shape = try SDFShape(
    baking: positions,
    indices: indices,
    resolution: 64,
    signMethod: .nodeWinding
)
```

The existing initializer and `.legacy` retain the upstream bake unchanged. `.nodeWinding` is an
opt-in C-facade correction; no vendored upstream files are modified. It computes a double-precision
solid-angle winding sum at each actual grid node, changes the stored distance and gradient signs
where needed, then regenerates mass properties and curvature seeds from the corrected field.

This addresses a concrete thin-box failure: upstream sign votes sample displaced ray lines but
apply votes to unjittered nodes. For a 12.4 × 0.2 × 4 m box at 64 cells across its longest side, the
only interior grid layer can be voted outside. The legacy baked box reports positive distance at
its centre and at y = ±0.09 m; node winding restores negative distance at those tested points.

## Precondition: the thinnest part must contain a grid node

Node winding only corrects the sign of nodes that exist. The baker scales the mesh so its longest
bounding-box side spans `resolution` cells (`bakedGridInfo.cellSize` = longest side / resolution,
in the shape's units) and places nodes at the bounds' minimum plus whole cells. A part with no node
strictly inside it has no negative sample, and the interpolated field stays positive throughout
it: overlap queries see a phantom gap.

- An axis-aligned slab has an interior node only when its thickness exceeds one cell. A 0.2 m
  slab at resolution 64 keeps its interior up to a 12.8 m longest side, and loses it beyond
  (with 13–20 m sides, distances at the slab centre are positive, up to about +0.036 m).
- A slab at an arbitrary orientation is only guaranteed an interior node when its thickness is at
  least √3 cells.
- So choose **resolution ≥ 2 × longest side / thinnest thickness** (two cells across the
  thinnest part). `SDFShape.minimumResolution(forThickness:longestExtent:)` computes this.

To detect the failure after baking, compare the known thickness with `bakedGridInfo.cellSize`
(`thickness >= 2 * cellSize`), and check `bakedGridInfo.interiorSampleCount`: zero means no sample
of the field is inside, so no query can report penetration. A nonzero count does not prove that
every thin part kept its interior; only the cell-size comparison covers individual parts. The
same precondition applies to the legacy bake.

## What it does not guarantee

This is **node sign correction, not an exact continuous signed-distance or collision guarantee**.
The field still uses trilinear interpolation of a finite grid. Features with no interior grid
samples can still disappear; corners and finite-grid distances remain approximate. Choose a
resolution suitable for the smallest important geometry and validate critical contacts separately.

The opt-in initializer requires closed, consistently oriented welded triangle connectivity.
`NodeWindingBakeError.invalidMeshTopology` reports invalid topology. Unsafe normalization or grid
size arithmetic is rejected with `BakeError.invalidMesh`, before entering the legacy constructor.
The checks do not promise that a valid but enormous grid fits available memory. Nested cavity
shells must be wound opposite to their enclosing material shell; validation does not infer the
intended material/void semantics or rule out self-intersections.

Cost is O(grid nodes × triangle count), **in addition to the legacy bake**, and includes a second
mass/curvature-seed finalization. Run large bakes off the main actor. This is a bake-time operation;
query APIs and the runtime overlap solver are unchanged.

## Caching and compatibility

```swift
let shape = try SDFShape.cached(
    baking: positions,
    indices: indices,
    resolution: 64,
    signMethod: .nodeWinding
)
```

The explicit-method `bakeKey` overload preserves old keys for `.legacy` and uses a separate
versioned domain for `.nodeWinding`, preventing corrected requests from loading legacy entries.
Generic `cached(key:...)` remains caller-keyed; callers supplying their own keys must include the
chosen bake method in their own key policy.

The baked-data format/version is unchanged. It already stores distances, gradients, mass
properties and seeds, so corrected data round-trips through the existing loader without needing
method metadata. There is no sign-method flag or automatic legacy cache invalidation. Existing
calls continue to use the existing field and existing cache entries.
