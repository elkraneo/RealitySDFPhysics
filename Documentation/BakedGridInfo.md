# Baked grid diagnostics

`SDFShape.bakedGridInfo` reports the actual voxel sample dimensions, spacing and live sample
payload of a baked shape. Analytical shapes return `nil`. The getter reads existing immutable
metadata; it does not serialize or copy the grid. Loading `bakedData()` preserves the information.
The C facade exposes the same values through `sdf_shape_baked_grid_info`.

```swift
if let info = shape.bakedGridInfo {
    print(info.sampleDimensions) // Includes boundary samples and padding
    print(info.cellSize)         // Original shape-local units, normalization undone
    print(info.sampleCount)
    print(info.gridPayloadBytes)
}
```

## Grid dimensions are cropped to the mesh bounds

The baker normalizes the mesh's longest side to one unit, sets `cell = 1 / resolution`, and creates
each grid dimension from that axis's actual normalized extent:

```
samples[axis] = ceil(normalizedExtent[axis] / cell) + 1 + 2 * padding
padding = 3
```

Thus `resolution³` is not the sample count. Short axes have fewer samples; every axis also includes
boundary/padding samples. An earlier cube-per-shape memory estimate for the placement experiment
was incorrect. The live grid dimensions, rather than that estimate, should guide its resolution
and storage evaluation. No vendored engine or serialization format changes were made for this API.

For a 10 × 2 × 0.2 unit box at resolution 64, the metadata test reports:

| Measurement | Value |
| --- | ---: |
| Sample dimensions | 71 × 20 × 9 |
| Samples | 12,780 |
| Local cell size | 0.15625 units |
| Live sample payload | 204,480 bytes (about 0.195 MiB) |

The corresponding 10 × 10 × 10 cube has 71³ samples and a 5,726,576 byte payload. This example
is not a measurement of the game's building fixture. Rotating a mesh can change its axis-aligned
bounds and therefore its grid dimensions even if its geometry has the same volume.

## Coordinate units and memory limits

`cellSize` is `voxel.cell / shape.unitScale`: the original caller's shape-local units. It does not
know a body's entity transform. For simulation-root units, multiply by the body's uniform scale;
for physical meters multiply that result by the simulation's meters-per-unit. A rendered RealityKit
world transform is a separate coordinate conversion. Grid spacing is not a collision-error bound,
contact margin, or guarantee that every thin feature is resolved.

`gridPayloadBytes` is `grid.size() * sizeof(float4)`, covering the live distance/gradient samples.
It **excludes** spare vector capacity, retained meshes and indices, curvature seeds, solver bodies,
query buffers, transient baking allocations, rendering resources and process/runtime costs. It is
neither process RSS nor peak memory footprint. To estimate the live grid payload of a scene, sum
across its distinct shapes; bodies can share one immutable shape, so multiplying by body count can
overcount storage. Process memory must be measured separately with the appropriate profiling tool.

Four tests cover nil metadata for analytical/null shapes, exact cropped dimensions with padding,
source scaling and normalization, thin-versus-cubic payload, and saved-data round trips.
