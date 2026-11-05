# Checking a mesh before signed-distance baking

`SDFShape.validateMesh(positions:indices:)` checks a triangle mesh without allocating a voxel grid.
It welds seam vertices using the baker's extent-based quantization and reports invalid indices,
incomplete triangles, nonfinite coordinates, degenerate triangles, open edges, nonmanifold edges
and inconsistent traversal of shared edges.

A mesh can be closed while some faces point inward. Check both `isClosed` and
`isConsistentlyOriented` before relying on the signed interior. Existing `isWatertight` describes
closed connectivity; it is not a winding guarantee. The new diagnostics do not mutate input or
change the existing bake initializer's behavior.

For example, when extruding a polygon along normal N, the starting cap must point against N, the
translated cap along N, and the side faces outward. Copying the original polygon's arbitrary
winding onto the starting cap can create a closed slab whose caps disagree with its sides.
The geometry then encloses the intended volume, yet the winding-based voxel interior can have
positive distance where negative distance was expected. More random overlap samples cannot
repair the field's input orientation.

The placement reproduction contained six such slabs. Correcting their cap winding restored all
six overlap queries at both 64 and 128 cells. A local full-building fixture also verified all 57
wall-centre overlaps at both resolutions and negative known-interior samples at 128 cells. The
fixture is not shipped in this library; optional local verification uses `SDF_PLACEMENT_FIXTURE`.
The regular regression tests contain only six wall shapes rebased to local centroids.

These checks are necessary, not a signed-distance accuracy certificate. They do not detect
self-intersections, establish the intended material/void orientation of disconnected or nested
shells, or ensure a voxel grid captures thin features. In the corrected-wall reproduction, one
known interior sample was still positive at 64 cells, even though the panel overlap was detected;
at 128 cells every checked interior sample was negative. Choose resolution relative to feature
thickness and test critical shapes independently.
