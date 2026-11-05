# Sampled shape casts

`SDFSimulation.sampledShapeCast` queries the same world snapshot as `overlaps`, along a linear
translation and shortest-arc quaternion rotation. It supports a target collision-group mask and
exact-entity exclusions. Results include the queried contacts, their entities, the sampled pose
and motion fraction. It does not move a body or advance simulation time.

```swift
let result = try simulation.sampledShapeCast(
    of: panelShape, from: oldPose, to: requestedPose,
    maxTranslationStep: 0.01, maxRotationStep: .pi / 180,
    maxSamples: 256, mask: [.default], excluding: [heldPanel])
switch result {
case let .hit(hit):
    // A contact at this sampled pose; not an exact first time of impact.
    print(hit.fraction, hit.contacts.map(\.entity))
case .noHitInSamples:
    // The requested sampling completed without finding contact. Gaps remain unchecked.
    break
case .incomplete:
    // Budget expired before reaching the requested destination; do not accept as clear.
    break
}
```

The start is always checked; zero motion uses one query. The destination is included when the
sample budget permits: any change of pose, however small, is sampled at both ends, and the last
sample is exactly the requested destination. The rotation angle is measured in double precision,
so rotations far below a step (or below Float's `acos` resolution) are not mistaken for none. A collision at the start has fraction zero and no preceding sample.
The default budget, 512 samples at 1 cm, reaches the destination of a translation of
at most about 5.1 m; longer motions return `.incomplete` unless `maxSamples` or the step is raised.
Positive uniform scale must stay unchanged. Invalid transforms, steps, margin or budget throw.
Queries inspect already-synchronized bodies, including triggers; pending RealityKit changes must
be synchronized through the normal simulation update first. Exclusions filter results, not query
work, and do not implicitly exclude descendants. Mask selection follows `overlaps` semantics:
it matches target groups, rather than applying a querying body's reciprocal collision filter.

## It is not an exact shape sweep

A thin obstacle can be entirely between two sampled poses. The overlap query itself operates on
the supplied SDF geometry and approximate contact search. A previous clear sample therefore is
not a guaranteed safe pose, and `noHitInSamples` is deliberately not named `clear` or `noCollision`.
The rotation step is angular, not a bound on a shape corner's travel; large or offset shapes need
smaller angular steps. There is no automatic sliding, snapping, support or engineering rule here.

The regression suite includes a thin wall that coarse sampling misses but fine sampling detects.
Other tests cover crossing with clear endpoints, pure rotation, the quaternion sign boundary,
initial and destination contacts, query masks, self exclusion, incomplete budgets and validation.
These analytic fixtures establish API behavior, not an accuracy bound for arbitrary baked shapes.

## What continuous casts still require

True conservative advancement needs a trustworthy lower bound on distance between the two shapes,
not just an overlap optimizer that found no contact. A closest-point search that returns an
approximate candidate distance may overestimate the true minimum; using it as a safe step can
skip a thin obstacle. Voxel interpolation and baking errors must also be bounded or handled with
an explicit conservative dilation.

A future continuous implementation needs:

- A narrow-phase separation lower bound with a stated error model for analytic and baked SDFs.
- A shape bound about the rotation pivot, to bound surface-point speed by translation plus angular
  speed times radius. That bound must include scale and off-centre geometry.
- Broadphase candidates covering the whole translation and rotation, not only endpoint boxes.
- A bounded conservative-advancement/refinement algorithm, with explicit initial-overlap and
  numerical/budget-failure results. A failure must not become an unchecked clear answer.
- Tests against independently known times of impact for thin walls, grazing/rotational contact,
  near-parallel motion, concave shapes, masks and excluded entities.

The sampled implementation provides an engine-backed comparison tool now. It is not the final
continuous sweep requested for a physical drag/contact mechanic.
