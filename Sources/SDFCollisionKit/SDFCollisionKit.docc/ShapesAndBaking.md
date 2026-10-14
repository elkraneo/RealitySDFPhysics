# Shapes and Baking

Describe a body's geometry with an analytical shape or a baked mesh.

## Overview

An ``SDFShape`` is a signed distance field: for any point, the distance to the surface, negative
inside. Shapes are immutable, can be created on any thread, and can be shared by any number of
bodies. A body's shape is given in its entity's frame and scales with the entity.

## Analytical shapes

Primitives are exact and cheap to query:

- ``SDFShape/box(halfExtents:)``, ``SDFShape/sphere(radius:)``
- ``SDFShape/capsule(radius:halfLength:)``, ``SDFShape/cylinder(radius:halfHeight:)`` and
  ``SDFShape/cone(radius:halfHeight:)``, along y
- ``SDFShape/torus(majorRadius:minorRadius:)``, around y

The paper's test shapes are concave composites and distorted boxes:
``SDFShape/spikePlate(halfExtents:count:spikeRadius:spikeHeights:)``,
``SDFShape/spikyWheel(hubRadius:hubHalfWidth:spikeRadius:spikeLength:spikes:)`` and
``SDFShape/warpedBox(_:halfExtents:amount:)``.

## Baked shapes

``SDFShape/bake(from:resolution:)`` reads the meshes of an entity and its descendants on the main
actor and bakes them in the background into a voxel grid with `resolution` cells across the
longest side. ``SDFShape/init(baking:indices:resolution:)`` bakes a triangle list you provide.

```swift
let chair = try await Entity(named: "Chair", in: realityKitContentBundle)
let shape = try await SDFShape.bake(from: chair, resolution: 64)
```

- **Close the mesh.** Inside and outside are decided from the triangles, so the meshes should
  form a closed surface. ``SDFShape/isWatertight`` is false when some edges are not shared by two
  triangles. Vertices duplicated at normal and texture seams are welded first, so split vertices
  are fine.
- **Choose a resolution.** Features thinner than about two cells are lost. Baking time and memory
  grow with the cube of the resolution: 100 cells take up to about a second on an Apple M1.
- **Bake once.** Share a shape between all the bodies that use it, and keep it around across scene
  loads.

## Queries

``SDFShape/distance(at:)`` evaluates the field and its gradient at a point of the shape's frame.
``SDFShape/volume`` is the enclosed volume, from which the default mass comes, and
`MeshResource.generate(from:)` builds a display mesh of the surface, useful to see what a baked
shape looks like to the simulation.
