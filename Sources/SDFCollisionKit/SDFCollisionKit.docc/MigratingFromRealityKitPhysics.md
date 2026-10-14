# Migrating from RealityKit Physics

Move entities from RealityKit's physics to SDF bodies, and know what differs.

## Overview

SDFCollisionKit mirrors the names and shapes of RealityKit's physics API, and reuses its
`PhysicsBodyMode`, `CollisionFilter`, `CollisionGroup`, `CollisionCastQueryType` and
`PhysicsSimulationComponent` option types. The two simulations are separate: SDF bodies and
RealityKit physics bodies don't collide with each other, so give each entity one or the other.

## Counterparts

| RealityKit | SDFCollisionKit |
| --- | --- |
| `PhysicsSimulationComponent` | ``SDFSimulationComponent`` |
| `PhysicsBodyComponent` with `CollisionComponent` | ``SDFBodyComponent``, whose ``SDFBodyComponent/shape`` replaces the collision shapes |
| `ShapeResource` | ``SDFShape`` |
| `PhysicsMaterialResource.generate(staticFriction:dynamicFriction:restitution:)` | ``SDFMaterial/init(staticFriction:dynamicFriction:restitution:)`` |
| `PhysicsMassProperties` | ``SDFMassProperties`` |
| `PhysicsMotionComponent` | ``SDFMotionComponent`` |
| `CollisionComponent.Mode.trigger` | ``SDFBodyComponent/isTrigger`` |
| `HasPhysicsBody` forces and impulses | ``SDFPhysics``, from `entity.sdfPhysics` |
| `CollisionEvents` | ``SDFCollisionEvents`` |
| `PhysicsSimulationEvents` | ``SDFSimulationEvents`` |
| `scene.subscribe(to:on:_:)` | ``SDFSimulation/subscribe(to:on:_:)`` |
| `scene.raycast(origin:direction:length:query:mask:relativeTo:)` | ``SDFSimulation/raycast(origin:direction:length:query:mask:)`` |
| `scene.convexCast(...)` | ``SDFSimulation/overlaps(of:at:margin:mask:)`` tests a pose rather than sweeping a path |

## Step by step

1. Put an ``SDFSimulationComponent`` on the entity that contains the bodies. Move the settings of a
   `PhysicsSimulationComponent`: `gravity`, `solverIterations` and `collisionOptions` keep their
   names and types.
2. Replace each `PhysicsBodyComponent` and `CollisionComponent` pair with an ``SDFBodyComponent``.
   Bake the entity's meshes with ``SDFShape/bake(from:resolution:)``, or use an analytical shape.
   `mode`, `massProperties`, `material`, the translation and rotation locks,
   `isAffectedByGravity`, `linearDamping` and `angularDamping` keep their names. The collision
   filter moves from `CollisionComponent` to ``SDFBodyComponent/filter``.
3. Remove the RealityKit components from those entities, so that RealityKit doesn't simulate
   them too.
4. Replace `PhysicsMotionComponent` with ``SDFMotionComponent``, and calls such as
   `entity.addForce(_:relativeTo:)` with `entity.sdfPhysics.addForce(_:relativeTo:)`.
5. Subscribe to collision events through the simulation instead of the scene.

## Differences

- **Frame and units.** A simulation works in its root entity's local frame. Contacts, query
  results, velocities and the locked axes are in that frame. ``SDFPhysics`` converts from the
  reference entity's space as RealityKit does.
- **Object size.** RealityKit's physics works for objects of any size in meters. Here the
  collision tolerances suit objects about 1 unit across: model small or large objects at about
  that size in a scaled root, and set ``SDFSimulationComponent/metersPerUnit``.
- **Defaults.** 60 steps per second of 2 substeps each, 10 velocity and 4 position iterations
  (``SDFSimulationComponent/Preset/realTime``; ``SDFSimulationComponent/Preset/paper`` has the
  paper's 4 substeps and 20 and 20 iterations), friction 0.5, no restitution, linear damping 0.05 and angular
  damping 0.3 per second, and the mass of the shape's volume of water.
- **Material combination.** A contact's friction is the geometric mean of the two bodies'
  coefficients, and its restitution the larger of the two. Contacts approaching slower than
  ``SDFSimulationComponent/restitutionThreshold`` don't bounce.
- **Mass properties.** The center of mass and inertia always come from the shape at a uniform
  density; you set the mass or the density.
- **Scale.** Bodies scale with their entities, but the scale must be uniform.
- **Events.** Events are delivered while the simulation steps, after the entities have their new
  poses. A pair is in contact for a step when it touched in any of the step's substeps. `Began`
  and `Updated` carry the step's total normal impulse, summed over the substeps, and the contacts
  of the last substep. Handlers can't step the simulation: `update` and `step` do nothing when
  called from them.
- **Not yet available:** joints, continuous collision detection (fast thin bodies can pass
  through others), sleeping, sweeps of a shape along a path, and the simulation `clock`.
