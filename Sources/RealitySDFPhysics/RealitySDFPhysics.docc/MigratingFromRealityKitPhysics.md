# Migrating from RealityKit Physics

Move entities from RealityKit's physics to SDF bodies, and know what differs.

## Overview

RealitySDFPhysics mirrors the names and shapes of RealityKit's physics API, and reuses its
`PhysicsBodyMode`, `CollisionFilter`, `CollisionGroup`, `CollisionCastQueryType` and
`PhysicsSimulationComponent` option types. The two simulations are separate: SDF bodies and
RealityKit physics bodies don't push each other, so give each moving entity one or the other.
For the static and kinematic colliders a scene already has, such as floors, walls and tables,
`addSDFColliders(mode:material:resolution:)` adds SDF bodies for an entity's `CollisionComponent`
shapes, so that SDF bodies collide with them too. Axis-aligned boxes, spheres and capsules become
exact analytical shapes; other shapes are baked from their meshes with
``SDFShape/make(from:scale:resolution:)``.

## Counterparts

| RealityKit | RealitySDFPhysics |
| --- | --- |
| `PhysicsSimulationComponent` | ``SDFPhysicsSimulationComponent`` |
| `PhysicsBodyComponent` with `CollisionComponent` | ``SDFPhysicsBodyComponent``, whose ``SDFPhysicsBodyComponent/shape`` replaces the collision shapes |
| `ShapeResource` | ``SDFShape`` |
| `PhysicsMaterialResource.generate(staticFriction:dynamicFriction:restitution:)` | ``SDFPhysicsMaterial/init(staticFriction:dynamicFriction:restitution:)`` |
| `PhysicsMassProperties` | ``SDFPhysicsMassProperties`` |
| `PhysicsMotionComponent` | ``SDFPhysicsMotionComponent`` |
| `CollisionComponent.Mode.trigger` | ``SDFPhysicsBodyComponent/isTrigger`` |
| `HasPhysicsBody` forces and impulses | ``SDFPhysics``, from `entity.sdf` |
| `CollisionEvents` | ``SDFCollisionEvents`` |
| `PhysicsSimulationEvents` | ``SDFPhysicsSimulationEvents`` |
| `scene.subscribe(to:on:_:)` | ``SDFSimulation/subscribe(to:on:_:)`` |
| `scene.raycast(origin:direction:length:query:mask:relativeTo:)` | ``SDFSimulation/raycast(origin:direction:length:query:mask:)`` |
| `scene.convexCast(...)` | ``SDFSimulation/overlaps(of:at:margin:mask:)`` tests a pose rather than sweeping a path |

## Step by step

1. Put an ``SDFPhysicsSimulationComponent`` on the entity that contains the bodies. Move the settings of a
   `PhysicsSimulationComponent`: `gravity`, `solverIterations` and `collisionOptions` keep their
   names and types.
2. Replace each `PhysicsBodyComponent` and `CollisionComponent` pair with an ``SDFPhysicsBodyComponent``.
   Bake the entity's meshes with ``SDFShape/bake(from:resolution:)``, or use an analytical shape.
   `mode`, `massProperties`, `material`, the translation and rotation locks,
   `isAffectedByGravity`, `linearDamping` and `angularDamping` keep their names. The collision
   filter moves from `CollisionComponent` to ``SDFPhysicsBodyComponent/filter``.
3. Remove the RealityKit components from those entities, so that RealityKit doesn't simulate
   them too.
4. Replace `PhysicsMotionComponent` with ``SDFPhysicsMotionComponent``, and calls such as
   `entity.addForce(_:relativeTo:)` with `entity.sdf.addForce(_:relativeTo:)`.
5. Subscribe to collision events through the simulation instead of the scene.

## Differences

- **Frame and units.** A simulation works in its root entity's local frame. Contacts, query
  results, velocities and the locked axes are in that frame. ``SDFPhysics`` converts from the
  reference entity's space as RealityKit does.
- **Object size.** RealityKit's physics works for objects of any size in meters. Here the
  collision tolerances suit objects about 1 unit across: model small or large objects at about
  that size in a scaled root, and set ``SDFPhysicsSimulationComponent/metersPerUnit``.
- **Defaults.** 60 steps per second of 2 substeps each, 10 velocity and 4 position iterations
  (``SDFPhysicsSimulationComponent/Preset/realTime``; ``SDFPhysicsSimulationComponent/Preset/paper`` has the
  paper's 4 substeps and 20 and 20 iterations), friction 0.5, no restitution, linear damping 0.05 and angular
  damping 0.3 per second, and the mass of the shape's volume of water.
- **Material combination.** A contact's friction is the geometric mean of the two bodies'
  coefficients, and its restitution the larger of the two. Contacts approaching slower than
  ``SDFPhysicsSimulationComponent/restitutionThreshold`` don't bounce. Faster ones bounce pair by
  pair before the contacts are solved, so a row of touching bodies passes a bounce on whole, as a
  Newton's cradle does; RealityKit shares it out among the row.
- **Small bodies.** The contact margin and the restitution threshold are sized for bodies of about
  a meter. For bodies of a few centimeters, scale ``SDFCollisionDetection/contactMargin``,
  ``SDFCollisionDetection/allowedPenetration`` and the threshold down with them, as the Newton's
  cradle sample does, or touching bodies are held apart by the margin.
- **Mass properties.** The center of mass and inertia always come from the shape at a uniform
  density; you set the mass or the density.
- **Scale.** Bodies scale with their entities, but the scale must be uniform.
- **Events.** Events are delivered while the simulation steps, after the entities have their new
  poses. A pair is in contact for a step when it touched in any of the step's substeps. `Began`
  and `Updated` carry the step's total normal impulse, summed over the substeps, and the contacts
  of the last substep. Handlers can't step the simulation: `update` and `step` do nothing when
  called from them.
- **Not yet available:** joints, continuous collision detection (fast thin bodies can pass
  through others), sweeps of a shape along a path, and the simulation `clock`.
- **Sleeping.** Bodies at rest fall asleep and wake when touched, moved or pushed. Turn it off
  with ``SDFPhysicsSimulationComponent/isSleepingEnabled``, and ask ``SDFSimulation/isSleeping(_:)``.
  While asleep, a pair in contact keeps reporting `Updated` with the contacts and impulses it had
  when it fell asleep.
