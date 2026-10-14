import Foundation
import RealityKit
internal import SDFCore
import simd

/// A rigid body world whose collision detection is SDF-SDF optimization with temporal caching and
/// importance sampling (Giles and Andrews, Pacific Graphics 2026).
///
/// Set it on a root entity with `SDFSimulationComponent`. Every frame it adds, updates and removes
/// bodies for the root's descendants with an `SDFBodyComponent`, advances in fixed steps and writes
/// the poses of dynamic bodies back to their entities. Everything happens in the root's local
/// frame: scale the root to scale the whole simulation.
///
/// RealityKit's own physics is not involved.
@MainActor
public final class SDFSimulation {
    nonisolated(unsafe) let world: OpaquePointer

    public var isPaused = false

    /// Upper bound on the steps taken in one frame, so that slow frames do not spiral
    public var maximumStepsPerFrame = 4

    /// Steps taken so far
    public private(set) var stepCount = 0

    private var bodies: [Entity.ID: Body] = [:]
    private var entityIDs: [SDFBodyID: Entity.ID] = [:]
    private var accumulator: TimeInterval = 0

    public init() {
        _ = Self.systemRegistration
        world = sdf_world_create()
    }

    deinit {
        sdf_world_destroy(world)
    }

    private static let systemRegistration: Void = SDFSimulationSystem.registerSystem()

    // MARK: Stepping

    /// Synchronizes the bodies with the root's descendants and advances by real time in fixed
    /// steps. `SDFSimulationSystem` calls this every frame.
    public func update(root: Entity, deltaTime: TimeInterval) {
        let dynamic = synchronize(root: root, deltaTime: deltaTime)
        guard !isPaused else { return }
        let timeStep = TimeInterval(parameters.timeStep)
        accumulator += min(deltaTime, 0.25)
        var steps = 0
        while accumulator >= timeStep && steps < max(maximumStepsPerFrame, 1) {
            sdf_world_step(world)
            accumulator -= timeStep
            steps += 1
        }
        // Drop the whole steps a slow frame could not catch up on, keep the fraction
        accumulator = accumulator.truncatingRemainder(dividingBy: timeStep)
        stepCount += steps
        if steps > 0 { writeBack(dynamic, root: root) }
    }

    /// Synchronizes and takes exactly one step, also while paused
    public func step(root: Entity) {
        let dynamic = synchronize(root: root, deltaTime: TimeInterval(parameters.timeStep))
        sdf_world_step(world)
        stepCount += 1
        writeBack(dynamic, root: root)
    }

    // MARK: Bodies

    private struct Pose {
        var position: SIMD3<Float>
        var orientation: simd_quatf

        @MainActor init(of entity: Entity, relativeTo root: Entity) {
            position = entity.position(relativeTo: root)
            orientation = entity.orientation(relativeTo: root)
        }

        init(_ pose: SDFPose) {
            position = SIMD3(pose.position.0, pose.position.1, pose.position.2)
            orientation = simd_quatf(ix: pose.orientation.0, iy: pose.orientation.1, iz: pose.orientation.2, r: pose.orientation.3)
        }

        func engine(scale: Float) -> SDFPose {
            let q = orientation.vector
            return SDFPose(position: (position.x, position.y, position.z), orientation: (q.x, q.y, q.z, q.w), scale: scale)
        }

        func isClose(to other: Pose) -> Bool {
            simd_distance(position, other.position) < 1e-5 && abs(simd_dot(orientation.vector, other.orientation.vector)) > 1 - 1e-6
        }
    }

    private struct Body {
        var id: SDFBodyID
        /// Kept alive, so a new shape at the same address is not mistaken for it
        var shape: SDFShape
        var mode: SDFBodyComponent.Mode
        var scale: Float
        var defaultMass: Float
        var mass: Float?
        var friction: Float
        /// The entity's pose when last read or written
        var pose: Pose
        /// The motion component when last read or written
        var motion: SDFMotionComponent?
    }

    /// Updates the bodies from the root's descendants. Returns the entities of dynamic bodies.
    private func synchronize(root: Entity, deltaTime: TimeInterval) -> [Entity] {
        var dynamic: [Entity] = []
        var attached: Set<Entity.ID> = []
        func visit(_ entity: Entity) {
            for child in entity.children where !child.components.has(SDFSimulationComponent.self) {
                if let component = child.components[SDFBodyComponent.self] {
                    attach(child, component, root: root, deltaTime: deltaTime)
                    attached.insert(child.id)
                    if component.mode == .dynamic { dynamic.append(child) }
                }
                visit(child)
            }
        }
        visit(root)
        for entityID in bodies.keys where !attached.contains(entityID) {
            removeBody(of: entityID)
        }
        return dynamic
    }

    private func attach(_ entity: Entity, _ component: SDFBodyComponent, root: Entity, deltaTime: TimeInterval) {
        let pose = Pose(of: entity, relativeTo: root)
        let scale = entity.scale(relativeTo: root).x
        let motion = entity.components[SDFMotionComponent.self]
        guard var body = bodies[entity.id], body.shape === component.shape, body.mode == component.mode,
              abs(body.scale - scale) <= 1e-5 * scale
        else {
            removeBody(of: entity.id)
            addBody(of: entity, component, pose: pose, scale: scale, motion: motion)
            return
        }

        if body.friction != component.friction {
            sdf_world_set_body_friction(world, body.id, component.friction)
            body.friction = component.friction
        }
        if body.mass != component.mass {
            sdf_world_set_body_mass(world, body.id, component.mass ?? body.defaultMass)
            body.mass = component.mass
        }
        switch component.mode {
        case .static:
            if !pose.isClose(to: body.pose) { sdf_world_set_body_pose(world, body.id, pose.engine(scale: scale)) }
        case .kinematic:
            let (linear, angular) = velocity(from: body.pose, to: pose, over: deltaTime)
            sdf_world_set_body_pose(world, body.id, pose.engine(scale: scale))
            sdf_world_set_body_velocity(world, body.id, [linear.x, linear.y, linear.z], [angular.x, angular.y, angular.z])
        case .dynamic:
            // The entity moved since the last write: teleport the body
            if !pose.isClose(to: body.pose) { sdf_world_set_body_pose(world, body.id, pose.engine(scale: scale)) }
            if let motion, motion != body.motion { setVelocity(of: body.id, motion) }
        }
        body.pose = pose
        body.motion = motion
        bodies[entity.id] = body
    }

    private func addBody(of entity: Entity, _ component: SDFBodyComponent, pose: Pose, scale: Float,
                         motion: SDFMotionComponent?) {
        let mode: SDFBodyMode = switch component.mode {
        case .dynamic: SDFBodyDynamic
        case .kinematic: SDFBodyKinematic
        case .static: SDFBodyStatic
        }
        let id = sdf_world_add_body(world, component.shape.handle, pose.engine(scale: scale), mode)
        guard id != 0 else { return }
        let defaultMass = sdf_world_body_mass(world, id)
        if let mass = component.mass { sdf_world_set_body_mass(world, id, mass) }
        sdf_world_set_body_friction(world, id, component.friction)
        if let motion { setVelocity(of: id, motion) }
        bodies[entity.id] = Body(id: id, shape: component.shape, mode: component.mode, scale: scale,
                                 defaultMass: defaultMass, mass: component.mass, friction: component.friction,
                                 pose: pose, motion: motion)
        entityIDs[id] = entity.id
    }

    private func removeBody(of entityID: Entity.ID) {
        guard let body = bodies.removeValue(forKey: entityID) else { return }
        entityIDs[body.id] = nil
        sdf_world_remove_body(world, body.id)
    }

    private func writeBack(_ dynamic: [Entity], root: Entity) {
        for entity in dynamic {
            guard var body = bodies[entity.id] else { continue }
            let pose = Pose(sdf_world_body_pose(world, body.id))
            entity.setPosition(pose.position, relativeTo: root)
            entity.setOrientation(pose.orientation, relativeTo: root)
            // Read back, so rounding in the transform is not mistaken for a teleport
            body.pose = Pose(of: entity, relativeTo: root)
            if entity.components.has(SDFMotionComponent.self) {
                var linear: [Float] = [0, 0, 0], angular: [Float] = [0, 0, 0]
                sdf_world_body_velocity(world, body.id, &linear, &angular)
                let motion = SDFMotionComponent(linearVelocity: SIMD3(linear[0], linear[1], linear[2]),
                                                angularVelocity: SIMD3(angular[0], angular[1], angular[2]))
                entity.components.set(motion)
                body.motion = motion
            }
            bodies[entity.id] = body
        }
    }

    private func setVelocity(of id: SDFBodyID, _ motion: SDFMotionComponent) {
        let v = motion.linearVelocity, w = motion.angularVelocity
        sdf_world_set_body_velocity(world, id, [v.x, v.y, v.z], [w.x, w.y, w.z])
    }

    /// Velocities that move a pose to another over a time
    private func velocity(from a: Pose, to b: Pose, over time: TimeInterval) -> (SIMD3<Float>, SIMD3<Float>) {
        guard time > 0 else { return (.zero, .zero) }
        var delta = b.orientation * a.orientation.inverse
        if delta.real < 0 { delta = simd_quatf(vector: -delta.vector) }
        let angular = delta.angle > 1e-6 ? delta.axis * (delta.angle / Float(time)) : .zero
        return ((b.position - a.position) / Float(time), angular)
    }

    // MARK: Queries and interaction

    /// Mass of an entity's body, nil when it has none (yet)
    public func mass(of entity: Entity) -> Float? {
        bodies[entity.id].map { sdf_world_body_mass(world, $0.id) }
    }

    /// Applies an impulse at a point (in the root's frame) to a dynamic body
    public func applyImpulse(_ impulse: SIMD3<Float>, at point: SIMD3<Float>, to entity: Entity) {
        guard let body = bodies[entity.id] else { return }
        sdf_world_apply_impulse(world, body.id, [impulse.x, impulse.y, impulse.z], [point.x, point.y, point.z])
    }

    public struct Contact: Sendable {
        public var entityA: Entity.ID
        public var entityB: Entity.ID
        /// In the root's frame
        public var position: SIMD3<Float>
        /// From entity A to entity B
        public var normal: SIMD3<Float>
        /// Gap (positive) or overlap (negative) of the surfaces
        public var separation: Float
        /// Tracked from the previous step by the temporal cache, rather than newly found
        public var isCached: Bool
    }

    /// Active contacts of the last substep
    public func contacts(limit: Int = 4096) -> [Contact] {
        var buffer = [SDFContact](repeating: SDFContact(), count: limit)
        let count = Int(buffer.withUnsafeMutableBufferPointer { sdf_world_contacts(world, $0.baseAddress, Int32(limit)) })
        return buffer.prefix(min(count, limit)).compactMap { c in
            guard let a = entityIDs[c.bodyA], let b = entityIDs[c.bodyB] else { return nil }
            return Contact(entityA: a, entityB: b, position: SIMD3(c.position.0, c.position.1, c.position.2),
                           normal: SIMD3(c.normal.0, c.normal.1, c.normal.2), separation: c.separation, isCached: c.cached)
        }
    }

    /// Picks the closest dynamic body on a ray (in the root's frame) and holds it at the hit point,
    /// which then follows `drag(to:)`. Returns the entity and the hit point.
    public func beginDrag(origin: SIMD3<Float>, direction: SIMD3<Float>) -> (entity: Entity.ID, hit: SIMD3<Float>)? {
        var hit: [Float] = [0, 0, 0]
        let id = sdf_world_begin_drag(world, [origin.x, origin.y, origin.z], [direction.x, direction.y, direction.z], &hit)
        guard let entityID = entityIDs[id] else { return nil }
        return (entityID, SIMD3(hit[0], hit[1], hit[2]))
    }

    public func drag(to target: SIMD3<Float>) {
        sdf_world_drag_to(world, [target.x, target.y, target.z])
    }

    public func endDrag() {
        sdf_world_end_drag(world)
    }
}
