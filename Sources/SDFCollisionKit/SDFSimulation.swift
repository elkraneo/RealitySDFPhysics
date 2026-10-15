import Foundation
import RealityKit
internal import SDFCore
import simd

/// A rigid body world whose collision detection is SDF-SDF optimization with temporal caching and
/// importance sampling (Giles and Andrews, Pacific Graphics 2026).
///
/// Set it on a root entity with `SDFSimulationComponent`, which also holds its settings. Every frame
/// it adds, updates and removes bodies for the root's descendants with an `SDFBodyComponent`,
/// advances in fixed steps and writes the poses of dynamic bodies back to their entities. Positions,
/// directions and distances of the simulation are in the root's local frame and units.
///
/// RealityKit's own physics is not involved.
@MainActor
public final class SDFSimulation {
    nonisolated(unsafe) let world: OpaquePointer

    /// Steps taken so far
    public private(set) var stepCount = 0

    var bodies: [Entity.ID: Body] = [:]
    private var entities: [SDFBodyID: Entity] = [:]
    private var accumulator: TimeInterval = 0
    /// Impulses for bodies that are created on the next update
    private var pending: [Entity.ID: [(SDFBodyID) -> Void]] = [:]
    /// Forces added outside the steps since the last update
    private var updatePushes: [Entity.ID: [Push]] = [:]
    /// The forces that act during every step of an update: per entity, those of the latest update
    /// they were added before. Kept through updates that take no step, without adding up.
    private var activePushes: [Entity.ID: [Push]] = [:]
    /// Forces added by event handlers during a step, which act during the next step
    private var stepPushes: [Entity.ID: [Push]] = [:]
    /// Bodies whose forces are set in the engine
    private var forced: Set<SDFBodyID> = []
    private var isStepping = false
    /// The length of a unit of the root being simulated, from its component
    private var metersPerUnit: Float = 1
    var events = EventRouter()
    /// Pairs touching after the last step
    var touching: [PairKey: (entityA: Entity, entityB: Entity)] = [:]
    /// The engine's joints, by their pins
    var jointStates: [JointKey: JointState] = [:]

    public init() {
        _ = registration
        _ = Self.systemRegistration
        world = sdf_world_create()
    }

    private static let systemRegistration: Void = SDFSimulationSystem.registerSystem()

    deinit {
        sdf_world_destroy(world)
    }

    // MARK: Stepping

    /// Synchronizes the bodies with the root's descendants and advances by real time in fixed
    /// steps. `SDFSimulationSystem` calls this every frame. Does nothing when called from an event
    /// handler of this simulation.
    public func update(root: Entity, deltaTime: TimeInterval) {
        guard !isStepping, let settings = root.components[SDFSimulationComponent.self] else { return }
        // Event handlers, including those of contacts that end when bodies are removed, can't step
        isStepping = true
        defer { isStepping = false }
        let dynamic = prepare(root: root, settings: settings, deltaTime: deltaTime)
        let timeStep = TimeInterval(settings.timeStep)
        guard !settings.isPaused, timeStep > 0, timeStep.isFinite else { return }
        accumulator += min(max(deltaTime, 0), 0.25)
        var steps = 0
        while accumulator >= timeStep && steps < max(settings.maximumStepsPerFrame, 1) {
            step(root: root, dynamic: dynamic, timeStep: timeStep)
            accumulator -= timeStep
            steps += 1
        }
        // Drop the whole steps a slow frame could not catch up on, keep the fraction
        accumulator = accumulator.truncatingRemainder(dividingBy: timeStep)
        if steps > 0 { activePushes = [:] }
    }

    /// Synchronizes and takes exactly one step, also while paused. Does nothing when called from an
    /// event handler of this simulation, or when the time step isn't positive.
    public func step(root: Entity) {
        guard !isStepping, let settings = root.components[SDFSimulationComponent.self] else { return }
        isStepping = true
        defer { isStepping = false }
        let timeStep = TimeInterval(settings.timeStep)
        let dynamic = prepare(root: root, settings: settings, deltaTime: timeStep)
        guard timeStep > 0, timeStep.isFinite else { return }
        step(root: root, dynamic: dynamic, timeStep: timeStep)
        activePushes = [:]
    }

    /// Applies the settings, synchronizes the bodies, and takes the forces added since the last
    /// update. Returns the entities of dynamic bodies.
    private func prepare(root: Entity, settings: SDFSimulationComponent, deltaTime: TimeInterval) -> [Entity] {
        sdf_world_set_params(world, settings.engine)
        metersPerUnit = settings.metersPerUnit
        let dynamic = synchronize(root: root, deltaTime: deltaTime)
        synchronizeJoints(root: root)
        // A force added every frame stays the same at any frame rate, or while paused
        activePushes.merge(updatePushes) { _, new in new }
        updatePushes = [:]
        return dynamic
    }

    /// Takes one step; `isStepping` is set
    private func step(root: Entity, dynamic: [Entity], timeStep: TimeInterval) {
        send(SDFSimulationEvents.WillSimulate(deltaTime: timeStep, simulationEntity: root))
        let pushes = stepPushes
        stepPushes = [:]
        applyForces(activePushes, pushes)
        sdf_world_step(world)
        stepCount += 1
        writeBack(dynamic, root: root)
        sendCollisionEvents()
        send(SDFSimulationEvents.DidSimulate(deltaTime: timeStep, simulationEntity: root))
    }

    // MARK: Bodies

    struct Pose {
        var position: SIMD3<Float>
        var orientation: simd_quatf

        @MainActor init(of entity: Entity, relativeTo root: Entity) {
            // A direct child's local transform is its pose in the root; the general query
            // composes and decomposes matrices up the chain, several times the cost. The rotation
            // is documented as a unit quaternion but stored as given: one off unit is normalized,
            // as the query's decomposition does, and the zero quaternion, which simd_quatf() is,
            // goes to the query, which returns identity for it (observed), as RealityKit renders it.
            if entity.parent === root {
                let transform = entity.transform
                let length = simd_length_squared(transform.rotation.vector)
                if length > 1e-6, length.isFinite {
                    position = transform.translation
                    orientation = abs(length - 1) < 1e-6 ? transform.rotation : simd_normalize(transform.rotation)
                    return
                }
            }
            position = entity.position(relativeTo: root)
            orientation = entity.orientation(relativeTo: root)
        }

        /// Writes the pose to the entity, keeping its scale
        @MainActor func write(to entity: Entity, relativeTo root: Entity) {
            if entity.parent === root {
                var transform = entity.transform
                transform.translation = position
                transform.rotation = orientation
                entity.transform = transform
            } else {
                entity.setPosition(position, relativeTo: root)
                entity.setOrientation(orientation, relativeTo: root)
            }
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

    struct Body {
        var id: SDFBodyID
        var entity: Entity
        /// Kept alive, so that a new shape at the same address is not mistaken for it
        var shape: SDFShape
        var mode: PhysicsBodyMode
        var scale: Float
        var massProperties: SDFMassProperties
        var options: SDFBodyComponent.Options
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
        // Work for bodies that were to be created now; that of entities without a body is dropped
        for (entityID, work) in pending {
            if let id = bodies[entityID]?.id { work.forEach { $0(id) } }
        }
        pending.removeAll()
        return dynamic
    }

    private func attach(_ entity: Entity, _ component: SDFBodyComponent, root: Entity, deltaTime: TimeInterval) {
        let pose = Pose(of: entity, relativeTo: root)
        let scale = entity.parent === root ? entity.scale.x : entity.scale(relativeTo: root).x
        let motion = entity.components[SDFMotionComponent.self]
        guard var body = bodies[entity.id], body.shape === component.shape, body.mode == component.mode,
              abs(body.scale - scale) <= 1e-5 * scale
        else {
            removeBody(of: entity.id)
            addBody(of: entity, component, pose: pose, scale: scale, motion: motion)
            return
        }

        let options = component.options
        if body.options != options {
            sdf_world_set_body_options(world, body.id, options.engine)
            body.options = options
        }
        if body.massProperties != component.massProperties {
            sdf_world_set_body_mass(world, body.id, component.massProperties.mass(of: component.shape, scale: scale, metersPerUnit: metersPerUnit))
            body.massProperties = component.massProperties
        }
        switch component.mode {
        case .kinematic:
            let (linear, angular) = velocity(from: body.pose, to: pose, over: deltaTime)
            sdf_world_set_body_pose(world, body.id, pose.engine(scale: scale))
            sdf_world_set_body_velocity(world, body.id, [linear.x, linear.y, linear.z], [angular.x, angular.y, angular.z])
        case .dynamic:
            // The entity moved since the last write: teleport the body
            if !pose.isClose(to: body.pose) { sdf_world_set_body_pose(world, body.id, pose.engine(scale: scale)) }
            if let motion, motion != body.motion { setVelocity(of: body.id, motion) }
        default:
            if !pose.isClose(to: body.pose) { sdf_world_set_body_pose(world, body.id, pose.engine(scale: scale)) }
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
        default: SDFBodyStatic
        }
        let id = sdf_world_add_body(world, component.shape.handle, pose.engine(scale: scale), mode)
        guard id != 0 else { return }
        let options = component.options
        sdf_world_set_body_options(world, id, options.engine)
        // The engine computes the default mass itself
        if component.massProperties != .default {
            sdf_world_set_body_mass(world, id, component.massProperties.mass(of: component.shape, scale: scale, metersPerUnit: metersPerUnit))
        }
        if let motion { setVelocity(of: id, motion) }
        bodies[entity.id] = Body(id: id, entity: entity, shape: component.shape, mode: component.mode, scale: scale,
                                 massProperties: component.massProperties, options: options, pose: pose, motion: motion)
        entities[id] = entity
    }

    private func removeBody(of entityID: Entity.ID) {
        guard let body = bodies.removeValue(forKey: entityID) else { return }
        endContacts(of: body.id)
        entities[body.id] = nil
        forced.remove(body.id)
        sdf_world_remove_body(world, body.id)
    }

    private func writeBack(_ dynamic: [Entity], root: Entity) {
        for entity in dynamic {
            guard var body = bodies[entity.id], !sdf_world_body_rested_through_step(world, body.id) else { continue }
            let pose = Pose(sdf_world_body_pose(world, body.id))
            pose.write(to: entity, relativeTo: root)
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

    func entity(of id: SDFBodyID) -> Entity? {
        entities[id]
    }

    /// The body IDs of the entities with bodies
    var bodyIDs: [Entity.ID: SDFBodyID] {
        bodies.mapValues(\.id)
    }

    /// Whether an entity's dynamic body is asleep (see ``SDFSimulationComponent/isSleepingEnabled``)
    public func isSleeping(_ entity: Entity) -> Bool {
        guard let id = bodies[entity.id]?.id else { return false }
        return sdf_world_body_is_asleep(world, id)
    }

    /// Sets a dynamic body's velocities now, rather than on the next update as setting its
    /// `SDFMotionComponent` does, and sets that component too. For example before a forecast.
    public func setMotion(_ motion: SDFMotionComponent, of entity: Entity) {
        entity.components.set(motion)
        guard var body = bodies[entity.id] else { return }
        setVelocity(of: body.id, motion)
        body.motion = motion
        bodies[entity.id] = body
    }

    // MARK: Forces and impulses

    /// Runs work on an entity's body now, or once the body is created on the next update
    private func withBody(of entity: Entity, _ work: @escaping (SDFBodyID) -> Void) {
        if let id = bodies[entity.id]?.id {
            work(id)
        } else {
            pending[entity.id, default: []].append(work)
        }
    }

    /// A force at the center of mass or at a point, or a torque, in the root's frame
    private struct Push {
        var force: SIMD3<Float> = .zero
        var point: SIMD3<Float>?
        var torque: SIMD3<Float> = .zero
    }

    private func add(_ push: Push, to entity: Entity) {
        if isStepping {
            stepPushes[entity.id, default: []].append(push)
        } else {
            updatePushes[entity.id, default: []].append(push)
        }
    }

    /// Sets the engine's forces for the next step
    private func applyForces(_ pushes: [Entity.ID: [Push]]...) {
        for id in forced { sdf_world_clear_body_forces(world, id) }
        forced.removeAll()
        for (entityID, list) in pushes.joined() {
            guard let id = bodies[entityID]?.id else { continue }
            for push in list {
                let f = push.force, t = push.torque
                if let p = push.point {
                    sdf_world_add_body_force(world, id, [f.x, f.y, f.z], [p.x, p.y, p.z])
                } else {
                    sdf_world_add_body_force(world, id, [f.x, f.y, f.z], nil)
                }
                sdf_world_add_body_torque(world, id, [t.x, t.y, t.z])
            }
            forced.insert(id)
        }
    }

    /// Adds a force (in the root's frame) at the center of mass, or at a point. A force added from
    /// an event handler of this simulation acts during the next step; otherwise during every step
    /// of the next update. Forces add up.
    public func addForce(_ force: SIMD3<Float>, at point: SIMD3<Float>? = nil, to entity: Entity) {
        add(Push(force: force, point: point), to: entity)
    }

    /// Adds a torque (in the root's frame), which acts like a force added at the same time
    public func addTorque(_ torque: SIMD3<Float>, to entity: Entity) {
        add(Push(torque: torque), to: entity)
    }

    /// Removes the forces and torques that have not acted yet
    public func clearForcesAndTorques(of entity: Entity) {
        updatePushes[entity.id] = nil
        activePushes[entity.id] = nil
        stepPushes[entity.id] = nil
    }

    /// Changes a dynamic body's velocity by an impulse (in the root's frame) at its center of mass,
    /// or at a point
    public func applyImpulse(_ impulse: SIMD3<Float>, at point: SIMD3<Float>? = nil, to entity: Entity) {
        withBody(of: entity) { [unowned self] id in
            if let point {
                sdf_world_apply_impulse(world, id, [impulse.x, impulse.y, impulse.z], [point.x, point.y, point.z])
            } else {
                sdf_world_apply_impulse(world, id, [impulse.x, impulse.y, impulse.z], nil)
            }
        }
    }

    /// Changes a dynamic body's angular velocity by an angular impulse (in the root's frame)
    public func applyAngularImpulse(_ impulse: SIMD3<Float>, to entity: Entity) {
        withBody(of: entity) { [unowned self] id in
            sdf_world_apply_angular_impulse(world, id, [impulse.x, impulse.y, impulse.z])
        }
    }

    /// Mass of an entity's body, nil when it has none (yet)
    public func mass(of entity: Entity) -> Float? {
        bodies[entity.id].map { sdf_world_body_mass(world, $0.id) }
    }

    // MARK: Statistics and contacts

    /// Statistics of the last step
    public struct Statistics: Equatable, Sendable {
        public var bodies: Int
        /// Body pairs with overlapping bounds
        public var pairs: Int
        public var contacts: Int
        public var optimizerIterations: Int
        /// Evaluations of the collision objective, each querying both shapes
        public var queries: Int
        /// Narrow phase time, summed over the substeps
        public var collisionMilliseconds: Double
        /// Contact solver time, summed over the substeps
        public var solverMilliseconds: Double
        public var stepMilliseconds: Double
        /// Dynamic bodies asleep after the step
        public var sleepingBodies: Int
    }

    public var statistics: Statistics {
        let s = sdf_world_stats(world)
        return Statistics(bodies: Int(s.bodies), pairs: Int(s.pairs), contacts: Int(s.contacts),
                          optimizerIterations: Int(s.iterations), queries: Int(s.queries),
                          collisionMilliseconds: s.collisionMilliseconds, solverMilliseconds: s.solverMilliseconds,
                          stepMilliseconds: s.stepMilliseconds,
                          sleepingBodies: Int(s.sleepingBodies))
    }

    /// A contact point between two bodies
    public struct Contact: Sendable {
        public var entityA: Entity.ID
        public var entityB: Entity.ID
        /// In the root's frame
        public var position: SIMD3<Float>
        /// From entity A to entity B
        public var normal: SIMD3<Float>
        /// Gap (positive) or overlap (negative) of the surfaces
        public var separation: Float
        /// Normal impulse of the last substep
        public var impulse: Float
        /// Tracked from the previous step by the temporal cache, rather than newly found
        public var isCached: Bool
    }

    /// Active contacts of the last substep
    public func contacts() -> [Contact] {
        let count = Int(sdf_world_contacts(world, nil, 0))
        var buffer = [SDFContact](repeating: SDFContact(), count: count)
        _ = buffer.withUnsafeMutableBufferPointer { sdf_world_contacts(world, $0.baseAddress, Int32(count)) }
        return buffer.compactMap { c in
            guard let a = entities[c.bodyA], let b = entities[c.bodyB] else { return nil }
            return Contact(entityA: a.id, entityB: b.id, position: SIMD3(c.position.0, c.position.1, c.position.2),
                           normal: SIMD3(c.normal.0, c.normal.1, c.normal.2), separation: c.separation,
                           impulse: c.impulse, isCached: c.cached)
        }
    }

    // MARK: Dragging

    /// Picks the closest dynamic body on a ray (in the root's frame) and holds it at the hit point,
    /// which then follows `drag(to:)`. Returns the entity and the hit point.
    public func beginDrag(origin: SIMD3<Float>, direction: SIMD3<Float>) -> (entity: Entity, hit: SIMD3<Float>)? {
        var hit: [Float] = [0, 0, 0]
        let id = sdf_world_begin_drag(world, [origin.x, origin.y, origin.z], [direction.x, direction.y, direction.z], &hit)
        guard let entity = entities[id] else { return nil }
        return (entity, SIMD3(hit[0], hit[1], hit[2]))
    }

    public func drag(to target: SIMD3<Float>) {
        sdf_world_drag_to(world, [target.x, target.y, target.z])
    }

    public func endDrag() {
        sdf_world_end_drag(world)
    }
}
