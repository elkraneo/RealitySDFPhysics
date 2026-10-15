import RealityKit
internal import SDFCore
import simd

/// A hinge between two bodies, with one rotational degree of freedom about the x axis of two
/// entity pins, as `PhysicsRevoluteJoint`. The pins' positions coincide and their x axes stay
/// aligned; `angularLimit` bounds the rotation of `pin1`'s entity relative to `pin0`'s, a `motor`
/// drives it and a `spring` pulls it toward a target angle, which RealityKit's joints don't have.
///
/// ```swift
/// let hinge = door.pins.set(named: "hinge", position: [-0.5, 0, 0], orientation: simd_quatf(angle: .pi / 2, axis: [0, 0, 1]))
/// let frame = wall.pins.set(named: "frame", position: [0, 0, 0], orientation: hinge.offsetOrientation)
/// var joint = SDFPhysicsRevoluteJoint(pin0: frame, pin1: hinge, angularLimit: -.pi / 2 ... 0)
/// try joint.addToSimulation()
/// ```
public struct SDFPhysicsRevoluteJoint: Equatable {
    /// Drives the rotation at a speed, with up to a torque
    public struct Motor: Equatable, Sendable {
        /// Radians per second of `pin1`'s entity relative to `pin0`'s, about the axis
        public var targetVelocity: Float
        /// The most torque the motor applies, in the simulation's units
        public var maxTorque: Float

        public init(targetVelocity: Float, maxTorque: Float) {
            self.targetVelocity = targetVelocity
            self.maxTorque = maxTorque
        }
    }

    /// Pulls the rotation toward a target angle, the short way round, as a damped spring: one
    /// that swings at `hertz` a body turning about its center of mass, as Box3D's, and loses its
    /// swing as `dampingRatio` says, 1 coming to rest without overshoot. A body turning about a
    /// pin away from its center swings slower.
    public struct Spring: Equatable, Sendable {
        /// Radians of `pin1`'s entity relative to `pin0`'s, about the axis
        public var targetAngle: Float
        public var hertz: Float
        public var dampingRatio: Float

        public init(targetAngle: Float, hertz: Float, dampingRatio: Float = 1) {
            self.targetAngle = targetAngle
            self.hertz = hertz
            self.dampingRatio = dampingRatio
        }
    }

    public var pin0: GeometricPin
    public var pin1: GeometricPin
    /// The rotation of `pin1`'s entity relative to `pin0`'s, in radians within (-π, π)
    public var angularLimit: ClosedRange<Float>?
    public var motor: Motor?
    public var spring: Spring?
    /// Whether the two bodies collide with each other
    public var checksForInternalCollisions: Bool

    public init(pin0: GeometricPin, pin1: GeometricPin, angularLimit: ClosedRange<Float>? = nil, motor: Motor? = nil,
                spring: Spring? = nil, checksForInternalCollisions: Bool = false) {
        self.pin0 = pin0
        self.pin1 = pin1
        self.angularLimit = angularLimit
        self.motor = motor
        self.spring = spring
        self.checksForInternalCollisions = checksForInternalCollisions
    }

    /// Adds the joint to the nearest simulation above `pin0`'s entity, in its `SDFPhysicsJointsComponent`,
    /// and returns that entity. Throws when a pin has no entity or no simulation holds it.
    @discardableResult
    @MainActor
    public func addToSimulation() throws -> Entity {
        try SDFPhysicsJoint.revolute(self).addToSimulation()
    }
}

/// A slider between two bodies, with one translational degree of freedom along the x axis of two
/// entity pins, as `PhysicsPrismaticJoint`. `pin1`'s position stays on the line through `pin0`'s
/// along its x axis, and the pins' orientations stay as they are when they coincide: the bodies
/// don't turn relative to each other. `linearLimit` bounds the translation of `pin1` from `pin0`
/// along the axis; a `motor` drives it and a `spring` pulls it toward a target, which
/// RealityKit's joints don't have.
///
/// ```swift
/// let rail = base.pins.set(named: "rail", orientation: simd_quatf(angle: .pi / 2, axis: [0, 0, 1]))
/// let slide = block.pins.set(named: "slide", orientation: rail.offsetOrientation)
/// var joint = SDFPhysicsPrismaticJoint(pin0: rail, pin1: slide, linearLimit: -1 ... 1)
/// try joint.addToSimulation()
/// ```
public struct SDFPhysicsPrismaticJoint: Equatable {
    /// Drives the translation at a speed, with up to a force
    public struct Motor: Equatable, Sendable {
        /// Units per second of `pin1` relative to `pin0`, along the axis
        public var targetVelocity: Float
        /// The most force the motor applies, in the simulation's units
        public var maxForce: Float

        public init(targetVelocity: Float, maxForce: Float) {
            self.targetVelocity = targetVelocity
            self.maxForce = maxForce
        }
    }

    /// Pulls the translation toward a target position, as ``SDFPhysicsRevoluteJoint/Spring`` does the angle
    public struct Spring: Equatable, Sendable {
        /// Units of `pin1` from `pin0` along the axis
        public var targetPosition: Float
        public var hertz: Float
        public var dampingRatio: Float

        public init(targetPosition: Float, hertz: Float, dampingRatio: Float = 1) {
            self.targetPosition = targetPosition
            self.hertz = hertz
            self.dampingRatio = dampingRatio
        }
    }

    public var pin0: GeometricPin
    public var pin1: GeometricPin
    /// The translation of `pin1` from `pin0` along the axis, in the simulation's units
    public var linearLimit: ClosedRange<Float>?
    public var motor: Motor?
    public var spring: Spring?
    /// Whether the two bodies collide with each other
    public var checksForInternalCollisions: Bool

    public init(pin0: GeometricPin, pin1: GeometricPin, linearLimit: ClosedRange<Float>? = nil, motor: Motor? = nil,
                spring: Spring? = nil, checksForInternalCollisions: Bool = false) {
        self.pin0 = pin0
        self.pin1 = pin1
        self.linearLimit = linearLimit
        self.motor = motor
        self.spring = spring
        self.checksForInternalCollisions = checksForInternalCollisions
    }

    /// Adds the joint to the nearest simulation above `pin0`'s entity, in its `SDFPhysicsJointsComponent`,
    /// and returns that entity. Throws when a pin has no entity or no simulation holds it.
    @discardableResult
    @MainActor
    public func addToSimulation() throws -> Entity {
        try SDFPhysicsJoint.prismatic(self).addToSimulation()
    }
}

/// A joint of either kind
public enum SDFPhysicsJoint: Equatable {
    case revolute(SDFPhysicsRevoluteJoint)
    case prismatic(SDFPhysicsPrismaticJoint)

    public var pin0: GeometricPin {
        switch self {
        case .revolute(let joint): joint.pin0
        case .prismatic(let joint): joint.pin0
        }
    }

    public var pin1: GeometricPin {
        switch self {
        case .revolute(let joint): joint.pin1
        case .prismatic(let joint): joint.pin1
        }
    }

    public var checksForInternalCollisions: Bool {
        switch self {
        case .revolute(let joint): joint.checksForInternalCollisions
        case .prismatic(let joint): joint.checksForInternalCollisions
        }
    }

    /// Adds the joint to the nearest simulation above `pin0`'s entity, in its `SDFPhysicsJointsComponent`,
    /// and returns that entity. Throws when a pin has no entity or no simulation holds it.
    @discardableResult
    @MainActor
    public func addToSimulation() throws -> Entity {
        guard let entity = pin0.entity, pin1.entity != nil else { throw SDFJointError.pinWithoutEntity }
        var ancestor: Entity? = entity
        while let current = ancestor {
            if current.components.has(SDFPhysicsSimulationComponent.self) {
                var joints = current.components[SDFPhysicsJointsComponent.self] ?? SDFPhysicsJointsComponent()
                joints.joints.append(self)
                current.components.set(joints)
                return current
            }
            ancestor = current.parent
        }
        throw SDFJointError.noSimulation
    }

    // What the engine takes of the limit, motor and spring, the same for both kinds

    var limit: ClosedRange<Float>? {
        switch self {
        case .revolute(let joint): joint.angularLimit
        case .prismatic(let joint): joint.linearLimit
        }
    }

    /// The speed and the most torque or force
    var motor: SIMD2<Float>? {
        switch self {
        case .revolute(let joint): joint.motor.map { [$0.targetVelocity, $0.maxTorque] }
        case .prismatic(let joint): joint.motor.map { [$0.targetVelocity, $0.maxForce] }
        }
    }

    /// The target, the frequency and the damping ratio
    var spring: SIMD3<Float>? {
        switch self {
        case .revolute(let joint): joint.spring.map { [$0.targetAngle, $0.hertz, $0.dampingRatio] }
        case .prismatic(let joint): joint.spring.map { [$0.targetPosition, $0.hertz, $0.dampingRatio] }
        }
    }

    var isRevolute: Bool {
        if case .revolute = self { return true }
        return false
    }
}

public enum SDFJointError: Error {
    case pinWithoutEntity
    case noSimulation
}

/// The joints of a simulation, on the entity that has its ``SDFPhysicsSimulationComponent``
public struct SDFPhysicsJointsComponent: Component, Equatable {
    public var joints: [SDFPhysicsJoint]

    public init(joints: [SDFPhysicsJoint] = []) {
        _ = registration
        self.joints = joints
    }

    /// Hinges only
    public init<Hinges: Sequence>(joints: Hinges) where Hinges.Element == SDFPhysicsRevoluteJoint {
        self.init(joints: joints.map(SDFPhysicsJoint.revolute))
    }
}

// MARK: Synchronization

extension SDFSimulation {
    /// A joint's identity: its pins, by entity and name
    struct JointKey: Hashable {
        var entityA: Entity.ID, pinA: String
        var entityB: Entity.ID, pinB: String
    }

    /// A joint in the engine, with what it was made from, to tell a change from a repeat
    struct JointState {
        var id: SDFJointID
        var bodyA: SDFBodyID, bodyB: SDFBodyID
        var joint: SDFPhysicsJoint
    }

    /// Creates, updates and removes the engine's joints from the root's ``SDFPhysicsJointsComponent``.
    /// Runs after the bodies, which the pins' entities must have. A joint whose pins' entities
    /// have no body yet waits for them.
    func synchronizeJoints(root: Entity) {
        var seen: Set<JointKey> = []
        let joints = root.components[SDFPhysicsJointsComponent.self]?.joints ?? []
        for joint in joints {
            guard let entityA = joint.pin0.entity, let entityB = joint.pin1.entity,
                  let bodyA = bodies[entityA.id]?.id, let bodyB = bodies[entityB.id]?.id
            else { continue }
            let key = JointKey(entityA: entityA.id, pinA: joint.pin0.name, entityB: entityB.id, pinB: joint.pin1.name)
            guard !seen.contains(key) else { continue }
            seen.insert(key)
            if let state = jointStates[key] {
                if state.bodyA == bodyA, state.bodyB == bodyB, state.joint.isRevolute == joint.isRevolute,
                   state.joint.pin0 == joint.pin0, state.joint.pin1 == joint.pin1,
                   state.joint.checksForInternalCollisions == joint.checksForInternalCollisions {
                    // The same joint; its limit, motor and spring can change in place
                    if state.joint.limit != joint.limit {
                        let limit = joint.limit ?? 0...0
                        sdf_world_set_joint_limit(world, state.id, joint.limit != nil, limit.lowerBound, limit.upperBound)
                    }
                    if state.joint.motor != joint.motor {
                        let motor = joint.motor ?? .zero
                        sdf_world_set_joint_motor(world, state.id, joint.motor != nil, motor.x, motor.y)
                    }
                    if state.joint.spring != joint.spring {
                        let spring = joint.spring ?? [0, 0, 1]
                        sdf_world_set_joint_spring(world, state.id, joint.spring != nil, spring.x, spring.y, spring.z)
                    }
                    jointStates[key]?.joint = joint
                    continue
                }
                sdf_world_remove_joint(world, state.id)
                jointStates[key] = nil
            }
            // The pins' frames in the root's frame, where the engine works
            guard let positionA = joint.pin0.position(relativeTo: root), let orientationA = joint.pin0.orientation(relativeTo: root),
                  let positionB = joint.pin1.position(relativeTo: root), let orientationB = joint.pin1.orientation(relativeTo: root)
            else { continue }
            let frameA = SDFPose(position: (positionA.x, positionA.y, positionA.z),
                                 orientation: (orientationA.vector.x, orientationA.vector.y, orientationA.vector.z, orientationA.vector.w), scale: 1)
            let frameB = SDFPose(position: (positionB.x, positionB.y, positionB.z),
                                 orientation: (orientationB.vector.x, orientationB.vector.y, orientationB.vector.z, orientationB.vector.w), scale: 1)
            let limit = joint.limit ?? 0...0, motor = joint.motor ?? .zero, spring = joint.spring ?? [0, 0, 1]
            let id: SDFJointID
            if joint.isRevolute {
                var def = sdf_default_revolute_joint()
                def.bodyA = bodyA
                def.bodyB = bodyB
                def.frameA = frameA
                def.frameB = frameB
                def.hasLimit = joint.limit != nil
                def.lowerAngle = limit.lowerBound
                def.upperAngle = limit.upperBound
                def.hasMotor = joint.motor != nil
                def.motorSpeed = motor.x
                def.maxMotorTorque = motor.y
                def.hasSpring = joint.spring != nil
                def.targetAngle = spring.x
                def.springHertz = spring.y
                def.springDampingRatio = spring.z
                def.collideConnected = joint.checksForInternalCollisions
                id = sdf_world_add_revolute_joint(world, def)
            } else {
                var def = sdf_default_prismatic_joint()
                def.bodyA = bodyA
                def.bodyB = bodyB
                def.frameA = frameA
                def.frameB = frameB
                def.hasLimit = joint.limit != nil
                def.lowerTranslation = limit.lowerBound
                def.upperTranslation = limit.upperBound
                def.hasMotor = joint.motor != nil
                def.motorSpeed = motor.x
                def.maxMotorForce = motor.y
                def.hasSpring = joint.spring != nil
                def.targetTranslation = spring.x
                def.springHertz = spring.y
                def.springDampingRatio = spring.z
                def.collideConnected = joint.checksForInternalCollisions
                id = sdf_world_add_prismatic_joint(world, def)
            }
            if id != 0 {
                jointStates[key] = JointState(id: id, bodyA: bodyA, bodyB: bodyB, joint: joint)
            }
        }
        for (key, state) in jointStates where !seen.contains(key) {
            sdf_world_remove_joint(world, state.id)
            jointStates[key] = nil
        }
    }

    /// The engine's joint of a pair of pins
    /// The engine's joint on these pins, if it is of the kind asked for: a slider that replaced a
    /// hinge on the same pins is not that hinge
    private func jointID(_ pin0: GeometricPin, _ pin1: GeometricPin, revolute: Bool) -> SDFJointID? {
        guard let entityA = pin0.entity, let entityB = pin1.entity,
              let state = jointStates[JointKey(entityA: entityA.id, pinA: pin0.name, entityB: entityB.id, pinB: pin1.name)],
              state.joint.isRevolute == revolute
        else { return nil }
        return state.id
    }

    /// The angle of a joint: the rotation of `pin1`'s entity relative to `pin0`'s about the axis,
    /// in radians, or nil while the joint isn't in the simulation
    public func angle(of joint: SDFPhysicsRevoluteJoint) -> Float? {
        jointID(joint.pin0, joint.pin1, revolute: true).map { sdf_world_joint_angle(world, $0) }
    }

    /// The translation of a joint: how far `pin1` is from `pin0` along the axis, or nil while the
    /// joint isn't in the simulation
    public func translation(of joint: SDFPhysicsPrismaticJoint) -> Float? {
        jointID(joint.pin0, joint.pin1, revolute: false).map { sdf_world_joint_translation(world, $0) }
    }
}
