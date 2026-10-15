import RealityKit
internal import SDFCore
import simd

/// A hinge between two bodies, with one rotational degree of freedom about the x axis of two
/// entity pins, as `PhysicsRevoluteJoint`. The pins' positions coincide and their x axes stay
/// aligned; `angularLimit` bounds the rotation of `pin1`'s entity relative to `pin0`'s, and a
/// `motor` drives it, which RealityKit's joints don't have.
///
/// ```swift
/// let hinge = door.pins.set(named: "hinge", position: [-0.5, 0, 0], orientation: simd_quatf(angle: .pi / 2, axis: [0, 0, 1]))
/// let frame = wall.pins.set(named: "frame", position: [0, 0, 0], orientation: hinge.offsetOrientation)
/// var joint = SDFRevoluteJoint(pin0: frame, pin1: hinge, angularLimit: -.pi / 2 ... 0)
/// try joint.addToSimulation()
/// ```
public struct SDFRevoluteJoint: Equatable {
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

    public var pin0: GeometricPin
    public var pin1: GeometricPin
    /// The rotation of `pin1`'s entity relative to `pin0`'s, in radians within (-π, π)
    public var angularLimit: ClosedRange<Float>?
    public var motor: Motor?
    /// Whether the two bodies collide with each other
    public var checksForInternalCollisions: Bool

    public init(pin0: GeometricPin, pin1: GeometricPin, angularLimit: ClosedRange<Float>? = nil, motor: Motor? = nil,
                checksForInternalCollisions: Bool = false) {
        self.pin0 = pin0
        self.pin1 = pin1
        self.angularLimit = angularLimit
        self.motor = motor
        self.checksForInternalCollisions = checksForInternalCollisions
    }

    /// Adds the joint to the nearest simulation above `pin0`'s entity, in its `SDFJointsComponent`,
    /// and returns that entity. Throws when a pin has no entity or no simulation holds it.
    @discardableResult
    @MainActor
    public func addToSimulation() throws -> Entity {
        guard let entity = pin0.entity, pin1.entity != nil else { throw SDFJointError.pinWithoutEntity }
        var ancestor: Entity? = entity
        while let current = ancestor {
            if current.components.has(SDFSimulationComponent.self) {
                var joints = current.components[SDFJointsComponent.self] ?? SDFJointsComponent()
                joints.joints.append(self)
                current.components.set(joints)
                return current
            }
            ancestor = current.parent
        }
        throw SDFJointError.noSimulation
    }
}

public enum SDFJointError: Error {
    case pinWithoutEntity
    case noSimulation
}

/// The joints of a simulation, on the entity that has its ``SDFSimulationComponent``
public struct SDFJointsComponent: Component, Equatable {
    public var joints: [SDFRevoluteJoint]

    public init(joints: [SDFRevoluteJoint] = []) {
        _ = registration
        self.joints = joints
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
        var joint: SDFRevoluteJoint
    }

    /// Creates, updates and removes the engine's joints from the root's ``SDFJointsComponent``.
    /// Runs after the bodies, which the pins' entities must have. A joint whose pins' entities
    /// have no body yet waits for them.
    func synchronizeJoints(root: Entity) {
        var seen: Set<JointKey> = []
        let joints = root.components[SDFJointsComponent.self]?.joints ?? []
        for joint in joints {
            guard let entityA = joint.pin0.entity, let entityB = joint.pin1.entity,
                  let bodyA = bodies[entityA.id]?.id, let bodyB = bodies[entityB.id]?.id
            else { continue }
            let key = JointKey(entityA: entityA.id, pinA: joint.pin0.name, entityB: entityB.id, pinB: joint.pin1.name)
            guard !seen.contains(key) else { continue }
            seen.insert(key)
            if let state = jointStates[key] {
                if state.bodyA == bodyA, state.bodyB == bodyB, state.joint.pin0 == joint.pin0, state.joint.pin1 == joint.pin1,
                   state.joint.checksForInternalCollisions == joint.checksForInternalCollisions {
                    // The same joint; its limit and motor can change in place
                    if state.joint.angularLimit != joint.angularLimit {
                        let limit = joint.angularLimit ?? 0...0
                        sdf_world_set_joint_limit(world, state.id, joint.angularLimit != nil, limit.lowerBound, limit.upperBound)
                    }
                    if state.joint.motor != joint.motor {
                        let motor = joint.motor ?? .init(targetVelocity: 0, maxTorque: 0)
                        sdf_world_set_joint_motor(world, state.id, joint.motor != nil, motor.targetVelocity, motor.maxTorque)
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
            var def = sdf_default_revolute_joint()
            def.bodyA = bodyA
            def.bodyB = bodyB
            def.frameA = SDFPose(position: (positionA.x, positionA.y, positionA.z),
                                 orientation: (orientationA.vector.x, orientationA.vector.y, orientationA.vector.z, orientationA.vector.w), scale: 1)
            def.frameB = SDFPose(position: (positionB.x, positionB.y, positionB.z),
                                 orientation: (orientationB.vector.x, orientationB.vector.y, orientationB.vector.z, orientationB.vector.w), scale: 1)
            if let limit = joint.angularLimit {
                def.hasLimit = true
                def.lowerAngle = limit.lowerBound
                def.upperAngle = limit.upperBound
            }
            if let motor = joint.motor {
                def.hasMotor = true
                def.motorSpeed = motor.targetVelocity
                def.maxMotorTorque = motor.maxTorque
            }
            def.collideConnected = joint.checksForInternalCollisions
            let id = sdf_world_add_revolute_joint(world, def)
            if id != 0 {
                jointStates[key] = JointState(id: id, bodyA: bodyA, bodyB: bodyB, joint: joint)
            }
        }
        for (key, state) in jointStates where !seen.contains(key) {
            sdf_world_remove_joint(world, state.id)
            jointStates[key] = nil
        }
    }

    /// The angle of a joint: the rotation of `pin1`'s entity relative to `pin0`'s about the axis,
    /// in radians, or nil while the joint isn't in the simulation
    public func angle(of joint: SDFRevoluteJoint) -> Float? {
        guard let entityA = joint.pin0.entity, let entityB = joint.pin1.entity,
              let state = jointStates[JointKey(entityA: entityA.id, pinA: joint.pin0.name, entityB: entityB.id, pinB: joint.pin1.name)]
        else { return nil }
        return sdf_world_joint_angle(world, state.id)
    }
}
