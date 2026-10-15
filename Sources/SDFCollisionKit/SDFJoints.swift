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
        var seen: Set<Int> = []
        let joints = root.components[SDFJointsComponent.self]?.joints ?? []
        for (index, joint) in joints.enumerated() {
            guard let entityA = joint.pin0.entity, let entityB = joint.pin1.entity,
                  let bodyA = bodies[entityA.id]?.id, let bodyB = bodies[entityB.id]?.id
            else { continue }
            seen.insert(index)
            if let state = jointStates[index] {
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
                    jointStates[index]?.joint = joint
                    continue
                }
                sdf_world_remove_joint(world, state.id)
                jointStates[index] = nil
            }
            // The anchor and axis in the root's frame, where the engine works
            guard let anchor = joint.pin0.position(relativeTo: root), let orientation = joint.pin0.orientation(relativeTo: root)
            else { continue }
            var def = sdf_default_revolute_joint()
            def.bodyA = bodyA
            def.bodyB = bodyB
            def.anchor = (anchor.x, anchor.y, anchor.z)
            let axis = orientation.act([1, 0, 0])
            def.axis = (axis.x, axis.y, axis.z)
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
                jointStates[index] = JointState(id: id, bodyA: bodyA, bodyB: bodyB, joint: joint)
            }
        }
        for (index, state) in jointStates where !seen.contains(index) {
            sdf_world_remove_joint(world, state.id)
            jointStates[index] = nil
        }
    }

    /// The angle of a joint of the root's component: the rotation of `pin1`'s entity relative to
    /// `pin0`'s about the axis, in radians, or nil while the joint isn't in the simulation
    public func angle(ofJointAt index: Int) -> Float? {
        guard let state = jointStates[index] else { return nil }
        return sdf_world_joint_angle(world, state.id)
    }
}
