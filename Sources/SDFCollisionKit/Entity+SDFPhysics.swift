import RealityKit

extension Entity {
    /// Forces and impulses for the entity's SDF body, with the same methods as RealityKit's
    /// `HasPhysicsBody`:
    ///
    /// ```swift
    /// gear.sdfPhysics.addTorque([0, 5, 0], relativeTo: nil)
    /// ball.sdfPhysics.applyLinearImpulse([0, 2, 0], relativeTo: nil)
    /// ```
    public var sdfPhysics: SDFPhysics { SDFPhysics(entity: self) }
}

/// The forces and impulses of an entity's SDF body. Vectors and points are in the space of
/// `referenceEntity`, or world space when it is nil, as in RealityKit.
///
/// Forces and torques act during the next simulation update and are then cleared. Impulses change
/// the velocities immediately. Both apply to dynamic bodies only, and are kept for a body that
/// is created on the next update.
@MainActor
public struct SDFPhysics {
    public let entity: Entity

    /// The simulation of the nearest ancestor with an `SDFSimulationComponent`, and that ancestor
    private var simulation: (SDFSimulation, root: Entity)? {
        guard let parent = entity.parent, let root = SDFSimulationComponent.nearestSimulationEntity(for: parent),
              let component = root.components[SDFSimulationComponent.self] else { return nil }
        return (component.simulation, root)
    }

    public func addForce(_ force: SIMD3<Float>, relativeTo referenceEntity: Entity?) {
        guard let (simulation, root) = simulation else { return }
        simulation.addForce(root.convert(direction: force, from: referenceEntity), to: entity)
    }

    public func addForce(_ force: SIMD3<Float>, at position: SIMD3<Float>, relativeTo referenceEntity: Entity?) {
        guard let (simulation, root) = simulation else { return }
        simulation.addForce(root.convert(direction: force, from: referenceEntity),
                            at: root.convert(position: position, from: referenceEntity), to: entity)
    }

    public func addTorque(_ torque: SIMD3<Float>, relativeTo referenceEntity: Entity?) {
        guard let (simulation, root) = simulation else { return }
        simulation.addTorque(root.convert(direction: torque, from: referenceEntity), to: entity)
    }

    public func clearForcesAndTorques() {
        simulation?.0.clearForcesAndTorques(of: entity)
    }

    public func applyLinearImpulse(_ impulse: SIMD3<Float>, relativeTo referenceEntity: Entity?) {
        guard let (simulation, root) = simulation else { return }
        simulation.applyImpulse(root.convert(direction: impulse, from: referenceEntity), to: entity)
    }

    public func applyAngularImpulse(_ impulse: SIMD3<Float>, relativeTo referenceEntity: Entity?) {
        guard let (simulation, root) = simulation else { return }
        simulation.applyAngularImpulse(root.convert(direction: impulse, from: referenceEntity), to: entity)
    }

    public func applyImpulse(_ impulse: SIMD3<Float>, at position: SIMD3<Float>, relativeTo referenceEntity: Entity?) {
        guard let (simulation, root) = simulation else { return }
        simulation.applyImpulse(root.convert(direction: impulse, from: referenceEntity),
                                at: root.convert(position: position, from: referenceEntity), to: entity)
    }
}
