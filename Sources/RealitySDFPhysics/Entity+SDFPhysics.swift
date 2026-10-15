import RealityKit

extension Entity {
    /// Forces and impulses for the entity's SDF body, with the same methods as RealityKit's
    /// `HasPhysicsBody`:
    ///
    /// ```swift
    /// gear.sdf.addTorque([0, 5, 0], relativeTo: nil)
    /// ball.sdf.applyLinearImpulse([0, 2, 0], relativeTo: nil)
    /// ```
    public var sdf: SDFPhysics { SDFPhysics(entity: self) }
}

/// The forces and impulses of an entity's SDF body. Vectors and points are in the space of
/// `referenceEntity`, or world space when it is nil, as in RealityKit.
///
/// A force or torque added from your own code acts during every step of the next simulation
/// update; one added from a handler of the simulation's events acts during the next step. Forces
/// are then cleared. Impulses change the velocities immediately, or on the next update for a body
/// that is created then. Both apply to dynamic bodies only.
@MainActor
public struct SDFPhysics {
    public let entity: Entity

    /// The simulation of the nearest ancestor with an `SDFPhysicsSimulationComponent`, and that ancestor
    private var simulation: (SDFSimulation, root: Entity)? {
        guard let parent = entity.parent, let root = SDFPhysicsSimulationComponent.nearestSimulationEntity(for: parent),
              let component = root.components[SDFPhysicsSimulationComponent.self] else { return nil }
        return (component.simulation, root)
    }

    /// Converts a torque or angular impulse, a force times a length, which scales twice: once as a
    /// direction and once more for the length
    private func convert(moment: SIMD3<Float>, from referenceEntity: Entity?, to root: Entity) -> SIMD3<Float> {
        let scale = length(root.convert(direction: [1, 0, 0], from: referenceEntity))
        return root.convert(direction: moment, from: referenceEntity) * scale
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
        simulation.addTorque(convert(moment: torque, from: referenceEntity, to: root), to: entity)
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
        simulation.applyAngularImpulse(convert(moment: impulse, from: referenceEntity, to: root), to: entity)
    }

    public func applyImpulse(_ impulse: SIMD3<Float>, at position: SIMD3<Float>, relativeTo referenceEntity: Entity?) {
        guard let (simulation, root) = simulation else { return }
        simulation.applyImpulse(root.convert(direction: impulse, from: referenceEntity),
                                at: root.convert(position: position, from: referenceEntity), to: entity)
    }
}
