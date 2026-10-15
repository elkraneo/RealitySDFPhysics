import Foundation
import RealityKit
import SDFCollisionKit
import SDFDemoViewer
import SDFMechanicsScenes
import simd

/// The Absolut scene's hand on the pedals, which drives the crank's hinge every frame once
/// dragged, and the zone over the plato that activates the hung objects
@MainActor
final class AbsolutInteraction: DemoInteraction {
    private var root: Entity?
    private var simulation: SDFSimulation?
    private var crank: (entity: Entity, hinge: Int)?
    private var pedalDrive = PedalDrive()
    private var isPedaled = false
    private var carrier: CarrierZone?

    var status: String? { carrier?.active.map { "Active: \($0.name)" } }

    func attach(root: Entity, simulation: SDFSimulation) {
        self.root = root
        self.simulation = simulation
        crank = PedalDrive.crank(in: root)
        carrier = CarrierZone(root: root, simulation: simulation)
    }

    /// Takes the crank by a ray that meets the chain's plane within the pedals' reach of its axle;
    /// false without a crank there
    func beginDrag(origin: SIMD3<Float>, direction: SIMD3<Float>) -> Bool {
        guard let root, let crank, let angle = crankAngle,
              let point = PedalDrive.pointOnPlane(origin: origin, direction: direction)
        else { return false }
        let axle = crank.entity.position(relativeTo: root)
        guard simd_distance(point, axle) < 4 else { return false }
        pedalDrive.grab(at: PedalDrive.angle(of: point, about: axle), crankAngle: angle)
        isPedaled = true
        return true
    }

    func moveDrag(origin: SIMD3<Float>, direction: SIMD3<Float>) {
        guard let root, let crank, let point = PedalDrive.pointOnPlane(origin: origin, direction: direction) else { return }
        pedalDrive.move(to: PedalDrive.angle(of: point, about: crank.entity.position(relativeTo: root)))
    }

    func endDrag() {
        pedalDrive.release()
    }

    /// Sets the crank's motor from the drive once the pedals have been taken; until then the
    /// scene's own motor turns it
    func update(deltaTime: TimeInterval) {
        guard isPedaled, let root, let crank, var hinge, let angle = crankAngle else { return }
        let motor = pedalDrive.motor(crankAngle: angle)
        guard hinge.motor != motor else { return }
        hinge.motor = motor
        root.components[SDFJointsComponent.self]?.joints[crank.hinge] = .revolute(hinge)
    }

    /// The crank's own hinge, as the root holds it
    private var hinge: SDFRevoluteJoint? {
        guard let crank, let joints = root?.components[SDFJointsComponent.self]?.joints, joints.indices.contains(crank.hinge),
              case .revolute(let hinge) = joints[crank.hinge]
        else { return nil }
        return hinge
    }

    private var crankAngle: Float? {
        hinge.flatMap { simulation?.angle(of: $0) }
    }
}
