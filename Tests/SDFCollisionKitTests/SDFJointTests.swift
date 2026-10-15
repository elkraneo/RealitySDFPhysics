import RealityKit
import SDFCollisionKit
import Testing
import simd

@MainActor
@Suite struct SDFJointTests {
    let root = Entity()
    let simulation = SDFSimulation()

    init() {
        var settings = SDFSimulationComponent(simulation: simulation)
        settings.isSleepingEnabled = false
        root.components.set(settings)
    }

    @discardableResult
    func addBody(_ shape: SDFShape, mode: PhysicsBodyMode = .dynamic, at position: SIMD3<Float>) -> Entity {
        let entity = Entity()
        entity.position = position
        entity.components.set(SDFBodyComponent(shape: shape, mode: mode))
        root.addChild(entity)
        return entity
    }

    func run(_ steps: Int) {
        for _ in 0..<steps { simulation.step(root: root) }
    }

    /// A box hanging from a hinge on a static anchor, its pin at the top edge of the box
    func hangBox(at position: SIMD3<Float> = [0, 2, 0], halfHeight: Float = 0.5) -> (anchor: Entity, box: Entity, joint: SDFRevoluteJoint) {
        let anchor = addBody(.box(halfExtents: [0.1, 0.1, 0.1]), mode: .static, at: position + [0, halfHeight, 0])
        let box = addBody(.box(halfExtents: [0.1, halfHeight, 0.1]), at: position)
        let pin0 = anchor.pins.set(named: "hinge")
        let pin1 = box.pins.set(named: "top", position: [0, halfHeight, 0])
        return (anchor, box, SDFRevoluteJoint(pin0: pin0, pin1: pin1))
    }

    /// The distance between the pins of a joint, in the root's frame
    func gap(_ joint: SDFRevoluteJoint) -> Float {
        simd_distance(joint.pin0.position(relativeTo: root)!, joint.pin1.position(relativeTo: root)!)
    }

    @Test func pendulumSwingsOnItsPin() throws {
        let (_, box, joint) = hangBox()
        try joint.addToSimulation()
        box.components.set(SDFMotionComponent(linearVelocity: [0, 0, 3]))
        var farthest: Float = 0
        for _ in 0..<240 {
            run(1)
            farthest = max(farthest, abs(box.position.z))
            #expect(gap(joint) < 0.02, "pins \(gap(joint)) apart")
        }
        #expect(farthest > 0.3, "swung \(farthest)")
        // The hinge's x axis stays aligned: the box doesn't roll or yaw
        let up = box.orientation.act([0, 1, 0])
        #expect(abs(up.x) < 0.05)
    }

    @Test func limitHoldsTheAngle() throws {
        let (_, box, joint) = hangBox()
        var limited = joint
        limited.angularLimit = -0.3...0.3
        try limited.addToSimulation()
        box.components.set(SDFMotionComponent(linearVelocity: [0, 0, 6]))
        run(240)
        let angle = try #require(simulation.angle(ofJointAt: 0))
        #expect(abs(angle) <= 0.35, "angle \(angle)")
        #expect(gap(limited) < 0.02)
    }

    @Test func motorReachesItsSpeed() throws {
        let base = addBody(.box(halfExtents: [0.5, 0.1, 0.5]), mode: .static, at: [0, 0, 0])
        let wheel = addBody(.cylinder(radius: 0.5, halfHeight: 0.1), at: [0, 1, 0])
        wheel.components.set(SDFMotionComponent())
        let pin0 = base.pins.set(named: "axle", position: [0, 1, 0], orientation: simd_quatf(angle: .pi / 2, axis: [0, 0, 1]))
        let pin1 = wheel.pins.set(named: "hub", orientation: simd_quatf(angle: .pi / 2, axis: [0, 0, 1]))
        var joint = SDFRevoluteJoint(pin0: pin0, pin1: pin1)
        joint.motor = SDFRevoluteJoint.Motor(targetVelocity: 5, maxTorque: 1000)
        try joint.addToSimulation()
        run(120)
        let w = wheel.components[SDFMotionComponent.self]!.angularVelocity
        #expect(abs(w.y - 5) < 0.2, "spins at \(w)")
        #expect(gap(joint) < 0.01)
    }

    @Test func chainHangsWithoutStretching() throws {
        let links = 20
        let half: Float = 0.1
        let anchor = addBody(.box(halfExtents: [0.05, 0.05, 0.05]), mode: .static, at: [0, 5, 0])
        var previous = anchor.pins.set(named: "end")
        var joints: [SDFRevoluteJoint] = []
        var last: Entity = anchor
        for i in 0..<links {
            let y = 5 - half - Float(i) * 2 * half
            let link = addBody(.capsule(radius: 0.03, halfLength: half), at: [0, y, 0])
            let top = link.pins.set(named: "top", position: [0, half, 0])
            joints.append(SDFRevoluteJoint(pin0: previous, pin1: top))
            previous = link.pins.set(named: "bottom", position: [0, -half, 0])
            last = link
        }
        for joint in joints { try joint.addToSimulation() }
        // A push on the last link sets the chain swinging; it settles hanging, every pin holding
        last.components.set(SDFMotionComponent(linearVelocity: [0, 0, 2]))
        run(900)
        let length = Float(links) * 2 * half
        let hang = 5 - last.position.y
        let gaps = joints.map { gap($0) }
        let lean = simd_length(SIMD2(last.position.x, last.position.z))
        #expect(hang > length - 0.15 && hang < length + 0.02, "hangs \(hang) for a chain of \(length), lean \(lean)")
        #expect(gaps.reduce(0, +) < 0.02, "pins \(gaps.reduce(0, +)) apart in all, \(gaps.max()!) at most")
    }
}
