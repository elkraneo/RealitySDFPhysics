import RealityKit
import RealitySDFPhysics
import Testing
import simd

@MainActor
@Suite struct SDFJointTests {
    let root = Entity()
    let simulation = SDFSimulation()

    init() {
        var settings = SDFPhysicsSimulationComponent(simulation: simulation)
        settings.isSleepingEnabled = false
        root.components.set(settings)
    }

    @discardableResult
    func addBody(_ shape: SDFShape, mode: PhysicsBodyMode = .dynamic, at position: SIMD3<Float>) -> Entity {
        let entity = Entity()
        entity.position = position
        entity.components.set(SDFPhysicsBodyComponent(shape: shape, mode: mode))
        root.addChild(entity)
        return entity
    }

    func run(_ steps: Int) {
        for _ in 0..<steps { simulation.step(root: root) }
    }

    /// A box hanging from a hinge on a static anchor, its pin at the top edge of the box
    func hangBox(at position: SIMD3<Float> = [0, 2, 0], halfHeight: Float = 0.5) -> (anchor: Entity, box: Entity, joint: SDFPhysicsRevoluteJoint) {
        let anchor = addBody(.box(halfExtents: [0.1, 0.1, 0.1]), mode: .static, at: position + [0, halfHeight, 0])
        let box = addBody(.box(halfExtents: [0.1, halfHeight, 0.1]), at: position)
        let pin0 = anchor.pins.set(named: "hinge")
        let pin1 = box.pins.set(named: "top", position: [0, halfHeight, 0])
        return (anchor, box, SDFPhysicsRevoluteJoint(pin0: pin0, pin1: pin1))
    }

    /// The distance between the pins of a joint, in the root's frame
    func gap(_ joint: SDFPhysicsRevoluteJoint) -> Float {
        simd_distance(joint.pin0.position(relativeTo: root)!, joint.pin1.position(relativeTo: root)!)
    }

    @Test func pendulumSwingsOnItsPin() throws {
        let (_, box, joint) = hangBox()
        try joint.addToSimulation()
        box.components.set(SDFPhysicsMotionComponent(linearVelocity: [0, 0, 3]))
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
        box.components.set(SDFPhysicsMotionComponent(linearVelocity: [0, 0, 6]))
        run(240)
        let angle = try #require(simulation.angle(of: limited))
        #expect(abs(angle) <= 0.35, "angle \(angle)")
        #expect(gap(limited) < 0.02)
    }

    @Test func motorReachesItsSpeed() throws {
        let base = addBody(.box(halfExtents: [0.5, 0.1, 0.5]), mode: .static, at: [0, 0, 0])
        let wheel = addBody(.cylinder(radius: 0.5, halfHeight: 0.1), at: [0, 1, 0])
        wheel.components.set(SDFPhysicsMotionComponent())
        let pin0 = base.pins.set(named: "axle", position: [0, 1, 0], orientation: simd_quatf(angle: .pi / 2, axis: [0, 0, 1]))
        let pin1 = wheel.pins.set(named: "hub", orientation: simd_quatf(angle: .pi / 2, axis: [0, 0, 1]))
        var joint = SDFPhysicsRevoluteJoint(pin0: pin0, pin1: pin1)
        joint.motor = SDFPhysicsRevoluteJoint.Motor(targetVelocity: 5, maxTorque: 1000)
        try joint.addToSimulation()
        run(120)
        let w = wheel.components[SDFPhysicsMotionComponent.self]!.angularVelocity
        #expect(abs(w.y - 5) < 0.2, "spins at \(w)")
        #expect(gap(joint) < 0.01)
    }

    @Test func separatedPinsAreBroughtTogether() throws {
        // The pins start 0.3 apart and the box's pin turned 90° about the hinge: the joint pulls
        // the box up to the anchor, and the turn is the joint's angle, not an error
        let anchor = addBody(.box(halfExtents: [0.1, 0.1, 0.1]), mode: .static, at: [0, 2.5, 0])
        let box = addBody(.box(halfExtents: [0.1, 0.5, 0.1]), at: [0, 1.7, 0])
        let pin0 = anchor.pins.set(named: "hinge")
        let pin1 = box.pins.set(named: "top", position: [0, 0.5, 0], orientation: simd_quatf(angle: .pi / 2, axis: [1, 0, 0]))
        let joint = SDFPhysicsRevoluteJoint(pin0: pin0, pin1: pin1)
        try joint.addToSimulation()
        run(120)
        #expect(gap(joint) < 0.02, "pins \(gap(joint)) apart")
        let angle = try #require(simulation.angle(of: joint))
        #expect(abs(abs(angle) - .pi / 2) < 0.1, "angle \(angle)")
    }

    @Test func motorKeepsItsBodyAwake() throws {
        var settings = root.components[SDFPhysicsSimulationComponent.self]!
        settings.isSleepingEnabled = true
        root.components.set(settings)
        let base = addBody(.box(halfExtents: [0.5, 0.1, 0.5]), mode: .static, at: [0, 0, 0])
        let wheel = addBody(.cylinder(radius: 0.5, halfHeight: 0.1), at: [0, 1, 0])
        wheel.components.set(SDFPhysicsMotionComponent())
        let axle = simd_quatf(angle: .pi / 2, axis: [0, 0, 1])
        var joint = SDFPhysicsRevoluteJoint(pin0: base.pins.set(named: "axle", position: [0, 1, 0], orientation: axle),
                                     pin1: wheel.pins.set(named: "hub", orientation: axle))
        // Slower than the sleep threshold
        joint.motor = SDFPhysicsRevoluteJoint.Motor(targetVelocity: 0.02, maxTorque: 100)
        try joint.addToSimulation()
        run(240)
        #expect(!simulation.isSleeping(wheel))
        let w = wheel.components[SDFPhysicsMotionComponent.self]!.angularVelocity
        #expect(abs(w.y - 0.02) < 0.005, "spins at \(w)")
    }

    @Test func chainHangsWithoutStretching() throws {
        let links = 20
        let half: Float = 0.1
        let anchor = addBody(.box(halfExtents: [0.05, 0.05, 0.05]), mode: .static, at: [0, 5, 0])
        var previous = anchor.pins.set(named: "end")
        var joints: [SDFPhysicsRevoluteJoint] = []
        var last: Entity = anchor
        for i in 0..<links {
            let y = 5 - half - Float(i) * 2 * half
            let link = addBody(.capsule(radius: 0.03, halfLength: half), at: [0, y, 0])
            let top = link.pins.set(named: "top", position: [0, half, 0])
            joints.append(SDFPhysicsRevoluteJoint(pin0: previous, pin1: top))
            previous = link.pins.set(named: "bottom", position: [0, -half, 0])
            last = link
        }
        for joint in joints { try joint.addToSimulation() }
        // A push on the last link sets the chain swinging; it settles hanging, every pin holding
        last.components.set(SDFPhysicsMotionComponent(linearVelocity: [0, 0, 2]))
        run(900)
        let length = Float(links) * 2 * half
        let hang = 5 - last.position.y
        let gaps = joints.map { gap($0) }
        let lean = simd_length(SIMD2(last.position.x, last.position.z))
        #expect(hang > length - 0.15 && hang < length + 0.02, "hangs \(hang) for a chain of \(length), lean \(lean)")
        // Soft joints sag under load: 3 mm at the top pin, which carries the chain
        #expect(gaps.reduce(0, +) < 0.04, "pins \(gaps.reduce(0, +)) apart in all, \(gaps.max()!) at most")
        #expect(gaps.max()! < 0.005)
    }

    @Test func springTurnsADoorToItsTarget() throws {
        // A door on a vertical hinge: gravity pulls along the axis, so the spring alone sets the angle
        let frame = addBody(.box(halfExtents: [0.05, 0.5, 0.05]), mode: .static, at: [0, 1, 0])
        let door = addBody(.box(halfExtents: [0.4, 0.5, 0.05]), at: [0.45, 1, 0])
        let vertical = simd_quatf(angle: .pi / 2, axis: [0, 0, 1])
        var joint = SDFPhysicsRevoluteJoint(pin0: frame.pins.set(named: "hinge", orientation: vertical),
                                     pin1: door.pins.set(named: "hinge", position: [-0.45, 0, 0], orientation: vertical))
        joint.spring = SDFPhysicsRevoluteJoint.Spring(targetAngle: 1, hertz: 2, dampingRatio: 1)
        try joint.addToSimulation()
        run(180)
        let angle = try #require(simulation.angle(of: joint))
        #expect(abs(angle - 1) < 0.02, "angle \(angle)")
        #expect(gap(joint) < 0.02)
    }

    @Test func returnSpringBringsAPawlBack() throws {
        // A lever out along x on a hinge along z, held level against gravity by a stiff spring
        let post = addBody(.box(halfExtents: [0.05, 0.05, 0.05]), mode: .static, at: [0, 1, 0])
        let pawl = addBody(.box(halfExtents: [0.2, 0.02, 0.05]), at: [0.2, 1, 0])
        let alongZ = simd_quatf(angle: -.pi / 2, axis: [0, 1, 0])
        var joint = SDFPhysicsRevoluteJoint(pin0: post.pins.set(named: "pivot", orientation: alongZ),
                                     pin1: pawl.pins.set(named: "pivot", position: [-0.2, 0, 0], orientation: alongZ))
        joint.spring = SDFPhysicsRevoluteJoint.Spring(targetAngle: 0, hertz: 20, dampingRatio: 0.7)
        try joint.addToSimulation()
        run(30)
        // Pulled aside by a torque, as a ratchet's tooth would, then let go
        var pulled: Float = 0
        for _ in 0..<30 {
            pawl.sdf.addTorque([0, 0, 100], relativeTo: nil)
            run(1)
            pulled = max(pulled, abs(simulation.angle(of: joint)!))
        }
        #expect(pulled > 0.2, "pulled \(pulled)")
        run(120)
        var farthest: Float = 0
        for _ in 0..<60 {
            run(1)
            farthest = max(farthest, abs(simulation.angle(of: joint)!))
        }
        #expect(farthest < 0.02, "pulled \(pulled), then held within \(farthest)")
    }
}
