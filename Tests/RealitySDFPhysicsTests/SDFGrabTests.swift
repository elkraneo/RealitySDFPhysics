import RealityKit
import RealitySDFPhysics
import Testing
import simd

/// The drag as a hand holds a body: a soft pull toward the target, with a capped force, that a
/// body follows freely and that stops where something resists, rather than pushing through it.
@MainActor
@Suite struct SDFGrabTests {
    let root = Entity()
    let simulation = SDFSimulation()

    init() {
        var settings = SDFPhysicsSimulationComponent(simulation: simulation)
        settings.isSleepingEnabled = false
        root.components.set(settings)
    }

    func addBox(_ name: String, halfExtents: SIMD3<Float>, at position: SIMD3<Float>, mode: PhysicsBodyMode = .dynamic) -> Entity {
        let box = Entity()
        box.name = name
        box.position = position
        box.components.set(SDFPhysicsBodyComponent(shape: .box(halfExtents: halfExtents), mode: mode))
        root.addChild(box)
        return box
    }

    /// The least separation of the contacts between two entities in the last substep
    func separation(_ a: Entity, _ b: Entity) -> Float {
        simulation.contacts()
            .filter { Set([$0.entityA, $0.entityB]) == Set([a.id, b.id]) }
            .map(\.separation)
            .min() ?? .infinity
    }

    @Test func defaultsAreTheEngines() {
        let options = simulation.dragOptions
        #expect(options == SDFSimulation.DragOptions())
        var stiff = options
        stiff.hertz = 10
        stiff.maxForce = 100
        simulation.dragOptions = stiff
        #expect(simulation.dragOptions == stiff)
    }

    @Test func freeBoxFollowsTheTarget() {
        let box = addBox("box", halfExtents: [0.25, 0.25, 0.25], at: [0, 2, 0])
        simulation.step(root: root)
        #expect(simulation.beginDrag(box, at: box.position))
        let target = box.position + [1, 0, 0]
        simulation.drag(to: target)
        for _ in 0..<60 { simulation.step(root: root) }
        let miss = simd_distance(box.position, target)
        #expect(miss < 0.02, "the box is \(miss) from the target after a second")
        simulation.endDrag()
    }

    @Test func boxPulledIntoAWallStopsAtIt() {
        _ = addBox("ground", halfExtents: [4, 0.5, 4], at: [0, -0.5, 0], mode: .static)
        let wall = addBox("wall", halfExtents: [0.1, 1, 1], at: [1.5, 1, 0], mode: .static)
        let box = addBox("box", halfExtents: [0.25, 0.25, 0.25], at: [0, 0.25, 0])
        simulation.step(root: root)
        #expect(simulation.beginDrag(box, at: box.position))
        // 2 m past the wall's far face
        simulation.drag(to: [3.6, 0.25, 0])
        var deepest = Float.infinity, farthest = -Float.infinity
        for _ in 0..<180 {
            simulation.step(root: root)
            deepest = min(deepest, separation(box, wall))
            farthest = max(farthest, box.position.x)
        }
        let report = Comment(rawValue: "least separation \(deepest), box center up to x = \(farthest), wall face at 1.4")
        #expect(deepest > -0.02, report)
        #expect(farthest + 0.25 < 1.4 + 0.02, report)
        #expect(farthest > 1.0, report)
    }

    /// The spiky wheels of SDFJointedMotionTests, meshed as gears
    func addGears(driverMotor: SDFPhysicsRevoluteJoint.Motor?) throws -> (driver: Entity, driven: Entity) {
        try addSpikyWheels(to: root, driverMotor: driverMotor)
    }

    /// The angle a gear has turned about y from its start
    func turn(of gear: Entity, from start: simd_quatf) -> Float {
        let delta = gear.orientation * start.inverse
        return 2 * atan2(delta.imag.y, delta.real)
    }

    @Test func gearPulledAgainstAHeldGearDoesNotPushTeethThroughTeeth() throws {
        // The driver held still by a motor far stronger than the pull
        let (driver, driven) = try addGears(driverMotor: .init(targetVelocity: 0, maxTorque: 1e6))
        simulation.step(root: root)
        let start = driven.orientation
        // A point on the driven gear's pitch circle, where the spikes meet, pulled along the
        // tangent, 2 m on
        let grip = driven.position + [0, 0, 0.75]
        #expect(simulation.beginDrag(driven, at: grip))
        simulation.drag(to: grip + [-2, 0, 0])
        var deepest = Float.infinity
        for _ in 0..<120 {
            simulation.step(root: root)
            deepest = min(deepest, separation(driver, driven))
        }
        let turned = turn(of: driven, from: start)
        let report = Comment(rawValue: "least separation of the teeth \(deepest), driven gear turned \(turned) rad")
        #expect(deepest > -0.05, report)
        #expect(abs(turned) < 0.3, report)
    }

    @Test func gearPulledAlongItsTangentTurnsBoth() throws {
        let (driver, driven) = try addGears(driverMotor: nil)
        simulation.step(root: root)
        let driverStart = driver.orientation, drivenStart = driven.orientation
        let grip = driven.position + [0, 0, 0.75]
        #expect(simulation.beginDrag(driven, at: grip))
        simulation.drag(to: grip + [-2, 0, 0])
        var deepest = Float.infinity
        for _ in 0..<120 {
            simulation.step(root: root)
            deepest = min(deepest, separation(driver, driven))
        }
        let drivenTurn = turn(of: driven, from: drivenStart), driverTurn = turn(of: driver, from: driverStart)
        let report = Comment(rawValue: "driven turned \(drivenTurn) rad, driver \(driverTurn) rad, least separation \(deepest)")
        // Toward the target, about 1.2 rad round, and the driver about as far the other way
        #expect(abs(drivenTurn) > 0.8, report)
        #expect(driverTurn * drivenTurn < 0 && abs(driverTurn) > 0.3 * abs(drivenTurn), report)
        #expect(deepest > -0.05, report)
    }

    /// A chain of 12 capsule links on hinges about z, hung in a V from two static anchors 5.2
    /// apart: each half straight, 30° below the horizontal. The returned links are in order.
    func addChain() throws -> [Entity] {
        let pitch: Float = 0.5, count = 12, slope = Float.pi / 6
        let anchors = addBox("anchors", halfExtents: [0.1, 0.1, 0.1], at: [0, 4, 0], mode: .static)
        let axle = simd_quatf(angle: -.pi / 2, axis: [0, 1, 0]) // The pin's x axis along z
        let shape = SDFShape.capsule(radius: 0.06, halfLength: pitch / 2)
        // The pins from the left anchor: down to the middle, then up to the right anchor
        let span = Float(count) * pitch * cos(slope)
        var pins: [SIMD3<Float>] = [[-span / 2, 4, 0]]
        for i in 0..<count {
            let down = i < count / 2
            pins.append(pins[i] + pitch * SIMD3(cos(slope), down ? -sin(slope) : sin(slope), 0))
        }
        var links: [Entity] = []
        for i in 0..<count {
            let link = Entity()
            link.name = "link \(i)"
            link.position = (pins[i] + pins[i + 1]) / 2
            // The capsule's y axis along the link, from pin i to pin i + 1
            link.orientation = simd_quatf(from: [0, 1, 0], to: simd_normalize(pins[i + 1] - pins[i]))
            link.components.set(SDFPhysicsBodyComponent(shape: shape))
            link.components.set(SDFPhysicsMotionComponent())
            root.addChild(link)
            links.append(link)
        }
        func pin(_ entity: Entity, _ name: String, at position: SIMD3<Float>) -> GeometricPin {
            entity.pins.set(named: name, position: position, orientation: axle, relativeTo: root)
        }
        // Each link pinned to the one before, the first and the last to the anchors
        var previous = pin(anchors, "left", at: pins[0])
        for (i, link) in links.enumerated() {
            try SDFPhysicsRevoluteJoint(pin0: previous, pin1: pin(link, "start", at: pins[i])).addToSimulation()
            previous = pin(link, "end", at: pins[i + 1])
        }
        try SDFPhysicsRevoluteJoint(pin0: pin(anchors, "right", at: pins[count]), pin1: previous).addToSimulation()
        return links
    }

    @Test func chainLinkPulledAwayKeepsItsPins() throws {
        let links = try addChain()
        for _ in 0..<60 { simulation.step(root: root) }
        // The third link, pulled down, out of the hanging chain
        let link = links[2]
        let start = link.position
        #expect(simulation.beginDrag(link, at: start))
        simulation.drag(to: start + [0, -1.5, 0])
        let joints = root.components[SDFPhysicsJointsComponent.self]!.joints
        var worst: Float = 0
        for _ in 0..<120 {
            simulation.step(root: root)
            for joint in joints {
                worst = max(worst, simd_distance(joint.pin0.position(relativeTo: root)!, joint.pin1.position(relativeTo: root)!))
            }
        }
        let moved = simd_distance(link.position, start)
        let report = Comment(rawValue: "pins up to \(worst) apart, the link moved \(moved)")
        #expect(worst < 0.05, report)
        #expect(moved > 0.1, report)
    }
}
