import RealityKit
import RealitySDFPhysics
import SDFDemoScenes
import SDFMechanicsScenes
import Testing
import simd

/// The drag as a hand holds a body: a soft pull toward the target, with a capped force, that a
/// body follows freely and that stops where something resists, rather than pushing through it.
@MainActor
@Suite struct SDFGrabTests {
    let root = Entity()
    let simulation = SDFSimulation()

    init() {
        var settings = SDFSimulationComponent(simulation: simulation)
        settings.isSleepingEnabled = false
        root.components.set(settings)
    }

    func addBox(_ name: String, halfExtents: SIMD3<Float>, at position: SIMD3<Float>, mode: PhysicsBodyMode = .dynamic) -> Entity {
        let box = Entity()
        box.name = name
        box.position = position
        box.components.set(SDFBodyComponent(shape: .box(halfExtents: halfExtents), mode: mode))
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

    /// The gears of SDFGearTests: 20 and 10 teeth of module 0.1 on hinges to a static base
    func addGears(driverMotor: SDFRevoluteJoint.Motor?) throws -> (driver: Entity, driven: Entity) {
        let base = addBox("base", halfExtents: [1, 0.1, 1], at: [0, -5, 0], mode: .static)
        let axle = simd_quatf(angle: .pi / 2, axis: [0, 0, 1]) // The pin's x axis along y
        var gears: [Entity] = []
        for (name, teeth, x, turn) in [("driver", 20, Float(0), Float(0)), ("driven", 10, 1.58, Float.pi / 10)] {
            let gear = Entity()
            gear.name = name
            gear.position = [x, 0, 0]
            gear.orientation = simd_quatf(angle: turn, axis: [0, 1, 0])
            var body = SDFBodyComponent(shape: try GearMesh.shape(teeth: teeth, module: 0.1, thickness: 0.4))
            body.material = SDFMaterial(staticFriction: 0.1, dynamicFriction: 0.1, restitution: 0)
            gear.components.set(body)
            gear.components.set(SDFMotionComponent())
            root.addChild(gear)
            base.pins.set(named: name, position: gear.position, orientation: axle)
            gear.pins.set(named: "hub", orientation: gear.orientation.inverse * axle)
            var joint = SDFRevoluteJoint(pin0: base.pins[name]!, pin1: gear.pins["hub"]!)
            if name == "driver" { joint.motor = driverMotor }
            try joint.addToSimulation()
            gears.append(gear)
        }
        return (gears[0], gears[1])
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
        // A point on the driven gear's pitch circle, pulled along the tangent, 2 m on
        let grip = driven.position + [0, 0, 0.5]
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
        let grip = driven.position + [0, 0, 0.5]
        #expect(simulation.beginDrag(driven, at: grip))
        simulation.drag(to: grip + [-2, 0, 0])
        var deepest = Float.infinity
        for _ in 0..<120 {
            simulation.step(root: root)
            deepest = min(deepest, separation(driver, driven))
        }
        let drivenTurn = turn(of: driven, from: drivenStart), driverTurn = turn(of: driver, from: driverStart)
        let report = Comment(rawValue: "driven turned \(drivenTurn) rad, driver \(driverTurn) rad, least separation \(deepest)")
        // Toward the target, about 1.3 rad round, and the driver half as far the other way
        #expect(abs(drivenTurn) > 0.8, report)
        #expect(driverTurn * drivenTurn < 0 && abs(driverTurn) > 0.3 * abs(drivenTurn), report)
        #expect(deepest > -0.05, report)
    }

    @Test func chainLinkPulledAwayKeepsItsPins() async throws {
        root.components[SDFSimulationComponent.self]!.isSleepingEnabled = false
        try await MechanicsScenes.all.first { $0.name == "Chain" }!.load(into: root)
        for _ in 0..<60 { simulation.step(root: root) }
        let links = root.children.filter { $0.name.hasPrefix("link") }
        // The lowest link, pulled down and out of the loop
        let link = links.min { $0.position.y < $1.position.y }!
        let start = link.position
        #expect(simulation.beginDrag(link, at: start))
        simulation.drag(to: start + [0, -1.5, 0])
        let joints = root.components[SDFJointsComponent.self]!.joints
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
