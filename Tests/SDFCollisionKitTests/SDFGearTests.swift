import RealityKit
import SDFCollisionKit
import SDFDemoScenes
import Testing
import simd

/// Two involute gears on hinges, one driven by a motor: does the other turn by the contact of
/// their teeth alone, at the ratio of their teeth? This is the experiment behind the chain and
/// gear mechanics: nothing but shapes and hinges.
@MainActor
@Suite struct SDFGearTests {
    let root = Entity()
    let simulation = SDFSimulation()
    let module: Float = 0.1, thickness: Float = 0.4
    let friction: Float = 0.1

    init() {
        var settings = SDFSimulationComponent(simulation: simulation)
        settings.isSleepingEnabled = false
        root.components.set(settings)
    }

    /// A gear on a hinge to the base, its axis along y
    func addGear(teeth: Int, at position: SIMD3<Float>, turned angle: Float, base: Entity, name: String) throws -> Entity {
        let gear = Entity()
        gear.name = name
        gear.position = position
        gear.orientation = simd_quatf(angle: angle, axis: [0, 1, 0])
        var body = SDFBodyComponent(shape: try GearMesh.shape(teeth: teeth, module: module, thickness: thickness))
        // Steel on steel, oiled: the default 0.5 is a friction cone wider than the pressure angle
        body.material = SDFMaterial(staticFriction: friction, dynamicFriction: friction, restitution: 0)
        gear.components.set(body)
        gear.components.set(SDFMotionComponent())
        root.addChild(gear)
        let axle = simd_quatf(angle: .pi / 2, axis: [0, 0, 1]) // The pin's x axis along y
        base.pins.set(named: name, position: position, orientation: axle)
        gear.pins.set(named: "hub", orientation: gear.orientation.inverse * axle)
        return gear
    }

    @Test func bakedGearIsSolidInsideAndHollowOutside() throws {
        let shape = try GearMesh.shape(teeth: 20, module: module, thickness: thickness)
        let pitch = GearMesh.pitchRadius(teeth: 20, module: module)
        #expect(shape.distance(at: [0, 0, 0]).distance < -0.1)
        #expect(shape.distance(at: [pitch - 0.3, 0, 0]).distance < 0)
        #expect(shape.distance(at: [pitch + 0.3, 0, 0]).distance > 0.1)
        #expect(shape.distance(at: [0, thickness, 0]).distance > 0.1)
    }

    @Test func motoredGearDrivesTheOtherAtTheirRatio() throws {
        // Backlash: the pitch circles 0.08 apart, four times the contact skin, which keeps the
        // flanks a skin apart. Tighter, the teeth wedge; thinner skins jam sooner, not later.
        let clearance: Float = 0.08
        let base = Entity()
        base.position = [0, -5, 0]
        base.components.set(SDFBodyComponent(shape: .box(halfExtents: [1, 0.1, 1]), mode: .static))
        root.addChild(base)
        // Pitch circles touching: radii 1 and 0.5; the small gear turned by half a tooth to mesh
        let driver = try addGear(teeth: 20, at: [0, 0, 0], turned: 0, base: base, name: "driver")
        let driven = try addGear(teeth: 10, at: [1.5 + clearance, 0, 0], turned: .pi / 10, base: base, name: "driven")
        var motored = SDFRevoluteJoint(pin0: base.pins["driver"]!, pin1: driver.pins["hub"]!)
        motored.motor = SDFRevoluteJoint.Motor(targetVelocity: 1, maxTorque: 500)
        try motored.addToSimulation()
        try SDFRevoluteJoint(pin0: base.pins["driven"]!, pin1: driven.pins["hub"]!).addToSimulation()

        var speeds: [Float] = [], driverSpeeds: [Float] = []
        for _ in 0..<300 {
            simulation.step(root: root)
            speeds.append(driven.components[SDFMotionComponent.self]!.angularVelocity.y)
            driverSpeeds.append(driver.components[SDFMotionComponent.self]!.angularVelocity.y)
        }
        let settled = speeds.suffix(120), driverSettled = driverSpeeds.suffix(120)
        let mean = settled.reduce(0, +) / Float(settled.count)
        let driverMean = driverSettled.reduce(0, +) / Float(driverSettled.count)
        let report = Comment(rawValue: "driver \(driverMean) (\(driverSettled.min()!) to \(driverSettled.max()!)), driven \(mean) (\(settled.min()!) to \(settled.max()!)) over the last two seconds")
        #expect(abs(driverMean - 1) < 0.1, report)
        #expect(mean < -1.8 && mean > -2.2, report)
        // A tooth may hiccup now and then; the small gear mostly runs between 1.5 and 2.5
        let steady = settled.filter { $0 < -1.5 && $0 > -2.5 }.count
        #expect(steady >= settled.count * 9 / 10, report)
        #expect(simulation.statistics.pairs > 0)
    }
}
