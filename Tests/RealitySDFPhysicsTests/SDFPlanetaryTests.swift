import RealityKit
import RealitySDFPhysics
import SDFDemoScenes
import SDFMechanicsScenes
import Testing
import simd

/// The planetary set: a ring gear with inward teeth, one concave SDF, meshing with three planets
/// on a carrier around a motored sun. Braking one member selects the ratio.
@MainActor
@Suite struct SDFPlanetaryTests {
    let module: Float = 0.1, thickness: Float = 0.4

    @Test func bakedRingIsSolidInTheRimAndHollowAtTheCenter() throws {
        let shape = try GearMesh.ringShape(teeth: 36, module: module, thickness: thickness, outerRadius: 2.3)
        let pitch = GearMesh.pitchRadius(teeth: 36, module: module)
        #expect(shape.distance(at: [0, 0, 0]).distance > 1)
        #expect(shape.distance(at: [pitch - 1, 0, 0]).distance > 0.1)
        #expect(shape.distance(at: [2.1, 0, 0]).distance < -0.1)
        #expect(shape.distance(at: [2.5, 0, 0]).distance > 0.1)
        #expect(shape.distance(at: [2.1, thickness, 0]).distance > 0.1)
        // A tooth points inward halfway between two spaces, at 5°, and a space is open at 0°
        let between = Float.pi / 36
        #expect(shape.distance(at: [(pitch - 0.05) * cos(between), 0, (pitch - 0.05) * sin(between)]).distance < 0)
        #expect(shape.distance(at: [pitch - 0.05, 0, 0]).distance > 0)
    }

    /// Mean angular speeds about y over the last two of seven seconds, with the scene's backlash
    func run(brakingRing: Bool) async throws -> (sun: Float, carrier: Float, ring: Float, report: String) {
        let root = Entity(), simulation = SDFSimulation()
        var settings = SDFSimulationComponent(simulation: simulation, preset: .paper)
        settings.isSleepingEnabled = false
        root.components.set(settings)
        var builder = DemoBuilder(root: root)
        try await builder.spawn("Ground", at: [0, -1, 0], mode: .static, isVisible: false)
        try await builder.buildPlanetary(brakingRing: brakingRing)
        root.components.set(SDFJointsComponent(joints: builder.joints))
        let names = ["sun", "carrier", "ring"]
        var speeds = [[Float]](repeating: [], count: names.count)
        for _ in 0..<420 {
            simulation.step(root: root)
            for (i, name) in names.enumerated() {
                speeds[i].append(root.findEntity(named: name)!.components[SDFMotionComponent.self]!.angularVelocity.y)
            }
        }
        let summaries = speeds.map { all in
            let last = all.suffix(120)
            return (mean: last.reduce(0, +) / 120, min: last.min()!, max: last.max()!)
        }
        let report = zip(names, summaries).map { "\($0) \($1.mean) (\($1.min) to \($1.max))" }.joined(separator: ", ")
            + "; \(simulation.statistics.contacts) contacts, step \(simulation.statistics.stepMilliseconds) ms"
        return (summaries[0].mean, summaries[1].mean, summaries[2].mean, report)
    }

    @Test func brakedRingTurnsTheCarrierAQuarterAsFast() async throws {
        let speeds = try await run(brakingRing: true)
        let report = Comment(rawValue: speeds.report)
        #expect(abs(speeds.sun - 1) < 0.15, report)
        #expect(abs(speeds.carrier - 0.25) < 0.25 * 0.15, report)
        #expect(abs(speeds.ring) < 0.05, report)
    }

    @Test func brakedCarrierTurnsTheRingBackAThirdAsFast() async throws {
        let speeds = try await run(brakingRing: false)
        let report = Comment(rawValue: speeds.report)
        #expect(abs(speeds.sun - 1) < 0.15, report)
        #expect(abs(speeds.ring + 1 / 3) < 0.15 / 3, report)
        #expect(abs(speeds.carrier) < 0.05, report)
    }
}
