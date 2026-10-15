import RealityKit
import SDFCollisionKit
import SDFDemoScenes
import Testing
import simd

/// The Absolut scene: 48 links around a piñon of 18 teeth on the left and a plato of 29 on the
/// right, the plato motored clockwise, as a bicycle riding to the right pedals. The piñon must
/// turn by the chain alone, the same way, 29/18 as fast, with every body in the x-y plane.
@MainActor
@Suite struct SDFAbsolutTests {
    static let ratio: Float = 29.0 / 18

    /// The small sprocket on the left: the layout was first written for A larger than B
    @Test func layoutClosesWithTheSmallSprocketFirst() {
        let chain = ChainMesh()
        let layout = ChainLayout(chain: chain, links: 48, teethA: 18, teethB: 29)
        let pins = layout.pins
        #expect(pins.count == 48)
        let gaps = pins.indices.map { simd_distance(pins[$0], pins[($0 + 1) % pins.count]) }
        let report = Comment(rawValue: "center distance \(layout.centerDistance), gaps \(gaps.min()!) to \(gaps.max()!), closing \(gaps.last!)")
        #expect(abs(layout.centerDistance - 5.56) < 0.1, report)
        // A pitch around the sprockets, and on the runs closer by the slack, shared out: 4% of
        // the chain over the runs' 22 gaps is about 9% of each
        #expect(gaps.allSatisfy { $0 > 0.9 * chain.pitch && $0 < chain.pitch + 1e-4 }, report)
        #expect(abs(gaps.last! - chain.pitch) < 1e-3, report)
    }

    /// The scene loaded under a preset, with its sprockets and links
    func load(_ preset: SDFSimulationComponent.Preset) async throws
        -> (root: Entity, simulation: SDFSimulation, piñon: Entity, plato: Entity, links: [Entity]) {
        let root = Entity(), simulation = SDFSimulation()
        var settings = SDFSimulationComponent(simulation: simulation, preset: preset)
        settings.isSleepingEnabled = false
        root.components.set(settings)
        try await DemoScene.all.first { $0.name == "Absolut" }!.load(into: root)
        #expect(root.components[SDFSimulationComponent.self]!.gravity == [0, -3, 0])
        let links = root.children.filter { $0.name.hasPrefix("link") }
        #expect(links.count == 48)
        return (root, simulation, root.findEntity(named: "sprocket A")!, root.findEntity(named: "sprocket B")!, links)
    }

    /// The farthest any two pins of a joint are apart
    func worstPin(_ root: Entity) -> Float {
        root.components[SDFJointsComponent.self]!.joints.map {
            simd_distance($0.pin0.position(relativeTo: root)!, $0.pin1.position(relativeTo: root)!)
        }.max()!
    }

    @Test(arguments: [SDFSimulationComponent.Preset.realTime, .paper])
    func platoDrivesThePiñonThroughTheChain(preset: SDFSimulationComponent.Preset) async throws {
        let scene = try await load(preset)
        var speedsA: [Float] = [], speedsB: [Float] = []
        for _ in 0..<420 {
            scene.simulation.step(root: scene.root)
            speedsA.append(scene.piñon.components[SDFMotionComponent.self]!.angularVelocity.z)
            speedsB.append(scene.plato.components[SDFMotionComponent.self]!.angularVelocity.z)
        }
        let lastA = speedsA.suffix(120), lastB = speedsB.suffix(120)
        let meanA = lastA.reduce(0, +) / 120, meanB = lastB.reduce(0, +) / 120
        let worst = worstPin(scene.root)
        let drift = scene.links.map { abs($0.position.z) }.max()!
        let report = Comment(rawValue: "\(preset): piñon \(meanA) (\(lastA.min()!) to \(lastA.max()!)), plato \(meanB) (\(lastB.min()!) to \(lastB.max()!)) over the last two seconds; pins up to \(worst) apart, links up to \(drift) off the plane; step \(scene.simulation.statistics.stepMilliseconds) ms")
        #expect(abs(meanB + 1) < 0.15, report)
        #expect(abs(meanA + Self.ratio) < 0.15 * Self.ratio, report)
        #expect(worst < 0.03, report)
        #expect(drift < 1e-3, report)
        #expect(scene.links.allSatisfy { $0.position.y > 0 && $0.position.y < 8 }, report)
        print(report.rawValue)
    }

    /// A minute of pedaling: the piñon's turn stays 29/18 of the plato's. The chain's slack moves
    /// from the top run to the bottom one as the motor takes it up, which puts the piñon about a
    /// tooth behind in the first seconds and lets it wander a little after; a skipped tooth would
    /// stay.
    @Test func noToothSkippedInAMinute() async throws {
        let scene = try await load(.realTime)
        let timeStep = scene.root.components[SDFSimulationComponent.self]!.timeStep
        let tooth = 2 * Float.pi / 18
        var angleA: Float = 0, angleB: Float = 0, worst: Float = 0
        var slips: [Float] = [] // In teeth of the piñon, every second
        for step in 1...3600 {
            scene.simulation.step(root: scene.root)
            angleA += scene.piñon.components[SDFMotionComponent.self]!.angularVelocity.z * timeStep
            angleB += scene.plato.components[SDFMotionComponent.self]!.angularVelocity.z * timeStep
            // After the first second, which lays the laid out runs straight
            if step > 60 { worst = max(worst, worstPin(scene.root)) }
            if step % 60 == 0 { slips.append((angleA - Self.ratio * angleB) / tooth) }
        }
        let settled = slips[4], last = slips.last!
        let report = Comment(rawValue: "60 s: plato \(angleB) rad, piñon \(angleA) rad (\(angleA / angleB)×); piñon off by \(settled) teeth at 5 s, \(last) at 60 s, \(slips[4...].min()!) to \(slips[4...].max()!) in between; pins up to \(worst) apart, \(worstPin(scene.root)) at the end")
        #expect(abs(angleB + 60) < 6, report)
        #expect(abs(last - settled) < 0.5, report)
        #expect(worst < 0.15, report)
        #expect(worstPin(scene.root) < 0.03, report)
        print(report.rawValue)
    }
}
