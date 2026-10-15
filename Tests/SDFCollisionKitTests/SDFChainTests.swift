import RealityKit
import SDFCollisionKit
import SDFDemoScenes
import Testing
import simd

/// The chain scene: 48 links on pins around sprockets of 20 and 10 teeth, the big one motored.
/// The small one must turn by the chain alone, the same way, twice as fast.
@MainActor
@Suite struct SDFChainTests {
    @Test(arguments: [SDFSimulationComponent.Preset.realTime, .paper], [Float(0.04)])
    func motoredSprocketDrivesTheOtherThroughTheChain(preset: SDFSimulationComponent.Preset, slack: Float) async throws {
        let root = Entity(), simulation = SDFSimulation()
        var settings = SDFSimulationComponent(simulation: simulation, preset: preset)
        settings.isSleepingEnabled = false
        root.components.set(settings)
        var chain = ChainMesh()
        chain.slack = slack
        var builder = DemoBuilder(root: root)
        try await builder.spawn("Ground", at: [0, -1, 0], mode: .static, isVisible: false)
        try await builder.buildChain(chain: chain)
        root.components.set(SDFJointsComponent(joints: builder.joints))
        let a = root.findEntity(named: "sprocket A")!, b = root.findEntity(named: "sprocket B")!
        let links = root.children.filter { $0.name.hasPrefix("link") }
        #expect(links.count == 48)

        var speedsA: [Float] = [], speedsB: [Float] = []
        for _ in 0..<420 {
            simulation.step(root: root)
            speedsA.append(a.components[SDFMotionComponent.self]!.angularVelocity.z)
            speedsB.append(b.components[SDFMotionComponent.self]!.angularVelocity.z)
        }
        let lastA = speedsA.suffix(120), lastB = speedsB.suffix(120)
        let meanA = lastA.reduce(0, +) / 120, meanB = lastB.reduce(0, +) / 120
        let report = Comment(rawValue: "\(preset) slack \(slack): A \(meanA) (\(lastA.min()!) to \(lastA.max()!)), B \(meanB) (\(lastB.min()!) to \(lastB.max()!)) over the last two seconds; \(simulation.statistics.contacts) contacts, \(simulation.statistics.pairs) pairs, step \(simulation.statistics.stepMilliseconds) ms")
        #expect(abs(meanA - 1) < 0.15, report)
        #expect(meanB > 1.7 && meanB < 2.3, report)
        // Every link still on its pins, and none flung away
        let joints = root.components[SDFJointsComponent.self]!.joints
        var worst: Float = 0
        for joint in joints {
            worst = max(worst, simd_distance(joint.pin0.position(relativeTo: root)!, joint.pin1.position(relativeTo: root)!))
        }
        #expect(worst < 0.03, "\(preset) slack \(slack): pins up to \(worst) apart")
        #expect(links.allSatisfy { $0.position.y > 0 && $0.position.y < 6 })
    }
}
