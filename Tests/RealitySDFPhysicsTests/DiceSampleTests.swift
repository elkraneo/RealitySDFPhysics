import RealityKit
import RealitySDFPhysics
import SDFRealityKitSampleScenes
import Testing
import simd

/// The Dice sample's hand: a held die stays calm at the pointer, by whichever point it was
/// grabbed, and falls back onto the table when let go
@MainActor
@Suite struct DiceSampleTests {
    // The light D4, by its top and near a corner, and the D20 near a corner, which a spring added
    // as a force spun up to hundreds of radians per second
    @Test(arguments: [(0, SIMD3<Float>(0, 1, 0)), (0, SIMD3<Float>(0.7, 0.7, 0.7)), (6, SIMD3<Float>(0.7, 0.7, 0.7))])
    func heldDieStaysCalm(index: Int, grab: SIMD3<Float>) async throws {
        let scene = DiceScene()
        try await scene.load()
        let root = try #require(scene.content.children.first { $0.components.has(SDFPhysicsSimulationComponent.self) })
        let simulation = try #require(root.components[SDFPhysicsSimulationComponent.self]).simulation
        let die = try #require(root.findEntity(named: DieKind.allCases[index].rawValue))
        for _ in 0..<60 { simulation.step(root: root) }

        let center = try #require(simulation.transform(of: die)).translation
        scene.pickUp(die: index, at: center + grab, tossAll: false)
        let target = center + grab + [0, 6, 0]
        scene.move(heldTo: target)
        var fastestSpin: Float = 0
        for _ in 0..<180 {
            simulation.step(root: root)
            fastestSpin = max(fastestSpin, length(try #require(die.components[SDFPhysicsMotionComponent.self]).angularVelocity))
        }
        #expect(fastestSpin < 30)
        // Hanging by the grabbed point, at the target
        let held = try #require(simulation.transform(of: die)).translation
        #expect(abs(length(held - target) - length(grab)) < 0.2)

        scene.toss(maxScore: false)
        for _ in 0..<120 { simulation.step(root: root) }
        let landed = try #require(simulation.transform(of: die)).translation
        #expect(length(SIMD2(landed.x, landed.z)) < 50 && landed.y > 0 && landed.y < 3)
    }

    /// A die picked up by a script right after loading, before the simulation has its body, is
    /// held once the first step adds it
    @Test func pickUpBeforeFirstStep() async throws {
        let scene = DiceScene()
        try await scene.load()
        let root = try #require(scene.content.children.first { $0.components.has(SDFPhysicsSimulationComponent.self) })
        let simulation = try #require(root.components[SDFPhysicsSimulationComponent.self]).simulation
        let die = try #require(root.findEntity(named: DieKind.d6.rawValue))
        let center = die.position
        scene.pickUp(die: 1, tossAll: false)
        scene.move(heldTo: center + [0, 7, 0])
        for _ in 0..<120 { simulation.step(root: root) }
        // Hanging by the top of its center, 1 cm below the target
        #expect(length(try #require(simulation.transform(of: die)).translation - (center + [0, 6, 0])) < 0.3)
    }
}
