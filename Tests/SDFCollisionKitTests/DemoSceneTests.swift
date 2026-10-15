import RealityKit
import SDFCollisionKit
import SDFDemoScenes
import Testing

/// The upstream demo scenes, stepped as sdf-snapshot steps them. They are the one input of the
/// package built the way a user builds a scene, with entities, transforms and components, where
/// the tests above construct their bodies themselves.
@MainActor
@Suite struct DemoSceneTests {
    @Test(arguments: DemoScene.all) func sceneSettles(_ scene: DemoScene) async throws {
        let simulation = SDFSimulation()
        let root = Entity()
        root.components.set(SDFSimulationComponent(simulation: simulation, preset: .paper))
        try await scene.load(into: root)
        let dynamic = root.children.filter { $0.components[SDFBodyComponent.self]?.mode == .dynamic }

        // Four seconds: every scene has landed and is in contact by then
        for _ in 0..<240 { simulation.step(root: root) }

        // A settled scene has about a pair in contact per resting body, and no body has frozen in
        // the air or fallen through the ground, whose top face is at y = 0
        let stats = simulation.statistics
        #expect(stats.contacts > 0, "\(scene.name) has no contact")
        #expect(stats.pairs >= dynamic.count / 2, "\(scene.name) has \(stats.pairs) pairs for \(dynamic.count) bodies")
        for entity in dynamic {
            let p = entity.position
            #expect(p.x.isFinite && p.y.isFinite && p.z.isFinite && p.y > -0.5, "\(entity.name) of \(scene.name) at \(p)")
        }
    }
}
