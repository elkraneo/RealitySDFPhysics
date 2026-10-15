import RealityKit
import SDFCollisionKit
import SDFDemoScenes
import Testing

/// The upstream demo scenes, stepped as sdf-snapshot steps them. They are the one input of the
/// package built the way a user builds a scene, with entities, transforms and components, where
/// the tests above construct their bodies themselves.
@MainActor
@Suite struct DemoSceneTests {
    @Test(arguments: DemoScene.all) func sceneSimulates(_ scene: DemoScene) async throws {
        let simulation = SDFSimulation()
        let root = Entity()
        root.components.set(SDFSimulationComponent(simulation: simulation, preset: .paper))
        try await scene.load(into: root)

        var contacts = 0
        for _ in 0..<90 {
            simulation.step(root: root)
            contacts += simulation.statistics.contacts
        }
        #expect(contacts > 0, "\(scene.name) had no contact in 90 steps")
        // The ground's top face is at y = 0
        for entity in root.children where entity.components[SDFBodyComponent.self]?.mode == .dynamic {
            let p = entity.position
            #expect(p.x.isFinite && p.y.isFinite && p.z.isFinite && p.y > -0.5, "\(entity.name) of \(scene.name) at \(p)")
        }
    }
}
