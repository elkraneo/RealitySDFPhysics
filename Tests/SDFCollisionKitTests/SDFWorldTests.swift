import SDFCollisionKit
import Testing
import simd

@MainActor
@Suite struct SDFWorldTests {
    @Test func listsTheUpstreamScenes() {
        #expect(SDFWorld.scenes.map(\.name) == [
            "Box Stack", "Gear Stack", "Falling Boxes", "Falling Boxes: Bend", "Falling Boxes: Twist",
            "Falling Boxes: Displace", "Pile of Chairs", "Spiky Wheel", "Cows on Spikes", "Bunny Pile", "Primitives",
        ])
    }

    @Test func boxStackStaysStacked() {
        let world = SDFWorld()
        world.loadScene(0)
        let before = world.bodies()
        for _ in 0..<180 { world.step() }
        let after = world.bodies()

        #expect(after.count == before.count)
        for (a, b) in zip(before, after) where !a.isStatic {
            #expect(distance(a.origin, b.origin) < 0.05, "a box of the resting pyramid moved")
        }
    }

    @Test func meshShapesBakeAndRestOnTheGround() {
        let world = SDFWorld()
        world.loadScene(9) // Bunny Pile
        for _ in 0..<300 { world.step() }

        let dynamic = world.bodies().filter { !$0.isStatic }
        #expect(!dynamic.isEmpty)
        // Upstream falls back to a box when an asset fails to load
        #expect(dynamic.allSatisfy { world.shapeName($0.shape) == "Bunny" })
        #expect(dynamic.allSatisfy { $0.origin.y > -0.2 && $0.origin.y < 10 })
        #expect(world.stats.contacts > 0)
        let mesh = world.mesh(forShape: dynamic[0].shape)
        #expect(mesh.indices.count > 0 && mesh.indices.count % 3 == 0)
    }

    @Test func simulationIsDeterministic() {
        func run() -> [SDFWorld.Body] {
            let world = SDFWorld()
            world.loadScene(2) // Falling Boxes
            for _ in 0..<120 { world.step() }
            return world.bodies()
        }
        #expect(run() == run())
    }

    @Test func parametersRoundTrip() {
        let world = SDFWorld()
        var parameters = world.parameters
        #expect(parameters.cacheSize == 4 && parameters.randomSamples == 1 && parameters.substeps == 4)
        parameters.cacheSize = 6
        parameters.optimizer = 1
        world.parameters = parameters
        #expect(world.parameters.cacheSize == 6 && world.parameters.optimizer == 1)
    }
}
