import Foundation
import RealityKit
import SDFCollisionKit
import Testing

@MainActor
@Suite struct SDFSimulationTests {
    @Test func rebuildsWhenTheWorldLoadsAnotherSceneDirectly() {
        let simulation = SDFSimulation()
        simulation.loadScene(0) // Box Stack, 29 bodies
        simulation.step()
        simulation.world.loadScene(1) // Gear Stack, 11 bodies, bypassing the simulation
        simulation.synchronize()

        // Stale entities of the larger scene would remain if the mirror only grew
        let bodyEntities = simulation.root.children[0].children
        #expect(bodyEntities.count == simulation.world.bodyCount)
        #expect(simulation.stepCount == 0)
    }

    @Test func keepsTheFractionalStepWhenCappedPerFrame() {
        let simulation = SDFSimulation()
        simulation.loadScene(0)
        simulation.maximumStepsPerFrame = 1
        let timeStep = TimeInterval(simulation.world.parameters.timeStep)

        simulation.advance(by: 1.5 * timeStep) // One step, half a step left over
        simulation.advance(by: 0.6 * timeStep) // The leftover completes another step
        #expect(simulation.stepCount == 2)
    }
}
