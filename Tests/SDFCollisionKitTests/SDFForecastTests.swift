import RealityKit
import SDFCollisionKit
import Testing
import simd

@MainActor
@Suite struct SDFForecastTests {
    @Test(arguments: [false, true])
    func forecastsMatchTheSimulation(isMultithreaded: Bool) {
        let root = Entity(), simulation = SDFSimulation()
        var settings = SDFSimulationComponent(simulation: simulation)
        settings.isMultithreaded = isMultithreaded
        root.components.set(settings)
        let ground = Entity()
        ground.position = [0, -0.5, 0]
        ground.components.set(SDFBodyComponent(shape: .box(halfExtents: [5, 0.5, 5]), mode: .static))
        root.addChild(ground)
        let bodies = (0..<5).map { i in
            let entity = Entity()
            entity.position = [Float(i % 2) * 0.3, 0.6 + Float(i) * 0.9, 0]
            entity.orientation = simd_quatf(angle: Float(i), axis: normalize([1, 1, 0]))
            entity.components.set(SDFBodyComponent(shape: .torus(majorRadius: 0.4, minorRadius: 0.15)))
            root.addChild(entity)
            return entity
        }
        // Mid-pile, with warm contact caches, and a throw set right before the forecast
        for _ in 0..<60 { simulation.step(root: root) }
        simulation.setMotion(SDFMotionComponent(linearVelocity: [2, 3, 0], angularVelocity: [0, 0, 5]), of: bodies[4])
        let forecast = simulation.forecast()
        forecast.step(90)
        for _ in 0..<90 { simulation.step(root: root) }
        for body in bodies {
            let predicted = forecast.transform(of: body)!
            // Positions bit for bit; rotations as rotations, since the entity stores its own quaternion
            #expect(predicted.translation == body.position)
            #expect(abs(dot(predicted.rotation.vector, body.orientation.vector)) > 1 - 1e-6)
        }
        #expect(forecast.stepCount == 90)
    }
}
