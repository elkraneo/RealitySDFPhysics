import RealityKit
import SDFCollisionKit
import Testing
import simd

@MainActor
@Suite struct SDFSimulationTests {
    let root = Entity()
    let simulation = SDFSimulation()

    init() {
        root.components.set(SDFSimulationComponent(simulation: simulation))
        // Ground with its top face at y = 0
        addBody(.box(halfExtents: [5, 0.5, 5]), mode: .static, at: [0, -0.5, 0])
    }

    @discardableResult
    func addBody(_ shape: SDFShape, mode: PhysicsBodyMode = .dynamic, at position: SIMD3<Float>,
                 parent: Entity? = nil) -> Entity {
        let entity = Entity()
        entity.position = position
        entity.components.set(SDFBodyComponent(shape: shape, mode: mode))
        (parent ?? root).addChild(entity)
        return entity
    }

    func run(_ steps: Int) {
        for _ in 0..<steps { simulation.step(root: root) }
    }

    /// Resting bodies stay a skin apart, between half the contact margin and the margin
    func expectResting(_ entity: Entity, atHeight height: Float, sourceLocation: SourceLocation = #_sourceLocation) {
        let y = entity.position(relativeTo: root).y
        #expect(y > height && y < height + 0.04, "rests at \(y), expected about \(height)", sourceLocation: sourceLocation)
    }

    @Test func dynamicBoxFallsAndRests() {
        let box = addBody(.box(halfExtents: [0.5, 0.5, 0.5]), at: [0, 2, 0])
        run(180)
        expectResting(box, atHeight: 0.5)
        #expect(simulation.statistics.contacts > 0)
    }

    @Test func zeroRotationIsIdentity() {
        // simd_quatf() is the zero quaternion, which RealityKit renders as no rotation
        let slab = addBody(.box(halfExtents: [2, 0.5, 2]), mode: .static, at: [0, 0.5, 0])
        slab.transform.rotation = simd_quatf()
        let box = addBody(.box(halfExtents: [0.5, 0.5, 0.5]), at: [0, 3, 0])
        box.transform.rotation = simd_quatf()
        run(180)
        expectResting(box, atHeight: 1.5)
        #expect(simulation.statistics.contacts > 0)
    }

    @Test func bakedShapeRestsInTheEntityFrame() throws {
        // The mesh sits 2 units off the entity's origin; the entity itself must not drift
        let shape = try SDFShape.bakedBox(halfExtents: [0.5, 0.5, 0.5], center: [2, 0, 0])
        let box = addBody(shape, at: [-2, 2, 0])
        run(180)
        expectResting(box, atHeight: 0.5)
        #expect(abs(box.position(relativeTo: root).x + 2) < 0.05)
    }

    @Test func scaledEntityScalesItsBody() throws {
        let box = addBody(try .bakedBox(halfExtents: [0.5, 0.5, 0.5]), at: [0, 3, 0])
        box.scale = [2, 2, 2]
        run(180)
        expectResting(box, atHeight: 1)
    }

    @Test func bodiesFollowTheRootFrame() {
        // Bodies live in the root's frame: a scaled parent root does not change the simulation
        let scene = Entity()
        scene.scale = [0.1, 0.1, 0.1]
        scene.addChild(root)
        let box = addBody(.box(halfExtents: [0.5, 0.5, 0.5]), at: [0, 2, 0])
        run(180)
        expectResting(box, atHeight: 0.5)
    }

    @Test func removingABodyKeepsTheOthersSimulating() {
        let box: SDFShape = .box(halfExtents: [0.5, 0.5, 0.5])
        let stack = (0..<3).map { addBody(box, at: [0, 0.52 + Float($0) * 1.02, 0]) }
        run(60)
        stack[1].removeFromParent()
        run(120)
        expectResting(stack[0], atHeight: 0.5)
        expectResting(stack[2], atHeight: 1.5)
        let ids = Set(stack.map(\.id))
        #expect(simulation.contacts().allSatisfy { ids.contains($0.entityA) || ids.contains($0.entityB) })
        #expect(!simulation.contacts().contains { $0.entityA == stack[1].id || $0.entityB == stack[1].id })
    }

    @Test func kinematicPlatformCarriesABody() {
        let platform = addBody(.box(halfExtents: [1, 0.1, 1]), mode: .kinematic, at: [0, 1, 0])
        let box = addBody(.box(halfExtents: [0.25, 0.25, 0.25]), at: [0, 1.4, 0])
        box.components.set(SDFMotionComponent())
        run(60)
        for _ in 0..<60 {
            platform.position.x += 1.0 / 60 // 1 unit per second
            simulation.update(root: root, deltaTime: 1.0 / 60)
        }
        let velocity = box.components[SDFMotionComponent.self]!.linearVelocity
        #expect(velocity.x > 0.5, "friction should carry the box, velocity \(velocity)")
        #expect(box.position(relativeTo: root).x > 0.5)
    }

    @Test func movingADynamicEntityTeleportsItsBody() {
        let box = addBody(.box(halfExtents: [0.5, 0.5, 0.5]), at: [0, 0.52, 0])
        run(30)
        box.position = [3, 2, 0]
        run(180)
        expectResting(box, atHeight: 0.5)
        #expect(abs(box.position.x - 3) < 0.05)
    }

    @Test func motionComponentSetsVelocity() {
        let ball = addBody(.sphere(radius: 0.5), at: [0, 3, 0])
        ball.components.set(SDFMotionComponent(linearVelocity: [2, 0, 0]))
        run(30) // Half a second, before landing
        #expect(abs(ball.position.x - 1) < 0.05)
        #expect(abs(ball.components[SDFMotionComponent.self]!.linearVelocity.x - 2) < 0.05)
    }

    @Test func changingTheShapeRecreatesTheBody() {
        let body = addBody(.box(halfExtents: [0.5, 0.5, 0.5]), at: [0, 0.52, 0])
        run(30)
        body.components[SDFBodyComponent.self]!.shape = .box(halfExtents: [1, 1, 1])
        run(180)
        expectResting(body, atHeight: 1)
        #expect(simulation.statistics.bodies == 2)
    }

    @Test func massDefaultsToWaterAndCanBeSet() {
        let box = addBody(.box(halfExtents: [0.5, 0.5, 0.5]), at: [0, 2, 0])
        run(1)
        #expect(abs(simulation.mass(of: box)! - 1000) < 30) // Integrated over a voxelization
        box.components[SDFBodyComponent.self]!.massProperties = .init(mass: 5)
        run(1)
        #expect(simulation.mass(of: box) == 5)
    }

    @Test func presetsSetSubstepsAndIterations() {
        let realTime = SDFSimulationComponent()
        #expect(realTime.substeps == 2)
        #expect(realTime.solverIterations.velocityIterations == 10)
        #expect(realTime.solverIterations.positionIterations == 4)
        let paper = SDFSimulationComponent(preset: .paper)
        #expect(paper.substeps == 4)
        #expect(paper.solverIterations.velocityIterations == 20)
        #expect(paper.solverIterations.positionIterations == 20)
    }

    @Test(arguments: [false, true])
    func simulationIsDeterministic(isMultithreaded: Bool) {
        func pile() -> [SIMD3<Float>] {
            let root = Entity(), simulation = SDFSimulation()
            var settings = SDFSimulationComponent(simulation: simulation)
            settings.isMultithreaded = isMultithreaded
            root.components.set(settings)
            let ground = Entity()
            ground.position = [0, -0.5, 0]
            ground.components.set(SDFBodyComponent(shape: .box(halfExtents: [5, 0.5, 5]), mode: .static))
            root.addChild(ground)
            let bodies = (0..<6).map { i in
                let entity = Entity()
                entity.position = [Float(i % 2) * 0.3, 1 + Float(i) * 1.1, 0]
                entity.orientation = simd_quatf(angle: Float(i), axis: normalize([1, 1, 0]))
                entity.components.set(SDFBodyComponent(shape: .torus(majorRadius: 0.5, minorRadius: 0.2)))
                root.addChild(entity)
                return entity
            }
            for _ in 0..<120 { simulation.step(root: root) }
            return bodies.map(\.position)
        }
        #expect(pile() == pile())
    }
}
