import RealityKit
import SDFCollisionKit
import Testing
import simd

@MainActor
@Suite struct SDFColliderTests {
    @Test func boxesSpheresAndCapsulesBecomeAnalyticalShapes() async throws {
        // Analytical shapes are exact; a baked box is off by about a voxel
        let box = try await SDFShape.make(from: .generateBox(size: [1, 2, 3]).offsetBy(translation: [5, 0, 0]))
        #expect(simd_distance(box.transform.translation, [5, 0, 0]) < 1e-5)
        #expect(abs(box.shape.distance(at: .zero).distance + 0.5) < 1e-5)
        #expect(abs(box.shape.distance(at: [0, 0, 1.4]).distance + 0.1) < 1e-5)

        let sphere = try await SDFShape.make(from: .generateSphere(radius: 0.5))
        #expect(abs(sphere.shape.distance(at: .zero).distance + 0.5) < 1e-3)

        // A capsule 2 long along x
        let capsule = try await SDFShape.make(from: .generateCapsule(height: 2, radius: 0.5)
            .offsetBy(rotation: simd_quatf(angle: .pi / 2, axis: [0, 0, 1])))
        #expect(abs(abs(capsule.transform.rotation.act([0, 1, 0]).x) - 1) < 1e-4)
        #expect(abs(capsule.shape.distance(at: [0, 0.9, 0]).distance + 0.1) < 1e-3)
    }

    @Test func otherShapesAreBaked() async throws {
        let tetrahedron = try await SDFShape.make(from: .generateConvex(from: [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]]))
        #expect(tetrahedron.transform == Transform())
        #expect(tetrahedron.shape.isWatertight && tetrahedron.shape.isConvex)
        #expect(tetrahedron.shape.distance(at: [0.1, 0.1, 0.1]).distance < 0)
        #expect(tetrahedron.shape.distance(at: [1, 1, 1]).distance > 0)
    }

    @Test func aBodyRestsOnARealityKitFloor() async throws {
        let root = Entity(), simulation = SDFSimulation()
        root.components.set(SDFSimulationComponent(simulation: simulation))
        let floor = Entity()
        floor.position = [0, -0.05, 0]
        floor.components.set(CollisionComponent(shapes: [.generateBox(size: [10, 0.1, 10])]))
        floor.components.set(PhysicsBodyComponent(mode: .static))
        root.addChild(floor)
        let colliders = try await floor.addSDFColliders()
        #expect(colliders.count == 1)
        #expect(try await floor.addSDFColliders().count == 1)
        #expect(floor.children.count == 1)

        let box = Entity()
        box.position = [0, 1, 0]
        box.components.set(SDFBodyComponent(shape: .box(halfExtents: [0.5, 0.5, 0.5])))
        root.addChild(box)
        for _ in 0..<120 { simulation.step(root: root) }
        #expect(box.position.y > 0.5 && box.position.y < 0.54, "rests at \(box.position.y)")
    }
}
