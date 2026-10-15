import RealityKit
import RealitySDFPhysics
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
        root.components.set(SDFPhysicsSimulationComponent(simulation: simulation))
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
        box.components.set(SDFPhysicsBodyComponent(shape: .box(halfExtents: [0.5, 0.5, 0.5])))
        root.addChild(box)
        for _ in 0..<120 { simulation.step(root: root) }
        #expect(box.position.y > 0.5 && box.position.y < 0.54, "rests at \(box.position.y)")
    }

    @Test func lookalikesAreBakedNotRecognized() async throws {
        // An octahedron has every vertex at one distance from its center, but isn't a sphere
        let octahedron = try await SDFShape.make(from: .generateConvex(from: [
            [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1],
        ]))
        #expect(octahedron.shape.distance(at: [0.45, 0.45, 0.45]).distance > 0)

        // Two opposite faces of a box are not a box
        let faces = try? await SDFShape.make(from: .generateStaticMesh(
            positions: [[0, 0, 5], [1, 0, 5], [1, 1, 5], [0, 1, 5], [0, 0, 6], [1, 0, 6], [1, 1, 6], [0, 1, 6]],
            faceIndices: [0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7]))
        #expect(faces?.transform.translation == nil || faces?.transform == Transform())
    }

    @Test func aScaleThatIsNotUniformGoesIntoTheShapes() async throws {
        let root = Entity(), simulation = SDFSimulation()
        root.components.set(SDFPhysicsSimulationComponent(simulation: simulation))
        let floor = Entity()
        floor.transform = Transform(scale: [10, 0.1, 10], translation: [0, -0.05, 0])
        floor.components.set(CollisionComponent(shapes: [.generateBox(size: [1, 1, 1])]))
        root.addChild(floor)
        let colliders = try await floor.addSDFColliders()
        #expect(simd_distance(colliders[0].scale(relativeTo: root), .one) < 1e-5)

        let box = Entity()
        box.position = [4, 1, 0]
        box.components.set(SDFPhysicsBodyComponent(shape: .box(halfExtents: [0.5, 0.5, 0.5])))
        root.addChild(box)
        for _ in 0..<120 { simulation.step(root: root) }
        #expect(box.position.y > 0.5 && box.position.y < 0.54, "rests at \(box.position.y)")
    }

    @Test func importsFollowTheCollisionComponent() async throws {
        let entity = Entity()
        entity.components.set(CollisionComponent(shapes: [.generateSphere(radius: 1)], mode: .trigger))
        let other = Entity()
        other.name = "SDFCollider"
        entity.addChild(other)

        let colliders = try await entity.addSDFColliders()
        #expect(colliders[0].components[SDFPhysicsBodyComponent.self]?.isTrigger == true)
        #expect(entity.children.count == 2)

        // Without collision shapes, the bodies go, and children it didn't add stay
        entity.components.remove(CollisionComponent.self)
        #expect(try await entity.addSDFColliders().isEmpty)
        #expect(Array(entity.children) == [other])
    }

    @Test func aLaterImportCancelsAnEarlierOne() async throws {
        let entity = Entity()
        let tetrahedron = try await ShapeResource.generateConvex(from: [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
        entity.components.set(CollisionComponent(shapes: [tetrahedron]))
        func importColliders() async -> Error? {
            do { try await entity.addSDFColliders(); return nil } catch { return error }
        }
        async let first = importColliders()
        async let second = importColliders()
        let errors = await [first, second]
        #expect(errors.count { $0 == nil } == 1)
        #expect(errors.contains { $0 is CancellationError })
        #expect(entity.children.count == 1)
    }
}
