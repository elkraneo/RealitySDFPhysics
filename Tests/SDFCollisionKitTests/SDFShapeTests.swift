import RealityKit
import SDFCollisionKit
import Testing
import simd

@Suite struct SDFShapeTests {
    @Test func analyticalDistances() {
        let box = SDFShape.box(halfExtents: [1, 0.5, 0.5])
        #expect(abs(box.distance(at: [3, 0, 0]).distance - 2) < 1e-5)
        #expect(abs(box.distance(at: .zero).distance + 0.5) < 1e-5)
        #expect(box.distance(at: [3, 0, 0]).gradient == [1, 0, 0])
    }

    @Test func bakedShapeKeepsTheMeshFrame() throws {
        // A 2 x 1 x 1 box mesh centered away from the origin: baking normalizes it internally
        let center: SIMD3<Float> = [5, 1, -2]
        let shape = try SDFShape.bakedBox(halfExtents: [1, 0.5, 0.5], center: center)

        #expect(shape.isWatertight)
        #expect(abs(shape.distance(at: center).distance + 0.5) < 0.02)
        #expect(abs(shape.distance(at: center + [3, 0, 0]).distance - 2) < 0.02)
        let bounds = shape.surface.positions.reduce(into: (min: center, max: center)) {
            $0.min = simd_min($0.min, $1)
            $0.max = simd_max($0.max, $1)
        }
        #expect(simd_distance(bounds.min, center - [1, 0.5, 0.5]) < 0.02)
        #expect(simd_distance(bounds.max, center + [1, 0.5, 0.5]) < 0.02)
    }

    @MainActor
    @Test func bakesTheMeshesOfAnEntity() async throws {
        // RealityKit's box splits its vertices at the edges; baking welds them
        let parent = Entity()
        let model = ModelEntity(mesh: .generateBox(size: 1))
        model.position = [3, 0, 0]
        parent.addChild(model)
        let shape = try await SDFShape.bake(from: parent, resolution: 64)

        #expect(shape.isWatertight)
        #expect(abs(shape.volume - 1) < 0.05)
        #expect(abs(shape.distance(at: [3, 0, 0]).distance + 0.5) < 0.02)
    }

    @Test func bakingRejectsInvalidMeshes() {
        #expect(throws: SDFShape.BakeError.invalidMesh) { try SDFShape(baking: [], indices: []) }
        #expect(throws: SDFShape.BakeError.invalidMesh) { try SDFShape(baking: [.zero, .one, [1, 0, 0]], indices: [0, 1, 7]) }
    }

    @Test func bakedConvexMeshesAreConvex() throws {
        #expect(SDFShape.box(halfExtents: [1, 1, 1]).isConvex)
        #expect(!SDFShape.torus(majorRadius: 1, minorRadius: 0.3).isConvex)
        #expect(try SDFShape.bakedBox(halfExtents: [0.5, 0.3, 0.2]).isConvex)
        let torus = SDFShape.torus(majorRadius: 1, minorRadius: 0.3).surface
        #expect(try !SDFShape(baking: torus.positions, indices: torus.indices, resolution: 48).isConvex)
    }
}

extension SDFShape {
    /// Bakes the surface mesh of an analytical box, moved to a center
    static func bakedBox(halfExtents: SIMD3<Float>, center: SIMD3<Float> = .zero) throws -> SDFShape {
        let surface = SDFShape.box(halfExtents: halfExtents).surface
        return try SDFShape(baking: surface.positions.map { $0 + center }, indices: surface.indices, resolution: 64)
    }
}
