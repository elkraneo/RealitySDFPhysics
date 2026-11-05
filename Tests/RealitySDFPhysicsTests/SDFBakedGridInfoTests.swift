import RealitySDFPhysics
import SDFCore
import Testing
import simd

@Suite struct SDFBakedGridInfoTests {
    private func bake(halfExtents: SIMD3<Float>, resolution: Int = 64,
                      center: SIMD3<Float> = .zero) throws -> SDFShape {
        let mesh = SDFShape.box(halfExtents: halfExtents).surface
        return try SDFShape(baking: mesh.positions.map { $0 + center }, indices: mesh.indices, resolution: resolution)
    }

    @Test func analyticalAndInvalidHandlesHaveNoBakedGrid() {
        #expect(SDFShape.box(halfExtents: [5, 1, 0.1]).bakedGridInfo == nil)
        #expect(SDFShape.torus(majorRadius: 1, minorRadius: 0.2).bakedGridInfo == nil)
        var info = SDFBakedGridInfo()
        #expect(!sdf_shape_baked_grid_info(nil, &info))
    }

    @Test func thinGridIsCroppedAndCountsPaddingSamples() throws {
        // Original dimensions 10 x 2 x 0.2; normalized to 1 x 0.2 x 0.02.
        // ceil(extent * 64) + one boundary sample + six padding samples per axis.
        let thin = try #require(bake(halfExtents: [5, 1, 0.1]).bakedGridInfo)
        #expect(thin.sampleDimensions == [71, 20, 9])
        #expect(thin.sampleCount == 71 * 20 * 9)
        #expect(thin.gridPayloadBytes == thin.sampleCount * 16)
        #expect(abs(thin.cellSize - 10.0 / 64) < 1e-6)
        let cube = try #require(bake(halfExtents: [5, 5, 5]).bakedGridInfo)
        #expect(cube.sampleDimensions == [71, 71, 71])
        #expect(thin.gridPayloadBytes < cube.gridPayloadBytes / 10)
    }

    @Test func sourceScaleChangesSpacingButNotNormalizedGridDimensions() throws {
        let original = try #require(bake(halfExtents: [5, 1, 0.1]).bakedGridInfo)
        let scaled = try #require(bake(halfExtents: [15, 3, 0.3], center: [5, -3, 100]).bakedGridInfo)
        #expect(scaled.sampleDimensions == original.sampleDimensions)
        #expect(scaled.sampleCount == original.sampleCount)
        #expect(abs(scaled.cellSize - original.cellSize * 3) < 1e-6)
    }

    @Test func cacheRoundTripPreservesGridInformation() throws {
        let shape = try bake(halfExtents: [5, 1, 0.1], resolution: 48, center: [2, 4, -3])
        let data = try #require(shape.bakedData())
        let restored = try SDFShape(bakedData: data)
        #expect(try #require(restored.bakedGridInfo) == #require(shape.bakedGridInfo))
    }
}
