import Foundation
import RealitySDFPhysics
import Testing
import simd

@Suite struct SDFNodeWindingTests {
    func box(_ half: SIMD3<Float>) -> (positions: [SIMD3<Float>], indices: [UInt32]) {
        let mesh = SDFShape.box(halfExtents: half).surface
        return (mesh.positions,mesh.indices)
    }

    @Test func actualNodeWindingPreservesTheThinBoxInterior() throws {
        let mesh = box([6.2,0.1,2])
        let legacy = try SDFShape(baking: mesh.positions,indices: mesh.indices,resolution: 64)
        let corrected = try SDFShape(baking: mesh.positions,indices: mesh.indices,resolution: 64,signMethod: .nodeWinding)
        // A grid node really is inside the20cm box, although jittered vote locations cross its cap.
        #expect(legacy.distance(at: .zero).distance > 0)
        for y in [Float(0),0.09,-0.09] {
            #expect(corrected.distance(at: [0,y,0]).distance < 0)
        }
        #expect(corrected.distance(at: [0,0.15,0]).distance > 0)
        #expect(corrected.distance(at: [0,0.09,0]).gradient.y > 0.9)
        #expect(simd_normalize(corrected.distance(at: [0,-0.09,0]).gradient).y < -0.99)
    }

    /// Upstream grid nodes lie at the bounds' minimum plus whole cells, so an axis-aligned slab has
    /// an interior node only when its thickness exceeds the cell. A 0.2 m slab at 64 cells across
    /// its longest side keeps an interior up to a 12.8 m span and loses it beyond.
    @Test func thinSlabNeedsAnInteriorNodeAndReportsWhenItHasNone() throws {
        for (span, expectInterior) in [(Float(12.6), true), (13.0, false), (20.0, false)] {
            let mesh = box([span/2,0.1,span/3])
            let shape = try SDFShape(baking: mesh.positions,indices: mesh.indices,resolution: 64,signMethod: .nodeWinding)
            let grid = try #require(shape.bakedGridInfo)
            #expect(abs(grid.cellSize - span/64) < 1e-4)
            #expect((grid.cellSize < 0.2) == expectInterior)
            #expect((grid.interiorSampleCount > 0) == expectInterior, "span \(span)")
            #expect((shape.distance(at: .zero).distance < 0) == expectInterior, "span \(span)")
        }
        // The documented resolution rule restores an interior for any slab orientation.
        let mesh = box([10,0.1,20/3])
        let resolution = try #require(SDFShape.minimumResolution(forThickness: 0.2, longestExtent: 20))
        #expect(resolution == 200)
        #expect(SDFShape.minimumResolution(forThickness: 0.2, longestExtent: 0.1) == 8)
        #expect(SDFShape.minimumResolution(forThickness: 0, longestExtent: 20) == nil)
        #expect(SDFShape.minimumResolution(forThickness: .nan, longestExtent: 20) == nil)
        let shape = try SDFShape(baking: mesh.positions,indices: mesh.indices,resolution: resolution,signMethod: .nodeWinding)
        #expect(try #require(shape.bakedGridInfo).interiorSampleCount > 0)
        #expect(shape.distance(at: .zero).distance < 0)
    }

    @Test func cubeDistancesAndOutwardGradientsRemainConsistent() throws {
        let mesh = box([1,1,1])
        let shape = try SDFShape(baking: mesh.positions,indices: mesh.indices,resolution: 24,signMethod: .nodeWinding)
        #expect(abs(shape.distance(at: .zero).distance + 1) < 0.02)
        for axis in 0..<3 {
            for sign in [Float(-1),1] {
                var inside = SIMD3<Float>.zero, outside = inside
                inside[axis] = sign*0.8
                outside[axis] = sign*1.2
                #expect(shape.distance(at: inside).distance < 0)
                #expect(shape.distance(at: outside).distance > 0)
                #expect(shape.distance(at: inside).gradient[axis]*sign > 0.9)
                #expect(shape.distance(at: outside).gradient[axis]*sign > 0.9)
            }
        }
    }

    @Test func oppositelyWoundInnerShellRemainsACavity() throws {
        let outer = box([1,1,1]), inner = box([0.5,0.5,0.5])
        let positions = outer.positions+inner.positions
        let offset = UInt32(outer.positions.count)
        let reversed = stride(from: 0,to: inner.indices.count,by: 3).flatMap {
            [inner.indices[$0+2]+offset,inner.indices[$0+1]+offset,inner.indices[$0]+offset]
        }
        let shape = try SDFShape(baking: positions,indices: outer.indices+reversed,resolution: 24,signMethod: .nodeWinding)
        #expect(shape.distance(at: .zero).distance > 0)
        #expect(shape.distance(at: [0.7,0,0]).distance < 0)
        #expect(shape.distance(at: [1.2,0,0]).distance > 0)
        #expect(shape.distance(at: [0.55,0,0]).gradient.x < -0.8)
    }

    @Test func nodeWindingRejectsMalformedTopologyAndResolution() throws {
        let mesh = box([1,1,1])
        #expect(throws: SDFShape.NodeWindingBakeError.invalidMeshTopology) {
            try SDFShape(baking: mesh.positions,indices: Array(mesh.indices.dropLast(3)),resolution: 16,signMethod: .nodeWinding)
        }
        var inconsistent = mesh.indices
        inconsistent.swapAt(0,1)
        #expect(throws: SDFShape.NodeWindingBakeError.invalidMeshTopology) {
            try SDFShape(baking: mesh.positions,indices: inconsistent,resolution: 16,signMethod: .nodeWinding)
        }
        #expect(throws: SDFShape.BakeError.invalidMesh) {
            try SDFShape(baking: mesh.positions,indices: mesh.indices,resolution: .max,signMethod: .nodeWinding)
        }
        #expect(throws: SDFShape.BakeError.invalidMesh) {
            try SDFShape(baking: mesh.positions,indices: mesh.indices,resolution: Int(Int32.max),signMethod: .nodeWinding)
        }
        let huge = mesh.positions.map { $0 * 1e37 + SIMD3<Float>(repeating: 2e38) }
        #expect(throws: SDFShape.BakeError.invalidMesh) {
            try SDFShape(baking: huge,indices: mesh.indices,resolution: 16,signMethod: .nodeWinding)
        }
        var nonFinite = mesh.positions
        nonFinite[0].x = .infinity
        #expect(throws: SDFShape.NodeWindingBakeError.invalidMeshTopology) {
            try SDFShape(baking: nonFinite,indices: mesh.indices,resolution: 16,signMethod: .nodeWinding)
        }
    }

    @Test func legacyDefaultAndCacheKeysRemainUnchanged() throws {
        let mesh = box([1,0.5,0.3])
        let old = try SDFShape(baking: mesh.positions,indices: mesh.indices,resolution: 16)
        let explicit = try SDFShape(baking: mesh.positions,indices: mesh.indices,resolution: 16,signMethod: .legacy)
        #expect(old.bakedData() == explicit.bakedData())
        let key = SDFShape.bakeKey(positions: mesh.positions,indices: mesh.indices,resolution: 16)
        #expect(key == SDFShape.bakeKey(positions: mesh.positions,indices: mesh.indices,resolution: 16,signMethod: .legacy))
        #expect(key != SDFShape.bakeKey(positions: mesh.positions,indices: mesh.indices,resolution: 16,signMethod: .nodeWinding))
    }

    @Test func correctedDataRoundTripsAndCacheDoesNotLoadLegacyEntry() throws {
        let mesh = box([6.2,0.1,2])
        let directory = FileManager.default.temporaryDirectory.appending(path: "SDFNodeWindingTests-\(UUID().uuidString)")
        defer { try? FileManager.default.removeItem(at: directory) }
        let legacy = try SDFShape.cached(baking: mesh.positions,indices: mesh.indices,resolution: 64,in: directory)
        let corrected = try SDFShape.cached(baking: mesh.positions,indices: mesh.indices,resolution: 64,signMethod: .nodeWinding,in: directory)
        let loaded = try SDFShape.cached(baking: mesh.positions,indices: mesh.indices,resolution: 64,signMethod: .nodeWinding,in: directory)
        #expect(legacy.distance(at: .zero).distance > 0)
        #expect(corrected.distance(at: .zero).distance < 0)
        #expect(loaded.bakedData() == corrected.bakedData())
        #expect(try FileManager.default.contentsOfDirectory(atPath: directory.path).count == 2)
        let reloaded = try SDFShape(bakedData: #require(corrected.bakedData()))
        for y in [Float(-0.2),-0.09,0,0.09,0.2] {
            let a = corrected.distance(at: [0,y,0]), b = reloaded.distance(at: [0,y,0])
            #expect(a.distance.bitPattern == b.distance.bitPattern && a.gradient == b.gradient)
        }
        #expect(reloaded.volume == corrected.volume)
    }
}
