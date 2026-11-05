import RealitySDFPhysics
import Testing
import simd

@Suite struct SDFMeshValidationTests {
    let positions: [SIMD3<Float>] = [[0,0,0], [1,0,0], [0,1,0], [0,0,1]]
    let indices: [UInt32] = [0,2,1, 0,1,3, 0,3,2, 1,2,3]

    @Test func closedConnectivityDoesNotGuaranteeConsistentWinding() {
        let valid = SDFShape.validateMesh(positions: positions, indices: indices)
        #expect(valid.isClosed)
        #expect(valid.isConsistentlyOriented)
        var reversedFace = indices
        reversedFace.swapAt(0, 1)
        let inconsistent = SDFShape.validateMesh(positions: positions, indices: reversedFace)
        #expect(inconsistent.isClosed)
        #expect(!inconsistent.isConsistentlyOriented)
        #expect(inconsistent.inconsistentlyOrientedEdgeCount == 3)
    }

    @Test func globallyReversedShellIsStillConsistentlyOriented() {
        let reversed = stride(from: 0, to: indices.count, by: 3).flatMap { [indices[$0 + 2], indices[$0 + 1], indices[$0]] }
        let report = SDFShape.validateMesh(positions: positions, indices: reversed)
        #expect(report.isClosed)
        #expect(report.isConsistentlyOriented)
    }

    @Test func duplicatedSeamVerticesAreWeldedLikeTheBaker() {
        let duplicated = indices.map { positions[Int($0)] }
        let report = SDFShape.validateMesh(positions: duplicated, indices: duplicated.indices.map(UInt32.init))
        #expect(report.weldedVertexCount == 4)
        #expect(report.isClosed)
        #expect(report.isConsistentlyOriented)
    }

    @Test func seamWeldingUsesTheBakersMinimumOriginAndHalfAwayRounding() {
        var seam = indices.map { positions[Int($0)] }
        let sequential = seam.indices.map(UInt32.init)
        // Other vertices retain a zero minimum and unit extent, hence a 1e-5 cell.
        seam[0].x = 0.49e-5
        let sameBin = SDFShape.validateMesh(positions: seam, indices: sequential)
        #expect(sameBin.weldedVertexCount == 4)
        #expect(sameBin.isClosed)
        seam[0].x = 0.51e-5
        let otherBin = SDFShape.validateMesh(positions: seam, indices: sequential)
        #expect(otherBin.weldedVertexCount == 5)
        #expect(!otherBin.isClosed)
    }

    @Test func reportsOpenAndNonManifoldEdgesSeparately() {
        let open = SDFShape.validateMesh(positions: positions, indices: Array(indices.prefix(3)))
        #expect(open.boundaryEdgeCount == 3)
        #expect(!open.isClosed)
        #expect(open.isConsistentlyOriented)
        let nonManifold = SDFShape.validateMesh(positions: positions, indices: indices + indices.prefix(3))
        #expect(nonManifold.nonManifoldEdgeCount == 3)
        #expect(!nonManifold.isClosed)
        #expect(!nonManifold.isConsistentlyOriented)
    }

    /// A zero-area triangle encloses nothing, and the bakers leave it out, so it must not make an
    /// otherwise closed mesh fail validation. It is still counted.
    @Test func degenerateTrianglesAreCountedButDoNotOpenAClosedMesh() throws {
        let mesh = SDFShape.box(halfExtents: [1, 1, 1]).surface
        // Corners that weld together, and three distinct collinear corners on the top face.
        let positions = mesh.positions + [[-0.37, 1, 0.13], [0.01, 1, 0.13], [0.41, 1, 0.13]]
        let n = UInt32(mesh.positions.count)
        for extra: [UInt32] in [[mesh.indices[0], mesh.indices[0], mesh.indices[1]], [n, n + 1, n + 2]] {
            let indices = mesh.indices + extra
            let report = SDFShape.validateMesh(positions: positions, indices: indices)
            #expect(report.degenerateTriangleCount == 1)
            #expect(report.isClosed && report.isConsistentlyOriented)
            // Node winding accepts what validation accepts.
            let shape = try SDFShape(baking: positions, indices: indices, resolution: 16, signMethod: .nodeWinding)
            #expect(abs(shape.distance(at: .zero).distance + 1) < 0.05)
            #expect(shape.distance(at: [0, 1.2, 0]).distance > 0)
        }
    }

    @Test func malformedInputReturnsDiagnosticsWithoutTrapping() {
        let invalid = SDFShape.validateMesh(positions: positions, indices: [0,1,99,0,1])
        #expect(invalid.invalidIndexCount == 1)
        #expect(invalid.incompleteTriangleIndexCount == 2)
        #expect(!invalid.isClosed)
        let degenerate = SDFShape.validateMesh(positions: positions, indices: [0,1,1])
        #expect(degenerate.degenerateTriangleCount == 1)
        #expect(!degenerate.isConsistentlyOriented)
        let collinear = SDFShape.validateMesh(positions: [[0,0,0], [1,0,0], [2,0,0]], indices: [0,1,2])
        #expect(collinear.degenerateTriangleCount == 1)
        let empty = SDFShape.validateMesh(positions: [], indices: [])
        #expect(!empty.isClosed && !empty.isConsistentlyOriented)
        let nonFinite = SDFShape.validateMesh(positions: [[.nan,0,0], [0,1,0], [0,0,1]], indices: [0,1,2])
        #expect(nonFinite.nonFiniteVertexCount == 1)
        #expect(!nonFinite.hasFiniteNonzeroExtent)
        let overflowing = SDFShape.validateMesh(positions: [[-.greatestFiniteMagnitude,0,0], [.greatestFiniteMagnitude,0,0], [0,1,0]], indices: [0,1,2])
        #expect(!overflowing.hasFiniteNonzeroExtent)
        #expect(!overflowing.isConsistentlyOriented)
    }
}
