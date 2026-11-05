import simd

/// Topological checks on the triangle mesh before voxel baking. These do not establish that the
/// resolution captures every thin feature, or that a consistently oriented mesh is self-intersection free.
public struct SDFMeshValidation: Sendable, Equatable {
    /// Distinct vertices after welding positions closer than the baker's quantization cell
    /// (1e-5 of the longest bounding-box side). Zero when `hasFiniteNonzeroExtent` is false.
    public let weldedVertexCount: Int
    /// Complete index triples, `indices.count / 3`, including invalid and degenerate triangles.
    public let triangleCount: Int
    /// Positions with an infinite or NaN coordinate. Any of them makes the mesh invalid.
    public let nonFiniteVertexCount: Int
    /// True when every position is finite and the bounding box has a nonzero, finite longest side,
    /// so the baker can normalize the mesh. When false, no welding or edge checks were done.
    public let hasFiniteNonzeroExtent: Bool
    /// Indices that are not less than `positions.count`. Their triangles are left out of the edge
    /// checks, and any of them makes the mesh invalid.
    public let invalidIndexCount: Int
    /// Trailing indices that do not make a whole triangle (`indices.count % 3`), from 0 to 2.
    public let incompleteTriangleIndexCount: Int
    /// Triangles with two welded corners in one place, or with no area relative to the mesh's
    /// extent (squared normalized cross product at most 1e-20). They are reported here and left
    /// out of `isClosed` and `isConsistentlyOriented`, as the bakers leave them out: the voxel bake
    /// drops triangles whose welded corners coincide, and node winding skips zero-area triangles.
    /// The legacy bake still measures distance to a zero-area triangle with distinct corners.
    public let degenerateTriangleCount: Int
    /// Welded edges used by exactly one nondegenerate triangle: holes or open borders.
    public let boundaryEdgeCount: Int
    /// Welded edges used by more than two nondegenerate triangles.
    public let nonManifoldEdgeCount: Int
    /// Welded edges used by two nondegenerate triangles that traverse it in the same direction,
    /// so one of the two faces points the wrong way relative to its neighbour.
    public let inconsistentlyOrientedEdgeCount: Int

    /// Closed two-manifold connectivity, independent of triangle winding.
    public var isClosed: Bool {
        validTriangles && boundaryEdgeCount == 0 && nonManifoldEdgeCount == 0
    }

    /// Shared edges are traversed in opposite directions. A completely reversed closed shell is
    /// still consistent; the baker already reverses a shell whose total signed volume is negative.
    public var isConsistentlyOriented: Bool {
        validTriangles && nonManifoldEdgeCount == 0 && inconsistentlyOrientedEdgeCount == 0
    }

    /// Degenerate triangles are left out of the closed and oriented checks, as the bakers leave
    /// them out (see `degenerateTriangleCount`), but at least one triangle must have area.
    private var validTriangles: Bool {
        triangleCount > degenerateTriangleCount && hasFiniteNonzeroExtent && nonFiniteVertexCount == 0
            && invalidIndexCount == 0 && incompleteTriangleIndexCount == 0
    }
}

extension SDFShape {
    /// Validates connectivity and winding without allocating a voxel grid. Positions at seams are
    /// welded using the same extent-based quantization as `init(baking:indices:resolution:)`.
    /// A closed mesh can have inconsistent winding; `isWatertight` alone does not rule that out.
    /// The call neither changes the mesh nor changes the existing bake initializer's behavior.
    public static func validateMesh(positions: [SIMD3<Float>], indices: [UInt32]) -> SDFMeshValidation {
        struct Edge: Hashable {
            var low: Int
            var high: Int
        }
        struct Uses {
            var count = 0
            var direction = 0
        }
        let nonFinite = positions.filter { !$0.x.isFinite || !$0.y.isFinite || !$0.z.isFinite }.count
        let invalidIndices = indices.filter { UInt64($0) >= UInt64(positions.count) }.count
        let low = positions.filter { $0.x.isFinite && $0.y.isFinite && $0.z.isFinite }
            .reduce(SIMD3<Float>(repeating: .greatestFiniteMagnitude), simd_min)
        let high = positions.filter { $0.x.isFinite && $0.y.isFinite && $0.z.isFinite }
            .reduce(SIMD3<Float>(repeating: -.greatestFiniteMagnitude), simd_max)
        let extent = (high - low).max()
        // An overflowing coordinate range cannot be normalized by the baker either. Treat every
        // vertex in that range as invalid, so diagnostics never attempt a trapping Float-to-Int cast.
        let usableRange = nonFinite == 0 && !positions.isEmpty && extent.isFinite && extent > 0
        let cell = usableRange ? max(extent * 1e-5, 1e-9) : 1
        var unique: [SIMD3<Int>: Int] = [:]
        var remap: [Int] = []
        if usableRange {
            for p in positions {
                let q = (p - low) / cell
                let key = SIMD3<Int>(Int(q.x.rounded()), Int(q.y.rounded()), Int(q.z.rounded()))
                if let index = unique[key] { remap.append(index) }
                else { let index = unique.count; unique[key] = index; remap.append(index) }
            }
        }
        var edges: [Edge: Uses] = [:]
        var degenerate = 0
        if usableRange {
            for i in stride(from: 0, to: indices.count - indices.count % 3, by: 3) {
                let raw = [indices[i], indices[i + 1], indices[i + 2]]
                guard raw.allSatisfy({ UInt64($0) < UInt64(positions.count) }) else { continue }
                let vertex = raw.map { remap[Int($0)] }
                let a = positions[Int(raw[0])], b = positions[Int(raw[1])], c = positions[Int(raw[2])]
                let normalizedArea = simd_length_squared(simd_cross((b - a) / extent, (c - a) / extent))
                guard Set(vertex).count == 3, normalizedArea > 1e-20 else { degenerate += 1; continue }
                for j in 0..<3 {
                    let from = vertex[j], to = vertex[(j + 1) % 3]
                    let key = Edge(low: min(from, to), high: max(from, to))
                    var use = edges[key, default: Uses()]
                    use.count += 1
                    use.direction += from < to ? 1 : -1
                    edges[key] = use
                }
            }
        }
        return SDFMeshValidation(
            weldedVertexCount: unique.count, triangleCount: indices.count / 3,
            nonFiniteVertexCount: nonFinite, hasFiniteNonzeroExtent: usableRange,
            invalidIndexCount: invalidIndices, incompleteTriangleIndexCount: indices.count % 3,
            degenerateTriangleCount: degenerate,
            boundaryEdgeCount: edges.values.filter { $0.count == 1 }.count,
            nonManifoldEdgeCount: edges.values.filter { $0.count > 2 }.count,
            inconsistentlyOrientedEdgeCount: edges.values.filter { $0.count == 2 && $0.direction != 0 }.count)
    }
}
