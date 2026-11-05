import CryptoKit
import Foundation
internal import SDFCore
import simd

extension SDFShape {
    /// Controls voxel node sign classification; both methods still interpolate a finite grid.
    public enum BakeSignMethod: Sendable, Hashable {
        /// The unchanged upstream bake, retained by every preexisting initializer and cache key.
        case legacy
        /// Double-precision solid-angle winding at each actual grid node. Avoids displaced voting
        /// locations erasing a thin interior; does not make the interpolated field exact.
        case nodeWinding
    }

    public enum NodeWindingBakeError: Error {
        /// Node winding requires valid, closed, consistently oriented triangle connectivity.
        case invalidMeshTopology
    }

    /// Opt-in node sign correction. Cost is proportional to grid nodes times triangles, in
    /// addition to the legacy bake; call off the main actor for large meshes. No mesh repair or
    /// automatic resolution increase is performed. Nested cavities require oppositely wound shells.
    ///
    /// Only existing grid nodes are corrected: a part thinner than about one cell has no interior
    /// node and its field stays positive. Use a resolution of at least
    /// `minimumResolution(forThickness:longestExtent:)` and check `bakedGridInfo`.
    public convenience init(baking positions: [SIMD3<Float>], indices: [UInt32], resolution: Int = 100,
                            signMethod: BakeSignMethod) throws {
        if signMethod == .legacy {
            try self.init(baking: positions, indices: indices, resolution: resolution)
            return
        }
        let validation = Self.validateMesh(positions: positions, indices: indices)
        guard validation.isClosed && validation.isConsistentlyOriented else {
            throw NodeWindingBakeError.invalidMeshTopology
        }
        guard let vertexCount = Int32(exactly: positions.count), let indexCount = Int32(exactly: indices.count),
              let gridResolution = Int32(exactly: resolution) else { throw BakeError.invalidMesh }
        let flat = positions.flatMap { [$0.x,$0.y,$0.z] }
        guard let handle = sdf_shape_bake_node_winding(flat, vertexCount, indices, indexCount, gridResolution) else {
            throw BakeError.invalidMesh
        }
        self.init(handle)
    }

    /// Legacy keys are unchanged. The opt-in method uses a separate versioned domain so a
    /// corrected field is never loaded from a preexisting legacy cache entry.
    public static func bakeKey(positions: [SIMD3<Float>], indices: [UInt32], resolution: Int,
                               signMethod: BakeSignMethod) -> String {
        let legacy = bakeKey(positions: positions, indices: indices, resolution: resolution)
        guard signMethod == .nodeWinding else { return legacy }
        return SHA256.hash(data: Data(("node-winding-v1:"+legacy).utf8)).prefix(16)
            .map { String(format: "%02x",$0) }.joined()
    }

    /// Cached opt-in bake. The unchanged data format already stores the corrected distances,
    /// gradients, mass properties and seeds; no persisted sign-method flag is required.
    public static func cached(baking positions: [SIMD3<Float>], indices: [UInt32], resolution: Int = 100,
                              signMethod: BakeSignMethod, in directory: URL = cacheDirectory,
                              sizeLimit: Int = defaultCacheSizeLimit) throws -> SDFShape {
        let key = bakeKey(positions: positions, indices: indices, resolution: resolution, signMethod: signMethod)
        return try cached(key: key, in: directory, sizeLimit: sizeLimit) {
            try SDFShape(baking: positions, indices: indices, resolution: resolution, signMethod: signMethod)
        }
    }
}
