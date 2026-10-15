import CryptoKit
import Foundation

extension SDFShape {
    /// The folder "SDFCollisionKit" in the user's caches directory
    public static var cacheDirectory: URL {
        URL.cachesDirectory.appending(path: "SDFCollisionKit", directoryHint: .isDirectory)
    }

    /// A key that identifies a bake by its inputs: a hash of the mesh and the resolution, so that a
    /// changed mesh is baked again
    public static func bakeKey(positions: [SIMD3<Float>], indices: [UInt32], resolution: Int) -> String {
        var hash = SHA256()
        for value in [positions.count, indices.count, resolution] {
            withUnsafeBytes(of: Int64(value).littleEndian) { hash.update(bufferPointer: $0) }
        }
        positions.withUnsafeBytes { hash.update(bufferPointer: $0) }
        indices.withUnsafeBytes { hash.update(bufferPointer: $0) }
        return hash.finalize().prefix(16).map { String(format: "%02x", $0) }.joined()
    }

    /// The shape saved under `key` in a directory or, when there is none that loads, the one
    /// `bake` returns, which is then saved there. The cache only speeds things up: failing to save
    /// is ignored. Files are written atomically, so processes and threads can share a directory.
    public static func cached(key: String, in directory: URL = cacheDirectory, bake: () throws -> SDFShape) throws
        -> SDFShape {
        let url = directory.appending(path: key + ".sdf")
        if let data = try? Data(contentsOf: url, options: .mappedIfSafe), let shape = try? SDFShape(bakedData: data) {
            return shape
        }
        let shape = try bake()
        if let data = shape.bakedData() {
            try? FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
            try? data.write(to: url, options: .atomic)
        }
        return shape
    }

    /// Bakes a mesh as `init(baking:indices:resolution:)` does, once: later calls with the same
    /// mesh and resolution load the shape from the cache directory
    public static func cached(baking positions: [SIMD3<Float>], indices: [UInt32], resolution: Int = 100,
                              in directory: URL = cacheDirectory) throws -> SDFShape {
        let key = bakeKey(positions: positions, indices: indices, resolution: resolution)
        return try cached(key: key, in: directory) {
            try SDFShape(baking: positions, indices: indices, resolution: resolution)
        }
    }
}
