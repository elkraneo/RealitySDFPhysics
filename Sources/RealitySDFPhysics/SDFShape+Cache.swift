import CryptoKit
import Foundation

extension SDFShape {
    /// The folder "RealitySDFPhysics" in the user's caches directory
    public static var cacheDirectory: URL {
        URL.cachesDirectory.appending(path: "RealitySDFPhysics", directoryHint: .isDirectory)
    }

    /// The size the cache directory is trimmed to after each bake it saves: 512 MB
    public static let defaultCacheSizeLimit = 512 * 1024 * 1024

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
    ///
    /// A load marks the file as used. After saving, the directory is trimmed to `sizeLimit`
    /// bytes, the least recently used bakes first, so bakes a changed mesh left behind go away.
    public static func cached(key: String, in directory: URL = cacheDirectory, sizeLimit: Int = defaultCacheSizeLimit,
                              bake: () throws -> SDFShape) throws -> SDFShape {
        let url = directory.appending(path: key + ".sdf")
        if let data = try? Data(contentsOf: url, options: .mappedIfSafe), let shape = try? SDFShape(bakedData: data) {
            try? FileManager.default.setAttributes([.modificationDate: Date()], ofItemAtPath: url.path)
            return shape
        }
        let shape = try bake()
        if let data = shape.bakedData() {
            try? FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
            try? data.write(to: url, options: .atomic)
            trimCache(in: directory, to: sizeLimit)
        }
        return shape
    }

    /// Deletes the least recently used bakes in a directory until the rest take at most `sizeLimit`
    /// bytes. Only the cache's own ".sdf" files count and are deleted.
    public static func trimCache(in directory: URL = cacheDirectory, to sizeLimit: Int = defaultCacheSizeLimit) {
        let keys: Set<URLResourceKey> = [.contentModificationDateKey, .totalFileAllocatedSizeKey, .fileSizeKey]
        guard let urls = try? FileManager.default.contentsOfDirectory(at: directory, includingPropertiesForKeys: Array(keys))
        else { return }
        var files = urls.filter { $0.pathExtension == "sdf" }.compactMap { url -> (url: URL, date: Date, size: Int)? in
            guard let values = try? url.resourceValues(forKeys: keys) else { return nil }
            return (url, values.contentModificationDate ?? .distantPast, values.totalFileAllocatedSize ?? values.fileSize ?? 0)
        }
        var total = files.reduce(0) { $0 + $1.size }
        files.sort { $0.date < $1.date }
        // A file another process deleted first is gone all the same
        for file in files where total > sizeLimit {
            try? FileManager.default.removeItem(at: file.url)
            total -= file.size
        }
    }

    /// Bakes a mesh as `init(baking:indices:resolution:)` does, once: later calls with the same
    /// mesh and resolution load the shape from the cache directory
    public static func cached(baking positions: [SIMD3<Float>], indices: [UInt32], resolution: Int = 100,
                              in directory: URL = cacheDirectory, sizeLimit: Int = defaultCacheSizeLimit) throws -> SDFShape {
        let key = bakeKey(positions: positions, indices: indices, resolution: resolution)
        return try cached(key: key, in: directory, sizeLimit: sizeLimit) {
            try SDFShape(baking: positions, indices: indices, resolution: resolution)
        }
    }
}
