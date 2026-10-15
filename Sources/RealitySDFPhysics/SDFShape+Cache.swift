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
            // The cache's own files only: a link would date the file it points to
            if (try? url.resourceValues(forKeys: [.isRegularFileKey]))?.isRegularFile == true {
                try? FileManager.default.setAttributes([.modificationDate: Date()], ofItemAtPath: url.path)
            }
            return shape
        }
        let shape = try bake()
        if let data = shape.bakedData() {
            try? FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
            try? data.write(to: url, options: .atomic)
            trimCache(in: directory, to: sizeLimit, keeping: url)
        }
        return shape
    }

    /// Deletes the least recently used bakes in a directory until the rest take at most `sizeLimit`
    /// bytes. Only the cache's own ".sdf" files count and are deleted: regular files, not links or
    /// folders.
    public static func trimCache(in directory: URL = cacheDirectory, to sizeLimit: Int = defaultCacheSizeLimit) {
        trimCache(in: directory, to: sizeLimit, keeping: nil)
    }

    /// Trims the cache, never deleting `kept`, the bake just saved. Another process may use a bake
    /// while it is trimmed: one used since the listing is kept, and otherwise the worst is a bake
    /// done again.
    static func trimCache(in directory: URL, to sizeLimit: Int, keeping kept: URL?) {
        let keys: [URLResourceKey] = [.isRegularFileKey, .contentModificationDateKey, .totalFileAllocatedSizeKey, .fileSizeKey]
        func values(_ url: URL) -> (date: Date, size: Int)? {
            guard let values = try? url.resourceValues(forKeys: Set(keys)), values.isRegularFile == true else { return nil }
            return (values.contentModificationDate ?? .distantPast, values.totalFileAllocatedSize ?? values.fileSize ?? 0)
        }
        guard let urls = try? FileManager.default.contentsOfDirectory(at: directory, includingPropertiesForKeys: keys)
        else { return }
        var files = urls.filter { $0.pathExtension == "sdf" }.compactMap { url in values(url).map { (url, $0.date, $0.size) } }
        var total = files.reduce(0) { $0 + $1.2 }
        files.sort { $0.1 < $1.1 }
        for (url, date, size) in files where total > sizeLimit && url.lastPathComponent != kept?.lastPathComponent {
            // Read again: a bake used since the listing stays
            var url = url
            url.removeAllCachedResourceValues()
            guard let now = values(url) else { continue }
            if now.date > date { continue }
            do {
                try FileManager.default.removeItem(at: url)
                total -= size
            } catch {
                // Gone already, as another process trimmed it, or it stays and counts
                if !FileManager.default.fileExists(atPath: url.path) { total -= size }
            }
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
