import Foundation
import RealityKit
import RealitySDFPhysics
import Testing
import simd

@Suite struct SDFBakeCacheTests {
    /// A concave mesh away from the origin, so that the frame and the curvature seeds matter
    static let torus = SDFShape.torus(majorRadius: 1, minorRadius: 0.3).surface
    static let positions = torus.positions.map { $0 + [2, -1, 0.5] }

    /// A directory of its own for a test, removed after
    static func withTemporaryDirectory<T>(_ body: (URL) throws -> T) throws -> T {
        let directory = FileManager.default.temporaryDirectory.appending(path: "SDFBakeCacheTests-\(UUID().uuidString)")
        defer { try? FileManager.default.removeItem(at: directory) }
        return try body(directory)
    }

    @Test func reloadedShapeIsIdentical() throws {
        let shape = try SDFShape(baking: Self.positions, indices: Self.torus.indices, resolution: 48)
        let data = try #require(shape.bakedData())
        let loaded = try SDFShape(bakedData: data)

        #expect(loaded.bakedData() == data)
        #expect(loaded.volume == shape.volume)
        #expect(loaded.isConvex == shape.isConvex && loaded.isWatertight == shape.isWatertight)
        #expect(loaded.surface.positions == shape.surface.positions)
        #expect(loaded.surface.indices == shape.surface.indices)

        // Inside, on and well outside the grid
        var random = SplitMix64(seed: 1)
        for _ in 0..<1000 {
            let p = SIMD3<Float>(Float.random(in: -1...5, using: &random), Float.random(in: -3...1, using: &random),
                                 Float.random(in: -2...3, using: &random))
            let a = shape.distance(at: p), b = loaded.distance(at: p)
            #expect(a.distance.bitPattern == b.distance.bitPattern && a.gradient == b.gradient, "at \(p)")
        }
    }

    @MainActor
    @Test func reloadedShapeSimulatesTheSame() throws {
        // Mass properties and curvature seeds come back too: a drop onto the ground is identical
        let shape = try SDFShape(baking: Self.positions, indices: Self.torus.indices, resolution: 48)
        let loaded = try SDFShape(bakedData: try #require(shape.bakedData()))
        func drop(_ shape: SDFShape) -> Transform {
            let root = Entity()
            let simulation = SDFSimulation()
            root.components.set(SDFPhysicsSimulationComponent(simulation: simulation))
            let ground = Entity()
            ground.position = [0, -2, 0]
            ground.components.set(SDFPhysicsBodyComponent(shape: .box(halfExtents: [5, 0.5, 5]), mode: .static))
            let body = Entity()
            body.orientation = simd_quatf(angle: 0.4, axis: normalize([1, 0, 1]))
            body.components.set(SDFPhysicsBodyComponent(shape: shape))
            root.addChild(ground)
            root.addChild(body)
            for _ in 0..<120 { simulation.step(root: root) }
            return body.transform
        }
        let a = drop(shape), b = drop(loaded)
        #expect(a.translation == b.translation && a.rotation == b.rotation)
    }

    @Test func analyticalShapesHaveNoBakedData() {
        #expect(SDFShape.sphere(radius: 1).bakedData() == nil)
    }

    @Test func invalidDataIsRefused() throws {
        let shape = try SDFShape(baking: Self.positions, indices: Self.torus.indices, resolution: 16)
        let data = try #require(shape.bakedData())
        #expect(throws: SDFShape.BakeError.invalidData) { try SDFShape(bakedData: Data()) }
        #expect(throws: SDFShape.BakeError.invalidData) { try SDFShape(bakedData: data.dropLast()) }
        var version = data
        version[4] &+= 1
        #expect(throws: SDFShape.BakeError.invalidData) { try SDFShape(bakedData: version) }
    }

    @Test func cacheBakesOnceAndRebakesChangedMeshes() throws {
        try Self.withTemporaryDirectory { directory in
            var bakes = 0
            func shape(_ positions: [SIMD3<Float>], resolution: Int = 16) throws -> SDFShape {
                let key = SDFShape.bakeKey(positions: positions, indices: Self.torus.indices, resolution: resolution)
                return try SDFShape.cached(key: key, in: directory) {
                    bakes += 1
                    return try SDFShape(baking: positions, indices: Self.torus.indices, resolution: resolution)
                }
            }
            let first = try shape(Self.positions)
            let second = try shape(Self.positions)
            #expect(bakes == 1)
            #expect(second.bakedData() == first.bakedData())

            _ = try shape(Self.positions, resolution: 20)
            #expect(bakes == 2)
            _ = try shape(Self.positions.map { $0 * 1.01 })
            #expect(bakes == 3)
            #expect(try FileManager.default.contentsOfDirectory(atPath: directory.path).count == 3)
        }
    }

    @Test func cacheRebakesUnreadableFiles() throws {
        try Self.withTemporaryDirectory { directory in
            try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
            try Data("not a shape".utf8).write(to: directory.appending(path: "torus.sdf"))
            var bakes = 0
            let shape = try SDFShape.cached(key: "torus", in: directory) {
                bakes += 1
                return try SDFShape(baking: Self.positions, indices: Self.torus.indices, resolution: 16)
            }
            #expect(bakes == 1)
            let saved = try Data(contentsOf: directory.appending(path: "torus.sdf"))
            #expect(saved == shape.bakedData())
        }
    }

    @Test func cacheKeepsTheMostRecentlyUsedBakes() throws {
        try Self.withTemporaryDirectory { directory in
            let shape = try SDFShape(baking: Self.positions, indices: Self.torus.indices, resolution: 16)
            let size = try #require(shape.bakedData()).count
            func cache(_ key: String, limit: Int = .max) throws { _ = try SDFShape.cached(key: key, in: directory, sizeLimit: limit) { shape } }
            func keys() throws -> Set<String> {
                Set(try FileManager.default.contentsOfDirectory(atPath: directory.path).map { ($0 as NSString).deletingPathExtension })
            }
            // Each file's date a second apart, as file systems keep them only so finely
            for (index, key) in ["a", "b", "c"].enumerated() {
                try cache(key)
                try FileManager.default.setAttributes([.modificationDate: Date(timeIntervalSinceNow: Double(index - 10))],
                                                      ofItemAtPath: directory.appending(path: key + ".sdf").path)
            }
            // Loading "a" makes it the most recent; a fourth bake with room for about three drops "b"
            try cache("a")
            let other = directory.appending(path: "notes.txt")
            try Data(count: size * 4).write(to: other)
            try cache("d", limit: size * 3 + size / 2)
            #expect(try keys() == ["a", "c", "d", "notes"])

            SDFShape.trimCache(in: directory, to: 0)
            #expect(try keys() == ["notes"])
        }
    }
}

/// A seeded random number generator, for points that are the same in every run
private struct SplitMix64: RandomNumberGenerator {
    var state: UInt64
    init(seed: UInt64) { state = seed }
    mutating func next() -> UInt64 {
        state &+= 0x9E37_79B9_7F4A_7C15
        var z = state
        z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
        z = (z ^ (z >> 27)) &* 0x94D0_49BB_1330_11EB
        return z ^ (z >> 31)
    }
}
