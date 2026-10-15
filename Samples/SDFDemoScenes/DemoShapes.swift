import Foundation
import RealityKit
import RealitySDFPhysics

/// The named shapes of the upstream demo, created on first use and shared by all scenes
@MainActor
public final class DemoShapes {
    public static let shared = DemoShapes()

    /// Shapes that can be dropped into a scene, in the upstream order
    public static let names = ["Box", "Sphere", "Capsule", "Cylinder", "Cone", "Torus", "Plank", "Bent Plank",
                               "Twisted Plank", "Displaced Plank", "Spike Plate", "Spiky Wheel", "Bunny", "Cow",
                               "Chair", "Gear"]

    private var shapes: [String: SDFShape] = [:]
    private var meshes: [String: MeshResource] = [:]

    /// The shape of a name. Mesh shapes are baked from the bundled OBJ files on first use.
    public func shape(named name: String) async throws -> SDFShape {
        if let shape = shapes[name] { return shape }
        let shape = try await make(name)
        shapes[name] = shape
        return shape
    }

    /// Display mesh of a shape, generated from its surface
    public func mesh(named name: String) async throws -> MeshResource {
        if let mesh = meshes[name] { return mesh }
        let mesh = try MeshResource.generate(from: try await shape(named: name))
        meshes[name] = mesh
        return mesh
    }

    // Sizes are those of upstream scenes.cpp
    private func make(_ name: String) async throws -> SDFShape {
        switch name {
        case "Ground": .box(halfExtents: [60, 1, 60])
        case "Box": .box(halfExtents: [0.5, 0.5, 0.5])
        case "Plank": .box(halfExtents: [1.15, 0.28, 0.55])
        case "Bent Plank": .warpedBox(.bend, halfExtents: [1.15, 0.28, 0.55], amount: 0.95)
        case "Twisted Plank": .warpedBox(.twist, halfExtents: [1.15, 0.55, 0.28], amount: 1.25)
        case "Displaced Plank": .warpedBox(.displace, halfExtents: [1.15, 0.28, 0.55], amount: 0.1)
        case "Sphere": .sphere(radius: 0.5)
        case "Capsule": .capsule(radius: 0.3, halfLength: 0.4)
        case "Cylinder": .cylinder(radius: 0.4, halfHeight: 0.4)
        case "Cone": .cone(radius: 0.5, halfHeight: 0.5)
        case "Torus": .torus(majorRadius: 0.5, minorRadius: 0.2)
        case "Spike Plate": .spikePlate(halfExtents: [0.7, 0.08, 0.7], count: 10, spikeRadius: 0.045, spikeHeights: 0.12...0.26)
        case "Spiky Wheel": .spikyWheel(hubRadius: 0.35, hubHalfWidth: 0.48, spikeRadius: 0.08, spikeLength: 1.4, spikes: 16)
        default: try await bakeAsset(name.lowercased())
        }
    }

    /// Bakes a bundled OBJ, normalized to fit a unit cube centered at the origin as upstream does
    private func bakeAsset(_ name: String) async throws -> SDFShape {
        guard let url = Bundle.module.url(forResource: name, withExtension: "obj", subdirectory: "assets") else {
            throw CocoaError(.fileNoSuchFile)
        }
        return try await Task.detached(priority: .userInitiated) {
            let mesh = try OBJMesh(contentsOf: url).normalized()
            return try SDFShape.cached(baking: mesh.positions, indices: mesh.indices)
        }.value
    }
}

/// Positions and triangles of a Wavefront OBJ file. Polygons are fan triangulated.
struct OBJMesh {
    var positions: [SIMD3<Float>] = []
    var indices: [UInt32] = []

    init(contentsOf url: URL) throws {
        for line in try String(contentsOf: url, encoding: .utf8).split(whereSeparator: \.isNewline) {
            let fields = line.split(separator: " ", omittingEmptySubsequences: true)
            switch fields.first {
            case "v" where fields.count >= 4:
                positions.append(SIMD3(Float(fields[1]) ?? 0, Float(fields[2]) ?? 0, Float(fields[3]) ?? 0))
            case "f" where fields.count >= 4:
                // "v", "v/vt" or "v/vt/vn", 1-based or negative from the end
                let corners = fields.dropFirst().compactMap { field -> UInt32? in
                    guard let index = Int(field.split(separator: "/", omittingEmptySubsequences: false)[0]) else { return nil }
                    return UInt32(index < 0 ? positions.count + index : index - 1)
                }
                for i in corners.indices.dropFirst().dropLast() {
                    indices += [corners[0], corners[i], corners[i + 1]]
                }
            default:
                continue
            }
        }
    }

    func normalized() -> OBJMesh {
        let lower = positions.reduce(SIMD3(repeating: .infinity)) { simd_min($0, $1) }
        let upper = positions.reduce(SIMD3(repeating: -.infinity)) { simd_max($0, $1) }
        let center = (lower + upper) / 2, extent = (upper - lower).max()
        var mesh = self
        mesh.positions = positions.map { ($0 - center) / extent }
        return mesh
    }
}
