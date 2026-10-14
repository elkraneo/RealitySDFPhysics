import RealityKit
import simd

extension SDFShape {
    /// Bakes the meshes of an entity and its descendants into one voxel SDF, in the entity's
    /// frame. The meshes are read on the main actor and baked in the background.
    ///
    /// The meshes should together form a closed surface; check `isWatertight`.
    @MainActor
    public static func bake(from entity: Entity, resolution: Int = 100) async throws -> SDFShape {
        var positions: [SIMD3<Float>] = []
        var indices: [UInt32] = []
        appendTriangles(of: entity, relativeTo: entity, positions: &positions, indices: &indices)
        return try await Task.detached(priority: .userInitiated) {
            try SDFShape(baking: positions, indices: indices, resolution: resolution)
        }.value
    }

    /// Bakes several entities at once, each like `bake(from:resolution:)`, in parallel. Returns the
    /// shapes in the order of the entities.
    @MainActor
    public static func bake(from entities: [Entity], resolution: Int = 100) async throws -> [SDFShape] {
        let meshes = entities.map { entity in
            var positions: [SIMD3<Float>] = []
            var indices: [UInt32] = []
            appendTriangles(of: entity, relativeTo: entity, positions: &positions, indices: &indices)
            return (positions, indices)
        }
        return try await withThrowingTaskGroup(of: (Int, SDFShape).self) { group in
            for (index, mesh) in meshes.enumerated() {
                group.addTask {
                    (index, try SDFShape(baking: mesh.0, indices: mesh.1, resolution: resolution))
                }
            }
            var shapes = [SDFShape?](repeating: nil, count: meshes.count)
            for try await (index, shape) in group { shapes[index] = shape }
            return shapes.map { $0! }
        }
    }

    @MainActor
    private static func appendTriangles(of entity: Entity, relativeTo root: Entity, positions: inout [SIMD3<Float>],
                                        indices: inout [UInt32]) {
        if let mesh = entity.components[ModelComponent.self]?.mesh {
            let toRoot = entity.transformMatrix(relativeTo: root)
            let contents = mesh.contents
            for instance in contents.instances {
                guard let model = contents.models[instance.model] else { continue }
                let transform = toRoot * instance.transform
                for part in model.parts {
                    guard let triangles = part.triangleIndices else { continue }
                    let base = UInt32(positions.count)
                    positions += part.positions.elements.map { p in
                        let w = transform * SIMD4(p, 1)
                        return SIMD3(w.x, w.y, w.z)
                    }
                    indices += triangles.elements.map { base + $0 }
                }
            }
        }
        for child in entity.children {
            appendTriangles(of: child, relativeTo: root, positions: &positions, indices: &indices)
        }
    }
}

extension MeshResource {
    /// A mesh of the surface of a shape, for display or debugging
    @MainActor
    public static func generate(from shape: SDFShape) throws -> MeshResource {
        let surface = shape.surface
        var descriptor = MeshDescriptor(name: "SDFShape")
        descriptor.positions = MeshBuffers.Positions(surface.positions)
        if let normals = surface.normals {
            descriptor.normals = MeshBuffers.Normals(normals)
        }
        descriptor.primitives = .triangles(surface.indices)
        return try generate(from: [descriptor])
    }
}
