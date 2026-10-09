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
