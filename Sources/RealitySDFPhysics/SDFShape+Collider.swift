import RealityKit
import simd

extension SDFShape {
    /// The SDF of a RealityKit collision shape, and its pose in the collision shape's frame.
    /// Axis-aligned boxes, spheres and capsules become analytical shapes; any other shape is baked
    /// from its mesh in the background, like `bake(from:resolution:)`.
    ///
    /// - Parameter scale: A scale applied to the collision shape first. A box stays a box under any
    ///   scale, a sphere or a capsule only under a uniform one; others are baked.
    @MainActor
    public static func make(from collisionShape: ShapeResource, scale: SIMD3<Float> = .one, resolution: Int = 100) async throws
        -> (shape: SDFShape, transform: Transform) {
        var positions: [SIMD3<Float>] = []
        var indices: [UInt32] = []
        appendTriangles(of: try await MeshResource(shape: collisionShape), transform: matrix_identity_float4x4,
                        positions: &positions, indices: &indices)
        positions = positions.map { $0 * scale }
        if let primitive = primitive(of: positions, indices: indices) { return primitive }
        let shape = try await Task.detached(priority: .userInitiated) {
            try SDFShape(baking: positions, indices: indices, resolution: resolution)
        }.value
        return (shape, Transform())
    }

    /// The axis-aligned box, sphere or capsule that a closed mesh is a tessellation of, if any
    static func primitive(of positions: [SIMD3<Float>], indices: [UInt32]) -> (shape: SDFShape, transform: Transform)? {
        guard let first = positions.first, indices.count >= 3 else { return nil }
        let lower = positions.reduce(first, pointwiseMin), upper = positions.reduce(first, pointwiseMax)
        let center = (lower + upper) / 2, half = (upper - lower) / 2
        let tolerance = 1e-3 * max(half.max(), .leastNormalMagnitude)
        let triangles = stride(from: 0, to: indices.count - 2, by: 3).map { t in
            (positions[Int(indices[t])], positions[Int(indices[t + 1])], positions[Int(indices[t + 2])])
        }

        // A box: every triangle lies in a face of the bounding box, and they cover all six faces
        let isBox = half.min() > tolerance && triangles.allSatisfy { triangle in
            (0..<3).contains { axis in
                [lower[axis], upper[axis]].contains { plane in
                    [triangle.0, triangle.1, triangle.2].allSatisfy { abs($0[axis] - plane) <= tolerance }
                }
            }
        }
        let area = triangles.map { length(cross($0.1 - $0.0, $0.2 - $0.0)) / 2 }.reduce(0, +)
        let boxArea = 8 * (half.x * half.y + half.y * half.z + half.z * half.x)
        if isBox && abs(area - boxArea) <= 1e-3 * boxArea {
            return (.box(halfExtents: half), Transform(translation: center))
        }

        // A capsule, or a sphere: every vertex at one distance from a segment along the longest
        // side of the bounding box, and every triangle close to that surface, as in a fine tessellation
        let axis = half.x >= half.y && half.x >= half.z ? 0 : (half.y >= half.z ? 1 : 2)
        var direction = SIMD3<Float>.zero
        direction[axis] = 1
        let radius = positions.map { length(($0 - center) - dot($0 - center, direction) * direction) }.max()!
        let halfLength = max(half[axis] - radius, 0)
        func distance(_ p: SIMD3<Float>) -> Float {
            let along = simd_clamp(dot(p - center, direction), -halfLength, halfLength)
            return length(p - center - along * direction) - radius
        }
        let isRound = radius > tolerance && positions.allSatisfy { abs(distance($0)) <= tolerance } &&
            triangles.allSatisfy { abs(distance(($0.0 + $0.1 + $0.2) / 3)) <= 0.05 * radius }
        guard isRound else { return nil }
        if halfLength <= tolerance { return (.sphere(radius: radius), Transform(translation: center)) }
        return (.capsule(radius: radius, halfLength: halfLength),
                Transform(rotation: simd_quatf(from: [0, 1, 0], to: direction), translation: center))
    }
}

extension Entity {
    /// Lets SDF bodies collide with the entity's RealityKit collision shapes: adds a body for each
    /// shape of its `CollisionComponent`, on a child entity named `SDFCollider`, and removes those
    /// this method added before, or only removes them when the entity has no collision shapes
    /// anymore. The entity has to be in an `SDFPhysicsSimulationComponent`'s subtree.
    ///
    /// Bodies are static by default. Make them kinematic for colliders that move, such as the
    /// app's own kinematic bodies or a hand; RealityKit's physics keeps simulating the entity.
    /// Trigger shapes become trigger bodies. A scale that isn't uniform, from the entity or its
    /// ancestors, is applied to the shapes when they're made.
    ///
    /// When the method is called again for the entity before it returns, the earlier call throws
    /// `CancellationError` and leaves the bodies to the later one.
    ///
    /// - Returns: The child entities with the bodies
    @MainActor @discardableResult
    public func addSDFColliders(mode: PhysicsBodyMode = .static, material: SDFPhysicsMaterial = .default,
                                resolution: Int = 100) async throws -> [Entity] {
        ImportedColliderComponent.registerComponent()
        let generation = (importGenerations[id] ?? 0) + 1
        importGenerations[id] = generation
        defer { if importGenerations[id] == generation { importGenerations[id] = nil } }

        var colliders: [Entity] = []
        if let collision = components[CollisionComponent.self] {
            // The simulation takes one scale per body, so a scale that isn't uniform goes into the
            // shapes, and the child undoes it
            var root: Entity? = parent
            while let entity = root, !entity.components.has(SDFPhysicsSimulationComponent.self) { root = entity.parent }
            var scale = self.scale(relativeTo: root)
            scale = abs(scale.max() - scale.min()) <= 1e-5 * scale.max() ? .one : scale
            for collisionShape in collision.shapes {
                var (shape, transform) = try await SDFShape.make(from: collisionShape, scale: scale, resolution: resolution)
                guard importGenerations[id] == generation else { throw CancellationError() }
                // The child's scale cancels the parent's under the shape's rotation, which only
                // swaps axes (a capsule's) or is none
                transform.translation /= scale
                transform.scale = abs(transform.rotation.inverse.act(1 / scale))
                let collider = Entity()
                collider.name = SDFColliderName
                collider.transform = transform
                var body = SDFPhysicsBodyComponent(shape: shape, material: material, mode: mode)
                body.filter = collision.filter
                body.isTrigger = collision.mode == .trigger
                collider.components.set(body)
                collider.components.set(ImportedColliderComponent())
                colliders.append(collider)
            }
        }
        for child in children.filter({ $0.components.has(ImportedColliderComponent.self) }) { child.removeFromParent() }
        for collider in colliders { addChild(collider) }
        return colliders
    }
}

/// Marks the children that `addSDFColliders` added
private struct ImportedColliderComponent: Component {}

@MainActor private var importGenerations: [Entity.ID: Int] = [:]

private let SDFColliderName = "SDFCollider"
