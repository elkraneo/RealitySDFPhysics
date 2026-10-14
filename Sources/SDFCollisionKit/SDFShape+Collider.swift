import RealityKit
import simd

extension SDFShape {
    /// The SDF of a RealityKit collision shape, and its pose in the collision shape's frame.
    /// Axis-aligned boxes, spheres and capsules become analytical shapes; any other shape is baked
    /// from its mesh in the background, like `bake(from:resolution:)`.
    @MainActor
    public static func make(from collisionShape: ShapeResource, resolution: Int = 100) async throws
        -> (shape: SDFShape, transform: Transform) {
        var positions: [SIMD3<Float>] = []
        var indices: [UInt32] = []
        appendTriangles(of: try await MeshResource(shape: collisionShape), transform: matrix_identity_float4x4,
                        positions: &positions, indices: &indices)
        if let primitive = primitive(of: positions, indices: indices) { return primitive }
        let shape = try await Task.detached(priority: .userInitiated) {
            try SDFShape(baking: positions, indices: indices, resolution: resolution)
        }.value
        return (shape, Transform())
    }

    /// The axis-aligned box, sphere or capsule that a mesh is a tessellation of, if any
    static func primitive(of positions: [SIMD3<Float>], indices: [UInt32]) -> (shape: SDFShape, transform: Transform)? {
        guard let first = positions.first, indices.count >= 3 else { return nil }
        let lower = positions.reduce(first, pointwiseMin), upper = positions.reduce(first, pointwiseMax)
        let center = (lower + upper) / 2, half = (upper - lower) / 2
        let tolerance = 1e-3 * max(half.max(), .leastNormalMagnitude)

        // A box: every triangle lies in a face of the bounding box
        let isBox = stride(from: 0, to: indices.count - 2, by: 3).allSatisfy { t in
            let triangle = indices[t..<t + 3].map { positions[Int($0)] }
            return (0..<3).contains { axis in
                [lower[axis], upper[axis]].contains { plane in
                    triangle.allSatisfy { abs($0[axis] - plane) <= tolerance }
                }
            }
        }
        if isBox { return (.box(halfExtents: half), Transform(translation: center)) }

        // A capsule, or a sphere: every vertex at one distance from a segment along the longest
        // side of the bounding box
        let axis = half.x >= half.y && half.x >= half.z ? 0 : (half.y >= half.z ? 1 : 2)
        var direction = SIMD3<Float>.zero
        direction[axis] = 1
        let radius = positions.map { length(($0 - center) - dot($0 - center, direction) * direction) }.max()!
        let halfLength = max(half[axis] - radius, 0)
        let isRound = radius > tolerance && positions.allSatisfy { p in
            let along = simd_clamp(dot(p - center, direction), -halfLength, halfLength)
            return abs(length(p - center - along * direction) - radius) <= tolerance
        }
        guard isRound else { return nil }
        if halfLength <= tolerance { return (.sphere(radius: radius), Transform(translation: center)) }
        return (.capsule(radius: radius, halfLength: halfLength),
                Transform(rotation: simd_quatf(from: [0, 1, 0], to: direction), translation: center))
    }
}

extension Entity {
    /// Lets SDF bodies collide with the entity's RealityKit collision shapes: adds a body for each
    /// shape of its `CollisionComponent`, on a child entity named `SDFCollider`, and removes those
    /// added before. The entity has to be in an `SDFSimulationComponent`'s subtree.
    ///
    /// Bodies are static by default. Make them kinematic for colliders that move, such as the
    /// app's own kinematic bodies or a hand; RealityKit's physics keeps simulating the entity.
    ///
    /// - Returns: The child entities with the bodies
    @MainActor @discardableResult
    public func addSDFColliders(mode: PhysicsBodyMode = .static, material: SDFMaterial = .default,
                                resolution: Int = 100) async throws -> [Entity] {
        guard let collision = components[CollisionComponent.self] else { return [] }
        var colliders: [Entity] = []
        for collisionShape in collision.shapes {
            let (shape, transform) = try await SDFShape.make(from: collisionShape, resolution: resolution)
            let collider = Entity()
            collider.name = SDFColliderName
            collider.transform = transform
            var body = SDFBodyComponent(shape: shape, material: material, mode: mode)
            body.filter = collision.filter
            collider.components.set(body)
            colliders.append(collider)
        }
        for child in children.filter({ $0.name == SDFColliderName }) { child.removeFromParent() }
        for collider in colliders { addChild(collider) }
        return colliders
    }
}

private let SDFColliderName = "SDFCollider"
