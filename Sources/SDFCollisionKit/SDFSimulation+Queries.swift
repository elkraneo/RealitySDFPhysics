import RealityKit
internal import SDFCore
import simd

extension SDFSimulation {
    /// A ray hit on a body. The counterpart of `CollisionCastHit`.
    public struct RaycastHit {
        public let entity: Entity
        /// In the root's frame
        public let position: SIMD3<Float>
        /// The surface normal at the hit
        public let normal: SIMD3<Float>
        /// From the ray's origin
        public let distance: Float
    }

    /// Casts a ray (in the root's frame) against the bodies whose group is in the mask: triggers,
    /// static and kinematic ones included. A ray that starts inside a body hits it at distance 0.
    ///
    /// - Parameters:
    ///   - origin: Start of the ray
    ///   - direction: Direction of the ray, of any length
    ///   - length: Length of the ray
    ///   - query: `.nearest` and `.any` return one hit at most, `.all` every body's first hit,
    ///     sorted by distance.
    ///   - mask: The groups of the bodies to test
    public func raycast(origin: SIMD3<Float>, direction: SIMD3<Float>, length: Float = 100,
                        query: CollisionCastQueryType = .all, mask: CollisionGroup = .all) -> [RaycastHit] {
        // At most one hit per body
        var buffer = [SDFRayHit](repeating: SDFRayHit(), count: Int(sdf_world_stats(world).bodies))
        let count = buffer.withUnsafeMutableBufferPointer {
            sdf_world_raycast(world, [origin.x, origin.y, origin.z], [direction.x, direction.y, direction.z],
                              length, mask.rawValue, $0.baseAddress, Int32($0.count))
        }
        let hits = buffer.prefix(Int(count)).compactMap { hit -> RaycastHit? in
            guard let entity = entity(of: hit.body) else { return nil }
            return RaycastHit(entity: entity, position: SIMD3(hit.position.0, hit.position.1, hit.position.2),
                              normal: SIMD3(hit.normal.0, hit.normal.1, hit.normal.2), distance: hit.distance)
        }.sorted { $0.distance < $1.distance }
        return query == .all ? hits : Array(hits.prefix(1))
    }

    /// Casts a ray between two points (in the root's frame)
    public func raycast(from start: SIMD3<Float>, to end: SIMD3<Float>, query: CollisionCastQueryType = .all,
                        mask: CollisionGroup = .all) -> [RaycastHit] {
        raycast(origin: start, direction: end - start, length: simd_distance(start, end), query: query, mask: mask)
    }

    /// A body near a shape
    public struct Overlap {
        public let entity: Entity
        /// The deepest point found, in the root's frame
        public let position: SIMD3<Float>
        /// From the query shape towards the body
        public let normal: SIMD3<Float>
        /// Gap (positive) or overlap (negative) of the two surfaces
        public let separation: Float
    }

    /// The bodies that a shape at a pose (in the root's frame, with a uniform scale) overlaps, or
    /// comes within `margin` of. Answers "does this fit here?" with the same collision detection as
    /// the simulation, for any shape, concave ones included.
    ///
    /// The search is stochastic but seeded, so the same query on the same bodies always gives the
    /// same answer, and it does not change the simulation.
    public func overlaps(of shape: SDFShape, at transform: Transform, margin: Float = 0,
                         mask: CollisionGroup = .all) -> [Overlap] {
        let q = transform.rotation.vector
        let pose = SDFPose(position: (transform.translation.x, transform.translation.y, transform.translation.z),
                           orientation: (q.x, q.y, q.z, q.w), scale: transform.scale.x)
        // At most one result per body
        var buffer = [SDFOverlap](repeating: SDFOverlap(), count: Int(sdf_world_stats(world).bodies))
        let count = buffer.withUnsafeMutableBufferPointer {
            sdf_world_overlaps(world, shape.handle, pose, margin, mask.rawValue, $0.baseAddress, Int32($0.count))
        }
        return buffer.prefix(Int(count)).compactMap { overlap in
            guard let entity = entity(of: overlap.body) else { return nil }
            return Overlap(entity: entity, position: SIMD3(overlap.position.0, overlap.position.1, overlap.position.2),
                           normal: SIMD3(overlap.normal.0, overlap.normal.1, overlap.normal.2),
                           separation: overlap.separation)
        }
    }
}
