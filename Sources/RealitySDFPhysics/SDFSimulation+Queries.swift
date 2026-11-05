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
        guard mask.rawValue != 0 else { return [] }
        // The native functions write POD results, so no zero-initialized Swift scratch array is needed.
        let capacity = Int(sdf_world_stats(world).bodies)
        guard capacity > 0 else { return [] }
        return withUnsafeTemporaryAllocation(of: SDFRayHit.self, capacity: capacity) { buffer in
            let count = min(capacity, Int(sdf_world_raycast(
                world, [origin.x, origin.y, origin.z], [direction.x, direction.y, direction.z],
                length, mask.rawValue, buffer.baseAddress, Int32(capacity))))
            if query != .all {
                // Preserve the API's nearest-hit semantics for both .nearest and .any without
                // creating entity wrappers for every hit, sorting them, then copying a prefix.
                var nearest: SDFRayHit?
                for index in 0..<count {
                    let hit = buffer[index]
                    guard entity(of: hit.body) != nil else { continue }
                    if nearest == nil || hit.distance < nearest!.distance { nearest = hit }
                }
                guard let hit = nearest, let entity = entity(of: hit.body) else { return [] }
                return [RaycastHit(entity: entity, position: SIMD3(hit.position.0, hit.position.1, hit.position.2),
                                   normal: SIMD3(hit.normal.0, hit.normal.1, hit.normal.2), distance: hit.distance)]
            }
            var hits: [RaycastHit] = []
            hits.reserveCapacity(count)
            for index in 0..<count {
                let hit = buffer[index]
                guard let entity = entity(of: hit.body) else { continue }
                hits.append(RaycastHit(entity: entity, position: SIMD3(hit.position.0, hit.position.1, hit.position.2),
                                       normal: SIMD3(hit.normal.0, hit.normal.1, hit.normal.2), distance: hit.distance))
            }
            hits.sort { $0.distance < $1.distance }
            return hits
        }
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
        guard mask.rawValue != 0 else { return [] }
        let capacity = Int(sdf_world_stats(world).bodies)
        guard capacity > 0 else { return [] }
        let q = transform.rotation.vector
        let pose = SDFPose(position: (transform.translation.x, transform.translation.y, transform.translation.z),
                           orientation: (q.x, q.y, q.z, q.w), scale: transform.scale.x)
        return withUnsafeTemporaryAllocation(of: SDFOverlap.self, capacity: capacity) { buffer in
            let count = min(capacity, Int(sdf_world_overlaps(
                world, shape.handle, pose, margin, mask.rawValue, buffer.baseAddress, Int32(capacity))))
            var hits: [Overlap] = []
            hits.reserveCapacity(count)
            for index in 0..<count {
                let hit = buffer[index]
                guard let entity = entity(of: hit.body) else { continue }
                hits.append(Overlap(entity: entity, position: SIMD3(hit.position.0, hit.position.1, hit.position.2),
                                    normal: SIMD3(hit.normal.0, hit.normal.1, hit.normal.2), separation: hit.separation))
            }
            return hits
        }
    }
}
