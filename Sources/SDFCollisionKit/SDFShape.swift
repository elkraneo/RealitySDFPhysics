internal import SDFCore
import simd

/// The geometry of a body: a signed distance field, negative inside.
///
/// Shapes are immutable, so they can be created on any thread and shared by any number of bodies.
/// Every shape is given in the frame of the entity it is attached to.
public final class SDFShape: @unchecked Sendable {
    // The engine never mutates a shape after creating it
    let handle: OpaquePointer

    private init(_ handle: OpaquePointer) {
        self.handle = handle
    }

    deinit {
        sdf_shape_release(handle)
    }

    // MARK: Analytical shapes

    public static func box(halfExtents: SIMD3<Float>) -> SDFShape {
        SDFShape(sdf_shape_box(halfExtents.x, halfExtents.y, halfExtents.z))
    }

    public static func sphere(radius: Float) -> SDFShape {
        SDFShape(sdf_shape_sphere(radius))
    }

    /// Capsule along y; `halfLength` is that of the cylindrical part
    public static func capsule(radius: Float, halfLength: Float) -> SDFShape {
        SDFShape(sdf_shape_capsule(radius, halfLength))
    }

    /// Cylinder along y
    public static func cylinder(radius: Float, halfHeight: Float) -> SDFShape {
        SDFShape(sdf_shape_cylinder(radius, halfHeight))
    }

    /// Cone along y with its apex at +halfHeight
    public static func cone(radius: Float, halfHeight: Float) -> SDFShape {
        SDFShape(sdf_shape_cone(radius, halfHeight))
    }

    /// Torus around y
    public static func torus(majorRadius: Float, minorRadius: Float) -> SDFShape {
        SDFShape(sdf_shape_torus(majorRadius, minorRadius))
    }

    /// Box with a `count` × `count` grid of cone shaped spikes of varying height on its top face
    public static func spikePlate(halfExtents: SIMD3<Float>, count: Int, spikeRadius: Float,
                                  spikeHeights: ClosedRange<Float>) -> SDFShape {
        SDFShape(sdf_shape_spike_plate(halfExtents.x, halfExtents.y, halfExtents.z, Int32(count), spikeRadius,
                                       spikeHeights.lowerBound, spikeHeights.upperBound))
    }

    /// Cylindrical hub around y with two rings of radial cone shaped spikes
    public static func spikyWheel(hubRadius: Float, hubHalfWidth: Float, spikeRadius: Float, spikeLength: Float,
                                  spikes: Int) -> SDFShape {
        SDFShape(sdf_shape_spiky_wheel(hubRadius, hubHalfWidth, spikeRadius, spikeLength, Int32(spikes)))
    }

    /// The distortion operators of the paper (Sec. 4.5)
    public enum Warp: Sendable {
        case bend, twist, displace
    }

    /// A box distorted by an operator of strength `amount`
    public static func warpedBox(_ warp: Warp, halfExtents: SIMD3<Float>, amount: Float) -> SDFShape {
        let type: SDFWarp = switch warp {
        case .bend: SDFWarpBend
        case .twist: SDFWarpTwist
        case .displace: SDFWarpDisplace
        }
        return SDFShape(sdf_shape_warped_box(type, halfExtents.x, halfExtents.y, halfExtents.z, amount))
    }

    // MARK: Baked shapes

    public enum BakeError: Error {
        /// The mesh has no triangles, an index out of range or no extent
        case invalidMesh
    }

    /// Bakes a closed triangle mesh into a voxel SDF with `resolution` cells across its longest
    /// side. Takes up to about a second, so call it off the main actor for large meshes.
    public convenience init(baking positions: [SIMD3<Float>], indices: [UInt32], resolution: Int = 100) throws {
        let flat = positions.flatMap { [$0.x, $0.y, $0.z] }
        guard let handle = sdf_shape_bake(flat, Int32(positions.count), indices, Int32(indices.count), Int32(resolution))
        else { throw BakeError.invalidMesh }
        self.init(handle)
    }

    /// False for baked meshes with open edges, whose inside and outside may be ambiguous
    public var isWatertight: Bool { sdf_shape_is_watertight(handle) }

    /// True for the convex analytical shapes, and for meshes baked from a closed convex mesh. Two
    /// convex bodies touch in a single region, so their collision detection skips the random
    /// samples that look for others while their contact is tracked, which keeps bodies resting on
    /// a flat face still.
    public var isConvex: Bool { sdf_shape_is_convex(handle) }

    /// Enclosed volume. A dynamic body's default mass is that of this volume of water (1000 per
    /// unit volume) at its entity's scale.
    public var volume: Float { sdf_shape_volume(handle) }

    // MARK: Queries

    /// Signed distance from a point to the surface (negative inside) and its gradient, the
    /// outward direction
    public func distance(at point: SIMD3<Float>) -> (distance: Float, gradient: SIMD3<Float>) {
        var gradient: [Float] = [0, 0, 0]
        let distance = sdf_shape_distance(handle, [point.x, point.y, point.z], &gradient)
        return (distance, SIMD3(gradient[0], gradient[1], gradient[2]))
    }

    /// Triangle mesh of the surface, for display
    public struct Surface: Sendable {
        public var positions: [SIMD3<Float>]
        public var normals: [SIMD3<Float>]?
        public var indices: [UInt32]
    }

    public var surface: Surface {
        let mesh = sdf_shape_mesh(handle)
        let count = Int(mesh.vertexCount)
        func vectors(_ p: UnsafePointer<Float>?) -> [SIMD3<Float>]? {
            p.map { p in (0..<count).map { SIMD3(p[3 * $0], p[3 * $0 + 1], p[3 * $0 + 2]) } }
        }
        let indices = mesh.indices.map { Array(UnsafeBufferPointer(start: $0, count: Int(mesh.indexCount))) } ?? []
        return Surface(positions: vectors(mesh.positions) ?? [], normals: vectors(mesh.normals), indices: indices)
    }
}
