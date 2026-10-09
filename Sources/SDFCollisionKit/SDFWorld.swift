import Foundation
@_exported import SDFCore
import simd

/// A rigid body simulation whose collision detection is SDF-SDF optimization with temporal caching
/// and importance sampling (Giles and Andrews, Pacific Graphics 2026).
///
/// Wraps one upstream `Solver`. Not thread safe; the shape library behind the built-in scenes is
/// shared by all worlds, so create and use worlds on the main actor.
@MainActor
public final class SDFWorld {
    nonisolated(unsafe) let handle: OpaquePointer

    public init() {
        Self.configureResources()
        handle = sdf_world_create()
    }

    deinit {
        sdf_world_destroy(handle)
    }

    // MARK: Scenes

    public struct SceneInfo: Identifiable, Hashable, Sendable {
        public let id: Int
        public let name: String
        public let description: String
    }

    /// The built-in scenes of the upstream demo, in its order
    public static var scenes: [SceneInfo] {
        (0..<Int(sdf_scene_count())).map {
            SceneInfo(id: $0, name: String(cString: sdf_scene_name(Int32($0))),
                      description: String(cString: sdf_scene_description(Int32($0))))
        }
    }

    /// Shapes that can be dropped into a scene by name
    public static var shapeNames: [String] {
        (0..<Int(sdf_shape_name_count())).map { String(cString: sdf_shape_name(Int32($0))) }
    }

    /// Incremented by every scene load, so mirrors of the world know to rebuild
    public private(set) var sceneGeneration = 0

    /// Replaces the world with a built-in scene. Scenes with mesh shapes bake their voxel SDFs on
    /// first use, which takes a moment.
    public func loadScene(_ index: Int) {
        sdf_world_load_scene(handle, Int32(index))
        sceneGeneration += 1
    }

    /// Adds a dynamic body of a named shape with a random orientation
    public func spawn(_ shape: String, at position: SIMD3<Float>) {
        sdf_world_spawn(handle, shape, [position.x, position.y, position.z])
    }

    // MARK: Simulation

    public func step() {
        sdf_world_step(handle)
    }

    public var stats: SDFStats { sdf_world_stats(handle) }

    public var parameters: SDFParams {
        get { sdf_world_params(handle) }
        set { sdf_world_set_params(handle, newValue) }
    }

    public static var defaultParameters: SDFParams { sdf_default_params() }

    /// Orbit camera chosen by the current scene
    public var camera: SDFCamera { sdf_world_camera(handle) }

    // MARK: Bodies and shapes

    public struct Body: Equatable, Sendable {
        /// World position of the shape's local origin
        public var origin: SIMD3<Float>
        public var orientation: simd_quatf
        public var scale: Float
        public var color: SIMD3<Float>
        public var shape: Int
        public var isVisible: Bool
        public var isStatic: Bool
    }

    public var bodyCount: Int { Int(sdf_world_body_count(handle)) }

    public func bodies() -> [Body] {
        let count = bodyCount
        var states = [SDFBodyState](repeating: SDFBodyState(), count: count)
        _ = states.withUnsafeMutableBufferPointer { sdf_world_body_states(handle, $0.baseAddress, Int32(count)) }
        return states.map { s in
            Body(origin: SIMD3(s.origin.0, s.origin.1, s.origin.2),
                 orientation: simd_quatf(ix: s.orientation.0, iy: s.orientation.1, iz: s.orientation.2, r: s.orientation.3),
                 scale: s.scale,
                 color: SIMD3(s.color.0, s.color.1, s.color.2),
                 shape: Int(s.shape), isVisible: s.visible, isStatic: s.isStatic)
        }
    }

    public struct Mesh: Sendable {
        public var positions: [SIMD3<Float>]
        public var normals: [SIMD3<Float>]?
        public var indices: [UInt32]
    }

    /// Triangle mesh of a shape in its local frame
    public func mesh(forShape shape: Int) -> Mesh {
        let m = sdf_world_shape_mesh(handle, Int32(shape))
        func vectors(_ p: UnsafePointer<Float>?) -> [SIMD3<Float>]? {
            guard let p else { return nil }
            return (0..<Int(m.vertexCount)).map { SIMD3(p[3 * $0], p[3 * $0 + 1], p[3 * $0 + 2]) }
        }
        let indices = m.indices.map { Array(UnsafeBufferPointer(start: $0, count: Int(m.indexCount))) } ?? []
        return Mesh(positions: vectors(m.positions) ?? [], normals: vectors(m.normals), indices: indices)
    }

    /// Name of a mesh shape (such as "Bunny"), empty for analytical shapes
    public func shapeName(_ shape: Int) -> String {
        String(cString: sdf_world_shape_name(handle, Int32(shape)))
    }

    // MARK: Contacts

    public struct Contact: Sendable {
        public var position: SIMD3<Float>
        public var normal: SIMD3<Float>
        public var separation: Float
        /// Warm started from the temporal cache, as opposed to a new stochastic sample
        public var isCached: Bool
    }

    /// Active contacts of the last substep
    public func contacts(limit: Int = 4096) -> [Contact] {
        var buffer = [SDFContact](repeating: SDFContact(), count: limit)
        let count = buffer.withUnsafeMutableBufferPointer { sdf_world_contacts(handle, $0.baseAddress, Int32(limit)) }
        return buffer.prefix(min(Int(count), limit)).map { c in
            Contact(position: SIMD3(c.position.0, c.position.1, c.position.2),
                    normal: SIMD3(c.normal.0, c.normal.1, c.normal.2),
                    separation: c.separation, isCached: c.cached)
        }
    }

    // MARK: Dragging

    /// Picks the closest dynamic body on the ray and holds it at the hit point. Returns the hit
    /// point, or nil when the ray misses.
    public func beginDrag(origin: SIMD3<Float>, direction: SIMD3<Float>) -> SIMD3<Float>? {
        var hit: [Float] = [0, 0, 0]
        let body = sdf_world_begin_drag(handle, [origin.x, origin.y, origin.z], [direction.x, direction.y, direction.z], &hit)
        return body >= 0 ? SIMD3(hit[0], hit[1], hit[2]) : nil
    }

    public func drag(to target: SIMD3<Float>) {
        sdf_world_drag_to(handle, [target.x, target.y, target.z])
    }

    public func endDrag() {
        sdf_world_end_drag(handle)
    }

    // MARK: Resources

    private static var resourcesConfigured = false

    /// Points the upstream shape library at the bundled OBJ assets
    private static func configureResources() {
        guard !resourcesConfigured, let directory = Bundle.module.resourceURL else { return }
        sdf_set_resource_directory(directory.path)
        resourcesConfigured = true
    }
}

extension SDFCamera {
    /// Eye position of the orbit camera
    public var eye: SIMD3<Float> {
        let t = SIMD3(target.0, target.1, target.2)
        return t + SIMD3(cos(pitch) * sin(yaw), sin(pitch), cos(pitch) * cos(yaw)) * distance
    }

    public var targetPoint: SIMD3<Float> { SIMD3(target.0, target.1, target.2) }
}
