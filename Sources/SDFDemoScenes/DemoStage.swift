import CoreGraphics
import RealityKit
import simd

/// Floor, light and orbit camera matching the upstream demo's presentation
@MainActor
public enum DemoStage {
    /// Vertical field of view of the upstream camera
    public nonisolated static let fieldOfView: Float = 45

    /// The ground body of every scene is invisible; upstream draws a floor at y = 0 instead
    public static func makeFloor() -> Entity {
        var material = PhysicallyBasedMaterial()
        material.baseColor = .init(tint: .init(white: 0.82, alpha: 1))
        material.roughness = 0.9
        let floor = ModelEntity(mesh: .generatePlane(width: 60, depth: 60), materials: [material])
        floor.name = "Floor"
        return floor
    }

    public static func makeLight() -> Entity {
        let light = DirectionalLight()
        light.light.intensity = 2500
        light.shadow = DirectionalLightComponent.Shadow(maximumDistance: 40, depthBias: 2)
        light.look(at: .zero, from: [4, 10, 6], relativeTo: nil)
        return light
    }

    public static func makeCamera() -> PerspectiveCamera {
        let camera = PerspectiveCamera()
        camera.camera.fieldOfViewInDegrees = fieldOfView
        camera.camera.fieldOfViewOrientation = .vertical
        camera.camera.near = 0.05
        camera.camera.far = 500
        return camera
    }
}

/// Camera orbiting a target, as in the upstream demo
public struct OrbitCamera: Equatable, Sendable {
    public var target: SIMD3<Float>
    public var distance: Float
    public var yaw: Float = 0.6
    public var pitch: Float = 0.35

    public init(for scene: DemoScene) {
        target = scene.target
        distance = scene.distance
    }

    public var eye: SIMD3<Float> {
        target + SIMD3(cos(pitch) * sin(yaw), sin(pitch), cos(pitch) * cos(yaw)) * distance
    }

    @MainActor
    public func place(_ camera: Entity) {
        camera.look(at: target, from: eye, relativeTo: nil)
    }

    /// Ray through a point of a view of the given size (y pointing down)
    public func ray(through point: CGPoint, in size: CGSize) -> (origin: SIMD3<Float>, direction: SIMD3<Float>) {
        let forward = normalize(target - eye)
        let right = normalize(cross(forward, [0, 1, 0]))
        let up = cross(right, forward)
        let t = tan(DemoStage.fieldOfView * .pi / 360)
        let aspect = Float(size.width / max(size.height, 1))
        let x = Float(2 * point.x / max(size.width, 1) - 1)
        let y = Float(1 - 2 * point.y / max(size.height, 1))
        return (eye, normalize(forward + right * (x * t * aspect) + up * (y * t)))
    }
}
