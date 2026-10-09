import RealityKit
import simd

/// Floor, light and camera matching the upstream demo's presentation. The ground body of every
/// scene is invisible, upstream draws an infinite floor at y = 0 instead.
@MainActor
public enum SDFDemoStage {
    /// Vertical field of view of the upstream camera
    public static let fieldOfView: Float = 45

    public static func makeFloor() -> Entity {
        var material = PhysicallyBasedMaterial()
        material.baseColor = .init(tint: PlatformColor(white: 0.82, alpha: 1))
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

    public static func place(_ camera: Entity, at orbit: SDFCamera) {
        camera.look(at: orbit.targetPoint, from: orbit.eye, relativeTo: nil)
    }
}
