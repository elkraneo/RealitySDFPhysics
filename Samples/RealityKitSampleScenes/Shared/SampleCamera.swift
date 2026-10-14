import CoreGraphics
import RealityKit
import simd

/// A fixed perspective camera, and the world space ray through a point of the view for picking.
/// Apple's samples run in visionOS volumes, which macOS windows replace with a camera.
package struct SampleCamera {
    package var eye: SIMD3<Float>
    package var target: SIMD3<Float>
    package var fieldOfView: Float

    package init(eye: SIMD3<Float>, target: SIMD3<Float> = .zero, fieldOfView: Float = 50) {
        self.eye = eye
        self.target = target
        self.fieldOfView = fieldOfView
    }

    @MainActor
    package func makeEntity() -> PerspectiveCamera {
        let camera = PerspectiveCamera()
        camera.camera.fieldOfViewInDegrees = fieldOfView
        camera.camera.fieldOfViewOrientation = .vertical
        camera.camera.near = 0.01
        camera.look(at: target, from: eye, relativeTo: nil)
        return camera
    }

    package func ray(through point: CGPoint, in size: CGSize) -> (origin: SIMD3<Float>, direction: SIMD3<Float>) {
        let forward = normalize(target - eye)
        let right = normalize(cross(forward, [0, 1, 0]))
        let up = cross(right, forward)
        let t = tan(fieldOfView * .pi / 360)
        let aspect = Float(size.width / max(size.height, 1))
        let x = Float(2 * point.x / max(size.width, 1) - 1)
        let y = Float(1 - 2 * point.y / max(size.height, 1))
        return (eye, normalize(forward + right * (x * t * aspect) + up * (y * t)))
    }
}
