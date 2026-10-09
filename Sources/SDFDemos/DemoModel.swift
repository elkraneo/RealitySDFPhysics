import Foundation
import Observation
import RealityKit
import SDFCollisionKit
import simd

/// State of the demo window: the simulation, the selected scene and the orbit camera
@MainActor
@Observable
final class DemoModel {
    @ObservationIgnored let simulation = SDFSimulation()
    @ObservationIgnored let cameraEntity = SDFDemoStage.makeCamera()

    private(set) var sceneIndex = 0
    private(set) var sceneDescription = ""

    var isPaused = false {
        didSet { simulation.isPaused = isPaused }
    }

    var showsContacts = false {
        didSet { simulation.showsContacts = showsContacts }
    }

    var dropShape = "Box"

    var parameters = SDFWorld.defaultParameters {
        didSet { simulation.world.parameters = parameters }
    }

    // Orbit camera, applied to the camera entity directly rather than observed by views
    @ObservationIgnored private var target: SIMD3<Float> = [0, 1, 0]
    @ObservationIgnored private var distance: Float = 10
    @ObservationIgnored private var yaw: Float = 0.6
    @ObservationIgnored private var pitch: Float = 0.35

    init() {
        loadScene(0)
    }

    func loadScene(_ index: Int) {
        simulation.loadScene(index)
        simulation.world.parameters = parameters
        sceneIndex = index
        sceneDescription = SDFWorld.scenes[index].description
        let camera = simulation.world.camera
        target = camera.targetPoint
        distance = camera.distance
        yaw = camera.yaw
        pitch = camera.pitch
        updateCamera()
    }

    func reset() {
        loadScene(sceneIndex)
    }

    func resetParameters() {
        parameters = SDFWorld.defaultParameters
    }

    /// Drops the selected shape above the camera target, as the Space key of the upstream demo
    func drop() {
        let offset = SIMD3<Float>(.random(in: -0.5...0.5), 4, .random(in: -0.5...0.5))
        simulation.spawn(dropShape, at: target + offset)
    }

    // MARK: Camera

    var eye: SIMD3<Float> {
        target + SIMD3(cos(pitch) * sin(yaw), sin(pitch), cos(pitch) * cos(yaw)) * distance
    }

    func orbit(by delta: CGSize) {
        yaw -= Float(delta.width) * 0.006
        pitch = min(max(pitch + Float(delta.height) * 0.006, -0.2), 1.5)
        updateCamera()
    }

    func zoom(by factor: Float) {
        distance = min(max(distance / factor, 1), 100)
        updateCamera()
    }

    /// World space ray through a point of a view of the given size (y pointing down)
    func ray(through point: CGPoint, in size: CGSize) -> (origin: SIMD3<Float>, direction: SIMD3<Float>) {
        let forward = normalize(target - eye)
        let right = normalize(cross(forward, [0, 1, 0]))
        let up = cross(right, forward)
        let t = tan(SDFDemoStage.fieldOfView * .pi / 360)
        let aspect = Float(size.width / max(size.height, 1))
        let x = Float(2 * point.x / max(size.width, 1) - 1)
        let y = Float(1 - 2 * point.y / max(size.height, 1))
        return (eye, normalize(forward + right * (x * t * aspect) + up * (y * t)))
    }

    private func updateCamera() {
        cameraEntity.look(at: target, from: eye, relativeTo: nil)
    }
}
