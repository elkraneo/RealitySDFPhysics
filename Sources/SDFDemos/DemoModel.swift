import Foundation
import Observation
import RealityKit
import SDFCollisionKit
import SDFDemoScenes
import simd

/// State of the demo window: the simulation root, the selected scene and the orbit camera
@MainActor
@Observable
final class DemoModel {
    @ObservationIgnored let simulation = SDFSimulation()
    /// Parent of the bodies, which `SDFSimulationSystem` advances every frame
    @ObservationIgnored let root = Entity()
    @ObservationIgnored let cameraEntity = DemoStage.makeCamera()
    @ObservationIgnored private let contactMarkers = ContactMarkers.makeEntity()
    // Applied to the camera entity directly rather than observed by views
    @ObservationIgnored private var camera = OrbitCamera(for: DemoScene.all[0])

    private(set) var sceneIndex = 0
    private(set) var isLoading = false
    private(set) var errorMessage: String?

    var isPaused = false {
        didSet { simulation.isPaused = isPaused }
    }

    var showsContacts = false {
        didSet { showsContacts ? root.addChild(contactMarkers) : contactMarkers.removeFromParent() }
    }

    var showsRealityKitStatistics = false

    var dropShape = "Box"

    var parameters = SDFSimulation.Parameters() {
        didSet { simulation.parameters = parameters }
    }

    var sceneDescription: String { DemoScene.all[sceneIndex].description }

    init() {
        root.components.set(SDFSimulationComponent(simulation: simulation))
        camera.place(cameraEntity)
    }

    func loadScene(_ index: Int) async {
        sceneIndex = index
        isLoading = true
        defer { isLoading = false }
        do {
            try await DemoScene.all[index].load(into: root)
            errorMessage = nil
        } catch {
            errorMessage = "Could not load \(DemoScene.all[index].name): \(error.localizedDescription)"
        }
        camera = OrbitCamera(for: DemoScene.all[index])
        camera.place(cameraEntity)
    }

    func reset() async {
        await loadScene(sceneIndex)
    }

    func resetParameters() {
        parameters = SDFSimulation.Parameters()
    }

    func step() {
        simulation.step(root: root)
    }

    /// Drops the selected shape above the camera target, as the Space key of the upstream demo
    func drop() async {
        var dropper = DemoBuilder(root: root, seed: .random(in: 0...UInt64.max))
        do {
            try await dropper.drop(dropShape, above: camera.target)
        } catch {
            errorMessage = "Could not drop \(dropShape): \(error.localizedDescription)"
        }
    }

    /// Cycles through scenes with a slowly orbiting camera, for recording the app
    func runShowcase() async {
        showsRealityKitStatistics = true
        let reel = [(scene: 7, contacts: false), (8, false), (6, false), (1, true), (3, false)]
        while !Task.isCancelled {
            for (scene, contacts) in reel {
                showsContacts = contacts
                await loadScene(scene)
                for _ in 0..<300 { // About 5 s
                    orbit(by: CGSize(width: -0.4, height: 0))
                    try? await Task.sleep(for: .milliseconds(16))
                }
            }
        }
    }

    // MARK: Camera

    func orbit(by delta: CGSize) {
        camera.yaw -= Float(delta.width) * 0.006
        camera.pitch = min(max(camera.pitch + Float(delta.height) * 0.006, -0.2), 1.5)
        camera.place(cameraEntity)
    }

    func zoom(by factor: Float) {
        camera.distance = min(max(camera.distance / factor, 1), 100)
        camera.place(cameraEntity)
    }

    func ray(through point: CGPoint, in size: CGSize) -> (origin: SIMD3<Float>, direction: SIMD3<Float>) {
        camera.ray(through: point, in: size)
    }
}
