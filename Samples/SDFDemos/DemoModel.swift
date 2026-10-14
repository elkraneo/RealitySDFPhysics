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
    /// A new one for every scene load, so that a scene always starts from the same random state
    private(set) var simulation: SDFSimulation
    /// Parent of the bodies, which `SDFSimulationSystem` advances every frame
    @ObservationIgnored let root = Entity()
    @ObservationIgnored let cameraEntity = DemoStage.makeCamera()
    @ObservationIgnored private let contactMarkers = ContactMarkers.makeEntity()
    // Applied to the camera entity directly rather than observed by views
    @ObservationIgnored private var camera = OrbitCamera(for: DemoScene.all[0])

    private(set) var sceneIndex = 0
    private(set) var isLoading = false
    private(set) var errorMessage: String?

    /// The root's simulation settings, applied on change
    var settings: SDFSimulationComponent {
        didSet { root.components.set(settings) }
    }

    var isPaused: Bool {
        get { settings.isPaused }
        set { settings.isPaused = newValue }
    }

    var showsContacts = false {
        didSet { showsContacts ? root.addChild(contactMarkers) : contactMarkers.removeFromParent() }
    }

    var dropShape = "Box"

    var sceneDescription: String { DemoScene.all[sceneIndex].description }

    init() {
        let simulation = SDFSimulation()
        self.simulation = simulation
        settings = Self.paperSettings(simulation)
        root.components.set(settings)
        camera.place(cameraEntity)
    }

    /// The paper's settings on one thread, as in the upstream demo
    private static func paperSettings(_ simulation: SDFSimulation) -> SDFSimulationComponent {
        var settings = SDFSimulationComponent(simulation: simulation)
        settings.isMultithreaded = false
        return settings
    }

    func loadScene(_ index: Int) async {
        sceneIndex = index
        isLoading = true
        defer { isLoading = false }
        // No simulation while the scene is built, then a new one with the same settings, so that
        // loading a scene, or resetting it, always plays out the same
        root.components.remove(SDFSimulationComponent.self)
        do {
            try await DemoScene.all[index].load(into: root)
            errorMessage = nil
        } catch {
            errorMessage = "Could not load \(DemoScene.all[index].name): \(error.localizedDescription)"
        }
        simulation = SDFSimulation()
        settings = settings.with(simulation)
        camera = OrbitCamera(for: DemoScene.all[index])
        camera.place(cameraEntity)
    }

    func reset() async {
        await loadScene(sceneIndex)
    }

    func resetParameters() {
        let isPaused = isPaused
        settings = Self.paperSettings(simulation)
        settings.isPaused = isPaused
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

private extension SDFSimulationComponent {
    /// The same settings for another simulation
    @MainActor
    func with(_ simulation: SDFSimulation) -> SDFSimulationComponent {
        var copy = SDFSimulationComponent(simulation: simulation, metersPerUnit: metersPerUnit)
        copy.gravity = gravity
        copy.solverIterations = solverIterations
        copy.collisionOptions = collisionOptions
        copy.timeStep = timeStep
        copy.substeps = substeps
        copy.maximumStepsPerFrame = maximumStepsPerFrame
        copy.isPaused = isPaused
        copy.restitutionThreshold = restitutionThreshold
        copy.collisionDetection = collisionDetection
        copy.isMultithreaded = isMultithreaded
        return copy
    }
}
