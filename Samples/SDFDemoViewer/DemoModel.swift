import Foundation
import Observation
import RealityKit
import RealitySDFPhysics
import SDFDemoScenes
import simd

/// State of the demo window: the simulation root, the selected scene and the orbit camera
@MainActor
@Observable
final class DemoModel {
    let scenes: [DemoScene]
    @ObservationIgnored private let makeInteraction: ((DemoScene) -> DemoInteraction?)?

    /// A new one for every scene load, so that a scene always starts from the same random state
    private(set) var simulation: SDFSimulation
    /// Parent of the bodies, which `SDFSimulationSystem` advances every frame
    @ObservationIgnored let root = Entity()
    @ObservationIgnored let cameraEntity = DemoStage.makeCamera()
    @ObservationIgnored private let contactMarkers = ContactMarkers.makeEntity()
    // Applied to the camera entity directly rather than observed by views
    @ObservationIgnored private var camera: OrbitCamera

    private(set) var sceneIndex = 0
    private(set) var isLoading = false
    private(set) var errorMessage: String?
    /// The scene's own interaction, if it has one, made anew on every load
    @ObservationIgnored private(set) var interaction: DemoInteraction?
    @ObservationIgnored var frameUpdates: EventSubscription?

    /// The root's simulation settings, applied on change
    var settings: SDFPhysicsSimulationComponent {
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

    var sceneDescription: String { scenes[sceneIndex].description }

    init(scenes: [DemoScene], interaction: ((DemoScene) -> DemoInteraction?)?) {
        self.scenes = scenes
        makeInteraction = interaction
        camera = OrbitCamera(for: scenes[0])
        let simulation = SDFSimulation()
        self.simulation = simulation
        settings = Self.paperSettings(simulation)
        root.components.set(settings)
        camera.place(cameraEntity)
    }

    /// The paper's settings on one thread, as in the upstream demo
    private static func paperSettings(_ simulation: SDFSimulation) -> SDFPhysicsSimulationComponent {
        var settings = SDFPhysicsSimulationComponent(simulation: simulation, preset: .paper)
        settings.isMultithreaded = false
        return settings
    }

    func loadScene(_ index: Int) async {
        sceneIndex = index
        isLoading = true
        defer { isLoading = false }
        // No simulation while the scene is built, then a new one with the same settings, so that
        // loading a scene, or resetting it, always plays out the same
        root.components.remove(SDFPhysicsSimulationComponent.self)
        do {
            try await scenes[index].load(into: root)
            errorMessage = nil
        } catch {
            errorMessage = "Could not load \(scenes[index].name): \(error.localizedDescription)"
        }
        simulation = SDFSimulation()
        var settings = settings.with(simulation)
        scenes[index].configure(&settings)
        self.settings = settings
        interaction = makeInteraction?(scenes[index])
        interaction?.attach(root: root, simulation: simulation)
        camera = OrbitCamera(for: scenes[index])
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

private extension SDFPhysicsSimulationComponent {
    /// The same settings for another simulation
    @MainActor
    func with(_ simulation: SDFSimulation) -> SDFPhysicsSimulationComponent {
        var copy = SDFPhysicsSimulationComponent(simulation: simulation, metersPerUnit: metersPerUnit)
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
        copy.isSleepingEnabled = isSleepingEnabled
        return copy
    }
}
