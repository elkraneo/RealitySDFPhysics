import CoreMedia
import Metal
import RealityKit
import SDFCollisionKit

/// Milliseconds per step of a run, and a check of where the pile ended
public struct StepTiming: Sendable {
    public var mean: Double
    public var worst: Double
    /// The five slowest steps, as (step, milliseconds)
    public var slowest: [(Int, Double)]
    public var check: String

    public var text: String { String(format: "%6.2f / %6.2f", mean, worst) }
}

/// The parts of a step, in milliseconds, and the work of the last substep
public struct Phases: Sendable {
    public var collision: Double, solver: Double, engine: Double, step: Double
    public var contacts: Int, pairs: Int, iterations: Int
    /// The broad phase, integration and bookkeeping
    public var rest: Double { engine - collision - solver }
    /// The synchronization of the entities
    public var swift: Double { step - engine }
}

/// The pile of tori timed, headless: SDFCollisionKit stepped directly, RealityKit's physics
/// stepped by its clock between its simulation events.
public enum Benchmark {
    /// The steps of the standard table
    public static let steps = 300

    @MainActor
    public static func timeSDF(count: Int, steps: Int = steps, isMultithreaded: Bool,
                               preset: SDFSimulationComponent.Preset = .paper,
                               configure: (inout SDFSimulationComponent) -> Void = { _ in }) -> StepTiming {
        var settings = SDFSimulationComponent(simulation: SDFSimulation(), preset: preset)
        settings.isMultithreaded = isMultithreaded
        configure(&settings)
        let (root, tori) = TorusPile.makeSDF(count: count, settings: settings)
        var times: [Double] = []
        for _ in 0..<steps {
            let start = DispatchTime.now().uptimeNanoseconds
            settings.simulation.step(root: root)
            times.append(Double(DispatchTime.now().uptimeNanoseconds - start) / 1e6)
        }
        let slowest = times.enumerated().sorted { $0.element > $1.element }.prefix(5).map { ($0.offset, $0.element) }
        return StepTiming(mean: times.reduce(0, +) / Double(times.count), worst: times.max() ?? 0, slowest: slowest,
                          check: TorusPile.check(tori))
    }

    /// Mean milliseconds per step of each phase, with the real-time preset without sleeping
    @MainActor
    public static func phases(count: Int, steps: Int = steps, isMultithreaded: Bool,
                              configure: (inout SDFSimulationComponent) -> Void = { _ in }) -> Phases {
        var settings = SDFSimulationComponent(simulation: SDFSimulation())
        settings.isSleepingEnabled = false
        settings.isMultithreaded = isMultithreaded
        configure(&settings)
        let (root, _) = TorusPile.makeSDF(count: count, settings: settings)
        var collision = 0.0, solver = 0.0, engine = 0.0, step = 0.0, contacts = 0, pairs = 0, iterations = 0
        for _ in 0..<steps {
            let start = DispatchTime.now().uptimeNanoseconds
            settings.simulation.step(root: root)
            step += Double(DispatchTime.now().uptimeNanoseconds - start) / 1e6
            let statistics = settings.simulation.statistics
            collision += statistics.collisionMilliseconds
            solver += statistics.solverMilliseconds
            engine += statistics.stepMilliseconds
            contacts += statistics.contacts
            pairs += statistics.pairs
            iterations += statistics.optimizerIterations
        }
        let n = Double(steps)
        return Phases(collision: collision / n, solver: solver / n, engine: engine / n, step: step / n,
                      contacts: contacts / steps, pairs: pairs / steps, iterations: iterations / steps)
    }

    /// RealityKit's physics on the same pile, rendered offscreen to a tiny texture. Returns the
    /// simulate step, measured between the events RealityKit sends around it, and an estimate of
    /// what a step adds to a frame.
    @MainActor
    public static func timeRealityKit(count: Int, steps: Int = steps) async throws -> (timing: StepTiming, frameEstimate: Double) {
        var timebase: CMTimebase?
        CMTimebaseCreateWithSourceClock(allocator: nil, sourceClock: CMClockGetHostTimeClock(), timebaseOut: &timebase)
        guard let timebase, let device = MTLCreateSystemDefaultDevice() else { throw CocoaError(.featureUnsupported) }
        CMTimebaseSetRate(timebase, rate: 0)
        CMTimebaseSetTime(timebase, time: .zero)
        let (root, tori) = TorusPile.makeRealityKit(count: count, clock: timebase)

        let renderer = try RealityRenderer()
        let camera = PerspectiveCamera()
        renderer.entities.append(contentsOf: [root, camera])
        renderer.activeCamera = camera
        let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm_srgb, width: 8, height: 8, mipmapped: false)
        descriptor.usage = [.renderTarget]
        descriptor.storageMode = .private
        guard let texture = device.makeTexture(descriptor: descriptor) else { throw CocoaError(.featureUnsupported) }
        let output = try RealityRenderer.CameraOutput(.singleProjection(colorTexture: texture))

        var willSimulate: UInt64 = 0, simulated: [Double] = []
        let subscriptions = [
            renderer.subscribe(to: PhysicsSimulationEvents.WillSimulate.self) { _ in willSimulate = DispatchTime.now().uptimeNanoseconds },
            renderer.subscribe(to: PhysicsSimulationEvents.DidSimulate.self) { _ in
                simulated.append(Double(DispatchTime.now().uptimeNanoseconds - willSimulate) / 1e6)
            },
        ]
        defer { _ = subscriptions }

        func frame(advancing: Bool, to step: Int) async throws -> Double {
            if advancing { CMTimebaseSetTime(timebase, time: CMTime(value: CMTimeValue(step), timescale: 60)) }
            let start = DispatchTime.now().uptimeNanoseconds
            try await withCheckedThrowingContinuation { (done: CheckedContinuation<Void, any Error>) in
                do { try renderer.updateAndRender(deltaTime: 1.0 / 60, cameraOutput: output, onComplete: { _ in done.resume() }) }
                catch { done.resume(throwing: error) }
            }
            return Double(DispatchTime.now().uptimeNanoseconds - start) / 1e6
        }
        // The cost of a frame without physics, then frames that each take a physics step
        var baseline: [Double] = []
        for _ in 0..<60 { baseline.append(try await frame(advancing: false, to: 0)) }
        let overhead = baseline.sorted()[baseline.count / 2]
        simulated.removeAll()
        var frames: [Double] = []
        for step in 1...steps { frames.append(max(0, try await frame(advancing: true, to: step) - overhead)) }
        guard simulated.count == steps else { throw CocoaError(.featureUnsupported, userInfo: ["steps": simulated.count]) }
        let slowest = simulated.enumerated().sorted { $0.element > $1.element }.prefix(5).map { ($0.offset, $0.element) }
        let timing = StepTiming(mean: simulated.reduce(0, +) / Double(steps), worst: simulated.max() ?? 0, slowest: slowest,
                                check: TorusPile.check(tori))
        return (timing, frames.reduce(0, +) / Double(steps))
    }
}
