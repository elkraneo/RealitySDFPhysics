// Times the same pile of tori with SDFCollisionKit and with RealityKit's physics, from 25 to 400
// bodies, and prints a table.
//
//   swift run -c release sdf-bench [steps]
//   swift run -c release sdf-bench quality [steps]
//   swift run -c release sdf-bench phases [steps] [tori ...]
//   swift run -c release sdf-bench soak [minutes] [tori ...]
//
// `quality` compares SDFCollisionKit's settings, from the paper's to RealityKit-like ones: step time,
// deepest penetration, how still the pile ends, and bodies out of the box; and, for the default
// real-time preset, the bodies asleep at the end and the step time over the last 30 steps.
//
// `phases` splits the step of the real-time preset, without sleeping so that every body keeps
// moving, into collision detection (the narrow phase, on several threads and on one), the rest of
// the engine (the solver, then the broad phase and integration) and the Swift synchronization.
//
// `soak` runs the pile for minutes of simulated time, with sleeping and without, and prints every
// minute the memory footprint, the step time and how far the pile has moved since it settled, to
// catch leaks and drift.
//
// The tori fall into a walled box. For RealityKit, each torus is the best it has for a moving torus:
// a compound of 12 capsules (from a mesh it would generate a convex hull, which closes the hole).
// SDFCollisionKit uses the torus itself. Both run in meters at 60 steps per second; the bodies have
// no meshes, so that rendering costs next to nothing.
//
// SDFCollisionKit's time is that of `SDFSimulation.step(root:)`, synchronization included.
// RealityKit's is measured between its `PhysicsSimulationEvents.WillSimulate` and `DidSimulate`
// events, and cross-checked with an estimate: the time of a frame with the physics clock advancing
// by a step, minus that of a frame with the clock still.

import CoreMedia
import Foundation
import Metal
import RealityKit
import SDFCollisionKit

let mode = CommandLine.arguments.dropFirst().first.flatMap { ["quality", "phases", "soak"].contains($0) ? $0 : nil }
let isQuality = mode == "quality"
let steps = CommandLine.arguments.dropFirst(mode == nil ? 1 : 2).first.flatMap { Int($0) } ?? 300
let countArguments = CommandLine.arguments.dropFirst(3).compactMap { Int($0) }
let counts = [25, 50, 100, 200, 400]
let major: Float = 0.15, minor: Float = 0.04, segments = 12

/// Where the tori start: layers of a 5 × 5 grid above a box 2 m across, tilted
func placements(_ count: Int) -> [(position: SIMD3<Float>, orientation: simd_quatf)] {
    (0..<count).map { i in
        let layer = i / 25, cell = i % 25
        let x = Float(cell % 5) * 0.36 - 0.72, z = Float(cell / 5) * 0.36 - 0.72
        let angle = Float(i % 7) * 0.4
        return ([x, 0.3 + Float(layer) * 0.25, z], simd_quatf(angle: angle, axis: normalize([1, 0, Float(i % 3) - 1])))
    }
}

/// Floor and four walls, 5 m high so that 400 tori stay in, around a box 2 m across: centers and
/// half extents
let walls: [(center: SIMD3<Float>, halfExtents: SIMD3<Float>)] = [
    ([0, -0.5, 0], [2, 0.5, 2]),
    ([-1.05, 2.5, 0], [0.05, 2.5, 1.1]), ([1.05, 2.5, 0], [0.05, 2.5, 1.1]),
    ([0, 2.5, -1.05], [1.1, 2.5, 0.05]), ([0, 2.5, 1.05], [1.1, 2.5, 0.05]),
]

/// Collision detection overrides for experiments: SDF_BENCH_CURVATURE=0 turns curvature sampling
/// off, SDF_BENCH_SAMPLES=n sets the random samples per pair and step
func applyEnvironment(to settings: inout SDFSimulationComponent) {
    let environment = ProcessInfo.processInfo.environment
    if environment["SDF_BENCH_CURVATURE"] == "0" {
        settings.collisionDetection.usesCurvatureSampling = false
    }
    if let samples = environment["SDF_BENCH_SAMPLES"].flatMap({ Int($0) }) {
        settings.collisionDetection.randomSamples = samples
    }
}

@MainActor
func timeSDF(count: Int, isMultithreaded: Bool, preset: SDFSimulationComponent.Preset = .paper)
    -> (mean: Double, worst: Double, check: String) {
    let root = Entity(), simulation = SDFSimulation()
    var settings = SDFSimulationComponent(simulation: simulation, preset: preset)
    settings.isMultithreaded = isMultithreaded
    applyEnvironment(to: &settings)
    root.components.set(settings)
    for wall in walls {
        let entity = Entity()
        entity.position = wall.center
        entity.components.set(SDFBodyComponent(shape: .box(halfExtents: wall.halfExtents), mode: .static))
        root.addChild(entity)
    }
    let torus = SDFShape.torus(majorRadius: major, minorRadius: minor)
    var bodies: [Entity] = []
    for placement in placements(count) {
        let entity = Entity()
        entity.transform = Transform(rotation: placement.orientation, translation: placement.position)
        entity.components.set(SDFBodyComponent(shape: torus))
        root.addChild(entity)
        bodies.append(entity)
    }
    var times: [Double] = []
    for _ in 0..<steps {
        let start = DispatchTime.now().uptimeNanoseconds
        simulation.step(root: root)
        times.append(Double(DispatchTime.now().uptimeNanoseconds - start) / 1e6)
    }
    return (times.reduce(0, +) / Double(times.count), times.max() ?? 0, check(bodies))
}

/// The mean height of the tori and how many left the box, to check that the pile fell and settled
@MainActor
func check(_ bodies: [Entity]) -> String {
    let positions = bodies.map { $0.position(relativeTo: nil) }
    let mean = positions.map(\.y).reduce(0, +) / Float(positions.count)
    let out = positions.filter { abs($0.x) > 1 || abs($0.z) > 1 || $0.y < -0.1 }.count
    return String(format: "mean y %.2f m, %d out", mean, out)
}

@MainActor
func timeRealityKit(count: Int) async throws -> (mean: Double, worst: Double, estimate: Double, check: String) {
    var timebase: CMTimebase?
    CMTimebaseCreateWithSourceClock(allocator: nil, sourceClock: CMClockGetHostTimeClock(), timebaseOut: &timebase)
    guard let timebase else { throw CocoaError(.featureUnsupported) }
    CMTimebaseSetRate(timebase, rate: 0)
    CMTimebaseSetTime(timebase, time: .zero)
    let root = Entity()
    var physics = PhysicsSimulationComponent()
    physics.clock = timebase
    root.components.set(physics)
    for wall in walls {
        let entity = Entity()
        entity.position = wall.center
        let shape = ShapeResource.generateBox(size: 2 * wall.halfExtents)
        entity.components.set(CollisionComponent(shapes: [shape]))
        entity.components.set(PhysicsBodyComponent(shapes: [shape], mass: 1, mode: .static))
        root.addChild(entity)
    }
    let chord = 2 * major * sin(.pi / Float(segments))
    let compound = (0..<segments).map { k in
        let angle = (Float(k) + 0.5) * 2 * .pi / Float(segments)
        return ShapeResource.generateCapsule(height: chord + 2 * minor, radius: minor)
            .offsetBy(rotation: simd_quatf(from: [0, 1, 0], to: [-sin(angle), 0, cos(angle)]),
                      translation: [major * cos(angle), 0, major * sin(angle)])
    }
    var bodies: [Entity] = []
    for placement in placements(count) {
        let entity = Entity()
        entity.transform = Transform(rotation: placement.orientation, translation: placement.position)
        entity.components.set(CollisionComponent(shapes: compound))
        entity.components.set(PhysicsBodyComponent(shapes: compound, density: 1000))
        root.addChild(entity)
        bodies.append(entity)
    }

    let renderer = try RealityRenderer()
    let camera = PerspectiveCamera()
    renderer.entities.append(contentsOf: [root, camera])
    renderer.activeCamera = camera
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm_srgb, width: 8, height: 8, mipmapped: false)
    descriptor.usage = [.renderTarget]
    descriptor.storageMode = .private
    let output = try RealityRenderer.CameraOutput(.singleProjection(colorTexture: MTLCreateSystemDefaultDevice()!.makeTexture(descriptor: descriptor)!))

    // The simulate step, between the events RealityKit sends around it
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
    var times: [Double] = []
    for step in 1...steps { times.append(max(0, try await frame(advancing: true, to: step) - overhead)) }
    guard simulated.count == steps else { throw CocoaError(.featureUnsupported, userInfo: ["steps": simulated.count]) }
    return (simulated.reduce(0, +) / Double(steps), simulated.max() ?? 0, times.reduce(0, +) / Double(steps), check(bodies))
}

/// A setting of the simulation: substeps, velocity and position iterations, and sleeping
struct Setting {
    let name: String, substeps: Int, velocityIterations: Int, positionIterations: Int
    var isSleepingEnabled = false
}

@MainActor
func quality(count: Int, setting: Setting) -> String {
    let root = Entity(), simulation = SDFSimulation()
    var settings = SDFSimulationComponent(simulation: simulation)
    settings.substeps = setting.substeps
    settings.solverIterations = .init(positionIterations: setting.positionIterations, velocityIterations: setting.velocityIterations)
    settings.isSleepingEnabled = setting.isSleepingEnabled
    root.components.set(settings)
    for wall in walls {
        let entity = Entity()
        entity.position = wall.center
        entity.components.set(SDFBodyComponent(shape: .box(halfExtents: wall.halfExtents), mode: .static))
        root.addChild(entity)
    }
    let torus = SDFShape.torus(majorRadius: major, minorRadius: minor)
    var bodies: [Entity] = []
    for placement in placements(count) {
        let entity = Entity()
        entity.transform = Transform(rotation: placement.orientation, translation: placement.position)
        entity.components.set(SDFBodyComponent(shape: torus))
        entity.components.set(SDFMotionComponent())
        root.addChild(entity)
        bodies.append(entity)
    }
    var time = 0.0, endTime = 0.0, deepest: Float = 0, deepestAtRest: Float = 0, endSpeed: Float = 0, fastest: Float = 0, spinning: Float = 0
    for step in 0..<steps {
        let start = DispatchTime.now().uptimeNanoseconds
        simulation.step(root: root)
        let stepTime = Double(DispatchTime.now().uptimeNanoseconds - start) / 1e6
        time += stepTime
        if step >= steps - 30 { endTime += stepTime / 30 }
        // Separation is the gap (> 0) or overlap (< 0) of the surfaces at a contact
        let overlap = -(simulation.contacts().map(\.separation).min() ?? 0)
        deepest = max(deepest, overlap)
        if step >= steps - 30 {
            deepestAtRest = max(deepestAtRest, overlap)
            endSpeed += bodies.map { length($0.components[SDFMotionComponent.self]!.linearVelocity) }.reduce(0, +) / Float(bodies.count) / 30
        }
        if step == steps - 1 {
            fastest = bodies.map { length($0.components[SDFMotionComponent.self]!.linearVelocity) }.max() ?? 0
            spinning = bodies.map { length($0.components[SDFMotionComponent.self]!.angularVelocity) }.max() ?? 0
        }
    }
    let name = setting.name.padding(toLength: 34, withPad: " ", startingAt: 0)
    return name + String(format: " %4d   %6.2f ms  %6.2f ms   %5.1f mm   %5.1f mm   %6.1f mm/s   %4d   ", count,
                         time / Double(steps), endTime, deepest * 1000, deepestAtRest * 1000, endSpeed * 1000,
                         simulation.statistics.sleepingBodies) + check(bodies)
        + String(format: " (fastest %.1f mm/s, %.3f rad/s)", fastest * 1000, spinning)
}

/// The process's physical memory footprint in megabytes
func footprintMB() -> Double {
    var info = task_vm_info_data_t()
    var count = mach_msg_type_number_t(MemoryLayout<task_vm_info_data_t>.size / MemoryLayout<natural_t>.size)
    let result = withUnsafeMutablePointer(to: &info) {
        $0.withMemoryRebound(to: integer_t.self, capacity: Int(count)) { task_info(mach_task_self_, task_flavor_t(TASK_VM_INFO), $0, &count) }
    }
    return result == KERN_SUCCESS ? Double(info.phys_footprint) / 1e6 : .nan
}

/// Minutes of the pile, with sleeping or without: memory, step time, and the drift of the pile
/// since it settled (20 s in)
@MainActor
func soak(count: Int, minutes: Int, isSleepingEnabled: Bool) {
    let root = Entity(), simulation = SDFSimulation()
    var settings = SDFSimulationComponent(simulation: simulation)
    settings.isSleepingEnabled = isSleepingEnabled
    root.components.set(settings)
    for wall in walls {
        let entity = Entity()
        entity.position = wall.center
        entity.components.set(SDFBodyComponent(shape: .box(halfExtents: wall.halfExtents), mode: .static))
        root.addChild(entity)
    }
    let torus = SDFShape.torus(majorRadius: major, minorRadius: minor)
    var bodies: [Entity] = []
    for placement in placements(count) {
        let entity = Entity()
        entity.transform = Transform(rotation: placement.orientation, translation: placement.position)
        entity.components.set(SDFBodyComponent(shape: torus))
        entity.components.set(SDFMotionComponent())
        root.addChild(entity)
        bodies.append(entity)
    }
    print("\(count) tori, sleeping \(isSleepingEnabled ? "on" : "off"): footprint and step time per simulated minute, drift of the pile since 20 s")
    print("minute   footprint   step (last 600)   contacts   asleep   max drift   max turn   max speed   invalid")
    var settled: [Transform] = []
    for minute in 0..<minutes {
        var time = 0.0
        for second in 0..<60 {
            for _ in 0..<60 {
                let start = DispatchTime.now().uptimeNanoseconds
                simulation.step(root: root)
                if second >= 50 { time += Double(DispatchTime.now().uptimeNanoseconds - start) / 1e6 / 600 }
            }
            if minute == 0 && second == 19 { settled = bodies.map(\.transform) }
        }
        var drift: Float = 0, turn: Float = 0, speed: Float = 0, invalid = 0
        for (body, rest) in zip(bodies, settled) {
            let t = body.transform
            if !t.translation.x.isFinite || !t.translation.y.isFinite || !t.translation.z.isFinite { invalid += 1; continue }
            drift = max(drift, simd_distance(t.translation, rest.translation))
            turn = max(turn, abs(simd_quatf(from: rest.rotation.act([0, 1, 0]), to: t.rotation.act([0, 1, 0])).angle))
            speed = max(speed, length(body.components[SDFMotionComponent.self]!.linearVelocity))
        }
        print(String(format: "%6d   %6.1f MB   %8.2f ms       %8d   %6d   %6.2f mm   %6.3f°   %7.2f mm/s   %7d", minute + 1, footprintMB(), time,
                     simulation.statistics.contacts, simulation.statistics.sleepingBodies, drift * 1000, turn * 180 / .pi, speed * 1000, invalid)
            + (invalid > 0 ? "   " + check(bodies) : ""))
    }
    print("   " + check(bodies))
}

if mode == "soak" {
    let minutes = steps == 300 ? 10 : steps
    for count in countArguments.isEmpty ? [100] : countArguments {
        soak(count: count, minutes: minutes, isSleepingEnabled: true)
        soak(count: count, minutes: minutes, isSleepingEnabled: false)
    }
    exit(0)
}

/// Mean milliseconds per step of each phase of the real-time preset, without sleeping
@MainActor
func phases(count: Int, isMultithreaded: Bool) -> (collision: Double, solver: Double, engine: Double, step: Double, contacts: Int) {
    let root = Entity(), simulation = SDFSimulation()
    var settings = SDFSimulationComponent(simulation: simulation)
    settings.isSleepingEnabled = false
    settings.isMultithreaded = isMultithreaded
    applyEnvironment(to: &settings)
    root.components.set(settings)
    for wall in walls {
        let entity = Entity()
        entity.position = wall.center
        entity.components.set(SDFBodyComponent(shape: .box(halfExtents: wall.halfExtents), mode: .static))
        root.addChild(entity)
    }
    let torus = SDFShape.torus(majorRadius: major, minorRadius: minor)
    for placement in placements(count) {
        let entity = Entity()
        entity.transform = Transform(rotation: placement.orientation, translation: placement.position)
        entity.components.set(SDFBodyComponent(shape: torus))
        root.addChild(entity)
    }
    var collision = 0.0, solver = 0.0, engine = 0.0, step = 0.0, contacts = 0
    for _ in 0..<steps {
        let start = DispatchTime.now().uptimeNanoseconds
        simulation.step(root: root)
        step += Double(DispatchTime.now().uptimeNanoseconds - start) / 1e6
        let statistics = simulation.statistics
        collision += statistics.collisionMilliseconds
        solver += statistics.solverMilliseconds
        engine += statistics.stepMilliseconds
        contacts += statistics.contacts
    }
    let n = Double(steps)
    return (collision / n, solver / n, engine / n, step / n, contacts / steps)
}

if mode == "phases" {
    print("ms/step, mean over \(steps) steps: real-time preset without sleeping. Collision is the narrow phase, solver the")
    print("contact solver; the rest of the engine is the broad phase and integration; Swift is the synchronization.")
    print("tori   contacts   collision (threads / one)   solver   rest of engine   Swift   step")
    for count in countArguments.isEmpty ? [100, 400, 800] : countArguments {
        let multi = phases(count: count, isMultithreaded: true), single = phases(count: count, isMultithreaded: false)
        print(String(format: "%4d   %8d   %8.2f / %8.2f   %6.2f   %8.2f   %6.2f   %6.2f", count, multi.contacts,
                     multi.collision, single.collision, multi.solver, multi.engine - multi.collision - multi.solver,
                     multi.step - multi.engine, multi.step))
    }
    exit(0)
}

if isQuality {
    let settings = [
        Setting(name: "paper: 4 substeps, 20/20", substeps: 4, velocityIterations: 20, positionIterations: 20),
        Setting(name: "2 substeps, 10/4", substeps: 2, velocityIterations: 10, positionIterations: 4),
        Setting(name: "real time: 2, 10/4, sleeping", substeps: 2, velocityIterations: 10, positionIterations: 4,
                isSleepingEnabled: true),
        Setting(name: "1 substep, 8/4", substeps: 1, velocityIterations: 8, positionIterations: 4),
        Setting(name: "RealityKit-like: 1, 6/2", substeps: 1, velocityIterations: 6, positionIterations: 2),
    ]
    print("setting                            tori   step       last 30     deepest    at rest    end speed    asleep  check")
    for count in countArguments.isEmpty ? [25, 100, 400] : countArguments {
        for setting in settings where countArguments.isEmpty || setting.name.hasPrefix("real time") {
            print(quality(count: count, setting: setting))
        }
    }
    exit(0)
}

print("ms/step, mean / worst. SDFCollisionKit: the default real-time preset, with sleeping, and the paper's settings,")
print("multithreaded and on one thread. RealityKit: 12-capsule tori, measured between its simulation events.")
print("tori   real time           paper               paper, one thread   RealityKit")
for count in counts {
    let realTime = timeSDF(count: count, isMultithreaded: true, preset: .realTime)
    let sdf = timeSDF(count: count, isMultithreaded: true)
    let single = timeSDF(count: count, isMultithreaded: false)
    let realityKit = try await timeRealityKit(count: count)
    print(String(format: "%4d   %6.2f / %6.2f     %6.2f / %6.2f     %6.2f / %6.2f     %6.2f / %6.2f", count,
                 realTime.mean, realTime.worst, sdf.mean, sdf.worst, single.mean, single.worst, realityKit.mean, realityKit.worst))
    print(String(format: "       after \(steps) steps: real time \(realTime.check), paper \(sdf.check), RealityKit \(realityKit.check) (frame estimate %.2f)",
                 realityKit.estimate))
}
