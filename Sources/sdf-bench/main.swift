// Times the same pile of tori with RealitySDFPhysics and with RealityKit's physics, from 25 to 400
// bodies, and prints a table.
//
//   swift run -c release sdf-bench [steps]
//   swift run -c release sdf-bench quality [steps]
//   swift run -c release sdf-bench phases [steps] [tori ...]
//   swift run -c release sdf-bench soak [minutes] [tori ...]
//
// `quality` compares RealitySDFPhysics's settings, from the paper's to RealityKit-like ones: step time,
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
// The scene and the timings are those of the SDFBenchmarks module, which the Benchmarks app runs
// on a Mac, an iPhone or a Vision Pro. The tori fall into a walled box. For RealityKit, each torus
// is the best it has for a moving torus: a compound of 12 capsules (from a mesh it would generate
// a convex hull, which closes the hole). RealitySDFPhysics uses the torus itself. Both run in meters
// at 60 steps per second; the bodies have no meshes, so that rendering costs next to nothing.
//
// RealitySDFPhysics's time is that of `SDFSimulation.step(root:)`, synchronization included.
// RealityKit's is measured between its `PhysicsSimulationEvents.WillSimulate` and `DidSimulate`
// events, and cross-checked with an estimate: the time of a frame with the physics clock advancing
// by a step, minus that of a frame with the clock still.

import Foundation
import RealityKit
import SDFBenchmarks
import RealitySDFPhysics

let mode = CommandLine.arguments.dropFirst().first.flatMap { ["quality", "phases", "soak"].contains($0) ? $0 : nil }
let steps = CommandLine.arguments.dropFirst(mode == nil ? 1 : 2).first.flatMap { Int($0) } ?? Benchmark.steps
let countArguments = CommandLine.arguments.dropFirst(3).compactMap { Int($0) }
let environment = ProcessInfo.processInfo.environment

/// Collision detection overrides for experiments: SDF_BENCH_CURVATURE=0 turns curvature sampling
/// off, SDF_BENCH_SAMPLES=n sets the random samples per pair and step, SDF_BENCH_ITERATIONS=n the
/// optimizer's limit and SDF_BENCH_CACHE=n the cached points
func applyEnvironment(to settings: inout SDFSimulationComponent) {
    if environment["SDF_BENCH_CURVATURE"] == "0" {
        settings.collisionDetection.usesCurvatureSampling = false
    }
    if let samples = environment["SDF_BENCH_SAMPLES"].flatMap({ Int($0) }) {
        settings.collisionDetection.randomSamples = samples
    }
    if let iterations = environment["SDF_BENCH_ITERATIONS"].flatMap({ Int($0) }) {
        settings.collisionDetection.maxIterations = iterations
    }
    if let cache = environment["SDF_BENCH_CACHE"].flatMap({ Int($0) }) {
        settings.collisionDetection.cachedSamples = cache
    }
}

/// A setting of the simulation: substeps, velocity and position iterations, and sleeping
struct Setting {
    let name: String, substeps: Int, velocityIterations: Int, positionIterations: Int
    var isSleepingEnabled = false
}

@MainActor
func quality(count: Int, setting: Setting) -> String {
    let simulation = SDFSimulation()
    var settings = SDFSimulationComponent(simulation: simulation)
    settings.substeps = setting.substeps
    settings.solverIterations = .init(positionIterations: setting.positionIterations, velocityIterations: setting.velocityIterations)
    settings.isSleepingEnabled = setting.isSleepingEnabled
    let (root, bodies) = TorusPile.makeSDF(count: count, settings: settings, motion: true)
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
                         simulation.statistics.sleepingBodies) + TorusPile.check(bodies)
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
    let simulation = SDFSimulation()
    var settings = SDFSimulationComponent(simulation: simulation)
    settings.isSleepingEnabled = isSleepingEnabled
    let (root, bodies) = TorusPile.makeSDF(count: count, settings: settings, motion: true)
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
            + (invalid > 0 ? "   " + TorusPile.check(bodies) : ""))
    }
    print("   " + TorusPile.check(bodies))
}

if mode == "soak" {
    let minutes = steps == Benchmark.steps ? 10 : steps
    for count in countArguments.isEmpty ? [100] : countArguments {
        soak(count: count, minutes: minutes, isSleepingEnabled: true)
        soak(count: count, minutes: minutes, isSleepingEnabled: false)
    }
    exit(0)
}

if mode == "phases" {
    print("ms/step, mean over \(steps) steps: real-time preset without sleeping. Collision is the narrow phase, solver the")
    print("contact solver; the rest of the engine is the broad phase and integration; Swift is the synchronization.")
    print("Pairs and contacts are those of the last substep; iterations are the optimizer's, per pair and substep.")
    print("tori   pairs   contacts   iterations   collision (threads / one)   solver   rest of engine   Swift   step")
    for count in countArguments.isEmpty ? [100, 400, 800] : countArguments {
        let multi = Benchmark.phases(count: count, steps: steps, isMultithreaded: true, configure: applyEnvironment)
        let single = Benchmark.phases(count: count, steps: steps, isMultithreaded: false, configure: applyEnvironment)
        let substeps = Double(SDFSimulationComponent().substeps)
        print(String(format: "%4d   %5d   %8d   %10.1f   %8.2f / %8.2f   %6.2f   %8.2f   %6.2f   %6.2f", count,
                     multi.pairs, multi.contacts, Double(multi.iterations) / Double(max(1, multi.pairs)) / substeps,
                     multi.collision, single.collision, multi.solver, multi.rest, multi.swift, multi.step))
    }
    exit(0)
}

if mode == "quality" {
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

/// The slowest steps of a run, with SDF_BENCH_TRACE=1
func trace(_ timing: StepTiming, _ name: String) {
    guard environment["SDF_BENCH_TRACE"] != nil else { return }
    print("       slowest steps (\(name)): " + timing.slowest.map { String(format: "#%d %.1f ms", $0.0, $0.1) }.joined(separator: ", "))
}

print(DeviceInfo.current.text)
print("ms/step, mean / worst. RealitySDFPhysics: the default real-time preset, with sleeping, and the paper's settings,")
print("multithreaded and on one thread. RealityKit: 12-capsule tori, measured between its simulation events.")
print("tori   real time           paper               paper, one thread   RealityKit")
for count in TorusPile.counts {
    let realTime = Benchmark.timeSDF(count: count, steps: steps, isMultithreaded: true, preset: .realTime, configure: applyEnvironment)
    trace(realTime, "real time")
    let sdf = Benchmark.timeSDF(count: count, steps: steps, isMultithreaded: true, configure: applyEnvironment)
    trace(sdf, "paper, threads")
    let single = Benchmark.timeSDF(count: count, steps: steps, isMultithreaded: false, configure: applyEnvironment)
    trace(single, "paper, one thread")
    let realityKit = try await Benchmark.timeRealityKit(count: count, steps: steps)
    print(String(format: "%4d   %@     %@     %@     %@", count, realTime.text, sdf.text, single.text, realityKit.timing.text))
    print(String(format: "       after \(steps) steps: real time \(realTime.check), paper \(sdf.check), RealityKit \(realityKit.timing.check) (frame estimate %.2f)",
                 realityKit.frameEstimate))
}
