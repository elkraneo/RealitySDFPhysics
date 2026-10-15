import RealityKit
import RealitySDFPhysics
import SDFDemoScenes
import SDFMechanicsScenes
import SwiftUI

/// Runs the standard table of `sdf-bench` on the machine at hand and shows it: the pile of tori
/// at each count, with the real-time preset, the paper's settings on threads and on one thread,
/// and RealityKit's physics. The report is also printed, for the console.
public struct BenchmarkView: View {
    @State private var model = BenchmarkModel()

    public init() {}

    public var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 16) {
                Text(model.device.text)
                    .font(.footnote)
                    .foregroundStyle(.secondary)
                HStack {
                    Button(model.isRunning ? "Running…" : "Run") { model.run() }
                        .disabled(model.isRunning)
                    Button("Scenes") { model.run(scenes: true) }
                        .disabled(model.isRunning)
                    Button("Copy") { model.copyReport() }
                        .disabled(model.report.isEmpty || model.isRunning)
                    if model.isRunning { ProgressView() }
                }
                Text(model.report.isEmpty ? "ms per step, mean / worst, over \(Benchmark.steps) steps." : model.report)
                    .font(.system(.footnote, design: .monospaced))
                    .textSelection(.enabled)
                    .frame(maxWidth: .infinity, alignment: .leading)
            }
            .padding()
        }
        .navigationTitle("SDF Benchmarks")
        // Launched with --run, as from a Mac over devicectl, the table runs and prints by itself
        .task {
            if CommandLine.arguments.contains("--run") { model.run() }
            if CommandLine.arguments.contains("--scenes") { model.run(scenes: true) }
        }
    }
}

@MainActor
@Observable
final class BenchmarkModel {
    let device = DeviceInfo.current
    private(set) var isRunning = false
    private(set) var report = ""

    func run(scenes: Bool = false) {
        guard !isRunning else { return }
        isRunning = true
        report = ""
        Task { @MainActor in
            if scenes { await runScenes() } else { await runTable() }
            isRunning = false
        }
    }

    /// The mechanics scenes stepped with the real-time preset: step time, mean and worst
    private func runScenes() async {
        emit("\(device.text)")
        emit("ms/step, mean / worst over 300 steps, real-time preset. Load is the baking.")
        emit("scene         bodies   load ms   step   contacts")
        for scene in MechanicsScenes.all {
            await Task.yield()
            let simulation = SDFSimulation(), root = Entity()
            var settings = SDFSimulationComponent(simulation: simulation)
            settings.isSleepingEnabled = false
            root.components.set(settings)
            let loadStart = Date()
            do { try await scene.load(into: root) } catch { emit("\(scene.name): \(error)"); continue }
            let load = Date().timeIntervalSince(loadStart) * 1000
            var times: [Double] = []
            for _ in 0..<300 {
                let start = DispatchTime.now().uptimeNanoseconds
                simulation.step(root: root)
                times.append(Double(DispatchTime.now().uptimeNanoseconds - start) / 1e6)
            }
            let stats = simulation.statistics
            emit(String(format: "%-13@ %6d %9.0f   %5.2f / %5.2f   %5d", scene.name, stats.bodies, load,
                        times.reduce(0, +) / Double(times.count), times.max() ?? 0, stats.contacts))
        }
        emit("thermal state at the end: \(DeviceInfo.current.thermalState)")
    }

    private func runTable() async {
        emit("\(device.text), solver threads \(ProcessInfo.processInfo.environment["SDF_SOLVER_THREADS"] ?? "\(device.solverThreads)")")
        emit("ms/step, mean / worst over \(Benchmark.steps) steps. RealitySDFPhysics: the real-time preset with sleeping, and the")
        emit("paper's settings on threads and on one thread. RealityKit: 12-capsule tori, between its simulation events.")
        emit("tori   real time           paper               paper, one thread   RealityKit")
        for count in TorusPile.counts {
            // Yield between runs so the view can show the rows so far
            await Task.yield()
            let realTime = Benchmark.timeSDF(count: count, isMultithreaded: true, preset: .realTime)
            await Task.yield()
            let paper = Benchmark.timeSDF(count: count, isMultithreaded: true)
            await Task.yield()
            let single = Benchmark.timeSDF(count: count, isMultithreaded: false)
            await Task.yield()
            let realityKit: String, realityKitCheck: String
            do {
                let result = try await Benchmark.timeRealityKit(count: count)
                realityKit = result.timing.text
                realityKitCheck = result.timing.check
            } catch {
                realityKit = "failed"
                realityKitCheck = "\(error)"
            }
            emit(String(format: "%4d   %@     %@     %@     %@", count, realTime.text, paper.text, single.text, realityKit))
            emit("       after \(Benchmark.steps) steps: real time \(realTime.check), paper \(paper.check), RealityKit \(realityKitCheck)")
        }
        emit("thermal state at the end: \(DeviceInfo.current.thermalState)")
    }

    private func emit(_ line: String) {
        report += (report.isEmpty ? "" : "\n") + line
        print(line)
    }

    func copyReport() {
        #if os(macOS)
        NSPasteboard.general.clearContents()
        NSPasteboard.general.setString(report, forType: .string)
        #else
        UIPasteboard.general.string = report
        #endif
    }
}
