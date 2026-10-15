import SDFCollisionKit
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
        .task { if CommandLine.arguments.contains("--run") { model.run() } }
    }
}

@MainActor
@Observable
final class BenchmarkModel {
    let device = DeviceInfo.current
    private(set) var isRunning = false
    private(set) var report = ""

    func run() {
        guard !isRunning else { return }
        isRunning = true
        report = ""
        Task { @MainActor in
            await runTable()
            isRunning = false
        }
    }

    private func runTable() async {
        emit("\(device.text), solver threads \(ProcessInfo.processInfo.environment["SDF_SOLVER_THREADS"] ?? "\(device.solverThreads)")")
        emit("ms/step, mean / worst over \(Benchmark.steps) steps. SDFCollisionKit: the real-time preset with sleeping, and the")
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
