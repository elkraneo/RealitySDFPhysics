import AppKit
import SDFBenchmarks
import SwiftUI

@main
struct BenchmarksApp: App {
    @NSApplicationDelegateAdaptor private var appDelegate: AppDelegate

    var body: some Scene {
        WindowGroup("SDF Benchmarks") {
            BenchmarkView()
                .frame(minWidth: 720, minHeight: 420)
        }
    }
}

/// Run from `swift run`, the executable is not an app bundle, so it has to ask to be a regular,
/// frontmost app to get a Dock icon and keyboard focus.
final class AppDelegate: NSObject, NSApplicationDelegate {
    func applicationDidFinishLaunching(_ notification: Notification) {
        NSApplication.shared.setActivationPolicy(.regular)
        NSApplication.shared.activate()
    }

    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }
}
