// Mechanisms on the joints: gears, chains, the Absolut chain with its pedals, a planetary set

import AppKit
import SDFDemoViewer
import SDFMechanicsScenes
import SwiftUI

@main
struct SDFMechanicsApp: App {
    @NSApplicationDelegateAdaptor private var appDelegate: AppDelegate

    var body: some Scene {
        WindowGroup("SDF Mechanics") {
            DemoViewer(scenes: MechanicsScenes.all, interaction: { $0.name == "Absolut" ? AbsolutInteraction() : nil })
                .frame(minWidth: 960, minHeight: 600)
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
