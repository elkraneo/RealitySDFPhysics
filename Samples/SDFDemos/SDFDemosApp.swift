// The 11 scenes of the paper's demo, rebuilt on the public API

import AppKit
import SDFDemoScenes
import SDFDemoViewer
import SwiftUI

@main
struct SDFDemosApp: App {
    @NSApplicationDelegateAdaptor private var appDelegate: AppDelegate

    var body: some Scene {
        WindowGroup("SDF Collision Demos") {
            DemoViewer(scenes: DemoScene.all)
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
