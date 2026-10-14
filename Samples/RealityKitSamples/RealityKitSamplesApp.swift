// Apple's RealityKit physics samples, ported from RealityKit's physics to SDF bodies. Each sample
// keeps the structure and values of Apple's, and says where and why it differs.

import RealityKit
import SwiftUI

@main
struct RealityKitSamplesApp: App {
    init() {
        AttractionComponent.registerComponent()
        AttractionSystem.registerSystem()
    }

    var body: some SwiftUI.Scene {
        WindowGroup("RealityKit Samples on SDF Bodies") {
            SamplesView()
        }
        .defaultSize(width: 1200, height: 800)
    }
}

private enum Sample: String, CaseIterable, Identifiable {
    case physicsBodies = "Physics Bodies"

    var id: Self { self }

    var source: String {
        switch self {
        case .physicsBodies: "Simulating physics with collisions in your visionOS app"
        }
    }
}

private struct SamplesView: View {
    @State private var selection: Sample? = .physicsBodies

    var body: some View {
        NavigationSplitView {
            List(Sample.allCases, selection: $selection) { sample in
                VStack(alignment: .leading) {
                    Text(sample.rawValue)
                    Text(sample.source).font(.caption).foregroundStyle(.secondary)
                }
            }
            .navigationSplitViewColumnWidth(min: 220, ideal: 260)
        } detail: {
            switch selection {
            case .physicsBodies: PhysicsBodiesView()
            case nil: Text("Select a sample")
            }
        }
    }
}
