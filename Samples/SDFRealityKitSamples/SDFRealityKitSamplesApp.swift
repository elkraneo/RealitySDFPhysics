// Apple's RealityKit physics samples, ported from RealityKit's physics to SDF bodies. Each sample
// keeps the structure and values of Apple's, and says where and why it differs.

import RealityKit
import SDFRealityKitSampleScenes
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
    case dice = "Dice"
    case pendulum = "Newton's Cradle"

    var id: Self { self }

    /// How this port differs from Apple's sample
    var differences: [String] {
        switch self {
        case .physicsBodies: [
            "SDF bodies in place of PhysicsBodyComponent and CollisionComponent; the same sizes, masses and forces.",
            "Shape swaps the spheres for concave tori, which RealityKit's physics can't move.",
            "A fixed 1 m volume on macOS. Dragging picks with the simulation's raycast and pulls on every step.",
        ]
        case .dice: [
            "SDF bodies baked from Apple's dice models, in place of TabletopKit, which is visionOS only.",
            "The roll is forecast from an exact copy of the simulation, which is deterministic; Max score toss turns each die by a symmetry to land on its highest face, without adjusting the roll.",
            "Invisible walls keep the dice on the table; the pointer holds a die with a spring, and releases it with its velocity.",
        ]
        case .pendulum: [
            "SDF bodies and revolute joints in place of RealityKit's; Apple's sizes, masses, materials and slow-motion scale.",
            "It behaves as a Newton's cradle: one ball in, one out; two in, two out. RealityKit shares the bounce among the touching balls, so one ball in sends four out.",
            "The contact margin and restitution threshold are sized for 4 cm balls. Lift lets one to three balls go from 30 degrees.",
        ]
        }
    }

    /// Apple's sample this one ports
    var source: (title: String, url: URL) {
        switch self {
        case .physicsBodies:
            ("Simulating physics with collisions in your visionOS app",
             URL(string: "https://developer.apple.com/documentation/realitykit/simulating-physics-with-collisions-in-your-visionos-app")!)
        case .dice:
            ("Simulating dice rolls as a component for your game",
             URL(string: "https://developer.apple.com/documentation/tabletopkit/simulating-dice-rolls-as-a-component-for-your-game")!)
        case .pendulum:
            ("Simulating physics joints in your RealityKit app",
             URL(string: "https://developer.apple.com/documentation/realitykit/simulating-physics-joints-in-your-realitykit-app")!)
        }
    }
}

private struct SamplesView: View {
    /// The first sample, or the one named after --sample at launch
    @State private var selection: Sample? = {
        let arguments = CommandLine.arguments
        guard let index = arguments.firstIndex(of: "--sample"), index + 1 < arguments.count else { return .physicsBodies }
        return Sample.allCases.first { $0.rawValue.caseInsensitiveCompare(arguments[index + 1]) == .orderedSame } ?? .physicsBodies
    }()

    var body: some View {
        NavigationSplitView {
            List(Sample.allCases, selection: $selection) { sample in
                VStack(alignment: .leading) {
                    Text(sample.rawValue)
                    Text(sample.source.title).font(.caption).foregroundStyle(.secondary)
                }
            }
            .navigationSplitViewColumnWidth(min: 220, ideal: 260)
        } detail: {
            if let selection {
                VStack(spacing: 0) {
                    switch selection {
                    case .physicsBodies: PhysicsBodiesView()
                    case .dice: DiceView()
                    case .pendulum: PendulumView()
                    }
                    Divider()
                    SampleReference(sample: selection)
                }
            } else {
                Text("Select a sample")
            }
        }
    }
}

/// A bar naming the Apple sample a port reimplements, with a link to it and its differences
private struct SampleReference: View {
    let sample: Sample
    @State private var showsDifferences = false

    var body: some View {
        HStack(spacing: 12) {
            Text("Port of Apple's sample").foregroundStyle(.secondary)
            Text(sample.source.title).fontWeight(.semibold)
            Spacer()
            Button("Differences", systemImage: "arrow.triangle.branch") { showsDifferences.toggle() }
                .popover(isPresented: $showsDifferences, arrowEdge: .top) {
                    VStack(alignment: .leading, spacing: 8) {
                        ForEach(sample.differences, id: \.self) { Label($0, systemImage: "arrow.triangle.branch") }
                    }
                    .padding()
                    .frame(width: 420)
                }
            Link(destination: sample.source.url) {
                Label("Open Apple's Sample", systemImage: "arrow.up.right.square")
            }
            .buttonStyle(.borderedProminent)
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 10)
    }
}
