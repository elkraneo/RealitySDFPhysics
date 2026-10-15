// Port of MainView from Apple's sample "Simulating physics joints in your RealityKit app":
// https://developer.apple.com/documentation/realitykit/simulating-physics-joints-in-your-realitykit-app

import RealityKit
import SDFRealityKitSampleScenes
import SwiftUI

struct PendulumView: View {
    @State private var scene = PendulumScene()
    @State private var pendulumCount = 5
    @State private var pushButtonDisabled = false

    var body: some View {
        RealityView { content in
            scene.reset(pendulumCount: pendulumCount)
            content.add(scene.content)
        }
        .toolbar {
            Picker("Pendulums", selection: $pendulumCount) {
                ForEach([1, 3, 5], id: \.self) { Text("\($0)").tag($0) }
            }
            .pickerStyle(.segmented)
            Menu("Lift", systemImage: "arrow.up.left") {
                ForEach(1..<pendulumCount, id: \.self) { count in
                    Button(count == 1 ? "One ball" : "\(count) balls") { scene.lift(count) }
                }
            }
            .disabled(pendulumCount < 2)
            Button("Push ball", systemImage: "hand.point.right") {
                pushButtonDisabled = true
                scene.pushBall()
                Task {
                    try? await Task.sleep(for: .seconds(0.5))
                    pushButtonDisabled = false
                }
            }
            .disabled(pushButtonDisabled)
            Button("Reset", systemImage: "arrow.counterclockwise") { scene.reset(pendulumCount: pendulumCount) }
        }
        .onChange(of: pendulumCount) { scene.reset(pendulumCount: pendulumCount) }
    }
}
