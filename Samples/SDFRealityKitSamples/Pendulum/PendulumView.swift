// Port of MainView from Apple's sample "Simulating physics joints in your RealityKit app":
// https://developer.apple.com/documentation/realitykit/simulating-physics-joints-in-your-realitykit-app

import RealityKit
import SDFRealityKitSampleScenes
import SwiftUI

struct PendulumView: View {
    @State private var scene = PendulumScene()
    @State private var pendulumCount = 5
    @State private var pushButtonDisabled = false

    @State private var isDragging = false

    var body: some View {
        GeometryReader { geometry in
            RealityView { content in
                scene.reset(pendulumCount: pendulumCount)
                content.add(scene.content)
            }
            .gesture(drag(in: geometry.size))
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

    /// Pulls a ball around with the pointer, and lets it go with its speed
    private func drag(in size: CGSize) -> some Gesture {
        DragGesture(minimumDistance: 0)
            .onChanged { value in
                if !isDragging { isDragging = scene.beginDrag(at: value.startLocation, in: size) }
                if isDragging { scene.drag(to: value.location, in: size) }
            }
            .onEnded { _ in
                if isDragging { scene.endDrag() }
                isDragging = false
            }
    }
}
