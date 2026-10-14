// Port of MainView from Apple's sample "Simulating physics with collisions in your visionOS app":
// https://developer.apple.com/documentation/realitykit/simulating-physics-with-collisions-in-your-visionos-app
//
// Apple's sample drags with entity-targeted gestures, which hit-test RealityKit collision shapes.
// SDF bodies have none, so the drag picks with the simulation's own raycast.

import RealityKit
import SDFCollisionKit
import SwiftUI

struct PhysicsBodiesView: View {
    @State private var scene = PhysicsBodiesScene()
    @State private var shape = PhysicsBodiesScene.Shape.spheres
    @State private var drag: (entity: Entity, distance: Float)?

    var body: some View {
        GeometryReader { geometry in
            ZStack {
                RealityView { content in
                    scene.reset(shape: shape)
                    content.add(scene.content)
                }
                // Gestures on a SwiftUI layer: on macOS, RealityView keeps the mouse events
                Color.clear
                    .contentShape(.rect)
                    .gesture(forceDrag(in: geometry.size))
            }
        }
        .toolbar {
            Picker("Shape", selection: $shape) {
                ForEach(PhysicsBodiesScene.Shape.allCases) { Text($0.rawValue).tag($0) }
            }
            .pickerStyle(.segmented)
            Button("Reset", systemImage: "arrow.counterclockwise") { scene.reset(shape: shape) }
        }
        .onChange(of: shape) { scene.reset(shape: shape) }
    }

    /// Pulls the body under the pointer towards it, with Apple's spring-like force
    private func forceDrag(in size: CGSize) -> some Gesture {
        DragGesture(minimumDistance: 0)
            .onChanged { value in
                if drag == nil { drag = scene.pick(at: value.startLocation, in: size) }
                guard let drag else { return }
                let ray = scene.ray(through: value.location, in: size)
                scene.pull(drag.entity, towards: ray.origin + ray.direction * drag.distance)
            }
            .onEnded { _ in drag = nil }
    }
}
