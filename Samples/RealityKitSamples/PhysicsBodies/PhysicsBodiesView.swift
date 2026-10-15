// Port of MainView from Apple's sample "Simulating physics with collisions in your visionOS app":
// https://developer.apple.com/documentation/realitykit/simulating-physics-with-collisions-in-your-visionos-app
//
// Apple's sample drags with entity-targeted gestures, which hit-test RealityKit collision shapes.
// SDF bodies have none, so the drag picks with the simulation's own raycast, leaving out the
// walls: the camera is outside the volume.

import RealityKit
import RealitySDFPhysics
import RealityKitSampleScenes
import SwiftUI

struct PhysicsBodiesView: View {
    @State private var scene = PhysicsBodiesScene()
    @State private var shape = PhysicsBodiesScene.Shape.spheres
    /// Distance from the camera of the dragged body when the drag began
    @State private var dragDistance: Float?

    var body: some View {
        GeometryReader { geometry in
            RealityView { content in
                scene.reset(shape: shape)
                content.add(scene.content)
            }
            .gesture(forceDrag(in: geometry.size))
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
                if dragDistance == nil { dragDistance = scene.beginDrag(at: value.startLocation, in: size) }
                guard let dragDistance else { return }
                let ray = scene.camera.ray(through: value.location, in: size)
                scene.drag(to: ray.origin + ray.direction * dragDistance)
            }
            .onEnded { _ in
                dragDistance = nil
                scene.endDrag()
            }
    }
}
