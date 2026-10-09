import RealityKit
import SDFCollisionKit
import SwiftUI

struct ContentView: View {
    @State private var model = DemoModel()

    var body: some View {
        NavigationSplitView {
            Sidebar(model: model)
                .navigationSplitViewColumnWidth(min: 280, ideal: 320)
        } detail: {
            ZStack(alignment: .topLeading) {
                Viewport(model: model)
                StatsOverlay(simulation: model.simulation, sceneDescription: model.sceneDescription)
                    .padding(12)
            }
        }
    }
}

// MARK: Sidebar

private struct Sidebar: View {
    @Bindable var model: DemoModel

    var body: some View {
        Form {
            Section("Scene") {
                Picker("Scene", selection: Binding(get: { model.sceneIndex }, set: { model.loadScene($0) })) {
                    ForEach(SDFWorld.scenes) { scene in
                        Text(scene.name).tag(scene.id)
                    }
                }
                HStack {
                    Button(model.isPaused ? "Play" : "Pause") { model.isPaused.toggle() }
                        .keyboardShortcut("p", modifiers: [])
                    Button("Step") { model.simulation.step() }
                        .disabled(!model.isPaused)
                    Button("Reset") { model.reset() }
                        .keyboardShortcut("r", modifiers: [])
                }
                Toggle("Show contacts", isOn: $model.showsContacts)
            }

            Section("Drop") {
                Picker("Shape", selection: $model.dropShape) {
                    ForEach(SDFWorld.shapeNames, id: \.self) { name in
                        Text(name).tag(name)
                    }
                }
                Button("Drop Shape") { model.drop() }
                    .keyboardShortcut(.space, modifiers: [])
            }

            ParametersSection(parameters: $model.parameters, reset: model.resetParameters)

            Section {
                Text("Drag a body to move it, drag the background to orbit, pinch to zoom.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
        .formStyle(.grouped)
    }
}

private struct ParametersSection: View {
    @Binding var parameters: SDFParams
    let reset: () -> Void

    var body: some View {
        Section("Collision Detection") {
            Picker("Optimizer", selection: $parameters.optimizer) {
                Text("Nelder–Mead").tag(Int32(0))
                Text("Ellipsoid").tag(Int32(1))
                Text("Gradient Descent").tag(Int32(2))
            }
            Stepper("Cached samples: \(parameters.cacheSize)", value: $parameters.cacheSize, in: 0...16)
            Stepper("Random samples: \(parameters.randomSamples)", value: $parameters.randomSamples, in: 0...16)
            Toggle("Temporal cache", isOn: $parameters.warmStart)
            Toggle("Curvature sampling", isOn: $parameters.curvatureSampling)
            Toggle("Skip convex samples", isOn: $parameters.skipConvexSamples)
            LabeledSlider(title: "Softmax ε", value: $parameters.epsilon, range: 0...0.5)
            LabeledSlider(title: "Repulsion α", value: $parameters.alpha, range: 0...0.1)
            LabeledSlider(title: "Tolerance τ", value: $parameters.tolerance, range: 0.001...0.05)
            Stepper("Max iterations: \(parameters.maxIterations)", value: $parameters.maxIterations, in: 5...200, step: 5)
        }
        Section("Solver") {
            Stepper("Substeps: \(parameters.substeps)", value: $parameters.substeps, in: 1...16)
            Stepper("Iterations: \(parameters.solverIterations)", value: $parameters.solverIterations, in: 1...100)
            LabeledSlider(title: "Contact margin", value: $parameters.margin, range: 0...0.1)
            Button("Paper Defaults", action: reset)
        }
    }
}

private struct LabeledSlider: View {
    let title: String
    @Binding var value: Float
    let range: ClosedRange<Float>

    var body: some View {
        LabeledContent {
            Slider(value: $value, in: range)
        } label: {
            Text(title)
            Text(value, format: .number.precision(.fractionLength(3)))
                .monospacedDigit()
        }
    }
}

// MARK: Stats

private struct StatsOverlay: View {
    let simulation: SDFSimulation
    let sceneDescription: String

    var body: some View {
        // The simulation is not observable; sample its statistics a few times per second
        TimelineView(.periodic(from: .now, by: 0.25)) { _ in
            let stats = simulation.world.stats
            VStack(alignment: .leading, spacing: 2) {
                Text(sceneDescription).font(.headline)
                Text("\(stats.bodies) bodies · \(stats.pairs) pairs · \(stats.contacts) contacts")
                Text("step \(stats.stepMilliseconds, format: .number.precision(.fractionLength(2))) ms · collision \(stats.collisionMilliseconds, format: .number.precision(.fractionLength(2))) ms")
                Text("\(stats.iterations) iterations · \(stats.queries) queries")
            }
            .font(.caption.monospacedDigit())
            .padding(8)
            .background(.regularMaterial, in: .rect(cornerRadius: 8))
        }
    }
}

// MARK: Viewport

private struct Viewport: View {
    let model: DemoModel

    private enum Interaction {
        case body(distance: Float)
        case orbit(last: CGPoint)
    }

    @State private var interaction: Interaction?
    @State private var lastMagnification: CGFloat = 1

    var body: some View {
        GeometryReader { geometry in
            RealityView { content in
                content.add(model.simulation.root)
                content.add(SDFDemoStage.makeFloor())
                content.add(SDFDemoStage.makeLight())
                content.add(model.cameraEntity)
            }
            .gesture(dragGesture(in: geometry.size))
            .simultaneousGesture(magnifyGesture)
        }
    }

    private func dragGesture(in size: CGSize) -> some Gesture {
        DragGesture(minimumDistance: 0)
            .onChanged { value in
                switch interaction {
                case nil:
                    let ray = model.ray(through: value.startLocation, in: size)
                    if let hit = model.simulation.world.beginDrag(origin: ray.origin, direction: ray.direction) {
                        interaction = .body(distance: simd_distance(hit, ray.origin))
                    } else {
                        interaction = .orbit(last: value.startLocation)
                    }
                case .body(let distance):
                    let ray = model.ray(through: value.location, in: size)
                    model.simulation.world.drag(to: ray.origin + ray.direction * distance)
                case .orbit(let last):
                    model.orbit(by: CGSize(width: value.location.x - last.x, height: value.location.y - last.y))
                    interaction = .orbit(last: value.location)
                }
            }
            .onEnded { _ in
                model.simulation.world.endDrag()
                interaction = nil
            }
    }

    private var magnifyGesture: some Gesture {
        MagnifyGesture()
            .onChanged { value in
                model.zoom(by: Float(value.magnification / lastMagnification))
                lastMagnification = value.magnification
            }
            .onEnded { _ in lastMagnification = 1 }
    }
}
