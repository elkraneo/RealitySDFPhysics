import RealityKit
import SDFCollisionKit
import SDFDemoScenes
import SwiftUI

struct ContentView: View {
    @State private var model = DemoModel()

    var body: some View {
        NavigationSplitView {
            Sidebar(model: model)
                .navigationSplitViewColumnWidth(min: 280, ideal: 320)
        } detail: {
            // Top trailing, clear of RealityKit's statistics in the top leading corner
            ZStack(alignment: .topTrailing) {
                Viewport(model: model)
                StatsOverlay(simulation: model.simulation, sceneDescription: model.sceneDescription,
                             isLoading: model.isLoading, errorMessage: model.errorMessage)
                    .padding(12)
            }
        }
        .task {
            if CommandLine.arguments.contains("--showcase") {
                await model.runShowcase()
            } else {
                await model.loadScene(0)
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
                Picker("Scene", selection: Binding(get: { model.sceneIndex }, set: { index in
                    Task { await model.loadScene(index) }
                })) {
                    ForEach(DemoScene.all) { scene in
                        Text(scene.name).tag(scene.id)
                    }
                }
                .disabled(model.isLoading)
                HStack {
                    Button(model.isPaused ? "Play" : "Pause") { model.isPaused.toggle() }
                        .keyboardShortcut("p", modifiers: [])
                    Button("Step") { model.step() }
                        .disabled(!model.isPaused)
                    Button("Reset") { Task { await model.reset() } }
                        .keyboardShortcut("r", modifiers: [])
                        .disabled(model.isLoading)
                }
                Toggle("Show contacts", isOn: $model.showsContacts)
                Toggle("RealityKit statistics (ARView)", isOn: $model.showsRealityKitStatistics)
            }

            Section("Drop") {
                Picker("Shape", selection: $model.dropShape) {
                    ForEach(DemoShapes.names, id: \.self) { name in
                        Text(name).tag(name)
                    }
                }
                Button("Drop Shape") { Task { await model.drop() } }
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
    @Binding var parameters: SDFSimulation.Parameters
    let reset: () -> Void

    var body: some View {
        Section("Collision Detection") {
            Picker("Optimizer", selection: $parameters.optimizer) {
                Text("Nelder–Mead").tag(SDFSimulation.Parameters.Optimizer.nelderMead)
                Text("Ellipsoid").tag(SDFSimulation.Parameters.Optimizer.ellipsoid)
                Text("Gradient Descent").tag(SDFSimulation.Parameters.Optimizer.gradientDescent)
            }
            Stepper("Cached samples: \(parameters.cachedSamples)", value: $parameters.cachedSamples, in: 0...16)
            Stepper("Random samples: \(parameters.randomSamples)", value: $parameters.randomSamples, in: 0...16)
            Toggle("Temporal cache", isOn: $parameters.usesTemporalCache)
            Toggle("Curvature sampling", isOn: $parameters.usesCurvatureSampling)
            Toggle("Skip convex samples", isOn: $parameters.skipsConvexSamples)
            LabeledSlider(title: "Softmax ε", value: $parameters.softmaxEpsilon, range: 0...0.5)
            LabeledSlider(title: "Repulsion α", value: $parameters.repulsionAlpha, range: 0...0.1)
            LabeledSlider(title: "Tolerance τ", value: $parameters.tolerance, range: 0.001...0.05)
            Stepper("Max iterations: \(parameters.maxIterations)", value: $parameters.maxIterations, in: 5...200, step: 5)
        }
        Section("Solver") {
            Stepper("Substeps: \(parameters.substeps)", value: $parameters.substeps, in: 1...16)
            Stepper("Iterations: \(parameters.solverIterations)", value: $parameters.solverIterations, in: 1...100)
            LabeledSlider(title: "Contact margin", value: $parameters.contactMargin, range: 0...0.1)
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
    let isLoading: Bool
    let errorMessage: String?

    var body: some View {
        // The simulation is not observable; sample its statistics a few times per second
        TimelineView(.periodic(from: .now, by: 0.25)) { _ in
            let stats = simulation.statistics
            VStack(alignment: .leading, spacing: 2) {
                Text(sceneDescription).font(.headline.monospaced())
                Text("physics: SDF engine (RealityKit physics off)")
                if isLoading {
                    Text("Baking shapes…")
                }
                if let errorMessage {
                    Text(errorMessage).foregroundStyle(.red)
                }
                Text("\(stats.bodies) bodies · \(stats.pairs) pairs · \(stats.contacts) contacts")
                Text("step \(stats.stepMilliseconds, format: .number.precision(.fractionLength(2))) ms · collision \(stats.collisionMilliseconds, format: .number.precision(.fractionLength(2))) ms")
                Text("\(stats.optimizerIterations) iterations · \(stats.queries) queries")
            }
            .font(.caption.monospaced())
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
            ZStack {
                // The same entities run in either RealityKit host
                if model.showsRealityKitStatistics {
                    ARViewport(model: model)
                } else {
                    RealityView { content in
                        content.add(model.root)
                        content.add(DemoStage.makeFloor())
                        content.add(DemoStage.makeLight())
                        content.add(model.cameraEntity)
                    }
                }
                // Gestures on a SwiftUI layer, since ARView handles its own mouse events
                Color.clear
                    .contentShape(.rect)
                    .gesture(dragGesture(in: geometry.size))
                    .simultaneousGesture(magnifyGesture)
            }
        }
    }

    private func dragGesture(in size: CGSize) -> some Gesture {
        DragGesture(minimumDistance: 0)
            .onChanged { value in
                switch interaction {
                case nil:
                    let ray = model.ray(through: value.startLocation, in: size)
                    // The root is at the scene origin, so its frame is the world's
                    if let picked = model.simulation.beginDrag(origin: ray.origin, direction: ray.direction) {
                        interaction = .body(distance: simd_distance(picked.hit, ray.origin))
                    } else {
                        interaction = .orbit(last: value.startLocation)
                    }
                case .body(let distance):
                    let ray = model.ray(through: value.location, in: size)
                    model.simulation.drag(to: ray.origin + ray.direction * distance)
                case .orbit(let last):
                    model.orbit(by: CGSize(width: value.location.x - last.x, height: value.location.y - last.y))
                    interaction = .orbit(last: value.location)
                }
            }
            .onEnded { _ in
                model.simulation.endDrag()
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

/// Hosts the scene in an ARView, which shows RealityKit's own statistics overlay
private struct ARViewport: NSViewRepresentable {
    let model: DemoModel

    func makeNSView(context: Context) -> ARView {
        let view = ARView(frame: .zero)
        view.debugOptions = [.showStatistics]
        view.environment.background = .color(NSColor(red: 0.16, green: 0.18, blue: 0.22, alpha: 1))
        let anchor = AnchorEntity(world: .zero)
        for entity in [model.root, DemoStage.makeFloor(), DemoStage.makeLight(), model.cameraEntity] {
            anchor.addChild(entity)
        }
        view.scene.addAnchor(anchor)
        return view
    }

    func updateNSView(_ view: ARView, context: Context) {}
}
