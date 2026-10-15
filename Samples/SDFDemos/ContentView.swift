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
            ZStack(alignment: .topTrailing) {
                Viewport(model: model)
                StatsOverlay(simulation: model.simulation, sceneDescription: model.sceneDescription,
                             isLoading: model.isLoading, errorMessage: model.errorMessage, activeObject: model.activeObject)
                    .padding(12)
            }
        }
        .task {
            await model.loadScene(0)
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

            ParametersSection(settings: $model.settings, reset: model.resetParameters)

            Section {
                Text("Drag a body to move it, drag the background to orbit, pinch to zoom. In Absolut, drag around the plato to pedal.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
        .formStyle(.grouped)
    }
}

private struct ParametersSection: View {
    @Binding var settings: SDFSimulationComponent
    let reset: () -> Void

    private var parameters: Binding<SDFCollisionDetection> { $settings.collisionDetection }

    var body: some View {
        Section("Collision Detection") {
            Picker("Optimizer", selection: parameters.optimizer) {
                Text("Nelder–Mead").tag(SDFCollisionDetection.Optimizer.nelderMead)
                Text("Ellipsoid").tag(SDFCollisionDetection.Optimizer.ellipsoid)
                Text("Gradient Descent").tag(SDFCollisionDetection.Optimizer.gradientDescent)
            }
            Stepper("Cached samples: \(settings.collisionDetection.cachedSamples)", value: parameters.cachedSamples, in: 0...16)
            Stepper("Random samples: \(settings.collisionDetection.randomSamples)", value: parameters.randomSamples, in: 0...16)
            Toggle("Temporal cache", isOn: parameters.usesTemporalCache)
            Toggle("Curvature sampling", isOn: parameters.usesCurvatureSampling)
            Toggle("Skip convex samples", isOn: parameters.skipsConvexSamples)
            LabeledSlider(title: "Softmax ε", value: parameters.softmaxEpsilon, range: 0...0.5)
            LabeledSlider(title: "Repulsion α", value: parameters.repulsionAlpha, range: 0...0.1)
            LabeledSlider(title: "Tolerance τ", value: parameters.tolerance, range: 0.001...0.05)
            Stepper("Max iterations: \(settings.collisionDetection.maxIterations)", value: parameters.maxIterations, in: 5...200, step: 5)
            LabeledSlider(title: "Contact margin", value: parameters.contactMargin, range: 0...0.1)
        }
        Section("Solver") {
            Stepper("Substeps: \(settings.substeps)", value: $settings.substeps, in: 1...16)
            Stepper("Velocity iterations: \(settings.solverIterations.velocityIterations)",
                    value: $settings.solverIterations.velocityIterations, in: 1...100)
            Stepper("Position iterations: \(settings.solverIterations.positionIterations)",
                    value: $settings.solverIterations.positionIterations, in: 0...100)
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
    let activeObject: String?

    var body: some View {
        // The simulation is not observable; sample its statistics a few times per second
        TimelineView(.periodic(from: .now, by: 0.25)) { _ in
            let stats = simulation.statistics
            VStack(alignment: .leading, spacing: 2) {
                Text(sceneDescription).font(.headline.monospaced())
                if isLoading {
                    Text("Baking shapes…")
                }
                if let errorMessage {
                    Text(errorMessage).foregroundStyle(.red)
                }
                if let activeObject {
                    Text("Active: \(activeObject)").foregroundStyle(.orange)
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
        case pedal
        case body(distance: Float)
        case orbit(last: CGPoint)
    }

    @State private var interaction: Interaction?
    @State private var lastMagnification: CGFloat = 1

    var body: some View {
        GeometryReader { geometry in
            ZStack {
                RealityView { content in
                    content.add(model.root)
                    content.add(DemoStage.makeFloor())
                    content.add(DemoStage.makeLight())
                    content.add(model.cameraEntity)
                    model.frameUpdates = content.subscribe(to: SceneEvents.Update.self) { _ in model.updatePedals() }
                }
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
                    // The root is at the scene origin, so its frame is the world's. The drag holds
                    // the body as a hand would, with a capped force: a gear or a link pulled into
                    // another stops against it rather than passing through.
                    if model.beginPedaling(origin: ray.origin, direction: ray.direction) {
                        interaction = .pedal
                    } else if let picked = model.simulation.beginDrag(origin: ray.origin, direction: ray.direction) {
                        interaction = .body(distance: simd_distance(picked.hit, ray.origin))
                    } else {
                        interaction = .orbit(last: value.startLocation)
                    }
                case .pedal:
                    let ray = model.ray(through: value.location, in: size)
                    model.pedal(origin: ray.origin, direction: ray.direction)
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
                model.endPedaling()
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
