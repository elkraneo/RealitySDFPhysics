// Renders short videos of the demo scenes and the RealityKit samples offscreen, with a HUD stating
// what runs.
//
//   swift run -c release sdf-video <output.mp4> [demos | tori | dice | compare]
//
// Every frame is one simulation step at 60 Hz, so the videos play in real time. `demos` is a reel
// of the paper's scenes; `tori` and `dice` drive the samples' scenes as the app does, with the
// RealityKit systems stepping the simulation and a scripted toss. `compare` drops the same rings over
// a peg with RealityKit's physics, each ring a compound of capsules, and with SDF bodies.

import AppKit
import AVFoundation
import CoreMedia
import CoreText
import Metal
import RealityKit
import RealityKitSampleScenes
import SDFCollisionKit
import SDFDemoScenes

let width = 1280, height = 720, framesPerSecond: Int32 = 60
let paperCredit = "Giles & Andrews, Real-Time Collision Handling for SDFs via Caching and Importance Sampling (PG 2026)"

let arguments = CommandLine.arguments.dropFirst()
guard let output = arguments.first, arguments.count <= 2, ["demos", "tori", "dice", "compare"].contains(arguments.dropFirst().first ?? "demos")
else {
    print("usage: sdf-video <output.mp4> [demos | tori | dice | compare]")
    exit(1)
}
let reel = arguments.dropFirst().first ?? "demos"

/// Writes rendered frames, with a HUD, to an H.264 MP4
@MainActor
final class Recorder {
    let texture: MTLTexture
    let cameraOutput: RealityRenderer.CameraOutput
    private let writer: AVAssetWriter
    private let input: AVAssetWriterInput
    private let adaptor: AVAssetWriterInputPixelBufferAdaptor
    private var frame: Int64 = 0

    init(url: URL) throws {
        try? FileManager.default.removeItem(at: url)
        writer = try AVAssetWriter(outputURL: url, fileType: .mp4)
        input = AVAssetWriterInput(mediaType: .video, outputSettings: [
            AVVideoCodecKey: AVVideoCodecType.h264, AVVideoWidthKey: width, AVVideoHeightKey: height,
            AVVideoCompressionPropertiesKey: [AVVideoAverageBitRateKey: 12_000_000],
        ])
        adaptor = AVAssetWriterInputPixelBufferAdaptor(assetWriterInput: input, sourcePixelBufferAttributes: [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA,
            kCVPixelBufferWidthKey as String: width, kCVPixelBufferHeightKey as String: height,
        ])
        writer.add(input)
        guard writer.startWriting() else { throw writer.error ?? CocoaError(.fileWriteUnknown) }
        writer.startSession(atSourceTime: .zero)

        let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm_srgb, width: width, height: height,
                                                                  mipmapped: false)
        descriptor.usage = [.renderTarget, .shaderRead]
        descriptor.storageMode = .shared
        texture = MTLCreateSystemDefaultDevice()!.makeTexture(descriptor: descriptor)!
        cameraOutput = try RealityRenderer.CameraOutput(.singleProjection(colorTexture: texture))
    }

    /// Updates the renderer's scene by a time, which runs the RealityKit systems, and renders it
    func render(_ renderer: RealityRenderer, deltaTime: TimeInterval) async throws {
        try await withCheckedThrowingContinuation { (done: CheckedContinuation<Void, any Error>) in
            do {
                try renderer.updateAndRender(deltaTime: deltaTime, cameraOutput: cameraOutput, onComplete: { _ in done.resume() })
            } catch {
                done.resume(throwing: error)
            }
        }
    }

    /// Appends the rendered frame with HUD lines
    func append(lines: [String], contactLegend: Bool = false, credit: String = paperCredit) async throws {
        while !input.isReadyForMoreMediaData { try await Task.sleep(for: .milliseconds(2)) }
        guard let pool = adaptor.pixelBufferPool else { throw writer.error ?? CocoaError(.fileWriteUnknown) }
        var buffer: CVPixelBuffer?
        CVPixelBufferPoolCreatePixelBuffer(nil, pool, &buffer)
        guard let buffer else { throw CocoaError(.fileWriteUnknown) }

        CVPixelBufferLockBaseAddress(buffer, [])
        let base = CVPixelBufferGetBaseAddress(buffer)!, bytesPerRow = CVPixelBufferGetBytesPerRow(buffer)
        texture.getBytes(base, bytesPerRow: bytesPerRow, from: MTLRegionMake2D(0, 0, width, height), mipmapLevel: 0)
        drawHUD(into: base, bytesPerRow: bytesPerRow, lines: lines, contactLegend: contactLegend, credit: credit)
        CVPixelBufferUnlockBaseAddress(buffer, [])

        adaptor.append(buffer, withPresentationTime: CMTime(value: frame, timescale: framesPerSecond))
        frame += 1
    }

    func finish() async throws -> Int64 {
        input.markAsFinished()
        await writer.finishWriting()
        guard writer.status == .completed else { throw writer.error ?? CocoaError(.fileWriteUnknown) }
        return frame
    }
}

// MARK: Reels

/// The paper's scenes, each with a slowly orbiting camera
@MainActor
func renderDemos(_ recorder: Recorder) async throws {
    // Scenes in order, with their length in seconds and whether contacts are shown
    let reel: [(scene: Int, seconds: Double, contacts: Bool)] = [
        (7, 4.5, false), // Spiky Wheel
        (8, 5, false),   // Cows on Spikes
        (6, 5, false),   // Pile of Chairs
        (1, 4.5, true),  // Gear Stack, with the contact points
        (3, 4.5, false), // Bent planks
    ]
    for item in reel {
        let scene = DemoScene.all[item.scene]
        let simulation = SDFSimulation()
        let root = Entity()
        root.components.set(SDFSimulationComponent(simulation: simulation))
        try await scene.load(into: root)
        if item.contacts { root.addChild(ContactMarkers.makeEntity()) }

        let renderer = try RealityRenderer()
        let camera = DemoStage.makeCamera()
        renderer.entities.append(contentsOf: [root, DemoStage.makeFloor(), DemoStage.makeLight(), camera])
        renderer.activeCamera = camera
        renderer.cameraSettings.colorBackground = .color(CGColor(red: 0.62, green: 0.70, blue: 0.80, alpha: 1))
        var orbit = OrbitCamera(for: scene)

        var stepMilliseconds: [Double] = []
        for _ in 0..<Int(item.seconds * Double(framesPerSecond)) {
            simulation.step(root: root)
            stepMilliseconds = Array((stepMilliseconds + [simulation.statistics.stepMilliseconds]).suffix(30))
            orbit.yaw += 0.0025
            orbit.place(camera)
            // Without time passing the simulation only synchronizes, and the contact markers update
            try await recorder.render(renderer, deltaTime: 0)
            let stats = simulation.statistics
            let mean = stepMilliseconds.reduce(0, +) / Double(stepMilliseconds.count)
            try await recorder.append(lines: [
                "SDFCollisionKit · SDF-SDF rigid bodies in RealityKit",
                "physics: SDF engine (RealityKit physics off)",
                "\(scene.name.lowercased()) · \(stats.bodies) bodies · \(stats.contacts) contacts · \(String(format: "%.1f", mean)) ms/step",
            ] + (item.contacts ? ["contacts: ● cached  ● new"] : []), contactLegend: item.contacts)
        }
        print("\(scene.name): \(String(format: "%.1f", item.seconds)) s")
    }
}

/// A renderer for a sample's scene, with the scene's own camera and a dark background
@MainActor
func makeRenderer(for content: Entity) throws -> RealityRenderer {
    let renderer = try RealityRenderer()
    renderer.entities.append(content)
    renderer.activeCamera = content.children.first { $0 is PerspectiveCamera }
    renderer.cameraSettings.colorBackground = .color(CGColor(red: 0.12, green: 0.13, blue: 0.15, alpha: 1))
    return renderer
}

/// Apple's physics bodies sample with concave tori, which appear one at a time and clump
@MainActor
func renderTori(_ recorder: Recorder) async throws {
    AttractionComponent.registerComponent()
    AttractionSystem.registerSystem()
    let scene = PhysicsBodiesScene()
    scene.reset(shape: .tori)
    let renderer = try makeRenderer(for: scene.content)
    let timeStep = TimeInterval(SDFSimulationComponent().timeStep)
    for _ in 0..<Int(9 * Double(framesPerSecond)) {
        try await recorder.render(renderer, deltaTime: timeStep)
        let stats = scene.simulation.statistics
        try await recorder.append(lines: [
            "Apple's \"Simulating physics with collisions\", ported to SDFCollisionKit",
            "tori in place of spheres: concave moving bodies, which RealityKit's physics can't simulate",
            "\(stats.bodies - 6) tori · \(stats.contacts) contacts · \(String(format: "%.1f", stats.stepMilliseconds)) ms/step",
        ])
    }
}

/// Apple's dice sample: all seven dice gathered, thrown, and forecast to land on their highest faces
@MainActor
func renderDice(_ recorder: Recorder) async throws {
    let scene = DiceScene()
    try await scene.load()
    let renderer = try makeRenderer(for: scene.content)
    let timeStep = TimeInterval(SDFSimulationComponent().timeStep)
    func describe(_ rolls: [DiceScene.Roll]) -> String {
        rolls.map { "\($0.kind.label) \($0.value)" }.joined(separator: " · ") + " = \(rolls.map(\.value).reduce(0, +))"
    }
    // Rest, then pick up the D8 with the others around it, lift, swing, and throw
    let start = SIMD3<Float>(9, 6, 0)
    for frame in 0..<Int(7.5 * Double(framesPerSecond)) {
        switch frame {
        case 30: scene.pickUp(die: 3, tossAll: true)
        // Wind up to the left, then swing across the table and a little towards the camera
        case 31..<100: scene.move(heldTo: start + [-Float(frame - 31) * 0.12, 0, 0])
        case 100..<112: scene.move(heldTo: start + [-8.3 + Float(frame - 100) * 1.4, 0, Float(frame - 100) * 0.3])
        case 112: scene.toss(maxScore: true)
        default: break
        }
        try await recorder.render(renderer, deltaTime: timeStep)
        var lines = ["Apple's \"Simulating dice rolls\", ported to SDFCollisionKit", "toss all, max score"]
        if let rolled = scene.rolled {
            let asForecast = scene.predicted.map { $0.map(\.value) == rolled.map(\.value) } ?? false
            lines.append("rolled    \(describe(rolled))\(asForecast ? ", as forecast" : "")")
        } else if let predicted = scene.predicted {
            lines.append("forecast  \(describe(predicted))")
        } else if frame >= 112 {
            lines.append("forecasting…")
        }
        try await recorder.append(lines: lines, credit: "Dice models from Apple's sample · deterministic forecast with SDFSimulation.forecast()")
    }
}

/// A ring toss: three tori dropped over a peg. Left, RealityKit's physics, with each ring as the best
/// RealityKit has for a moving torus: a compound of 12 capsules around it, built by hand (from a mesh,
/// it would generate a convex hull, which closes the hole). Right, SDFCollisionKit with the torus
/// itself. Each side draws what its physics collides with, and its contact points.
@MainActor
func renderComparison(_ recorder: Recorder) async throws {
    // In meters
    let major: Float = 0.15, minor: Float = 0.04, pegHalfWidth: Float = 0.03, pegHeight: Float = 0.5, segments = 12
    let drops: [(offset: SIMD3<Float>, height: Float, tilt: Float)] = [
        ([0.03, 0, -0.02], 0.8, 0.15), ([-0.04, 0, 0.03], 1.1, -0.2), ([0.02, 0, 0.04], 1.4, 0.1),
    ]
    func material(_ red: CGFloat, _ green: CGFloat, _ blue: CGFloat) -> PhysicallyBasedMaterial {
        var material = PhysicallyBasedMaterial()
        material.baseColor = .init(tint: .init(red: red, green: green, blue: blue, alpha: 1))
        material.roughness = 0.5
        return material
    }
    let colors = [material(0.9, 0.35, 0.3), material(0.3, 0.7, 0.45), material(0.35, 0.5, 0.9)]
    let pegMaterial = material(0.75, 0.72, 0.68)
    let pegShape = ShapeResource.generateBox(size: [2 * pegHalfWidth, pegHeight, 2 * pegHalfWidth])
    let pegMesh = MeshResource.generateBox(size: [2 * pegHalfWidth, pegHeight, 2 * pegHalfWidth])

    // The capsules of a ring: each spans a chord, and their caps overlap at the joints
    let chord = 2 * major * sin(.pi / Float(segments))
    let capsuleMesh = try MeshResource.generate(from: SDFShape.capsule(radius: minor, halfLength: chord / 2))
    let capsules: [(rotation: simd_quatf, translation: SIMD3<Float>)] = (0..<segments).map { k in
        let angle = (Float(k) + 0.5) * 2 * .pi / Float(segments)
        return (simd_quatf(from: [0, 1, 0], to: [-sin(angle), 0, cos(angle)]), [major * cos(angle), 0, major * sin(angle)])
    }
    let compound = capsules.map {
        ShapeResource.generateCapsule(height: chord + 2 * minor, radius: minor).offsetBy(rotation: $0.rotation, translation: $0.translation)
    }

    // RealityKit, with its physics clock driven frame by frame
    var timebase: CMTimebase?
    CMTimebaseCreateWithSourceClock(allocator: nil, sourceClock: CMClockGetHostTimeClock(), timebaseOut: &timebase)
    guard let timebase else { return }
    CMTimebaseSetRate(timebase, rate: 0)
    CMTimebaseSetTime(timebase, time: .zero)
    let left = Entity()
    left.position = [-0.45, 0, 0]
    var physics = PhysicsSimulationComponent()
    physics.clock = timebase
    left.components.set(physics)
    let ground = Entity()
    ground.position = [0, -0.5, 0]
    ground.components.set(CollisionComponent(shapes: [.generateBox(size: [4, 1, 4])]))
    ground.components.set(PhysicsBodyComponent(shapes: [.generateBox(size: [4, 1, 4])], mass: 1, mode: .static))
    let leftPeg = ModelEntity(mesh: pegMesh, materials: [pegMaterial])
    leftPeg.position = [0, pegHeight / 2, 0]
    leftPeg.components.set(CollisionComponent(shapes: [pegShape], mode: .default, collisionOptions: .fullContactInformation))
    leftPeg.components.set(PhysicsBodyComponent(shapes: [pegShape], mass: 1, mode: .static))
    left.addChild(ground)
    left.addChild(leftPeg)
    var leftRings: [Entity] = []
    for (index, drop) in drops.enumerated() {
        let ring = Entity()
        // Draw the capsules: what RealityKit's physics sees
        for capsule in capsules {
            let part = ModelEntity(mesh: capsuleMesh, materials: [colors[index]])
            part.transform = Transform(rotation: capsule.rotation, translation: capsule.translation)
            ring.addChild(part)
        }
        ring.position = drop.offset + [0, drop.height, 0]
        ring.orientation = simd_quatf(angle: drop.tilt, axis: [1, 0, 0])
        ring.components.set(CollisionComponent(shapes: compound, mode: .default, collisionOptions: .fullContactInformation))
        ring.components.set(PhysicsBodyComponent(shapes: compound, density: 1000))
        left.addChild(ring)
        leftRings.append(ring)
    }

    // SDFCollisionKit, in a root of decimeters, with the contact markers of the demo
    let right = Entity()
    right.position = [0.45, 0, 0]
    right.scale = SIMD3(repeating: 0.1)
    let simulation = SDFSimulation()
    right.components.set(SDFSimulationComponent(simulation: simulation, metersPerUnit: 0.1))
    let sdfGround = Entity()
    sdfGround.position = [0, -5, 0]
    sdfGround.components.set(SDFBodyComponent(shape: .box(halfExtents: [20, 5, 20]), mode: .static))
    let rightPeg = ModelEntity(mesh: pegMesh, materials: [pegMaterial])
    rightPeg.scale = SIMD3(repeating: 10)
    rightPeg.position = [0, 10 * pegHeight / 2, 0]
    rightPeg.components.set(SDFBodyComponent(shape: .box(halfExtents: [pegHalfWidth, pegHeight / 2, pegHalfWidth]), mode: .static))
    right.addChild(sdfGround)
    right.addChild(rightPeg)
    right.addChild(ContactMarkers.makeEntity())
    let torus = SDFShape.torus(majorRadius: major, minorRadius: minor)
    let torusMesh = try MeshResource.generate(from: torus)
    var rightRings: [Entity] = []
    for (index, drop) in drops.enumerated() {
        // The mesh is the collision shape. In meters, scaled to the root's decimeters.
        let ring = ModelEntity(mesh: torusMesh, materials: [colors[index]])
        ring.scale = SIMD3(repeating: 10)
        ring.position = 10 * (drop.offset + [0, drop.height, 0])
        ring.orientation = simd_quatf(angle: drop.tilt, axis: [1, 0, 0])
        ring.components.set(SDFBodyComponent(shape: torus))
        right.addChild(ring)
        rightRings.append(ring)
    }

    let renderer = try RealityRenderer()
    let camera = PerspectiveCamera()
    camera.camera.fieldOfViewInDegrees = 40
    camera.look(at: [0, 0.3, 0], from: [0, 0.9, 1.9], relativeTo: nil)
    renderer.entities.append(contentsOf: [left, right, DemoStage.makeFloor(), DemoStage.makeLight(), camera])
    renderer.activeCamera = camera
    renderer.cameraSettings.colorBackground = .color(CGColor(red: 0.62, green: 0.70, blue: 0.80, alpha: 1))

    // RealityKit's contact points, drawn as yellow dots
    var realityKitContacts: [SIMD3<Float>] = []
    let began = renderer.subscribe(to: CollisionEvents.Began.self) { event in realityKitContacts += event.contacts.map(\.point) }
    let updated = renderer.subscribe(to: CollisionEvents.Updated.self) { event in realityKitContacts += event.contacts.map(\.point) }
    var dots: [ModelEntity] = []
    let dotMesh = MeshResource.generateSphere(radius: 0.006)
    let dotMaterial = UnlitMaterial(color: .init(red: 1, green: 0.85, blue: 0.1, alpha: 1))

    /// Rings with the peg through their hole, below its top
    func ringsOnPeg(_ rings: [Entity], in side: Entity) -> Int {
        rings.filter { ring in
            // In meters: the right side's root is scaled to decimeters
            let position = ring.position(relativeTo: nil) - side.position(relativeTo: nil)
            return simd_length(SIMD2(position.x, position.z)) < major - minor && position.y < pegHeight
        }.count
    }

    let timeStep = TimeInterval(SDFSimulationComponent().timeStep)
    for frame in 1...Int(6 * Double(framesPerSecond)) {
        realityKitContacts = []
        CMTimebaseSetTime(timebase, time: CMTime(value: CMTimeValue(frame), timescale: framesPerSecond))
        try await recorder.render(renderer, deltaTime: timeStep)
        // Move the dots to this frame's contacts; they show in the next frame
        while dots.count < realityKitContacts.count {
            let dot = ModelEntity(mesh: dotMesh, materials: [dotMaterial])
            renderer.entities.append(dot)
            dots.append(dot)
        }
        for (index, dot) in dots.enumerated() {
            dot.isEnabled = index < realityKitContacts.count
            if index < realityKitContacts.count { dot.position = realityKitContacts[index] }
        }
        try await recorder.append(lines: [
            "Ring toss: three rings dropped over a peg, same sizes, drops and simulated time",
            "left   RealityKit physics: each ring is 12 capsules (a compound built by hand)",
            "right  SDFCollisionKit: the torus itself",
            "contacts  RealityKit ● \(realityKitContacts.count) · SDFCollisionKit ● \(simulation.contacts().count)",
            "rings around the peg  RealityKit \(ringsOnPeg(leftRings, in: left))/3 · SDFCollisionKit \(ringsOnPeg(rightRings, in: right))/3",
        ])
    }
    withExtendedLifetime((began, updated)) {}
    print("rings around the peg: RealityKit \(ringsOnPeg(leftRings, in: left))/3, SDFCollisionKit \(ringsOnPeg(rightRings, in: right))/3")
}

do {
    let url = URL(fileURLWithPath: output)
    let recorder = try Recorder(url: url)
    switch reel {
    case "tori": try await renderTori(recorder)
    case "dice": try await renderDice(recorder)
    case "compare": try await renderComparison(recorder)
    default: try await renderDemos(recorder)
    }
    let frames = try await recorder.finish()
    print("\(url.path): \(frames) frames")
} catch {
    print("error: \(error)")
    exit(1)
}

// MARK: HUD

/// Draws a HUD panel in the top left and a credit in the bottom left, in a monospaced font
func drawHUD(into base: UnsafeMutableRawPointer, bytesPerRow: Int, lines: [String], contactLegend: Bool, credit: String) {
    let info = CGImageAlphaInfo.premultipliedFirst.rawValue | CGBitmapInfo.byteOrder32Little.rawValue
    guard let context = CGContext(data: base, width: width, height: height, bitsPerComponent: 8, bytesPerRow: bytesPerRow,
                                  space: CGColorSpace(name: CGColorSpace.sRGB)!, bitmapInfo: info) else { return }
    let font = NSFont.monospacedSystemFont(ofSize: 17, weight: .medium)
    let lineHeight: CGFloat = 25, margin: CGFloat = 24, padding: CGFloat = 14

    func attributed(_ text: String, color: NSColor, size: CGFloat? = nil) -> CTLine {
        let font = size.map { NSFont.monospacedSystemFont(ofSize: $0, weight: .regular) } ?? font
        return CTLineCreateWithAttributedString(NSAttributedString(string: text, attributes: [.font: font, .foregroundColor: color]))
    }

    let ctLines = lines.enumerated().map { i, text in
        attributed(text, color: i == 0 ? .white : NSColor(white: 0.85, alpha: 1))
    }
    let panelWidth = ctLines.map { CTLineGetTypographicBounds($0, nil, nil, nil) }.max()! + 2 * padding
    let panelHeight = CGFloat(lines.count) * lineHeight + 2 * padding - 6
    // Bitmap context coordinates start at the bottom left
    let panel = CGRect(x: margin, y: CGFloat(height) - margin - panelHeight, width: panelWidth, height: panelHeight)
    context.setFillColor(CGColor(gray: 0.08, alpha: 0.72))
    context.addPath(CGPath(roundedRect: panel, cornerWidth: 10, cornerHeight: 10, transform: nil))
    context.fillPath()

    for (i, line) in ctLines.enumerated() {
        context.textPosition = CGPoint(x: panel.minX + padding, y: panel.maxY - padding - 15 - CGFloat(i) * lineHeight)
        if contactLegend && i == lines.count - 1 {
            // Color the two legend dots like the markers
            drawLegend(at: context.textPosition, in: context, attributed: attributed)
        } else {
            CTLineDraw(line, context)
        }
    }

    let creditLine = attributed(credit, color: NSColor(white: 0.8, alpha: 0.9), size: 13)
    let creditWidth = CTLineGetTypographicBounds(creditLine, nil, nil, nil)
    let creditPanel = CGRect(x: margin - 8, y: margin - 8, width: creditWidth + 16, height: 26)
    context.setFillColor(CGColor(gray: 0.08, alpha: 0.6))
    context.addPath(CGPath(roundedRect: creditPanel, cornerWidth: 6, cornerHeight: 6, transform: nil))
    context.fillPath()
    context.textPosition = CGPoint(x: margin, y: margin)
    CTLineDraw(creditLine, context)
}

func drawLegend(at origin: CGPoint, in context: CGContext, attributed: (String, NSColor, CGFloat?) -> CTLine) {
    var x = origin.x
    let parts: [(String, NSColor)] = [("contacts: ", NSColor(white: 0.85, alpha: 1)),
                                      ("● ", NSColor(red: 0.2, green: 0.8, blue: 0.3, alpha: 1)),
                                      ("cached  ", NSColor(white: 0.85, alpha: 1)),
                                      ("● ", NSColor(red: 0.95, green: 0.25, blue: 0.2, alpha: 1)),
                                      ("new", NSColor(white: 0.85, alpha: 1))]
    for (text, color) in parts {
        let part = attributed(text, color, nil)
        context.textPosition = CGPoint(x: x, y: origin.y)
        CTLineDraw(part, context)
        x += CTLineGetTypographicBounds(part, nil, nil, nil)
    }
}
