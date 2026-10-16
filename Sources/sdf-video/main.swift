// Renders short videos of the demo scenes and the RealityKit samples offscreen, with a HUD stating
// what runs.
//
//   swift run -c release sdf-video <output.mp4> [demos | spheres | tori | dice | cradle | samples | compare]
//
// Every frame is one simulation step at 60 Hz, so the videos play in real time. `demos` is a reel
// of the paper's scenes; `spheres`, `tori`, `dice` and `cradle` drive the samples' scenes as the
// app does, with the RealityKit systems stepping the simulation, a scripted toss and lifted balls.
// `samples` is Apple's three samples in a row: spheres, dice and cradle. `compare` drops the same
// rings over a peg with RealityKit's physics, each ring a compound of capsules, and with SDF bodies;
// 3 rings, or up to 20.
// Offscreen, RealityKit has no environment light, which a RealityView gives by default; every reel
// lights its scene with a studio.

import AppKit
import AVFoundation
import CoreMedia
import CoreText
import Metal
import RealityKit
import SDFRealityKitSampleScenes
import RealitySDFPhysics
import SDFDemoScenes

let width = 1280, height = 720, framesPerSecond: Int32 = 60
let paperCredit = "Giles & Andrews, Real-Time Collision Handling for SDFs via Caching and Importance Sampling (PG 2026)"

let arguments = CommandLine.arguments.dropFirst()
let reel = arguments.dropFirst().first ?? "demos"
// Only `compare` takes a third argument, its number of rings
let ringCount = arguments.count <= 2 ? 3 : arguments.count == 3 && reel == "compare" ? Int(arguments.last!) : nil
guard let output = arguments.first, ["demos", "spheres", "tori", "dice", "cradle", "samples", "compare"].contains(reel),
      let ringCount, (1...20).contains(ringCount)
else {
    print("usage: sdf-video <output.mp4> [demos | spheres | tori | dice | cradle | samples | compare [rings, 3 by default]]")
    exit(1)
}

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
    let scenes = DemoScene.all
    // Scenes in order, with their length in seconds and whether contacts are shown
    let reel: [(scene: Int, seconds: Double, contacts: Bool)] = [
        (7, 4.5, false), // Spiky Wheel
        (8, 5, false),   // Cows on Spikes
        (6, 5, false),   // Pile of Chairs
        (1, 4.5, true),  // Gear Stack, with the contact points
        (3, 4.5, false), // Bent planks
    ]
    for item in reel {
        let scene = scenes[item.scene]
        let simulation = SDFSimulation()
        let root = Entity()
        root.components.set(SDFPhysicsSimulationComponent(simulation: simulation, preset: .paper))
        try await scene.load(into: root)
        if item.contacts { root.addChild(ContactMarkers.makeEntity()) }

        let renderer = try RealityRenderer()
        try await lightWithStudio(renderer)
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
                "RealitySDFPhysics · SDF-SDF rigid bodies in RealityKit",
                "physics: SDF engine (RealityKit physics off)",
                "\(scene.name.lowercased()) · \(stats.bodies) bodies · \(stats.contacts) contacts · \(String(format: "%.1f", mean)) ms/step",
            ] + (item.contacts ? ["contacts: ● cached  ● new"] : []), contactLegend: item.contacts)
        }
        print("\(scene.name): \(String(format: "%.1f", item.seconds)) s")
    }
}

/// A renderer for a sample's scene, with the scene's own camera and a dark background
@MainActor
func makeRenderer(for content: Entity) async throws -> RealityRenderer {
    let renderer = try RealityRenderer()
    try await lightWithStudio(renderer)
    renderer.entities.append(content)
    renderer.activeCamera = content.children.first { $0 is PerspectiveCamera }
    renderer.cameraSettings.colorBackground = .color(CGColor(red: 0.12, green: 0.13, blue: 0.15, alpha: 1))
    return renderer
}

/// Lights a renderer's scene with the studio, as a RealityView's default environment would
@MainActor
func lightWithStudio(_ renderer: RealityRenderer) async throws {
    renderer.lighting.resource = try await EnvironmentResource(equirectangular: makeStudio())
    renderer.lighting.intensityExponent = 0.5
}

/// Apple's physics bodies sample: its metallic spheres, which attract each other, or concave tori,
/// which appear one at a time and clump
@MainActor
func renderPhysicsBodies(_ recorder: Recorder, shape: PhysicsBodiesScene.Shape, seconds: Double = 9) async throws {
    AttractionComponent.registerComponent()
    AttractionSystem.registerSystem()
    let scene = PhysicsBodiesScene()
    scene.reset(shape: shape)
    let renderer = try await makeRenderer(for: scene.content)
    let timeStep = TimeInterval(SDFPhysicsSimulationComponent().timeStep)
    for _ in 0..<Int(seconds * Double(framesPerSecond)) {
        try await recorder.render(renderer, deltaTime: timeStep)
        let stats = scene.simulation.statistics
        try await recorder.append(lines: [
            "Apple's \"Simulating physics with collisions\", ported to RealitySDFPhysics",
            shape == .tori ? "tori in place of spheres: concave moving bodies, which RealityKit's physics can't simulate"
                : "Apple's metallic spheres, without gravity, attract each other",
            "\(stats.bodies - 6) \(shape == .tori ? "tori" : "spheres") · \(stats.contacts) contacts · \(String(format: "%.1f", stats.stepMilliseconds)) ms/step",
        ])
    }
}

/// Apple's dice sample: all seven dice gathered, thrown, and forecast to land on their highest faces
@MainActor
func renderDice(_ recorder: Recorder) async throws {
    let scene = DiceScene()
    try await scene.load()
    let renderer = try await makeRenderer(for: scene.content)
    let timeStep = TimeInterval(SDFPhysicsSimulationComponent().timeStep)
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
        var lines = ["Apple's \"Simulating dice rolls\", ported to RealitySDFPhysics", "toss all, max score"]
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

/// Apple's Newton's cradle: one ball let go, then two, each coming out as as many on the far side
@MainActor
func renderCradle(_ recorder: Recorder) async throws {
    let scene = PendulumScene()
    scene.reset(pendulumCount: 5)
    let renderer = try await makeRenderer(for: scene.content)
    let timeStep = TimeInterval(SDFPhysicsSimulationComponent().timeStep)
    for (count, seconds) in [(1, 4.5), (2, 4.5)] {
        scene.lift(count)
        for _ in 0..<Int(seconds * Double(framesPerSecond)) {
            try await recorder.render(renderer, deltaTime: timeStep)
            try await recorder.append(lines: [
                "Apple's \"Simulating physics joints\", ported to RealitySDFPhysics",
                "Newton's cradle on revolute joints: \(count == 1 ? "one ball in, one ball out" : "two balls in, two balls out")",
                "RealityKit's physics, in Apple's sample: one ball in sends four out",
            ], credit: "Apple's sample settings, sizes and joints · bounces resolved pair by pair (Guendelman, Bridson & Fedkiw 2003)")
        }
    }
}

/// An equirectangular studio: a light ceiling with a bright strip, a dark floor
func makeStudio() -> CGImage {
    let width = 512, height = 256
    let context = CGContext(data: nil, width: width, height: height, bitsPerComponent: 8, bytesPerRow: 0,
                            space: CGColorSpace(name: CGColorSpace.sRGB)!,
                            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue)!
    let gradient = CGGradient(colorsSpace: CGColorSpace(name: CGColorSpace.sRGB)!, colors: [
        CGColor(gray: 0.12, alpha: 1), CGColor(gray: 0.3, alpha: 1), CGColor(gray: 0.85, alpha: 1), CGColor(gray: 0.6, alpha: 1),
    ] as CFArray, locations: [0, 0.48, 0.52, 1])!
    // Bitmap coordinates start at the bottom: the floor, then the horizon, then the ceiling
    context.drawLinearGradient(gradient, start: .zero, end: CGPoint(x: 0, y: height), options: [])
    context.setFillColor(CGColor(gray: 1, alpha: 1))
    context.fill(CGRect(x: 0, y: height * 3 / 4, width: width, height: 14))
    context.fill(CGRect(x: width / 3, y: height / 2 + 10, width: width / 6, height: height / 3))
    return context.makeImage()!
}

/// The outline of a capsule along y, in lines: circles around its cylinder, and meridians over it
/// from cap to cap
@MainActor
func makeCapsuleWire(radius: Float, halfLength: Float) throws -> MeshResource {
    var points: [SIMD3<Float>] = []
    func circle(y: Float) {
        for k in 0..<24 {
            let a = Float(k) / 24 * 2 * .pi, b = Float(k + 1) / 24 * 2 * .pi
            points += [[radius * cos(a), y, radius * sin(a)], [radius * cos(b), y, radius * sin(b)]]
        }
    }
    circle(y: -halfLength)
    circle(y: halfLength)
    for m in 0..<8 {
        let direction = SIMD3<Float>(cos(Float(m) * .pi / 4), 0, sin(Float(m) * .pi / 4))
        // Up the side, then over the cap halfway to its pole, and the same at the bottom
        points += [direction * radius - [0, halfLength, 0], direction * radius + [0, halfLength, 0]]
        for k in 0..<6 {
            let a = Float(k) / 12 * .pi, b = Float(k + 1) / 12 * .pi
            for sign: Float in [1, -1] {
                points += [direction * radius * cos(a) + [0, sign * (halfLength + radius * sin(a)), 0],
                           direction * radius * cos(b) + [0, sign * (halfLength + radius * sin(b)), 0]]
            }
        }
    }
    var descriptor = LowLevelMesh.Descriptor()
    descriptor.vertexAttributes = [.init(semantic: .position, format: .float3, offset: 0)]
    descriptor.vertexLayouts = [.init(bufferIndex: 0, bufferStride: MemoryLayout<SIMD3<Float>>.stride)]
    descriptor.vertexCapacity = points.count
    descriptor.indexCapacity = points.count
    descriptor.indexType = .uint32
    let mesh = try LowLevelMesh(descriptor: descriptor)
    mesh.withUnsafeMutableBytes(bufferIndex: 0) { raw in
        let vertices = raw.bindMemory(to: SIMD3<Float>.self)
        for (i, point) in points.enumerated() { vertices[i] = point }
    }
    mesh.withUnsafeMutableIndices { raw in
        let indices = raw.bindMemory(to: UInt32.self)
        for i in points.indices { indices[i] = UInt32(i) }
    }
    let extent = SIMD3<Float>(radius, halfLength + radius, radius)
    mesh.parts.replaceAll([LowLevelMesh.Part(indexCount: points.count, topology: .line, bounds: BoundingBox(min: -extent, max: extent))])
    return try MeshResource(from: mesh)
}

/// A ring toss: tori dropped over a peg. Left, RealityKit's physics, with each ring as the best
/// RealityKit has for a moving torus: a compound of 12 capsules around it, built by hand (from a mesh,
/// it would generate a convex hull, which closes the hole). Right, RealitySDFPhysics with the torus
/// itself. Each side draws its ring, what its physics collides with, and its contact points.
@MainActor
func renderComparison(_ recorder: Recorder, rings count: Int) async throws {
    // In meters. The peg is tall enough for the rings stacked flat.
    let major: Float = 0.15, minor: Float = 0.04, pegHalfWidth: Float = 0.03, segments = 12
    let pegHeight = max(0.5, Float(count) * 2 * minor + 0.15)
    // Plastic toss rings of 300 g, one let go every 0.4 s, 30 cm above the peg: three chosen
    // drops, then more on the same pattern
    let ringMass: Float = 0.3, releaseInterval = 0.4, dropHeight = pegHeight + 0.3
    let chosen: [(offset: SIMD3<Float>, tilt: Float)] = [([0.03, 0, -0.02], 0.15), ([-0.04, 0, 0.03], -0.2), ([0.02, 0, 0.04], 0.1)]
    let drops = (0..<count).map { i in
        let drop = chosen[i % 3]
        let turn = simd_quatf(angle: Float(i / 3) * 2.1, axis: [0, 1, 0])
        return (offset: turn.act(drop.offset), tilt: drop.tilt * (i / 3 % 2 == 0 ? 1 : -1),
                frame: 1 + Int((Double(i) * releaseInterval * Double(framesPerSecond)).rounded()))
    }
    func material(_ red: CGFloat, _ green: CGFloat, _ blue: CGFloat) -> PhysicallyBasedMaterial {
        var material = PhysicallyBasedMaterial()
        material.baseColor = .init(tint: .init(red: red, green: green, blue: blue, alpha: 1))
        material.roughness = 0.5
        return material
    }
    let palette = [material(0.9, 0.35, 0.3), material(0.3, 0.7, 0.45), material(0.35, 0.5, 0.9), material(0.95, 0.75, 0.25),
                   material(0.65, 0.4, 0.85)]
    let colors = drops.indices.map { palette[$0 % palette.count] }
    let pegMaterial = material(0.75, 0.72, 0.68)
    let pegShape = ShapeResource.generateBox(size: [2 * pegHalfWidth, pegHeight, 2 * pegHalfWidth])
    let pegMesh = MeshResource.generateBox(size: [2 * pegHalfWidth, pegHeight, 2 * pegHalfWidth])

    // The capsules of a ring: each spans a chord, and their caps overlap at the joints
    let chord = 2 * major * sin(.pi / Float(segments))
    let capsuleWire = try makeCapsuleWire(radius: minor, halfLength: chord / 2)
    let wireMaterial = UnlitMaterial(color: .init(red: 0.3, green: 1, blue: 0.4, alpha: 1))
    let torusMesh = try MeshResource.generate(from: SDFShape.torus(majorRadius: major, minorRadius: minor))
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
    left.position = [-0.6, 0, 0]
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
        // The ring as modeled, and over it, as RealityKit's physics debug view draws it, the
        // capsules its physics collides with
        let ring = ModelEntity(mesh: torusMesh, materials: [colors[index]])
        for capsule in capsules {
            let part = ModelEntity(mesh: capsuleWire, materials: [wireMaterial])
            part.transform = Transform(rotation: capsule.rotation, translation: capsule.translation)
            ring.addChild(part)
        }
        ring.position = drop.offset + [0, dropHeight, 0]
        ring.orientation = simd_quatf(angle: drop.tilt, axis: [1, 0, 0])
        ring.components.set(CollisionComponent(shapes: compound, mode: .default, collisionOptions: .fullContactInformation))
        var ringBody = PhysicsBodyComponent(shapes: compound, mass: ringMass)
        // RealityKit's best for falling rings: without it, more of them are lost
        ringBody.isContinuousCollisionDetectionEnabled = true
        ring.components.set(ringBody)
        leftRings.append(ring)
    }

    // RealitySDFPhysics, in a root of decimeters, with the contact markers of the demo
    let right = Entity()
    right.position = [0.6, 0, 0]
    right.scale = SIMD3(repeating: 0.1)
    let simulation = SDFSimulation()
    right.components.set(SDFPhysicsSimulationComponent(simulation: simulation, metersPerUnit: 0.1))
    let sdfGround = Entity()
    sdfGround.position = [0, -5, 0]
    sdfGround.components.set(SDFPhysicsBodyComponent(shape: .box(halfExtents: [20, 5, 20]), mode: .static))
    let rightPeg = ModelEntity(mesh: pegMesh, materials: [pegMaterial])
    rightPeg.scale = SIMD3(repeating: 10)
    rightPeg.position = [0, 10 * pegHeight / 2, 0]
    rightPeg.components.set(SDFPhysicsBodyComponent(shape: .box(halfExtents: [pegHalfWidth, pegHeight / 2, pegHalfWidth]), mode: .static))
    right.addChild(sdfGround)
    right.addChild(rightPeg)
    right.addChild(ContactMarkers.makeEntity())
    let torus = SDFShape.torus(majorRadius: major, minorRadius: minor)
    var rightRings: [Entity] = []
    for (index, drop) in drops.enumerated() {
        // The mesh is the collision shape. In meters, scaled to the root's decimeters.
        let ring = ModelEntity(mesh: torusMesh, materials: [colors[index]])
        ring.scale = SIMD3(repeating: 10)
        ring.position = 10 * (drop.offset + [0, dropHeight, 0])
        ring.orientation = simd_quatf(angle: drop.tilt, axis: [1, 0, 0])
        ring.components.set(SDFPhysicsBodyComponent(shape: torus, massProperties: .init(mass: ringMass)))
        rightRings.append(ring)
    }

    let renderer = try RealityRenderer()
    try await lightWithStudio(renderer)
    let camera = PerspectiveCamera()
    camera.camera.fieldOfViewInDegrees = 40
    camera.look(at: [0, 0.3 + (pegHeight - 0.5) / 2, 0], from: [0, 1 + (pegHeight - 0.5), 2.3 + 1.5 * (pegHeight - 0.5)], relativeTo: nil)
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

    /// Rings with the peg through their hole, below its top: the peg's axis crosses the ring's
    /// plane inside the hole
    func ringsOnPeg(_ rings: [Entity], in side: Entity) -> Int {
        rings.filter { ring in
            // In meters: the right side's root is scaled to decimeters
            let center = ring.position(relativeTo: nil) - side.position(relativeTo: nil)
            let axis = ring.orientation(relativeTo: nil).act([0, 1, 0])
            guard abs(axis.y) > 1e-3 else { return false }
            let height: Float = center.y + (axis.x * center.x + axis.z * center.z) / axis.y
            let hole: Float = major - minor
            return simd_length(SIMD3<Float>(0, height, 0) - center) < hole && (0..<pegHeight).contains(height)
        }.count
    }

    let timeStep = TimeInterval(SDFPhysicsSimulationComponent().timeStep)
    for frame in 1...(drops.last!.frame + 4 * Int(framesPerSecond)) {
        // Let go of the rings whose time it is, on both sides at once
        for (index, drop) in drops.enumerated() where drop.frame == frame {
            left.addChild(leftRings[index])
            right.addChild(rightRings[index])
        }
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
            "Ring toss: \(count) rings of 300 g dropped over a peg one by one, same sizes, drops and simulated time",
            "left   RealityKit physics: each ring collides as the 12 capsules drawn over it (a compound built by hand), with CCD",
            "right  RealitySDFPhysics: the torus itself",
            "contacts  RealityKit ● \(realityKitContacts.count) · RealitySDFPhysics ● \(simulation.contacts().count)",
            "rings around the peg  RealityKit \(ringsOnPeg(leftRings, in: left))/\(count) · RealitySDFPhysics \(ringsOnPeg(rightRings, in: right))/\(count)",
        ])
    }
    withExtendedLifetime((began, updated)) {}
    print("rings around the peg: RealityKit \(ringsOnPeg(leftRings, in: left))/\(count), RealitySDFPhysics \(ringsOnPeg(rightRings, in: right))/\(count)")
}

do {
    let url = URL(fileURLWithPath: output)
    let recorder = try Recorder(url: url)
    switch reel {
    case "spheres": try await renderPhysicsBodies(recorder, shape: .spheres)
    case "tori": try await renderPhysicsBodies(recorder, shape: .tori)
    case "dice": try await renderDice(recorder)
    case "cradle": try await renderCradle(recorder)
    case "samples":
        try await renderPhysicsBodies(recorder, shape: .spheres, seconds: 6)
        try await renderDice(recorder)
        try await renderCradle(recorder)
    case "compare": try await renderComparison(recorder, rings: ringCount)
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
