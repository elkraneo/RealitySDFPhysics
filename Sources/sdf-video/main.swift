// Renders a short showcase video of the demo scenes offscreen, with a HUD stating what runs.
//
//   swift run -c release sdf-video <output.mp4>
//
// Every frame is one simulation step at 60 Hz, so the video plays in real time.

import AppKit
import AVFoundation
import CoreText
import Metal
import RealityKit
import SDFCollisionKit
import SDFDemoScenes

let width = 1280, height = 720, framesPerSecond: Int32 = 60

/// Scenes in order, with their length in seconds and whether contacts are shown
let reel: [(scene: Int, seconds: Double, contacts: Bool)] = [
    (7, 4.5, false), // Spiky Wheel
    (8, 5, false),   // Cows on Spikes
    (6, 5, false),   // Pile of Chairs
    (1, 4.5, true),  // Gear Stack, with the contact points
    (3, 4.5, false), // Bent planks
]

guard CommandLine.arguments.count == 2 else {
    print("usage: sdf-video <output.mp4>")
    exit(1)
}
let url = URL(fileURLWithPath: CommandLine.arguments[1])
try? FileManager.default.removeItem(at: url)

do {
    let writer = try AVAssetWriter(outputURL: url, fileType: .mp4)
    let input = AVAssetWriterInput(mediaType: .video, outputSettings: [
        AVVideoCodecKey: AVVideoCodecType.h264, AVVideoWidthKey: width, AVVideoHeightKey: height,
        AVVideoCompressionPropertiesKey: [AVVideoAverageBitRateKey: 12_000_000],
    ])
    let adaptor = AVAssetWriterInputPixelBufferAdaptor(assetWriterInput: input, sourcePixelBufferAttributes: [
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
    let texture = MTLCreateSystemDefaultDevice()!.makeTexture(descriptor: descriptor)!
    let cameraOutput = try RealityRenderer.CameraOutput(.singleProjection(colorTexture: texture))

    var frame: Int64 = 0
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
            try await withCheckedThrowingContinuation { (done: CheckedContinuation<Void, any Error>) in
                do {
                    try renderer.updateAndRender(deltaTime: 0, cameraOutput: cameraOutput, onComplete: { _ in done.resume() })
                } catch {
                    done.resume(throwing: error)
                }
            }

            while !input.isReadyForMoreMediaData { try await Task.sleep(for: .milliseconds(2)) }
            guard let pool = adaptor.pixelBufferPool else { throw writer.error ?? CocoaError(.fileWriteUnknown) }
            var buffer: CVPixelBuffer?
            CVPixelBufferPoolCreatePixelBuffer(nil, pool, &buffer)
            guard let buffer else { throw CocoaError(.fileWriteUnknown) }

            CVPixelBufferLockBaseAddress(buffer, [])
            let base = CVPixelBufferGetBaseAddress(buffer)!, bytesPerRow = CVPixelBufferGetBytesPerRow(buffer)
            texture.getBytes(base, bytesPerRow: bytesPerRow, from: MTLRegionMake2D(0, 0, width, height), mipmapLevel: 0)
            let stats = simulation.statistics
            let mean = stepMilliseconds.reduce(0, +) / Double(stepMilliseconds.count)
            drawHUD(into: base, bytesPerRow: bytesPerRow, lines: [
                "SDFCollisionKit · SDF-SDF rigid bodies in RealityKit",
                "physics: SDF engine (RealityKit physics off)",
                "\(scene.name.lowercased()) · \(stats.bodies) bodies · \(stats.contacts) contacts · \(String(format: "%.1f", mean)) ms/step",
            ] + (item.contacts ? ["contacts: ● cached  ● new"] : []), contactLegend: item.contacts)
            CVPixelBufferUnlockBaseAddress(buffer, [])

            adaptor.append(buffer, withPresentationTime: CMTime(value: frame, timescale: framesPerSecond))
            frame += 1
        }
        print("\(scene.name): \(String(format: "%.1f", item.seconds)) s")
    }

    input.markAsFinished()
    await writer.finishWriting()
    guard writer.status == .completed else { throw writer.error ?? CocoaError(.fileWriteUnknown) }
    print("\(url.path): \(frame) frames")
} catch {
    print("error: \(error)")
    exit(1)
}

/// Draws a HUD panel in the top left and the paper credit in the bottom left, in a monospaced font
func drawHUD(into base: UnsafeMutableRawPointer, bytesPerRow: Int, lines: [String], contactLegend: Bool) {
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
            drawLegend(line: lines[i], at: context.textPosition, in: context, attributed: attributed)
        } else {
            CTLineDraw(line, context)
        }
    }

    context.textPosition = CGPoint(x: margin, y: margin)
    CTLineDraw(attributed("Giles & Andrews, Real-Time Collision Handling for SDFs via Caching and Importance Sampling (PG 2026)",
                          color: NSColor(white: 0.15, alpha: 0.85), size: 13), context)
}

func drawLegend(line: String, at origin: CGPoint, in context: CGContext,
                attributed: (String, NSColor, CGFloat?) -> CTLine) {
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
