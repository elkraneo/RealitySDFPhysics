// Simulates every demo scene and renders it offscreen with RealityKit to a PNG, and prints timings.
//
//   swift run -c release sdf-snapshot [--single-threaded] <output directory> [seconds] [scene name...]
//
// Seconds of simulated time default to 4, up to an hour. Scene names ignore case, spaces and
// punctuation ("falling-boxes-bend").

import CoreGraphics
import Foundation
import ImageIO
import Metal
import RealityKit
import RealitySDFPhysics
import SDFDemoScenes
import UniformTypeIdentifiers

func usage() -> Never {
    print("usage: sdf-snapshot [--single-threaded] <output directory> [seconds] [scene name...]")
    exit(1)
}

let isMultithreaded = !CommandLine.arguments.contains("--single-threaded")
let arguments = Array(CommandLine.arguments.dropFirst().filter { $0 != "--single-threaded" })
guard let output = arguments.first else { usage() }
let seconds = arguments.count > 1 ? Double(arguments[1]) : nil
guard (0...3600).contains(seconds ?? 0) else { usage() }
let steps = Int(((seconds ?? 4) * 60).rounded())
let names = arguments.dropFirst(seconds == nil ? 1 : 2)
let selected = names.isEmpty ? DemoScene.all : names.map { name in
    DemoScene.named(name) ?? { print("unknown scene \(name)"); usage() }()
}

do {
    try FileManager.default.createDirectory(atPath: output, withIntermediateDirectories: true)
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm_srgb, width: 1280, height: 800,
                                                              mipmapped: false)
    descriptor.usage = [.renderTarget, .shaderRead]
    descriptor.storageMode = .shared
    let texture = MTLCreateSystemDefaultDevice()!.makeTexture(descriptor: descriptor)!

    print("scene                     bodies  load ms  step ms (mean / max)  contacts  collision  pairs  queries/pair")
    for scene in selected {
        let simulation = SDFSimulation()
        let root = Entity()
        var settings = SDFPhysicsSimulationComponent(simulation: simulation, preset: .paper)
        settings.isMultithreaded = isMultithreaded
        root.components.set(settings)

        let loadStart = Date()
        try await scene.load(into: root)
        let loadMilliseconds = Date().timeIntervalSince(loadStart) * 1000
        root.addChild(ContactMarkers.makeEntity())

        var total = 0.0, slowest = 0.0, collision = 0.0, pairs = 0, queries = 0
        for _ in 0..<steps {
            simulation.step(root: root)
            let stats = simulation.statistics
            total += stats.stepMilliseconds
            slowest = max(slowest, stats.stepMilliseconds)
            collision += stats.collisionMilliseconds
            pairs += stats.pairs
            queries += stats.queries
        }

        let renderer = try RealityRenderer()
        let camera = DemoStage.makeCamera()
        OrbitCamera(for: scene).place(camera)
        renderer.entities.append(contentsOf: [root, DemoStage.makeFloor(), DemoStage.makeLight(), camera])
        renderer.activeCamera = camera
        renderer.cameraSettings.colorBackground = .color(CGColor(red: 0.62, green: 0.70, blue: 0.80, alpha: 1))
        // Without time passing the simulation only synchronizes, and the contact markers update
        let cameraOutput = try RealityRenderer.CameraOutput(.singleProjection(colorTexture: texture))
        try await withCheckedThrowingContinuation { (done: CheckedContinuation<Void, any Error>) in
            do {
                try renderer.updateAndRender(deltaTime: 0, cameraOutput: cameraOutput, onComplete: { _ in done.resume() })
            } catch {
                done.resume(throwing: error)
            }
        }

        let name = scene.name.replacingOccurrences(of: ":", with: "").replacingOccurrences(of: " ", with: "-")
        try writePNG(texture, to: URL(fileURLWithPath: output).appendingPathComponent(String(format: "%02d-%@.png", scene.id, name)))
        let stats = simulation.statistics
        // Approximate: pairs are those of the last substep, queries are summed over the substeps
        let queriesPerPair = Double(queries) / Double(max(pairs * settings.substeps, 1))
        print(scene.name.padding(toLength: 25, withPad: " ", startingAt: 0)
            + String(format: " %6d %8.0f %9.2f / %6.2f %9d %8.2f %6d %8.0f", stats.bodies, loadMilliseconds,
                     total / Double(max(steps, 1)), slowest, stats.contacts,
                     collision / Double(max(steps, 1)), pairs / max(steps, 1), queriesPerPair))
    }
} catch {
    print("error: \(error)")
    exit(1)
}

func writePNG(_ texture: MTLTexture, to url: URL) throws {
    let bytesPerRow = texture.width * 4
    let info = CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedFirst.rawValue | CGBitmapInfo.byteOrder32Little.rawValue)
    // The context owns its pixel memory, the texture is copied straight into it
    guard let context = CGContext(data: nil, width: texture.width, height: texture.height, bitsPerComponent: 8,
                                  bytesPerRow: bytesPerRow, space: CGColorSpace(name: CGColorSpace.sRGB)!, bitmapInfo: info.rawValue),
          let pixels = context.data
    else { throw CocoaError(.fileWriteUnknown) }
    texture.getBytes(pixels, bytesPerRow: bytesPerRow, from: MTLRegionMake2D(0, 0, texture.width, texture.height), mipmapLevel: 0)
    guard let image = context.makeImage(),
          let destination = CGImageDestinationCreateWithURL(url as CFURL, UTType.png.identifier as CFString, 1, nil)
    else { throw CocoaError(.fileWriteUnknown) }
    CGImageDestinationAddImage(destination, image, nil)
    guard CGImageDestinationFinalize(destination) else { throw CocoaError(.fileWriteUnknown) }
}
