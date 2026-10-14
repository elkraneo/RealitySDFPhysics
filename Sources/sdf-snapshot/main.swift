// Simulates every demo scene and renders it offscreen with RealityKit to a PNG, and prints timings.
//
//   swift run -c release sdf-snapshot [--single-threaded] <output directory> [steps >= 0] [scene index...]
//
// Steps default to 240 (4 s of simulated time).

import CoreGraphics
import Foundation
import ImageIO
import Metal
import RealityKit
import SDFCollisionKit
import SDFDemoScenes
import UniformTypeIdentifiers

func usage() -> Never {
    print("usage: sdf-snapshot [--single-threaded] <output directory> [steps >= 0] [scene index...]")
    exit(1)
}

let isMultithreaded = !CommandLine.arguments.contains("--single-threaded")
let arguments = CommandLine.arguments.dropFirst().filter { $0 != "--single-threaded" }
guard let output = arguments.first else { usage() }
let numbers = arguments.dropFirst().compactMap { Int($0) }
guard numbers.count == arguments.count - 1 else { usage() }
let steps = numbers.first ?? 240
let selected = numbers.count > 1 ? Array(numbers.dropFirst()) : DemoScene.all.map(\.id)
guard steps >= 0, selected.allSatisfy(DemoScene.all.indices.contains) else { usage() }

do {
    try FileManager.default.createDirectory(atPath: output, withIntermediateDirectories: true)
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm_srgb, width: 1280, height: 800,
                                                              mipmapped: false)
    descriptor.usage = [.renderTarget, .shaderRead]
    descriptor.storageMode = .shared
    let texture = MTLCreateSystemDefaultDevice()!.makeTexture(descriptor: descriptor)!

    print("scene                     bodies  load ms  step ms (mean / max)  contacts")
    for index in selected {
        let scene = DemoScene.all[index]
        let simulation = SDFSimulation()
        let root = Entity()
        var settings = SDFSimulationComponent(simulation: simulation, preset: .paper)
        settings.isMultithreaded = isMultithreaded
        root.components.set(settings)

        let loadStart = Date()
        try await scene.load(into: root)
        let loadMilliseconds = Date().timeIntervalSince(loadStart) * 1000
        root.addChild(ContactMarkers.makeEntity())

        var total = 0.0, slowest = 0.0
        for _ in 0..<steps {
            simulation.step(root: root)
            total += simulation.statistics.stepMilliseconds
            slowest = max(slowest, simulation.statistics.stepMilliseconds)
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
        try writePNG(texture, to: URL(fileURLWithPath: output).appendingPathComponent(String(format: "%02d-%@.png", index, name)))
        let stats = simulation.statistics
        print(scene.name.padding(toLength: 25, withPad: " ", startingAt: 0)
            + String(format: " %6d %8.0f %9.2f / %6.2f %9d", stats.bodies, loadMilliseconds,
                     total / Double(max(steps, 1)), slowest, stats.contacts))
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
