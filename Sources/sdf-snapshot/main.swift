// Renders every built-in scene offscreen with RealityKit after simulating it, and writes PNGs.
//
//   swift run -c release sdf-snapshot <output directory> [steps] [scene index...]
//
// Steps default to 240 (4 s of simulated time). Useful to compare with the upstream web demo,
// and to check the RealityKit side without a window.

import CoreGraphics
import Foundation
import ImageIO
import Metal
import RealityKit
import SDFCollisionKit
import UniformTypeIdentifiers

@MainActor
func run() throws {
    func usage() -> Never {
        print("usage: sdf-snapshot <output directory> [steps >= 0] [scene index...]")
        exit(1)
    }
    let arguments = CommandLine.arguments.dropFirst()
    guard let output = arguments.first else { usage() }
    let numbers = arguments.dropFirst().compactMap { Int($0) }
    guard numbers.count == arguments.dropFirst().count else { usage() }
    let steps = numbers.first ?? 240
    let selected = numbers.count > 1 ? Array(numbers.dropFirst()) : SDFWorld.scenes.map(\.id)
    guard steps >= 0, selected.allSatisfy(SDFWorld.scenes.indices.contains) else { usage() }
    try FileManager.default.createDirectory(atPath: output, withIntermediateDirectories: true)

    let width = 1280, height = 800
    let device = MTLCreateSystemDefaultDevice()!
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm_srgb, width: width, height: height, mipmapped: false)
    descriptor.usage = [.renderTarget, .shaderRead]
    descriptor.storageMode = .shared
    let texture = device.makeTexture(descriptor: descriptor)!

    for index in selected {
        // Simulate headless, then mirror the final state into entities once
        let world = SDFWorld()
        world.loadScene(index)
        for _ in 0..<steps { world.step() }
        let simulation = SDFSimulation(world: world)
        simulation.synchronize()
        simulation.showsContacts = true

        let renderer = try RealityRenderer()
        let camera = SDFDemoStage.makeCamera()
        SDFDemoStage.place(camera, at: simulation.world.camera)
        renderer.entities.append(contentsOf: [simulation.root, SDFDemoStage.makeFloor(), SDFDemoStage.makeLight(), camera])
        renderer.activeCamera = camera
        renderer.cameraSettings.colorBackground = .color(CGColor(red: 0.62, green: 0.70, blue: 0.80, alpha: 1))

        let done = DispatchSemaphore(value: 0)
        let cameraOutput = try RealityRenderer.CameraOutput(.singleProjection(colorTexture: texture))
        try renderer.updateAndRender(deltaTime: 0, cameraOutput: cameraOutput, onComplete: { _ in done.signal() })
        done.wait()

        let name = SDFWorld.scenes[index].name.replacingOccurrences(of: ":", with: "").replacingOccurrences(of: " ", with: "-")
        let url = URL(fileURLWithPath: output).appendingPathComponent(String(format: "%02d-%@.png", index, name))
        try writePNG(texture, to: url)
        print("\(url.path)  (\(simulation.world.stats.contacts) contacts)")
    }
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

MainActor.assumeIsolated {
    do { try run() } catch {
        print("error: \(error)")
        exit(1)
    }
}
