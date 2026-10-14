// Times the same pile of tori with SDFCollisionKit and with RealityKit's physics, from 25 to 400
// bodies, and prints a table.
//
//   swift run -c release sdf-bench [steps]
//
// The tori fall into a walled box. For RealityKit, each torus is the best it has for a moving torus:
// a compound of 12 capsules (from a mesh it would generate a convex hull, which closes the hole).
// SDFCollisionKit uses the torus itself. Both run in meters at 60 steps per second; the bodies have
// no meshes, so that rendering costs next to nothing.
//
// SDFCollisionKit's time is that of `SDFSimulation.step(root:)`, synchronization included.
// RealityKit doesn't report its physics time: it is estimated as the time of a frame with the
// physics clock advancing by a step, minus that of a frame with the clock still.

import CoreMedia
import Foundation
import Metal
import RealityKit
import SDFCollisionKit

let steps = CommandLine.arguments.dropFirst().first.flatMap { Int($0) } ?? 300
let counts = [25, 50, 100, 200, 400]
let major: Float = 0.15, minor: Float = 0.04, segments = 12

/// Where the tori start: layers of a 5 × 5 grid above a box 2 m across, tilted
func placements(_ count: Int) -> [(position: SIMD3<Float>, orientation: simd_quatf)] {
    (0..<count).map { i in
        let layer = i / 25, cell = i % 25
        let x = Float(cell % 5) * 0.36 - 0.72, z = Float(cell / 5) * 0.36 - 0.72
        let angle = Float(i % 7) * 0.4
        return ([x, 0.3 + Float(layer) * 0.25, z], simd_quatf(angle: angle, axis: normalize([1, 0, Float(i % 3) - 1])))
    }
}

/// Floor and four walls, 5 m high so that 400 tori stay in, around a box 2 m across: centers and
/// half extents
let walls: [(center: SIMD3<Float>, halfExtents: SIMD3<Float>)] = [
    ([0, -0.5, 0], [2, 0.5, 2]),
    ([-1.05, 2.5, 0], [0.05, 2.5, 1.1]), ([1.05, 2.5, 0], [0.05, 2.5, 1.1]),
    ([0, 2.5, -1.05], [1.1, 2.5, 0.05]), ([0, 2.5, 1.05], [1.1, 2.5, 0.05]),
]

@MainActor
func timeSDF(count: Int, isMultithreaded: Bool) -> (mean: Double, worst: Double, check: String) {
    let root = Entity(), simulation = SDFSimulation()
    var settings = SDFSimulationComponent(simulation: simulation)
    settings.isMultithreaded = isMultithreaded
    root.components.set(settings)
    for wall in walls {
        let entity = Entity()
        entity.position = wall.center
        entity.components.set(SDFBodyComponent(shape: .box(halfExtents: wall.halfExtents), mode: .static))
        root.addChild(entity)
    }
    let torus = SDFShape.torus(majorRadius: major, minorRadius: minor)
    var bodies: [Entity] = []
    for placement in placements(count) {
        let entity = Entity()
        entity.transform = Transform(rotation: placement.orientation, translation: placement.position)
        entity.components.set(SDFBodyComponent(shape: torus))
        root.addChild(entity)
        bodies.append(entity)
    }
    var times: [Double] = []
    for _ in 0..<steps {
        let start = DispatchTime.now().uptimeNanoseconds
        simulation.step(root: root)
        times.append(Double(DispatchTime.now().uptimeNanoseconds - start) / 1e6)
    }
    return (times.reduce(0, +) / Double(times.count), times.max() ?? 0, check(bodies))
}

/// The mean height of the tori and how many left the box, to check that the pile fell and settled
@MainActor
func check(_ bodies: [Entity]) -> String {
    let positions = bodies.map { $0.position(relativeTo: nil) }
    let mean = positions.map(\.y).reduce(0, +) / Float(positions.count)
    let out = positions.filter { abs($0.x) > 1 || abs($0.z) > 1 || $0.y < -0.1 }.count
    return String(format: "mean y %.2f m, %d out", mean, out)
}

@MainActor
func timeRealityKit(count: Int) async throws -> (mean: Double, worst: Double, check: String) {
    var timebase: CMTimebase?
    CMTimebaseCreateWithSourceClock(allocator: nil, sourceClock: CMClockGetHostTimeClock(), timebaseOut: &timebase)
    guard let timebase else { throw CocoaError(.featureUnsupported) }
    CMTimebaseSetRate(timebase, rate: 0)
    CMTimebaseSetTime(timebase, time: .zero)
    let root = Entity()
    var physics = PhysicsSimulationComponent()
    physics.clock = timebase
    root.components.set(physics)
    for wall in walls {
        let entity = Entity()
        entity.position = wall.center
        let shape = ShapeResource.generateBox(size: 2 * wall.halfExtents)
        entity.components.set(CollisionComponent(shapes: [shape]))
        entity.components.set(PhysicsBodyComponent(shapes: [shape], mass: 1, mode: .static))
        root.addChild(entity)
    }
    let chord = 2 * major * sin(.pi / Float(segments))
    let compound = (0..<segments).map { k in
        let angle = (Float(k) + 0.5) * 2 * .pi / Float(segments)
        return ShapeResource.generateCapsule(height: chord + 2 * minor, radius: minor)
            .offsetBy(rotation: simd_quatf(from: [0, 1, 0], to: [-sin(angle), 0, cos(angle)]),
                      translation: [major * cos(angle), 0, major * sin(angle)])
    }
    var bodies: [Entity] = []
    for placement in placements(count) {
        let entity = Entity()
        entity.transform = Transform(rotation: placement.orientation, translation: placement.position)
        entity.components.set(CollisionComponent(shapes: compound))
        entity.components.set(PhysicsBodyComponent(shapes: compound, density: 1000))
        root.addChild(entity)
        bodies.append(entity)
    }

    let renderer = try RealityRenderer()
    let camera = PerspectiveCamera()
    renderer.entities.append(contentsOf: [root, camera])
    renderer.activeCamera = camera
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm_srgb, width: 8, height: 8, mipmapped: false)
    descriptor.usage = [.renderTarget]
    descriptor.storageMode = .private
    let output = try RealityRenderer.CameraOutput(.singleProjection(colorTexture: MTLCreateSystemDefaultDevice()!.makeTexture(descriptor: descriptor)!))

    func frame(advancing: Bool, to step: Int) async throws -> Double {
        if advancing { CMTimebaseSetTime(timebase, time: CMTime(value: CMTimeValue(step), timescale: 60)) }
        let start = DispatchTime.now().uptimeNanoseconds
        try await withCheckedThrowingContinuation { (done: CheckedContinuation<Void, any Error>) in
            do { try renderer.updateAndRender(deltaTime: 1.0 / 60, cameraOutput: output, onComplete: { _ in done.resume() }) }
            catch { done.resume(throwing: error) }
        }
        return Double(DispatchTime.now().uptimeNanoseconds - start) / 1e6
    }
    // The cost of a frame without physics, then frames that each take a physics step
    var baseline: [Double] = []
    for _ in 0..<60 { baseline.append(try await frame(advancing: false, to: 0)) }
    let overhead = baseline.sorted()[baseline.count / 2]
    var times: [Double] = []
    for step in 1...steps { times.append(max(0, try await frame(advancing: true, to: step) - overhead)) }
    return (times.reduce(0, +) / Double(times.count), times.max() ?? 0, check(bodies))
}

print("tori   SDFCollisionKit ms/step (mean / worst)   one thread         RealityKit, 12-capsule tori (estimate)")
for count in counts {
    let sdf = timeSDF(count: count, isMultithreaded: true)
    let single = timeSDF(count: count, isMultithreaded: false)
    let realityKit = try await timeRealityKit(count: count)
    print(String(format: "%4d   %7.2f / %7.2f                      %7.2f / %7.2f  %7.2f / %7.2f", count, sdf.mean, sdf.worst,
                 single.mean, single.worst, realityKit.mean, realityKit.worst))
    print("       after \(steps) steps: SDFCollisionKit \(sdf.check), RealityKit \(realityKit.check)")
}
