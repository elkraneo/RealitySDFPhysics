import Foundation
import RealityKit
import RealitySDFPhysics
import Testing
import simd

@MainActor @Suite struct SDFPlacementOverlapTests {
  // Six thin trapezoidal wall shapes, rebased to their centroids; no site coordinates.
  let walls: [[SIMD3<Float>]] = [
    [[0.715, 5.59, 0.815], [0.715, -6.96, 0.815], [-0.715, -5.59, -0.815], [-0.715, 6.96, -0.815]],
    [[0.68, 5.71, 0.77], [0.68, -4.7, 0.77], [-0.68, -5.71, -0.77], [-0.68, 4.7, -0.77]],
    [[-0.87, 5.185, -0.135], [-0.87, -4.365, -0.135], [0.87, -5.185, 0.135], [0.87, 4.365, 0.135]],
    [[1.1, 4.57, 1.2], [1.1, -6.14, 1.2], [-1.1, -4.57, -1.2], [-1.1, 6.14, -1.2]],
    [[-1.425, 6.155, 1.27], [-1.425, -6.225, 1.27], [1.425, -6.155, -1.27], [1.425, 6.225, -1.27]],
    [[0.685, 6.8775, 0.775], [0.685, -5.5025, 0.775], [-0.685, -6.8825, -0.775], [-0.685, 5.5075, -0.775]],
  ]

  func slab(_ points: [SIMD3<Float>], sourceIndices: [UInt32] = [3,0,1,3,1,2]) -> (points: [SIMD3<Float>], indices: [UInt32], normal: SIMD3<Float>) {
    let normal = simd_normalize(simd_cross(points[1] - points[0], points[2] - points[0]))
    let count = UInt32(points.count)
    var indices: [UInt32] = []
    for i in stride(from: 0, to: sourceIndices.count, by: 3) {
      let a = sourceIndices[i], b = sourceIndices[i + 1], c = sourceIndices[i + 2]
      let facesExtrusion = simd_dot(simd_cross(points[Int(b)] - points[Int(a)], points[Int(c)] - points[Int(a)]), normal) > 0
      let cap = facesExtrusion ? [c,b,a] : [a,b,c]
      indices += cap
      indices += cap.reversed().map { $0 + count }
    }
    for i in 0..<points.count {
      let a = UInt32(i), b = UInt32((i + 1) % points.count)
      indices += [a,b,b + count,a,b + count,a + count]
    }
    return (points + points.map { $0 + normal * 0.2 }, indices, normal)
  }

  func query(_ points: [SIMD3<Float>], indices: [UInt32], centre: SIMD3<Float>, normal: SIMD3<Float>, resolution: Int) throws -> (overlaps: Bool, inside: Float) {
    let shape = try SDFShape(baking: points, indices: indices, resolution: resolution)
    let root = Entity(), sim = SDFSimulation(), wall = Entity()
    root.components.set(SDFPhysicsSimulationComponent(simulation: sim))
    wall.components.set(SDFPhysicsBodyComponent(shape: shape, mode: .static))
    root.addChild(wall)
    sim.step(root: root)
    let panel = SDFShape.box(halfExtents: [0.565,0.02,0.88])
    let overlap = sim.overlaps(of: panel, at: Transform(translation: centre))
    return (overlap.contains { $0.separation < 0 }, shape.distance(at: centre + normal * 0.1).distance)
  }

  @Test func outwardCapsRestoreTheSixThinWallContacts() throws {
    for (index, points) in walls.enumerated() {
      let mesh = slab(points)
      let report = SDFShape.validateMesh(positions: mesh.points, indices: mesh.indices)
      #expect(report.isClosed && report.isConsistentlyOriented)
      let centre = points.reduce(SIMD3<Float>.zero, +) / 4
      // Ground truth: the convex wall polygon passes through the panel's centre. Its two face
      // caps enclose centre + normal * 0.1, independent of SDF/query output.
      for resolution in [64, 128] {
        let result = try query(mesh.points, indices: mesh.indices, centre: centre, normal: mesh.normal, resolution: resolution)
        #expect(result.overlaps, "thin wall \(index), resolution \(resolution)")
        // Coarse trilinear grids can erase part of a 20 cm interior. Do not claim 64-cell sign accuracy.
        if resolution == 128 { #expect(result.inside < 0, "thin wall \(index)") }
      }
    }
  }

  @Test(.enabled(if: ProcessInfo.processInfo.environment["SDF_PLACEMENT_FIXTURE"] != nil))
  func localFullBuildingFixture() throws {
    struct Surface: Decodable { var kind: String; var points: [[Float]]; var triangles: [UInt32] }
    let path = try #require(ProcessInfo.processInfo.environment["SDF_PLACEMENT_FIXTURE"])
    let surfaces = try JSONDecoder().decode([Surface].self, from: Data(contentsOf: URL(fileURLWithPath: path)))
    var tested = 0
    for (index, surface) in surfaces.enumerated() where surface.kind == "wall" {
      let points = surface.points.map { SIMD3<Float>($0[0], $0[2], -$0[1]) }
      let mesh = slab(points, sourceIndices: surface.triangles)
      let report = SDFShape.validateMesh(positions: mesh.points, indices: mesh.indices)
      #expect(report.isClosed && report.isConsistentlyOriented, "wall \(index)")
      let centre = points.reduce(SIMD3<Float>.zero, +) / Float(points.count)
      for resolution in [64, 128] {
        let result = try query(mesh.points, indices: mesh.indices, centre: centre, normal: mesh.normal, resolution: resolution)
        #expect(result.overlaps, "wall \(index), resolution \(resolution)")
        if resolution == 128 { #expect(result.inside < 0, "wall \(index), 128-cell interior") }
      }
      tested += 1
    }
    #expect(tested == 57)
    print("Verified \(tested) consistently wound walls at 64 and 128 cells")
  }
}
