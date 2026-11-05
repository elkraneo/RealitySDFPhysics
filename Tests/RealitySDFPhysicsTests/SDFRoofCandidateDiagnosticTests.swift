import Foundation
import RealityKit
import RealitySDFPhysics
import Testing
import simd

/// Local investigation only: the fixture is supplied by the host, never bundled in the library.
@MainActor @Suite struct SDFRoofCandidateDiagnosticTests {
    struct Surface: Decodable {
        var kind: String
        var points: [[Float]]
        var triangles: [UInt32]
        var positions: [SIMD3<Float>] { points.map { [$0[0], $0[2], -$0[1]] } }
    }
    struct Triangle { var surface: Int; var points: [SIMD3<Float>] }
    let half = SIMD3<Float>(0.565, 0.02, 0.88)

    func rotation(_ normal: SIMD3<Float>, landscape: Bool) -> simd_quatf {
        let contour = simd_cross(SIMD3<Float>(0,1,0), normal)
        let width = simd_length(contour) > 0.001 ? simd_normalize(contour) : SIMD3<Float>(1,0,0)
        let length = simd_normalize(simd_cross(width, normal))
        return simd_quatf(simd_float3x3(columns: (width,normal,length))) * simd_quatf(angle: landscape ? .pi/2 : 0, axis: [0,1,0])
    }

    func slab(_ surface: Surface) -> (positions: [SIMD3<Float>], indices: [UInt32], inward: SIMD3<Float>) {
        let points = surface.positions
        let n = simd_normalize(points.indices.reduce(SIMD3<Float>.zero) { $0 + simd_cross(points[$1], points[($1 + 1) % points.count]) })
        let offset = n * 0.2
        let count = UInt32(points.count)
        var indices: [UInt32] = []
        for i in stride(from: 0, to: surface.triangles.count, by: 3) {
            let a = surface.triangles[i], b = surface.triangles[i+1], c = surface.triangles[i+2]
            let normal = simd_cross(points[Int(b)] - points[Int(a)], points[Int(c)] - points[Int(a)])
            let cap = simd_dot(normal,n) > 0 ? [c,b,a] : [a,b,c]
            indices += cap
            indices += cap.reversed().map { $0 + count }
        }
        for i in points.indices {
            let a = UInt32(i), b = UInt32((i+1) % points.count)
            indices += [a,b,b+count,a,b+count,a+count]
        }
        var positions = points + points.map { $0 + offset }
        if surface.kind == "roof", offset.y > 0 { positions = positions.map { $0 - offset } }
        return (positions,indices,surface.kind == "roof" && offset.y > 0 ? -n : n)
    }

    /// SAT in the panel frame; also returns a point of actual triangle/box intersection.
    func witness(_ triangle: [SIMD3<Float>], pose: Transform) -> (point: SIMD3<Float>, area: Float, depth: Float)? {
        var polygon = triangle.map { pose.rotation.inverse.act($0 - pose.translation) }
        let original = polygon
        let edges = [original[1]-original[0],original[2]-original[1],original[0]-original[2]]
        let basis: [SIMD3<Float>] = [[1,0,0],[0,1,0],[0,0,1]]
        let axes = basis + [simd_cross(edges[0],edges[1])] + edges.flatMap { edge in basis.map { simd_cross(edge,$0) } }
        var depth = Float.infinity
        for axis in axes where simd_length_squared(axis) > 1e-10 {
            let n = simd_normalize(axis)
            let projected = original.map { simd_dot($0,n) }
            let radius = simd_dot(half,abs(n))
            // Minimum translation of the finite triangle/box pair along a separating axis,
            // rather than penetration of the box into an infinite supporting plane.
            depth = min(depth,min(radius-projected.min()!,projected.max()!+radius))
        }
        for axis in 0..<3 {
            for sign in [Float(-1),1] {
                guard !polygon.isEmpty else { return nil }
                var next: [SIMD3<Float>] = []
                var previous = polygon.last!
                var previousDistance = sign * previous[axis] - half[axis]
                for current in polygon {
                    let currentDistance = sign * current[axis] - half[axis]
                    if (previousDistance <= 0) != (currentDistance <= 0) {
                        let t = previousDistance / (previousDistance - currentDistance)
                        next.append(previous + (current - previous) * t)
                    }
                    if currentDistance <= 0 { next.append(current) }
                    previous = current
                    previousDistance = currentDistance
                }
                polygon = next
            }
        }
        guard !polygon.isEmpty else { return nil }
        let area = simd_length(polygon.indices.reduce(SIMD3<Float>.zero) { $0 + simd_cross(polygon[$1],polygon[($1+1)%polygon.count]) }) / 2
        return (pose.translation + pose.rotation.act(polygon.reduce(SIMD3<Float>.zero,+) / Float(polygon.count)),area,depth)
    }

    @Test(.enabled(if: ProcessInfo.processInfo.environment["SDF_THIN_BOX_DIAGNOSTIC"] != nil))
    func axisAlignedThinBoxHasAnInteriorNode() throws {
        let box = try SDFShape.bakedBox(halfExtents: [6.2,0.1,2])
        let topology = SDFShape.validateMesh(positions: box.surface.positions,indices: box.surface.indices)
        #expect(topology.isClosed && topology.isConsistentlyOriented)
        for y in [Float(0),0.09,-0.09] {
            let distance = box.distance(at: [0,y,0]).distance
            print("THIN BOX y=\(y) distance=\(distance)")
            #expect(distance < 0, "known interior of a consistently wound12.4x0.2x4 box")
        }
    }

    @Test(.enabled(if: ProcessInfo.processInfo.environment["SDF_ROOF_CANDIDATE_DIAGNOSTIC"] != nil))
    func compareRoofCandidatesAgainstActualTriangles() throws {
        let path = try #require(ProcessInfo.processInfo.environment["SDF_ROOF_CANDIDATE_DIAGNOSTIC"])
        let surfaces = try JSONDecoder().decode([Surface].self, from: Data(contentsOf: URL(fileURLWithPath: path)))
        let root = Entity(), simulation = SDFSimulation()
        root.components.set(SDFPhysicsSimulationComponent(simulation: simulation))
        var frames: [Transform] = []
        var serializedBytes = 0
        var gridBytes: Int64 = 0
        var maxCell: Float = 0
        let aligned = ProcessInfo.processInfo.environment["SDF_ROOF_DIAGNOSTIC_ALIGNED"] == "1"
        let nodeWinding = ProcessInfo.processInfo.environment["SDF_ROOF_DIAGNOSTIC_NODE_WINDING"] == "1"
        let clock = ContinuousClock()
        let started = clock.now
        var shapes: [SDFShape] = [], inward: [SIMD3<Float>] = [], triangles: [Triangle] = []
        let resolution = Int(ProcessInfo.processInfo.environment["SDF_ROOF_DIAGNOSTIC_RESOLUTION"] ?? "64") ?? 64
        for (index,surface) in surfaces.enumerated() {
            let mesh = slab(surface)
            let topology = SDFShape.validateMesh(positions: mesh.positions, indices: mesh.indices)
            #expect(topology.isClosed && topology.isConsistentlyOriented, "surface \(index)")
            let frame: Transform
            if aligned {
                let centre = surface.positions.reduce(SIMD3<Float>.zero,+) / Float(surface.points.count)
                let normal = mesh.inward
                let edges = surface.positions.indices.map { surface.positions[($0+1)%surface.points.count] - surface.positions[$0] }
                let longest = edges.max { simd_length_squared($0) < simd_length_squared($1) }!
                let x = simd_normalize(longest - normal * simd_dot(longest,normal))
                let z = simd_normalize(simd_cross(x,normal))
                frame = Transform(rotation: simd_quatf(simd_float3x3(columns:(x,normal,z))),translation: centre)
            } else { frame = Transform() }
            let localPositions = mesh.positions.map { frame.rotation.inverse.act($0-frame.translation) }
            let error = zip(localPositions,mesh.positions).map { simd_distance(frame.translation+frame.rotation.act($0),$1) }.max() ?? 0
            #expect(error < 0.00001, "local frame must preserve source geometry")
            let shape = try SDFShape(baking: localPositions, indices: mesh.indices, resolution: resolution,signMethod: nodeWinding ? .nodeWinding : .legacy)
            serializedBytes += shape.bakedData()?.count ?? 0
            if let grid = shape.bakedGridInfo { gridBytes += grid.gridPayloadBytes; maxCell = max(maxCell,grid.cellSize) }
            frames.append(frame)
            let entity = Entity()
            entity.transform = frame
            entity.name = "surface-\(index)"
            entity.components.set(SDFPhysicsBodyComponent(shape: shape, mode: .static))
            root.addChild(entity)
            shapes.append(shape)
            inward.append(mesh.inward)
            for i in stride(from: 0, to: surface.triangles.count, by: 3) {
                triangles.append(Triangle(surface: index, points: (0..<3).map { surface.positions[Int(surface.triangles[i+$0])] }))
            }
        }
        let bakeDuration = clock.now-started
        simulation.step(root: root)
        let queryStarted = clock.now
        let panel = SDFShape.box(halfExtents: half)
        var samples = 0, truePositive = 0, falseNegative = 0, falsePositive = 0, trueNegative = 0
        var worst: (pose: Transform, source: Int, blocker: Int, witness: SIMD3<Float>, depth: Float, area: Float)?
        let margins: [Float] = ProcessInfo.processInfo.environment["SDF_ROOF_DIAGNOSTIC_SKIP_MARGINS"] == "1" ? [] : [0.03,0.05]
        var recovered = [Int](repeating: 0,count: margins.count)
        var remainingMisses = recovered, addedBlocks = recovered
        var nonGrazingMisses = 0
        var maxMissDepth: Float = 0
        var maxMissArea: Float = 0
        var missesOver1mm = 0, missesOver5mm = 0, missesOver10mm = 0
        for triangle in triangles where surfaces[triangle.surface].kind == "roof" {
            let p = triangle.points
            var normal = simd_normalize(simd_cross(p[1]-p[0], p[2]-p[0]))
            if normal.y < 0 { normal = -normal }
            for u in [Float(0.2),0.4,0.6] {
                for v in [Float(0.2),0.4] where u+v < 1 {
                    for landscape in [false,true] {
                        let centre = p[0] + (p[1]-p[0])*u + (p[2]-p[0])*v + normal*0.2
                        let pose = Transform(rotation: rotation(normal,landscape: landscape),translation: centre)
                        let hits = simulation.overlaps(of: panel,at: pose).filter { $0.separation < 0 }
                        let intersections = triangles.compactMap { other -> (Triangle,SIMD3<Float>,Float,Float)? in
                            guard let point = witness(other.points,pose: pose) else { return nil }
                            return (other,point.point,point.area,point.depth)
                        }
                        for (i,margin) in margins.enumerated() {
                            let blocked = !simulation.overlaps(of: panel,at: pose,margin: margin).isEmpty
                            if !intersections.isEmpty && !blocked { remainingMisses[i] += 1 }
                            if intersections.isEmpty && blocked { addedBlocks[i] += 1 }
                            if !intersections.isEmpty && hits.isEmpty && blocked { recovered[i] += 1 }
                        }
                        samples += 1
                        if intersections.isEmpty { if hits.isEmpty { trueNegative += 1 } else { falsePositive += 1 } }
                        else if !hits.isEmpty { truePositive += 1 }
                        else {
                            falseNegative += 1
                            var nonGrazing = false
                            var candidateDepth: Float = 0
                            for (other,point,area,depth) in intersections {
                                maxMissDepth = max(maxMissDepth,depth)
                                maxMissArea = max(maxMissArea,area)
                                candidateDepth = max(candidateDepth,depth)
                                if area > 0.0001 && depth > 0.0001 { nonGrazing = true }
                                if worst == nil || area > worst!.area { worst = (pose,triangle.surface,other.surface,point,depth,area) }
                            }
                            if nonGrazing { nonGrazingMisses += 1 }
                            if candidateDepth > 0.001 { missesOver1mm += 1 }
                            if candidateDepth > 0.005 { missesOver5mm += 1 }
                            if candidateDepth > 0.01 { missesOver10mm += 1 }
                        }
                    }
                }
            }
        }
        print("BAKE nodeWinding=\(nodeWinding) aligned=\(aligned) duration=\(bakeDuration) serializedBytes=\(serializedBytes) gridPayloadBytes=\(gridBytes) maxCell=\(maxCell) queryDuration=\(clock.now-queryStarted)")
        print("ROOF COMPARISON res=\(resolution) samples=\(samples) TP=\(truePositive) FN=\(falseNegative) FP=\(falsePositive) TN=\(trueNegative)")
        print("MARGIN remaining misses=\(remainingMisses) reference-clear blocks=\(addedBlocks) nongrazing base misses=\(nonGrazingMisses)")
        print("MISS SEVERITY maxFiniteSATDepth=\(maxMissDepth) maxClipArea=\(maxMissArea) candidatesOver1mm=\(missesOver1mm) over5mm=\(missesOver5mm) over10mm=\(missesOver10mm)")
        if let worst {
            let shape = shapes[worst.blocker]
            let point = worst.witness
            print("RECOVERED WITH MARGIN \(zip(margins,recovered).map { "\($0):\($1)" })")
            print("WORST source=\(worst.source) blocker=\(worst.blocker) clipArea=\(worst.area) finiteSATDepth=\(worst.depth) pose=\(worst.pose.translation) quat=\(worst.pose.rotation.vector) witness=\(point)")
            for offset in [Float(-0.02),-0.01,0,0.01,0.02,0.05,0.1] {
                let p = point + inward[worst.blocker]*offset
                let local = worst.pose.rotation.inverse.act(p-worst.pose.translation)
                let insidePanel = all(abs(local) .< half)
                print("WORST offset=\(offset) sdf=\(shape.distance(at:frames[worst.blocker].rotation.inverse.act(p-frames[worst.blocker].translation)).distance) insidePanel=\(insidePanel)")
            }
        }
    }
}
