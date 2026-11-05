import Foundation
import RealityKit
import SDFCore
import Testing
import simd
@testable import RealitySDFPhysics

/// A frozen copy of the 0.3.1 Swift wrapper, for same-process wrapper-cost comparisons.
/// Both paths deliberately share the current native narrow phase.
@MainActor private enum LegacyQueries {
    static func raycast(_ simulation: SDFSimulation, mask: CollisionGroup = .all,
                        nearest: Bool) -> [SDFSimulation.RaycastHit] {
        var buffer = [SDFRayHit](repeating: SDFRayHit(), count: Int(sdf_world_stats(simulation.world).bodies))
        let count = buffer.withUnsafeMutableBufferPointer {
            sdf_world_raycast(simulation.world, [0, 120, 0], [0, -1, 0], 200,
                              mask.rawValue, $0.baseAddress, Int32($0.count))
        }
        let hits = buffer.prefix(Int(count)).compactMap { hit -> SDFSimulation.RaycastHit? in
            guard let entity = simulation.entity(of: hit.body) else { return nil }
            return .init(entity: entity, position: [hit.position.0, hit.position.1, hit.position.2],
                         normal: [hit.normal.0, hit.normal.1, hit.normal.2], distance: hit.distance)
        }.sorted { $0.distance < $1.distance }
        return nearest ? Array(hits.prefix(1)) : hits
    }

    static func overlaps(_ simulation: SDFSimulation, shape: SDFShape, at position: SIMD3<Float>,
                         mask: CollisionGroup = .all) -> [SDFSimulation.Overlap] {
        let pose = SDFPose(position: (position.x, position.y, position.z), orientation: (0, 0, 0, 1), scale: 1)
        var buffer = [SDFOverlap](repeating: SDFOverlap(), count: Int(sdf_world_stats(simulation.world).bodies))
        let count = buffer.withUnsafeMutableBufferPointer {
            sdf_world_overlaps(simulation.world, shape.handle, pose, 0, mask.rawValue, $0.baseAddress, Int32($0.count))
        }
        return buffer.prefix(Int(count)).compactMap { hit -> SDFSimulation.Overlap? in
            guard let entity = simulation.entity(of: hit.body) else { return nil }
            return .init(entity: entity, position: [hit.position.0, hit.position.1, hit.position.2],
                         normal: [hit.normal.0, hit.normal.1, hit.normal.2], separation: hit.separation)
        }
    }
}

@MainActor @Suite(.serialized) struct SDFQueryPerformanceTests {
    private func scene(count: Int) -> (Entity, SDFSimulation) {
        let root = Entity()
        let simulation = SDFSimulation()
        root.components.set(SDFPhysicsSimulationComponent(simulation: simulation))
        for index in 0..<count {
            let body = Entity()
            body.name = "query-body-\(index)"
            body.position = [0, Float(index) * 2, 0]
            var component = SDFPhysicsBodyComponent(shape: .box(halfExtents: [0.5, 0.5, 0.5]), mode: .static)
            component.filter = .init(group: .init(rawValue: index.isMultiple(of: 2) ? 1 : 2), mask: .all)
            body.components.set(component)
            root.addChild(body)
        }
        simulation.step(root: root)
        return (root, simulation)
    }

    @Test func queriesPreserveOrderingFiltersAndOverlapResults() {
        for count in [1, 57] {
            let (root, simulation) = scene(count: count)
            withExtendedLifetime(root) {
                for mask in [CollisionGroup.all, .init(rawValue: 1), .init(rawValue: 2), .init(rawValue: 0)] {
                    let old = LegacyQueries.raycast(simulation, mask: mask, nearest: false)
                    let current = simulation.raycast(origin: [0, 120, 0], direction: [0, -1, 0], length: 200, mask: mask)
                    #expect(current.map(\.entity) == old.map(\.entity))
                    #expect(current.map(\.distance) == old.map(\.distance))
                    #expect(current.map(\.position) == old.map(\.position))
                    #expect(current.map(\.normal) == old.map(\.normal))
                    for query in [CollisionCastQueryType.nearest, .any] {
                        let single = simulation.raycast(origin: [0, 120, 0], direction: [0, -1, 0], length: 200,
                                                        query: query, mask: mask)
                        #expect(single.map(\.entity) == Array(old.prefix(1)).map(\.entity))
                        #expect(single.map(\.distance) == Array(old.prefix(1)).map(\.distance))
                    }
                    let shape = SDFShape.box(halfExtents: [0.25, 0.25, 0.25])
                    for position in [SIMD3<Float>.zero, [4, 0, 0]] {
                        let oldOverlap = LegacyQueries.overlaps(simulation, shape: shape, at: position, mask: mask)
                        let overlap = simulation.overlaps(of: shape, at: Transform(translation: position), mask: mask)
                        #expect(overlap.map(\.entity) == oldOverlap.map(\.entity))
                        #expect(overlap.map(\.separation) == oldOverlap.map(\.separation))
                        #expect(overlap.map(\.position) == oldOverlap.map(\.position))
                        #expect(overlap.map(\.normal) == oldOverlap.map(\.normal))
                    }
                }
            }
        }
    }

    @Test func queriesTrackEmptyGrowingAndShrinkingWorldsAndChangedGroups() {
        let (root, simulation) = scene(count: 0)
        #expect(simulation.raycast(origin: [0, 10, 0], direction: [0, -1, 0]).isEmpty)
        #expect(simulation.overlaps(of: .sphere(radius: 1), at: .identity).isEmpty)
        let body = Entity()
        var component = SDFPhysicsBodyComponent(shape: .box(halfExtents: [0.5, 0.5, 0.5]), mode: .static)
        component.filter = .init(group: .init(rawValue: 1), mask: .init(rawValue: 0))
        body.components.set(component)
        root.addChild(body)
        simulation.step(root: root)
        // Query masks select the body's group, independently of its physical collision mask.
        #expect(simulation.raycast(origin: [0, 10, 0], direction: [0, -1, 0], mask: .init(rawValue: 1)).count == 1)
        #expect(simulation.overlaps(of: .sphere(radius: 1), at: .identity, mask: .init(rawValue: 1)).count == 1)
        component.filter = .init(group: .init(rawValue: 2), mask: .all)
        body.components.set(component)
        simulation.step(root: root)
        #expect(simulation.raycast(origin: [0, 10, 0], direction: [0, -1, 0], mask: .init(rawValue: 1)).isEmpty)
        #expect(simulation.raycast(origin: [0, 10, 0], direction: [0, -1, 0], mask: .init(rawValue: 2)).count == 1)
        body.position = [10, 0, 0]
        simulation.step(root: root)
        #expect(simulation.raycast(origin: [0, 10, 0], direction: [0, -1, 0]).isEmpty)
        body.removeFromParent()
        simulation.step(root: root)
        #expect(simulation.overlaps(of: .sphere(radius: 100), at: .identity).isEmpty)
    }

    private func measure(_ iterations: Int, _ operation: () -> Int) -> (median: Double, checksum: Int) {
        for _ in 0..<20 { _ = operation() }
        var samples: [Double] = []
        var checksum = 0
        for _ in 0..<5 {
            let start = DispatchTime.now().uptimeNanoseconds
            for _ in 0..<iterations { checksum += operation() }
            let elapsed = DispatchTime.now().uptimeNanoseconds - start
            samples.append(Double(elapsed) / Double(iterations) / 1_000)
        }
        return (samples.sorted()[2], checksum)
    }

    @Test(.enabled(if: ProcessInfo.processInfo.environment["SDF_QUERY_BENCHMARK"] == "1"))
    func controlledWrapperBenchmark() {
        for count in [1, 57] {
            let (root, simulation) = scene(count: count)
            withExtendedLifetime(root) {
                let shape = SDFShape.box(halfExtents: [0.25, 0.25, 0.25])
                let cases: [(String, Int, () -> Int, () -> Int)] = [
                    ("ray-nearest", 3000,
                     { LegacyQueries.raycast(simulation, nearest: true).count },
                     { simulation.raycast(origin: [0, 120, 0], direction: [0, -1, 0], length: 200, query: .nearest).count }),
                    ("ray-all", 3000,
                     { LegacyQueries.raycast(simulation, nearest: false).count },
                     { simulation.raycast(origin: [0, 120, 0], direction: [0, -1, 0], length: 200).count }),
                    ("overlap-clear", 3000,
                     { LegacyQueries.overlaps(simulation, shape: shape, at: [4, 0, 0]).count },
                     { simulation.overlaps(of: shape, at: Transform(translation: [4, 0, 0])).count }),
                    ("overlap-one-hit", 100,
                     { LegacyQueries.overlaps(simulation, shape: shape, at: .zero).count },
                     { simulation.overlaps(of: shape, at: Transform(translation: .zero)).count }),
                    ("overlap-empty-mask", 10000,
                     { LegacyQueries.overlaps(simulation, shape: shape, at: .zero, mask: .init(rawValue: 0)).count },
                     { simulation.overlaps(of: shape, at: Transform(translation: .zero), mask: .init(rawValue: 0)).count })
                ]
                for (label, iterations, legacy, current) in cases {
                    let before = measure(iterations, legacy)
                    let after = measure(iterations, current)
                    #expect(before.checksum == after.checksum)
                    print("QUERY_BENCH bodies=\(count) \(label) iterations=\(iterations)x5 legacy-us=\(before.median) current-us=\(after.median) checksum=\(after.checksum)")
                }
            }
        }
    }
}
