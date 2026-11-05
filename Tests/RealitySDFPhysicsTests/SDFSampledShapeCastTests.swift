import RealityKit
import RealitySDFPhysics
import Testing
import simd

@MainActor
@Suite struct SDFSampledShapeCastTests {
    let root = Entity()
    let simulation = SDFSimulation()

    init() { root.components.set(SDFPhysicsSimulationComponent(simulation: simulation)) }

    @discardableResult
    func obstacle(_ shape: SDFShape = .box(halfExtents: [0.05, 2, 2]),
                  at position: SIMD3<Float> = .zero, group: CollisionGroup = .default) -> Entity {
        let entity = Entity()
        entity.position = position
        var body = SDFPhysicsBodyComponent(shape: shape, mode: .static)
        body.filter = CollisionFilter(group: group, mask: .all)
        entity.components.set(body)
        root.addChild(entity)
        return entity
    }

    @Test func crossesAnObstacleWhenBothEndpointsAreClear() throws {
        let wall = obstacle()
        simulation.step(root: root)
        let shape = SDFShape.box(halfExtents: [0.2, 0.2, 0.2])
        let from = Transform(translation: [-2, 0, 0]), to = Transform(translation: [2, 0, 0])
        #expect(simulation.overlaps(of: shape, at: from).isEmpty)
        #expect(simulation.overlaps(of: shape, at: to).isEmpty)
        let beforeStep = simulation.stepCount
        let result = try simulation.sampledShapeCast(of: shape, from: from, to: to, maxTranslationStep: 0.02)
        guard case let .hit(hit) = result else { Issue.record("Expected a sampled wall hit"); return }
        #expect(hit.fraction > 0.4 && hit.fraction < 0.5)
        #expect(hit.previousSampleFraction != nil)
        #expect(hit.contacts.contains { $0.entity == wall })
        #expect(simulation.stepCount == beforeStep)
        #expect(wall.position == .zero)
    }

    @Test func returnsInitialOverlapAtZero() throws {
        let wall = obstacle()
        simulation.step(root: root)
        let result = try simulation.sampledShapeCast(of: .sphere(radius: 0.2), from: Transform(), to: Transform(translation: [2, 0, 0]))
        guard case let .hit(hit) = result else { Issue.record("Expected initial overlap"); return }
        #expect(hit.fraction == 0)
        #expect(hit.previousSampleFraction == nil)
        #expect(hit.sampleCount == 1)
        #expect(hit.contacts.contains { $0.entity == wall })
    }

    @Test func rotationAloneFindsAnIntermediateContact() throws {
        let target = obstacle(.sphere(radius: 0.15), at: [0.7, 0, -0.7])
        simulation.step(root: root)
        let bar = SDFShape.box(halfExtents: [1, 0.1, 0.1])
        let start = Transform(), end = Transform(rotation: simd_quatf(angle: .pi / 2, axis: [0, 1, 0]))
        #expect(simulation.overlaps(of: bar, at: start).isEmpty)
        #expect(simulation.overlaps(of: bar, at: end).isEmpty)
        let result = try simulation.sampledShapeCast(of: bar, from: start, to: end, maxRotationStep: .pi / 180)
        guard case let .hit(hit) = result else { Issue.record("Expected rotation-only contact"); return }
        #expect(hit.fraction > 0 && hit.fraction < 1)
        #expect(hit.contacts.contains { $0.entity == target })
    }

    @Test func choosesTheShortArcAcrossTheQuaternionSignBoundary() throws {
        obstacle(.sphere(radius: 0.15), at: [0, 0, 1])
        simulation.step(root: root)
        let from = Transform(rotation: simd_quatf(angle: 179 * .pi / 180, axis: [0, 1, 0]))
        let to = Transform(rotation: simd_quatf(angle: -179 * .pi / 180, axis: [0, 1, 0]))
        let result = try simulation.sampledShapeCast(of: .box(halfExtents: [1.5, 0.1, 0.1]), from: from, to: to)
        guard case let .noHitInSamples(count) = result else { Issue.record("The short arc should not rotate through the obstacle"); return }
        #expect(count <= 4)
    }

    @Test func maskAndExactEntityExclusionSelectTargets() throws {
        let panels = CollisionGroup(rawValue: 1 << 4), walls = CollisionGroup(rawValue: 1 << 5)
        let selfBody = obstacle(.box(halfExtents: [0.2, 0.2, 0.2]), at: [-2, 0, 0], group: panels)
        let wall = obstacle(group: walls)
        simulation.step(root: root)
        let shape = SDFShape.box(halfExtents: [0.2, 0.2, 0.2])
        let from = Transform(translation: [-2, 0, 0]), to = Transform(translation: [2, 0, 0])
        let excluded = try simulation.sampledShapeCast(of: shape, from: from, to: to, excluding: [selfBody])
        guard case let .hit(hit) = excluded else { Issue.record("Expected wall beyond excluded self"); return }
        #expect(hit.fraction > 0)
        #expect(hit.contacts.allSatisfy { $0.entity == wall })
        let masked = try simulation.sampledShapeCast(of: shape, from: from, to: to, mask: panels, excluding: [selfBody])
        guard case .noHitInSamples = masked else { Issue.record("Walls should be outside the query mask"); return }
    }

    @Test func sampleBudgetCannotSilentlyReportAnUncheckedPathClear() throws {
        simulation.step(root: root)
        let result = try simulation.sampledShapeCast(of: .sphere(radius: 0.2), from: Transform(), to: Transform(translation: [10, 0, 0]), maxTranslationStep: 0.1, maxSamples: 3)
        guard case let .incomplete(fraction, samples) = result else { Issue.record("Expected explicit incomplete result"); return }
        #expect(samples == 3)
        #expect(fraction > 0 && fraction < 0.03)
    }

    /// 512 samples at the default 1 cm step cover about 5.1 m; a 6 m drag must not report clear.
    @Test func theDefaultBudgetEndsShortOfALongDrag() throws {
        obstacle(.box(halfExtents: [0.5, 0.5, 0.5]), at: [50, 0, 0])
        simulation.step(root: root)
        let short = try simulation.sampledShapeCast(of: .sphere(radius: 0.1), from: Transform(), to: Transform(translation: [5, 0, 0]))
        guard case let .noHitInSamples(count) = short else { Issue.record("Expected 5 m to fit, got \(short)"); return }
        #expect(count > 500 && count <= 512)
        let long = try simulation.sampledShapeCast(of: .sphere(radius: 0.1), from: Transform(), to: Transform(translation: [6, 0, 0]))
        guard case let .incomplete(fraction, samples) = long else { Issue.record("Expected incomplete, got \(long)"); return }
        #expect(samples == 512)
        #expect(fraction > 0.8 && fraction < 0.9)
    }

    @Test func destinationIsCheckedAndZeroMotionUsesOneSample() throws {
        simulation.step(root: root)
        let shape = SDFShape.sphere(radius: 0.2)
        let zero = try simulation.sampledShapeCast(of: shape, from: Transform(), to: Transform())
        guard case let .noHitInSamples(count) = zero else { Issue.record("Expected clear stationary query"); return }
        #expect(count == 1)
        let result = try simulation.sampledShapeCast(of: shape, from: Transform(), to: Transform(translation: [1, 0, 0]), maxTranslationStep: 0.5, maxSamples: 3)
        guard case let .noHitInSamples(count) = result else { Issue.record("Budget includes initial and final sample"); return }
        #expect(count == 3)
    }

    @Test func aDestinationOnlyContactIsNotSkipped() throws {
        obstacle(.sphere(radius: 0.1), at: [10, 0, 0])
        simulation.step(root: root)
        let result = try simulation.sampledShapeCast(of: .sphere(radius: 0.1), from: Transform(),
                                                   to: Transform(translation: [10, 0, 0]),
                                                   maxTranslationStep: 10, maxSamples: 2)
        guard case let .hit(hit) = result else { Issue.record("Expected destination contact"); return }
        #expect(hit.fraction == 1)
        #expect(hit.sampleCount == 2)
    }

    /// In Float, 2 * acos(dot) rounds a rotation under about 5e-4 radians to zero. With no
    /// translation the cast then checked only the start and missed a destination contact.
    @Test func aTinyRotationStillChecksTheDestination() throws {
        let block = obstacle(.box(halfExtents: [0.5, 0.5, 0.5]), at: [0, 0, 0.5 + 0.05 + 0.0002])
        simulation.step(root: root)
        let bar = SDFShape.box(halfExtents: [1, 0.05, 0.05])
        let start = Transform(), end = Transform(rotation: simd_quatf(angle: -4e-4, axis: [0, 1, 0]))
        #expect(simulation.overlaps(of: bar, at: start).isEmpty)
        #expect(!simulation.overlaps(of: bar, at: end).isEmpty)
        let result = try simulation.sampledShapeCast(of: bar, from: start, to: end)
        guard case let .hit(hit) = result else { Issue.record("Expected the destination contact, got \(result)"); return }
        #expect(hit.fraction == 1)
        #expect(hit.sampleCount == 2)
        #expect(hit.previousSampleFraction == 0)
        #expect(hit.contacts.contains { $0.entity == block })
    }

    @Test func theSameOrientationWrittenDifferentlyIsNoMotion() throws {
        simulation.step(root: root)
        let q = simd_quatf(vector: [0, 0, 0, 2])
        let same = try simulation.sampledShapeCast(of: .sphere(radius: 0.1), from: Transform(rotation: q), to: Transform(rotation: q))
        guard case let .noHitInSamples(count) = same else { Issue.record("Expected one clear sample"); return }
        #expect(count == 1)
        let flipped = try simulation.sampledShapeCast(of: .sphere(radius: 0.1), from: Transform(rotation: q),
                                                      to: Transform(rotation: simd_quatf(vector: -q.vector)))
        guard case let .noHitInSamples(count) = flipped else { Issue.record("Expected one clear sample"); return }
        #expect(count == 1)
    }

    @Test func coarseSamplesExplicitlyDoNotGuaranteeContinuousCollision() throws {
        obstacle(.box(halfExtents: [0.001, 1, 1]))
        simulation.step(root: root)
        let shape = SDFShape.sphere(radius: 0.01)
        let from = Transform(translation: [-1, 0, 0]), to = Transform(translation: [1, 0, 0])
        let coarse = try simulation.sampledShapeCast(of: shape, from: from, to: to, maxTranslationStep: 2)
        guard case .noHitInSamples = coarse else { Issue.record("Both coarse samples should be clear"); return }
        let fine = try simulation.sampledShapeCast(of: shape, from: from, to: to, maxTranslationStep: 0.01)
        guard case .hit = fine else { Issue.record("Fine samples should detect the wall"); return }
    }

    @Test func rejectsInvalidInputsAndChangingScale() throws {
        let shape = SDFShape.sphere(radius: 0.2)
        #expect(throws: SDFSimulation.SampledShapeCastError.invalidStep) {
            try simulation.sampledShapeCast(of: shape, from: Transform(), to: Transform(), maxTranslationStep: 0)
        }
        #expect(throws: SDFSimulation.SampledShapeCastError.invalidSampleBudget) {
            try simulation.sampledShapeCast(of: shape, from: Transform(), to: Transform(), maxSamples: 0)
        }
        #expect(throws: SDFSimulation.SampledShapeCastError.invalidMargin) {
            try simulation.sampledShapeCast(of: shape, from: Transform(), to: Transform(), margin: .nan)
        }
        #expect(throws: SDFSimulation.SampledShapeCastError.unsupportedScale) {
            try simulation.sampledShapeCast(of: shape, from: Transform(), to: Transform(scale: [2, 2, 2]))
        }
        #expect(throws: SDFSimulation.SampledShapeCastError.unsupportedScale) {
            try simulation.sampledShapeCast(of: shape, from: Transform(scale: [1, 2, 1]), to: Transform(scale: [1, 2, 1]))
        }
        #expect(throws: SDFSimulation.SampledShapeCastError.invalidTransform) {
            try simulation.sampledShapeCast(of: shape, from: Transform(translation: [.infinity, 0, 0]), to: Transform())
        }
    }
}
