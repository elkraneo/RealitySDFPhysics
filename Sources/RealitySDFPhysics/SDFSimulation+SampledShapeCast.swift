import RealityKit
import simd

extension SDFSimulation {
    /// The first sampled pose at which the overlap query found a contact.
    public struct SampledShapeCastHit {
        /// Fraction of the requested motion, in 0...1. This is not an exact time of impact.
        public let fraction: Float
        public let transform: Transform
        public let contacts: [Overlap]
        /// The preceding sampled fraction, or nil for an initial overlap. A clear sample does not
        /// prove the intervening path was clear.
        public let previousSampleFraction: Float?
        public let sampleCount: Int
    }

    public enum SampledShapeCastResult {
        case hit(SampledShapeCastHit)
        /// All requested samples were checked, not proof of a continuously clear path.
        case noHitInSamples(sampleCount: Int)
        /// The budget ended before the destination. Must not be treated as a clear result.
        case incomplete(lastSampleFraction: Float, sampleCount: Int)
    }

    public enum SampledShapeCastError: Error, Equatable {
        case invalidStep
        case invalidSampleBudget
        case invalidMargin
        case invalidTransform
        case unsupportedScale
    }

    /// Samples an SDF shape along a translation and shortest-arc rotation, without moving bodies
    /// or advancing the simulation. All poses and distances are in the simulation root's frame.
    ///
    /// This is **not continuous collision detection**. Thin obstacles can fall between samples,
    /// and this inherits the overlap query's approximate SDF geometry and contact search. In
    /// particular, `.noHitInSamples` is not a no-tunnelling guarantee. Rotation samples are bounded in
    /// radians, not in linear corner travel: choose a smaller angle for larger shapes.
    ///
    /// The initial and destination poses are included if the budget permits. Scale must be
    /// positive, uniform and unchanged. The mask selects target body groups (as with `overlaps`);
    /// it does not apply a querying body's reciprocal collision mask. Exclusions apply to exact
    /// entities, not their descendants. They filter query results, not broadphase work.
    ///
    /// The world must already have been synchronized with the root via the normal simulation
    /// update. This method queries that snapshot; it does not register pending entity changes.
    public func sampledShapeCast(
        of shape: SDFShape,
        from start: Transform,
        to end: Transform,
        maxTranslationStep: Float = 0.01,
        maxRotationStep: Float = .pi / 180,
        maxSamples: Int = 512,
        margin: Float = 0,
        mask: CollisionGroup = .all,
        excluding entities: [Entity] = []
    ) throws -> SampledShapeCastResult {
        guard maxTranslationStep.isFinite, maxTranslationStep > 0,
              maxRotationStep.isFinite, maxRotationStep > 0 else {
            throw SampledShapeCastError.invalidStep
        }
        guard maxSamples > 0 else { throw SampledShapeCastError.invalidSampleBudget }
        guard margin.isFinite, margin >= 0 else { throw SampledShapeCastError.invalidMargin }
        func finite(_ v: SIMD3<Float>) -> Bool { v.x.isFinite && v.y.isFinite && v.z.isFinite }
        func valid(_ pose: Transform) -> Bool {
            let q = pose.rotation.vector
            let length = simd_length(q)
            return finite(pose.translation) && finite(pose.scale)
                && q.x.isFinite && q.y.isFinite && q.z.isFinite && q.w.isFinite
                && length.isFinite && length > 0
        }
        guard valid(start), valid(end) else { throw SampledShapeCastError.invalidTransform }
        guard start.scale.x > 0, start.scale.x == start.scale.y, start.scale.x == start.scale.z,
              start.scale == end.scale else { throw SampledShapeCastError.unsupportedScale }

        let distance = simd_distance(start.translation, end.translation)
        guard distance.isFinite else { throw SampledShapeCastError.invalidTransform }
        let startRotation = simd_normalize(start.rotation)
        var endRotation = simd_normalize(end.rotation)
        var dot = simd_dot(startRotation.vector, endRotation.vector)
        if dot < 0 {
            endRotation = simd_quatf(vector: -endRotation.vector)
            dot = -dot
        }
        let angle = 2 * acos(min(1, max(0, dot)))
        // Double avoids conversion overflow for finite but extremely small requested steps.
        let intervals = max(ceil(Double(distance) / Double(maxTranslationStep)),
                            ceil(Double(angle) / Double(maxRotationStep)))
        let excluded = Set(entities.map { ObjectIdentifier($0) })
        var previous: Float?
        for sample in 0..<maxSamples {
            let fraction = intervals == 0 ? Float(0) : Float(Double(sample) / intervals)
            let pose = Transform(scale: start.scale,
                                 rotation: simd_slerp(startRotation, endRotation, fraction),
                                 translation: start.translation + (end.translation - start.translation) * fraction)
            let contacts = overlaps(of: shape, at: pose, margin: margin, mask: mask).filter {
                !excluded.contains(ObjectIdentifier($0.entity)) && $0.separation <= margin
            }
            if !contacts.isEmpty {
                return .hit(SampledShapeCastHit(fraction: fraction, transform: pose, contacts: contacts,
                                               previousSampleFraction: previous, sampleCount: sample + 1))
            }
            if intervals == 0 || Double(sample) >= intervals { return .noHitInSamples(sampleCount: sample + 1) }
            previous = fraction
        }
        return .incomplete(lastSampleFraction: previous ?? 0, sampleCount: maxSamples)
    }
}
