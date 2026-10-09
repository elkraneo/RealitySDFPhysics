internal import SDFCore

extension SDFSimulation {
    /// Collision detection and solver parameters. The defaults are those of the paper (Sec. 4).
    /// Lengths are in the simulation root's units; the defaults suit objects about 1 unit across.
    public struct Parameters: Equatable, Sendable {
        public enum Optimizer: Int32, Sendable, CaseIterable {
            case nelderMead, ellipsoid, gradientDescent
        }

        public var optimizer: Optimizer
        /// Points kept in the temporal cache of each pair (n_cache)
        public var cachedSamples: Int
        /// New stochastic samples per pair and step (n_rand)
        public var randomSamples: Int
        /// Warm start from the temporal cache, which also warm starts the contact solver
        public var usesTemporalCache: Bool
        /// Draw stochastic samples from high curvature seed points
        public var usesCurvatureSampling: Bool
        /// Skip random samples of convex pairs while the cache tracks them
        public var skipsConvexSamples: Bool
        /// Softmax smoothing of the objective (ε)
        public var softmaxEpsilon: Float
        /// Repulsion between the points of a manifold (α)
        public var repulsionAlpha: Float
        /// Improvement of the objective a new sample needs to replace a cached point
        public var replacementThreshold: Float
        /// Optimizer termination tolerance (τ), also the distance at which points are duplicates
        public var tolerance: Float
        public var maxIterations: Int
        /// Initial step size of gradient descent
        public var gradientStepSize: Float
        /// Skin around the bodies within which contacts are created and kept open
        public var contactMargin: Float
        /// Penetration of the skin left uncorrected
        public var allowedPenetration: Float
        /// Substeps per step, each with its own collision detection
        public var substeps: Int
        /// Solver iterations per substep
        public var solverIterations: Int
        /// Seconds per step
        public var timeStep: Float
        public var gravity: SIMD3<Float>

        public init() {
            self.init(sdf_default_params())
        }

        init(_ p: SDFParams) {
            optimizer = Optimizer(rawValue: p.optimizer) ?? .nelderMead
            cachedSamples = Int(p.cacheSize)
            randomSamples = Int(p.randomSamples)
            usesTemporalCache = p.warmStart
            usesCurvatureSampling = p.curvatureSampling
            skipsConvexSamples = p.skipConvexSamples
            softmaxEpsilon = p.epsilon
            repulsionAlpha = p.alpha
            replacementThreshold = p.replacementThreshold
            tolerance = p.tolerance
            maxIterations = Int(p.maxIterations)
            gradientStepSize = p.stepSize
            contactMargin = p.margin
            allowedPenetration = p.allowedPenetration
            substeps = Int(p.substeps)
            solverIterations = Int(p.solverIterations)
            timeStep = p.timeStep
            gravity = SIMD3(p.gravity.0, p.gravity.1, p.gravity.2)
        }

        var engine: SDFParams {
            SDFParams(optimizer: optimizer.rawValue, cacheSize: Int32(cachedSamples), randomSamples: Int32(randomSamples),
                      warmStart: usesTemporalCache, curvatureSampling: usesCurvatureSampling,
                      skipConvexSamples: skipsConvexSamples, epsilon: softmaxEpsilon, alpha: repulsionAlpha,
                      replacementThreshold: replacementThreshold, tolerance: tolerance,
                      maxIterations: Int32(maxIterations), stepSize: gradientStepSize, margin: contactMargin,
                      allowedPenetration: allowedPenetration, substeps: Int32(substeps),
                      solverIterations: Int32(solverIterations), timeStep: timeStep,
                      gravity: (gravity.x, gravity.y, gravity.z))
        }
    }

    /// Invalid values (non-finite, negative lengths) are ignored by the engine
    public var parameters: Parameters {
        get { Parameters(sdf_world_params(world)) }
        set { sdf_world_set_params(world, newValue.engine) }
    }

    /// Statistics of the last step
    public struct Statistics: Equatable, Sendable {
        public var bodies: Int
        /// Body pairs with overlapping bounds
        public var pairs: Int
        public var contacts: Int
        public var optimizerIterations: Int
        /// Evaluations of the collision objective, each querying both shapes
        public var queries: Int
        /// Narrow phase time, summed over the substeps
        public var collisionMilliseconds: Double
        public var stepMilliseconds: Double
    }

    public var statistics: Statistics {
        let s = sdf_world_stats(world)
        return Statistics(bodies: Int(s.bodies), pairs: Int(s.pairs), contacts: Int(s.contacts),
                          optimizerIterations: Int(s.iterations), queries: Int(s.queries),
                          collisionMilliseconds: s.collisionMilliseconds, stepMilliseconds: s.stepMilliseconds)
    }
}
