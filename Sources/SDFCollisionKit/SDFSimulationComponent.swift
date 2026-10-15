import RealityKit
internal import SDFCore

/// Runs an SDF rigid body simulation for the entity's descendants that have an `SDFBodyComponent`,
/// in the entity's local frame. The counterpart of `PhysicsSimulationComponent`.
///
/// Everything happens in the root's local frame and units: scale the root to scale the whole
/// simulation. Nested simulations own their own subtrees.
///
/// The collision tolerances suit objects about 1 unit across, so model your objects at about that
/// size and scale the root to show them at theirs; then say how long a unit is with
/// `metersPerUnit`, for the physical constants.
public struct SDFSimulationComponent: Component {
    /// The simulation's state: its bodies, statistics, contacts, queries and events
    public let simulation: SDFSimulation

    /// The length of a unit of the root, in meters. It sets the defaults that are physical rather
    /// than relative to the objects' size: gravity, the restitution threshold, the density of
    /// `SDFMassProperties.default`, and the limit on the force of `SDFSimulation.drag(to:)`. For
    /// dice modeled in centimeters, in a root scaled to 0.01, it is 0.01.
    public let metersPerUnit: Float

    /// Acceleration of gravity in the root's frame and units, per second squared. By default
    /// 9.81 m/s² down the y axis.
    public var gravity: SIMD3<Float>

    /// Contact solver iterations per substep. `velocityIterations` solve the contacts and friction,
    /// `positionIterations` push penetrating bodies apart.
    public var solverIterations: PhysicsSimulationComponent.SolverIterations

    /// Pairs without a dynamic body that get collision detection and events: kinematic bodies
    /// against static ones, and against each other. Off by default, like RealityKit's.
    public var collisionOptions: PhysicsSimulationComponent.CollisionOptions

    /// Steps per second are `1 / timeStep`. Every step runs `substeps` substeps, each with its own
    /// collision detection.
    public var timeStep: Float
    public var substeps: Int

    /// Upper bound on the steps taken in one frame, so that slow frames do not spiral
    public var maximumStepsPerFrame: Int

    /// Stops the simulation; bodies keep their velocities. `SDFSimulation.step(root:)` still steps.
    public var isPaused: Bool

    /// Speed below which contacts do not bounce, whatever their restitution, so that resting
    /// bodies settle, in units per second. By default 1 m/s.
    public var restitutionThreshold: Float
    /// How stiffly joints hold: the frequency, in hertz, and damping ratio of the spring and
    /// damper their constraints amount to. The defaults are Box3D's, 60 Hz and 2; the frequency is
    /// capped at a quarter of the substep rate, 30 Hz with 2 substeps at 60 Hz. Stiffer joints
    /// sag less under load, softer ones forgive an over-constrained assembly such as a taut chain.
    public var jointStiffness: (hertz: Float, dampingRatio: Float)

    /// The parameters of the paper's collision detection
    public var collisionDetection: SDFCollisionDetection

    /// Runs the collision detection of different pairs, and the contact solver, on several
    /// threads, which makes scenes of many bodies several times faster. Each pair then draws its
    /// random samples from its own seeded sequence, and the solver takes the contacts color by
    /// color (no two contacts of a color share a moving body), so results stay deterministic
    /// whatever the number of threads, but differ from those of a single thread. On by default;
    /// turn it off for upstream's single random sequence and contact order.
    public var isMultithreaded: Bool

    /// Puts bodies to sleep when they and the bodies they touch have been nearly still for half a
    /// second: below 0.05 units and 0.05 radians per second. Sleeping bodies cost nothing to
    /// simulate. A body wakes, with those it fell asleep with, when an awake dynamic body or a
    /// moving kinematic one touches it, and when you move it, push it, set its velocity or change
    /// its body component. On with ``Preset/realTime``, off with ``Preset/paper``.
    public var isSleepingEnabled: Bool

    /// The substeps and solver iterations a simulation starts from
    public enum Preset: Sendable {
        /// 2 substeps of 10 velocity and 4 position iterations, with sleeping: piles of hundreds
        /// of concave bodies settle without overlap, at about half the paper's cost. The default.
        case realTime
        /// The paper's: 4 substeps of 20 velocity and 20 position iterations, without sleeping,
        /// to reproduce its results, and for small, fast bodies that need more substeps
        case paper
    }

    /// A simulation of 60 steps per second, with the preset's substeps and solver iterations, the
    /// paper's collision detection, and Earth's gravity. Collision detection is multithreaded.
    ///
    /// - Parameters:
    ///   - simulation: The simulation, for example one you keep to query and subscribe to
    ///   - metersPerUnit: The length of a unit of the root, in meters
    ///   - preset: The substeps and solver iterations
    @MainActor
    public init(simulation: SDFSimulation = SDFSimulation(), metersPerUnit: Float = 1, preset: Preset = .realTime) {
        _ = registration
        self.simulation = simulation
        self.metersPerUnit = metersPerUnit > 0 && metersPerUnit.isFinite ? metersPerUnit : 1
        let p = sdf_default_params()
        gravity = SIMD3(p.gravity.0, p.gravity.1, p.gravity.2) / self.metersPerUnit
        switch preset {
        case .realTime:
            solverIterations = .init(positionIterations: 4, velocityIterations: 10)
            substeps = 2
            isSleepingEnabled = true
        case .paper:
            solverIterations = .init(positionIterations: Int(p.positionIterations), velocityIterations: Int(p.velocityIterations))
            substeps = Int(p.substeps)
            isSleepingEnabled = false
        }
        collisionOptions = .none
        timeStep = p.timeStep
        maximumStepsPerFrame = 4
        isPaused = false
        restitutionThreshold = p.restitutionThreshold / self.metersPerUnit
        jointStiffness = (p.jointHertz, p.jointDampingRatio)
        collisionDetection = SDFCollisionDetection(p)
        isMultithreaded = true
    }

    /// The nearest ancestor of an entity, or the entity itself, that has an `SDFSimulationComponent`
    @MainActor
    public static func nearestSimulationEntity(for entity: Entity) -> Entity? {
        var current: Entity? = entity
        while let candidate = current {
            if candidate.components.has(SDFSimulationComponent.self) { return candidate }
            current = candidate.parent
        }
        return nil
    }

    var engine: SDFParams {
        var p = collisionDetection.engine
        p.substeps = Int32(clamping: substeps)
        p.velocityIterations = Int32(clamping: solverIterations.velocityIterations)
        p.positionIterations = Int32(clamping: solverIterations.positionIterations)
        p.timeStep = timeStep
        p.gravity = (gravity.x, gravity.y, gravity.z)
        p.restitutionThreshold = restitutionThreshold
        p.jointHertz = jointStiffness.hertz
        p.jointDampingRatio = jointStiffness.dampingRatio
        p.parallel = isMultithreaded
        p.metersPerUnit = metersPerUnit
        p.sleeping = isSleepingEnabled
        p.pairOptions = (collisionOptions.contains(.reportKinematicVsStatic) ? SDFReportKinematicVsStatic.rawValue : 0)
            | (collisionOptions.contains(.reportKinematicVsKinematic) ? SDFReportKinematicVsKinematic.rawValue : 0)
        return p
    }
}

/// The parameters of the collision detection of Giles and Andrews (Sec. 3 and 4 of the paper).
/// The defaults are the paper's. Lengths are in the simulation root's units, and the defaults suit
/// objects about 1 unit across.
public struct SDFCollisionDetection: Equatable, Sendable {
    /// The optimizer that finds the deepest points of the intersection of two bodies (Sec. 3.3)
    public enum Optimizer: Int32, Sendable, CaseIterable {
        /// Derivative free simplex method, the paper's default
        case nelderMead
        /// Deep cut ellipsoid method
        case ellipsoid
        /// Stochastic gradient descent
        case gradientDescent
    }

    public var optimizer: Optimizer
    /// Points kept in the temporal cache of each pair (n_cache)
    public var cachedSamples: Int
    /// New stochastic samples per pair and substep (n_rand)
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
    /// Optimizer iteration limit
    public var maxIterations: Int
    /// Initial step size of gradient descent
    public var gradientStepSize: Float
    /// Skin around the bodies within which contacts are created and kept
    public var contactMargin: Float
    /// Penetration of the skin left uncorrected, so that resting contacts do not jitter
    public var allowedPenetration: Float

    /// The paper's parameters
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
    }

    /// Engine parameters with these collision parameters and default solver ones
    var engine: SDFParams {
        var p = sdf_default_params()
        p.optimizer = optimizer.rawValue
        p.cacheSize = Int32(clamping: cachedSamples)
        p.randomSamples = Int32(clamping: randomSamples)
        p.warmStart = usesTemporalCache
        p.curvatureSampling = usesCurvatureSampling
        p.skipConvexSamples = skipsConvexSamples
        p.epsilon = softmaxEpsilon
        p.alpha = repulsionAlpha
        p.replacementThreshold = replacementThreshold
        p.tolerance = tolerance
        p.maxIterations = Int32(clamping: maxIterations)
        p.stepSize = gradientStepSize
        p.margin = contactMargin
        p.allowedPenetration = allowedPenetration
        return p
    }
}

/// Registers the components on first use. The system is registered with the first simulation.
let registration: Void = {
    SDFBodyComponent.registerComponent()
    SDFMotionComponent.registerComponent()
    SDFSimulationComponent.registerComponent()
    SDFJointsComponent.registerComponent()
}()

/// Advances every simulation in the scene with the frame time. Order your own systems after it with
/// `.after(SDFSimulationSystem.self)` to see the poses of the current frame.
public struct SDFSimulationSystem: System {
    private static let query = EntityQuery(where: .has(SDFSimulationComponent.self))

    public init(scene: RealityKit.Scene) {}

    public mutating func update(context: SceneUpdateContext) {
        for root in context.entities(matching: Self.query, updatingSystemWhen: .rendering) {
            root.components[SDFSimulationComponent.self]?.simulation.update(root: root, deltaTime: context.deltaTime)
        }
    }
}
