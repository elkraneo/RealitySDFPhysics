import RealityKit

/// Makes an entity a rigid body of the nearest ancestor with an `SDFSimulationComponent`.
///
/// The body is created on the simulation's next update, and removed with the entity or the
/// component. Changing the shape, mode or scale recreates it. The entity's scale must be uniform.
public struct SDFBodyComponent: Component {
    public enum Mode: Sendable, Equatable {
        /// Moved by the simulation, which writes the entity's transform
        case dynamic
        /// Moved by you through the entity's transform; pushes dynamic bodies with its velocity
        case kinematic
        /// Does not move, or teleports when you set the entity's transform
        case `static`
    }

    public var shape: SDFShape
    public var mode: Mode
    /// Mass of a dynamic body. By default that of its volume of water.
    public var mass: Float?
    public var friction: Float

    public init(shape: SDFShape, mode: Mode = .dynamic, mass: Float? = nil, friction: Float = 0.5) {
        _ = registration
        self.shape = shape
        self.mode = mode
        self.mass = mass
        self.friction = friction
    }
}

/// Velocities of a dynamic body, in the simulation root's frame.
///
/// The simulation writes the body's velocities to this component after every step. Changes you make
/// are applied to the body before the next step.
public struct SDFMotionComponent: Component, Equatable {
    public var linearVelocity: SIMD3<Float>
    public var angularVelocity: SIMD3<Float>

    public init(linearVelocity: SIMD3<Float> = .zero, angularVelocity: SIMD3<Float> = .zero) {
        _ = registration
        self.linearVelocity = linearVelocity
        self.angularVelocity = angularVelocity
    }
}

/// Runs an `SDFSimulation` for the entity's descendants with an `SDFBodyComponent`, in the entity's
/// local frame. Nested simulations own their own subtrees.
public struct SDFSimulationComponent: Component {
    public let simulation: SDFSimulation

    @MainActor
    public init(simulation: SDFSimulation = SDFSimulation()) {
        _ = registration
        self.simulation = simulation
    }
}

/// Registers the components on first use
private let registration: Void = {
    SDFBodyComponent.registerComponent()
    SDFMotionComponent.registerComponent()
    SDFSimulationComponent.registerComponent()
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
