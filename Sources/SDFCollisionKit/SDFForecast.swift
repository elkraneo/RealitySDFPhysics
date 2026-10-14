import RealityKit
internal import SDFCore
import simd

extension SDFSimulation {
    /// A copy of the simulation as it is now, which steps on its own: to know where bodies will
    /// be without moving them, for example where a throw lands.
    ///
    /// The engine is deterministic, so the forecast takes exactly the steps the simulation will,
    /// as long as nothing changes the simulation in the meantime. Forces added for the next update
    /// are not included: the forecast is of the simulation left to itself.
    public func forecast() -> SDFForecast {
        SDFForecast(world: sdf_world_copy(world), ids: bodyIDs)
    }
}

/// A copy of an `SDFSimulation` that steps on its own, made by `SDFSimulation.forecast()`. It
/// moves no entities: read where their bodies are with `transform(of:)` and `motion(of:)`.
@MainActor
public final class SDFForecast {
    nonisolated(unsafe) private let world: OpaquePointer
    private let ids: [Entity.ID: SDFBodyID]

    /// Steps taken since the copy
    public private(set) var stepCount = 0

    init(world: OpaquePointer, ids: [Entity.ID: SDFBodyID]) {
        self.world = world
        self.ids = ids
        for id in ids.values { sdf_world_clear_body_forces(world, id) }
    }

    deinit {
        sdf_world_destroy(world)
    }

    /// Takes steps of the simulation's time step
    public func step(_ count: Int = 1) {
        for _ in 0..<max(count, 0) { sdf_world_step(world) }
        stepCount += max(count, 0)
    }

    /// The transform an entity's body has in the forecast, in the simulation root's frame
    public func transform(of entity: Entity) -> Transform? {
        guard let id = ids[entity.id] else { return nil }
        let pose = sdf_world_body_pose(world, id)
        return Transform(scale: SIMD3(repeating: pose.scale),
                         rotation: simd_quatf(ix: pose.orientation.0, iy: pose.orientation.1, iz: pose.orientation.2,
                                              r: pose.orientation.3),
                         translation: SIMD3(pose.position.0, pose.position.1, pose.position.2))
    }

    /// The velocities an entity's body has in the forecast, in the simulation root's frame
    public func motion(of entity: Entity) -> SDFMotionComponent? {
        guard let id = ids[entity.id] else { return nil }
        var linear: [Float] = [0, 0, 0], angular: [Float] = [0, 0, 0]
        sdf_world_body_velocity(world, id, &linear, &angular)
        return SDFMotionComponent(linearVelocity: SIMD3(linear[0], linear[1], linear[2]),
                                  angularVelocity: SIMD3(angular[0], angular[1], angular[2]))
    }
}
