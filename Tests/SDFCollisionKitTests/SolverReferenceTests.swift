import Foundation
import SDFCore
import Testing

/// Our solver adds options to upstream's. With every option at its default it must take exactly
/// the same steps, so that the behavior of the paper's engine is kept.
struct SolverReferenceTests {
    /// A pile of convex and concave bodies on a static ground, with a moving kinematic body
    func makeWorld() -> (OpaquePointer, [SDFBodyID]) {
        let world = sdf_world_create()!
        func pose(_ x: Float, _ y: Float, _ z: Float, _ angle: Float = 0.3) -> SDFPose {
            SDFPose(position: (x, y, z), orientation: (sin(angle / 2), 0, 0, cos(angle / 2)), scale: 1)
        }
        let shapes = [sdf_shape_box(0.2, 0.15, 0.25)!, sdf_shape_spiky_wheel(0.2, 0.1, 0.05, 0.2, 8)!,
                      sdf_shape_torus(0.25, 0.08)!, sdf_shape_warped_box(SDFWarpTwist, 0.3, 0.1, 0.1, 1)!]
        defer { shapes.forEach(sdf_shape_release) }
        let ground = sdf_shape_box(3, 0.5, 3)!
        defer { sdf_shape_release(ground) }
        _ = sdf_world_add_body(world, ground, pose(0, -0.5, 0, 0), SDFBodyStatic)
        var bodies: [SDFBodyID] = []
        for i in 0..<12 {
            let shape = shapes[i % shapes.count]
            bodies.append(sdf_world_add_body(world, shape, pose(Float(i % 3) * 0.3 - 0.3, 0.4 + Float(i) * 0.35, Float(i % 2) * 0.2),
                                             SDFBodyDynamic))
        }
        let kinematic = sdf_world_add_body(world, shapes[0], pose(1, 0.3, 0), SDFBodyKinematic)
        sdf_world_set_body_velocity(world, kinematic, [-0.5, 0, 0], [0, 1, 0])
        // Explicit defaults go through the same paths as a body component
        for id in bodies { sdf_world_set_body_options(world, id, sdf_default_body_options()) }
        return (world, bodies)
    }

    @Test func defaultOptionsMatchUpstreamSolver() {
        let (ours, bodies) = makeWorld(), (reference, _) = makeWorld()
        defer { sdf_world_destroy(ours); sdf_world_destroy(reference) }
        var contacts = 0
        for _ in 0..<240 {
            sdf_world_step(ours)
            sdf_world_step_reference(reference)
            contacts += Int(sdf_world_stats(ours).contacts)
        }
        #expect(contacts > 1000) // The bodies did pile up
        for id in bodies {
            let a = sdf_world_body_pose(ours, id), b = sdf_world_body_pose(reference, id)
            #expect([a.position.0, a.position.1, a.position.2, a.orientation.0, a.orientation.1, a.orientation.2, a.orientation.3]
                == [b.position.0, b.position.1, b.position.2, b.orientation.0, b.orientation.1, b.orientation.2, b.orientation.3])
        }
    }
}
