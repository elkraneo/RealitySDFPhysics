import Foundation
import RealityKit
import RealitySDFPhysics
import Testing
import simd

/// Frames that don't fall on steps: bodies show between their last two steps, so they move
/// smoothly, while the simulation itself is the same
@MainActor
@Suite struct SDFInterpolationTests {
    /// Frame times around 60 Hz, as a display delivers them, in seconds
    static let jittery: [TimeInterval] = (0..<240).map { 1.0 / 60 + [-0.0012, 0.0011, -0.0004, 0.0009, -0.0010, 0.0006][$0 % 6] }

    /// A ball falling freely, its simulation root and the simulation
    static func fallingBall(interpolated: Bool) -> (root: Entity, ball: Entity, simulation: SDFSimulation) {
        let root = Entity(), simulation = SDFSimulation()
        var settings = SDFPhysicsSimulationComponent(simulation: simulation)
        settings.isInterpolationEnabled = interpolated
        root.components.set(settings)
        let ball = Entity()
        ball.position = [0, 100, 0]
        var body = SDFPhysicsBodyComponent(shape: .sphere(radius: 0.1))
        body.linearDamping = 0
        ball.components.set(body)
        ball.components.set(SDFPhysicsMotionComponent(linearVelocity: [1, 0, 0]))
        root.addChild(ball)
        return (root, ball, simulation)
    }

    /// The ball's sideways move each frame over the move its speed of 1 should give it in that time
    static func moveRatios(interpolated: Bool, frames: [TimeInterval]) -> [Float] {
        let (root, ball, simulation) = fallingBall(interpolated: interpolated)
        simulation.update(root: root, deltaTime: 1.0 / 60)
        var x = ball.position.x, ratios: [Float] = []
        for frame in frames {
            simulation.update(root: root, deltaTime: frame)
            ratios.append((ball.position.x - x) / Float(frame))
            x = ball.position.x
        }
        return ratios.dropFirst(2).map { $0 } // Interpolation starts once a step has moved the ball
    }

    @Test func framesAroundSixtyHertzMoveEvenly() {
        let plain = Self.moveRatios(interpolated: false, frames: Self.jittery)
        let smooth = Self.moveRatios(interpolated: true, frames: Self.jittery)
        // Without interpolation some frames take no step, and the ball stands still in them
        #expect(plain.contains { $0 < 0.1 })
        #expect(smooth.allSatisfy { abs($0 - 1) < 0.15 }, "\(smooth.min()!)...\(smooth.max()!)")
    }

    @Test func framesAtNinetyAndOneTwentyHertzEachMove() {
        for rate in [90.0, 120.0] {
            let smooth = Self.moveRatios(interpolated: true, frames: Array(repeating: 1 / rate, count: 120))
            #expect(smooth.allSatisfy { abs($0 - 1) < 0.15 }, "\(rate) Hz: \(smooth.min()!)...\(smooth.max()!)")
        }
    }

    /// Interpolation only changes what the entities show: the simulated poses are those of a
    /// simulation stepped as often without it
    @Test func simulationIsTheSame() {
        var runs: [[SIMD3<Float>]] = []
        for interpolated in [false, true] {
            let (root, ball, simulation) = Self.fallingBall(interpolated: interpolated)
            var poses: [SIMD3<Float>] = []
            let subscription = simulation.subscribe(to: SDFPhysicsSimulationEvents.DidSimulate.self) { _ in poses.append(ball.position) }
            for frame in Self.jittery { simulation.update(root: root, deltaTime: frame) }
            withExtendedLifetime(subscription) {}
            runs.append(poses)
        }
        #expect(runs[0] == runs[1])
        #expect(runs[0].count > 200)
    }

    /// Handlers of the simulation's events see the simulated poses, not the interpolated ones
    @Test func handlersSeeSimulatedPoses() {
        let (root, ball, simulation) = Self.fallingBall(interpolated: true)
        var afterStep: SIMD3<Float>?, mismatches = 0
        let subscriptions = [
            simulation.subscribe(to: SDFPhysicsSimulationEvents.WillSimulate.self) { _ in
                if let afterStep, ball.position != afterStep { mismatches += 1 }
            },
            simulation.subscribe(to: SDFPhysicsSimulationEvents.DidSimulate.self) { _ in afterStep = ball.position },
        ]
        for frame in Self.jittery { simulation.update(root: root, deltaTime: frame) }
        withExtendedLifetime(subscriptions) {}
        #expect(afterStep != nil)
        #expect(mismatches == 0)
    }

    /// Moving the entity still moves the body, also while it shows an interpolated pose
    @Test func movingTheEntityTeleports() {
        let (root, ball, simulation) = Self.fallingBall(interpolated: true)
        for frame in Self.jittery.prefix(30) { simulation.update(root: root, deltaTime: frame) }
        ball.position = [5, 5, 5]
        simulation.update(root: root, deltaTime: 1.0 / 60)
        simulation.step(root: root)
        // From there, two steps on at its speed
        #expect(abs(ball.position.x - 5) < 0.05 && abs(ball.position.y - 5) < 0.5 && ball.position.z == 5)
    }

    /// A handler of the step's events that moves the entity moves its body
    @Test func handlersCanTeleport() {
        let (root, ball, simulation) = Self.fallingBall(interpolated: true)
        var moved = false
        let subscription = simulation.subscribe(to: SDFPhysicsSimulationEvents.DidSimulate.self) { _ in
            if !moved, simulation.stepCount == 10 { ball.position = [0, 50, 0]; moved = true }
        }
        for frame in Self.jittery.prefix(30) { simulation.update(root: root, deltaTime: frame) }
        withExtendedLifetime(subscription) {}
        #expect(moved)
        #expect(abs(ball.position.y - 50) < 2, "\(ball.position)")
    }

    /// A body made anew, for a new shape, starts from its simulated pose, not the one it shows
    @Test func newShapeKeepsTheSimulatedPose() {
        var finals: [SIMD3<Float>] = []
        for changesShape in [false, true] {
            let (root, ball, simulation) = Self.fallingBall(interpolated: true)
            for (index, frame) in Self.jittery.prefix(60).enumerated() {
                if changesShape, index == 30 {
                    var body = ball.components[SDFPhysicsBodyComponent.self]!
                    body.shape = .sphere(radius: 0.1)
                    ball.components.set(body)
                }
                simulation.update(root: root, deltaTime: frame)
            }
            simulation.step(root: root)
            finals.append(ball.position)
        }
        #expect(simd_distance(finals[0], finals[1]) < 1e-4, "\(finals)")
    }

    /// A position that isn't finite is refused, and doesn't spread into what the entity shows
    @Test func nonfiniteMoveIsRepaired() {
        let (root, ball, simulation) = Self.fallingBall(interpolated: true)
        for frame in Self.jittery.prefix(20) { simulation.update(root: root, deltaTime: frame) }
        ball.position = [.nan, 0, 0]
        for frame in Self.jittery.prefix(10) { simulation.update(root: root, deltaTime: frame) }
        #expect(ball.position.x.isFinite && ball.position.y.isFinite, "\(ball.position)")
    }

    /// Paused, a body shows its simulated pose, and resumes from it without stepping back
    @Test func pauseDoesNotStepBack() {
        let (root, ball, simulation) = Self.fallingBall(interpolated: true)
        for frame in Self.jittery.prefix(20) { simulation.update(root: root, deltaTime: frame) }
        root.components[SDFPhysicsSimulationComponent.self]!.isPaused = true
        simulation.update(root: root, deltaTime: 1.0 / 60)
        let paused = ball.position.x
        root.components[SDFPhysicsSimulationComponent.self]!.isPaused = false
        // A frame too short for a step
        simulation.update(root: root, deltaTime: 0)
        #expect(ball.position.x >= paused, "\(ball.position.x) after \(paused)")
    }
}
