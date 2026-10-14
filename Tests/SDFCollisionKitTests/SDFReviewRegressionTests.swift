import Combine
import RealityKit
import SDFCollisionKit
import Testing
import simd

/// Regressions found in review
@MainActor
@Suite struct SDFReviewRegressionTests {
    let root = Entity()
    let simulation = SDFSimulation()

    init() {
        root.components.set(SDFSimulationComponent(simulation: simulation))
    }

    func addBody(_ shape: SDFShape = .sphere(radius: 0.25), mode: PhysicsBodyMode = .dynamic, at position: SIMD3<Float>,
                 configure: (inout SDFBodyComponent) -> Void = { _ in }) -> Entity {
        let entity = Entity()
        entity.position = position
        var body = SDFBodyComponent(shape: shape, mode: mode)
        configure(&body)
        entity.components.set(body)
        root.addChild(entity)
        return entity
    }

    func floatingBall() -> Entity {
        let ball = addBody(at: [0, 2, 0]) {
            $0.isAffectedByGravity = false
            $0.linearDamping = 0
        }
        ball.components.set(SDFMotionComponent())
        simulation.step(root: root)
        return ball
    }

    @Test func forcesFromWillSimulateActOncePerStep() {
        let ball = floatingBall()
        let mass = simulation.mass(of: ball)!
        let subscription = simulation.subscribe(to: SDFSimulationEvents.WillSimulate.self) { _ in
            ball.sdfPhysics.addForce([mass * 60, 0, 0], relativeTo: nil) // 1 unit/s per step
        }
        withExtendedLifetime(subscription) {
            simulation.update(root: root, deltaTime: 2.0 / 60 + 1e-6) // Two steps in one update
        }
        let velocity = ball.components[SDFMotionComponent.self]!.linearVelocity.x
        #expect(abs(velocity - 2) < 0.01, "velocity \(velocity)")
    }

    @Test func forcesFromDidSimulateActDuringTheNextStep() {
        let ball = floatingBall()
        let mass = simulation.mass(of: ball)!
        var added = false
        let subscription = simulation.subscribe(to: SDFSimulationEvents.DidSimulate.self) { _ in
            if !added { ball.sdfPhysics.addForce([mass * 60, 0, 0], relativeTo: nil) }
            added = true
        }
        withExtendedLifetime(subscription) {
            simulation.step(root: root) // Adds the force after this step
            #expect(ball.components[SDFMotionComponent.self]!.linearVelocity.x == 0)
            simulation.step(root: root)
        }
        #expect(abs(ball.components[SDFMotionComponent.self]!.linearVelocity.x - 1) < 0.01)
    }

    @Test func clearingForcesKeepsImpulses() {
        let ball = addBody(at: [0, 2, 0]) { $0.isAffectedByGravity = false }
        ball.components.set(SDFMotionComponent())
        ball.sdfPhysics.applyLinearImpulse([0, 0, 1], relativeTo: nil)
        ball.sdfPhysics.clearForcesAndTorques()
        simulation.step(root: root)
        #expect(ball.components[SDFMotionComponent.self]!.linearVelocity.z > 0)
    }

    @Test func fastBodiesThroughTriggersReportBothEvents() {
        // A thin trigger crossed within a step. Substeps detect at heights 3.2375, 3.1125, 2.9875 and
        // 2.8625, and only the third is within the contact margin of the trigger.
        let trigger = addBody(.box(halfExtents: [1, 0.02, 1]), mode: .static, at: [0, 3, 0]) { $0.isTrigger = true }
        let ball = addBody(.sphere(radius: 0.05), at: [0, 3.2375, 0]) { $0.isAffectedByGravity = false }
        ball.components.set(SDFMotionComponent(linearVelocity: [0, -30, 0]))
        var events: [String] = []
        let subscriptions = [
            simulation.subscribe(to: SDFCollisionEvents.Began.self, on: trigger) { _ in events.append("began") },
            simulation.subscribe(to: SDFCollisionEvents.Ended.self, on: trigger) { _ in events.append("ended") },
        ]
        withExtendedLifetime(subscriptions) {
            for _ in 0..<6 { simulation.step(root: root) }
        }
        #expect(events == ["began", "ended"])
        #expect(ball.position.y < 2.5)
    }

    @Test func handlersCannotStepTheSimulation() {
        _ = addBody(.box(halfExtents: [2, 0.5, 2]), mode: .static, at: [0, -0.5, 0])
        _ = addBody(at: [0, 0.5, 0])
        var began = 0
        let subscription = simulation.subscribe(to: SDFCollisionEvents.Began.self) { [simulation, root] _ in
            began += 1
            let steps = simulation.stepCount
            simulation.step(root: root)
            #expect(simulation.stepCount == steps)
        }
        withExtendedLifetime(subscription) {
            for _ in 0..<40 { simulation.step(root: root) }
        }
        #expect(began == 1)
    }

    @Test func cancelledSubscriptionsReleaseTheirHandlers() {
        final class Token {}
        weak var weakToken: Token?
        var subscription: AnyCancellable?
        do {
            let token = Token()
            weakToken = token
            subscription = simulation.subscribe(to: SDFSimulationEvents.DidSimulate.self) { _ in _ = token }
        }
        subscription?.cancel()
        simulation.step(root: root)
        #expect(weakToken == nil)
        _ = subscription
    }

    @Test func raysAlongABoundingBoxFaceHit() {
        let box = addBody(.box(halfExtents: [1, 1, 1]), mode: .static, at: .zero)
        simulation.step(root: root)
        let hit = simulation.raycast(origin: [0, 2, 1], direction: [0, -1, 0], query: .nearest).first
        #expect(hit?.entity == box)
        #expect(abs((hit?.distance ?? 0) - 1) < 0.01)
    }
}
