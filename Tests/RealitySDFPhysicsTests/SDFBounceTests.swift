import RealityKit
import RealitySDFPhysics
import Testing
import simd

/// Bounces, which bodies take pair by pair before the contacts are solved: with friction, through
/// joints, and as contact events report them
@MainActor
@Suite struct SDFBounceTests {
    let root = Entity()
    let simulation = SDFSimulation()

    init() {
        var settings = SDFPhysicsSimulationComponent(simulation: simulation)
        settings.isSleepingEnabled = false
        settings.gravity = .zero
        settings.restitutionThreshold = 0.05
        root.components.set(settings)
    }

    func addGround(friction: Float, restitution: Float, tilt: Float = 0) -> Entity {
        let ground = Entity()
        ground.orientation = simd_quatf(angle: tilt, axis: [0, 0, 1])
        var body = SDFPhysicsBodyComponent(shape: .box(halfExtents: [5, 0.5, 5]), mode: .static)
        body.material = SDFPhysicsMaterial(staticFriction: friction, dynamicFriction: friction, restitution: restitution)
        ground.components.set(body)
        root.addChild(ground)
        return ground
    }

    func addBall(at position: SIMD3<Float>, velocity: SIMD3<Float>, friction: Float, restitution: Float,
                 rotationLocked: Bool = false) -> Entity {
        let ball = Entity()
        ball.position = position
        var body = SDFPhysicsBodyComponent(shape: .sphere(radius: 0.2))
        body.material = SDFPhysicsMaterial(staticFriction: friction, dynamicFriction: friction, restitution: restitution)
        body.linearDamping = 0
        body.angularDamping = 0
        if rotationLocked { body.isRotationLocked = (true, true, true) }
        ball.components.set(body)
        ball.components.set(SDFPhysicsMotionComponent(linearVelocity: velocity))
        root.addChild(ball)
        return ball
    }

    func velocity(of entity: Entity) -> SIMD3<Float> { entity.components[SDFPhysicsMotionComponent.self]!.linearVelocity }

    /// A ball that can't turn, sliding into a rough floor, is slowed along it by the bounce's friction
    @Test func bounceKeepsItsFriction() {
        _ = addGround(friction: 0.5, restitution: 0.8)
        let ball = addBall(at: [0, 0.75, 0], velocity: [1, -2, 0], friction: 0.5, restitution: 0.8, rotationLocked: true)
        for _ in 0..<20 { simulation.step(root: root) }
        let v = velocity(of: ball)
        // Up at the bounce's speed; along the floor, at most the dynamic friction's 0.5 × 3.6 off
        #expect(abs(v.y - 1.6) < 0.1, "\(v)")
        #expect(v.x < 0.1, "\(v)")
    }

    /// A ball on a slider, driven into a slope, bounces back along the slider. Bounces don't see
    /// joints: the joint takes the bounce's sideways part, so the ball comes back at about 0.72
    /// of the elastic 1 (0.67 before bounces were taken pair by pair). A known gap, see the README.
    @Test func bounceThroughAJoint() throws {
        // A 45 degree slope on the right
        let wall = addGround(friction: 0, restitution: 1, tilt: .pi / 4)
        wall.position = [1.2, 0, 0]
        let anchor = Entity()
        anchor.components.set(SDFPhysicsBodyComponent(shape: .box(halfExtents: [0.05, 0.05, 0.05]), mode: .static))
        anchor.position = [-3, 0, 0]
        root.addChild(anchor)
        let ball = addBall(at: [0, 0, 0], velocity: [1, 0, 0], friction: 0, restitution: 1)
        let pin0 = anchor.pins.set(named: "slide", position: .zero)
        let pin1 = ball.pins.set(named: "slide", position: anchor.position(relativeTo: ball))
        try SDFPhysicsPrismaticJoint(pin0: pin0, pin1: pin1).addToSimulation()
        for _ in 0..<90 { simulation.step(root: root) }
        let v = velocity(of: ball)
        #expect(v.x < -0.65, "\(v)")
        #expect(abs(v.y) < 0.1, "\(v)")
    }

    /// A bounce's contact event points along the impulse the bounce applied
    @Test func eventsReportTheBounce() {
        _ = addGround(friction: 0, restitution: 1)
        let ball = addBall(at: [0, 0.75, 0], velocity: [0, -2, 0], friction: 0, restitution: 1)
        var directions: [SIMD3<Float>] = []
        let subscription = simulation.subscribe(to: SDFCollisionEvents.Began.self) { event in
            directions.append(event.impulseDirection)
        }
        for _ in 0..<20 { simulation.step(root: root) }
        withExtendedLifetime(subscription) {}
        #expect(abs(velocity(of: ball).y - 2) < 0.05)
        #expect(directions.count == 1)
        if let direction = directions.first { #expect(abs(abs(direction.y) - 1) < 0.01, "\(direction)") }
    }
}
