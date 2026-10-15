import RealityKit
import RealitySDFPhysics
import SDFRealityKitSampleScenes
import Testing
import simd

/// A Newton's cradle of Apple's sample's sizes (balls of 4 cm and 2 kg on 30 cm strings, just
/// touching), with elastic, frictionless balls: what goes in on one side comes out on the other.
@MainActor
@Suite struct SDFNewtonsCradleTests {
    static let radius: Float = 0.04, stringLength: Float = 0.3

    /// The angle of each ball from straight down, in degrees, positive to +x, over `seconds` after
    /// the first `lifted` balls are let go from `degrees` to -x
    static func swing(balls count: Int = 5, lifted: Int, degrees: Float, seconds: Float = 1,
                      configure: (inout SDFPhysicsSimulationComponent) -> Void = { _ in }) throws -> [[Float]] {
        let root = Entity(), simulation = SDFSimulation()
        var settings = SDFPhysicsSimulationComponent(simulation: simulation)
        settings.isSleepingEnabled = false
        configure(&settings)
        root.components.set(settings)
        let hingeAxis = simd_quatf(from: [1, 0, 0], to: [0, 0, 1])
        var balls: [Entity] = [], hinges: [SIMD3<Float>] = []
        for index in 0..<count {
            let hinge = SIMD3<Float>((Float(index) - Float(count - 1) / 2) * 2 * radius, stringLength, 0)
            let anchor = Entity()
            anchor.position = hinge
            anchor.components.set(SDFPhysicsBodyComponent(shape: .box(halfExtents: [radius, radius / 4, radius]), mode: .static))
            root.addChild(anchor)
            let angle = index < lifted ? -degrees * .pi / 180 : 0
            let ball = Entity()
            ball.position = hinge + stringLength * SIMD3(sin(angle), -cos(angle), 0)
            var body = SDFPhysicsBodyComponent(shape: .sphere(radius: radius))
            body.massProperties = .init(mass: 2)
            body.material = SDFPhysicsMaterial(staticFriction: 0, dynamicFriction: 0, restitution: 1)
            body.linearDamping = 0
            body.angularDamping = 0
            ball.components.set(body)
            ball.components.set(SDFPhysicsMotionComponent())
            root.addChild(ball)
            let pin0 = anchor.pins.set(named: "hinge", position: .zero, orientation: hingeAxis)
            let pin1 = ball.pins.set(named: "hinge", position: anchor.position(relativeTo: ball), orientation: hingeAxis)
            try SDFPhysicsRevoluteJoint(pin0: pin0, pin1: pin1).addToSimulation()
            balls.append(ball)
            hinges.append(hinge)
        }
        var angles: [[Float]] = []
        for _ in 0..<Int(seconds * 60) {
            simulation.step(root: root)
            angles.append(zip(balls, hinges).map { ball, hinge in
                let v = ball.position(relativeTo: root) - hinge
                return atan2(v.x, -v.y) * 180 / .pi
            })
        }
        return angles
    }

    /// Balls this small need a skin to match: the default margin, sized for bodies of a meter,
    /// would hold them a centimeter apart. And they swing slower than the default threshold.
    static func smallBalls(_ settings: inout SDFPhysicsSimulationComponent) {
        settings.restitutionThreshold = 0.05
        settings.collisionDetection.contactMargin = 0.004
        settings.collisionDetection.allowedPenetration = 0.004
    }

    /// What goes in on one side comes out on the other: as many balls as were let go swing out as
    /// high on the far side, and the others stay put
    @Test(arguments: 1...3) func asManyBallsComeOut(lifted: Int) throws {
        // To the end of the first swing out, before anything comes back
        let angles = try Self.swing(lifted: lifted, degrees: 30, seconds: 0.6, configure: Self.smallBalls)
        let farthest = (0..<5).map { i in angles.map { $0[i] }.max()! }
        for (index, angle) in farthest.enumerated() {
            if index >= 5 - lifted {
                #expect(angle > 28, "ball \(index) swung out to \(angle)°")
            } else {
                #expect(angle < 1.5, "ball \(index) swung out to \(angle)°")
            }
        }
    }

    /// A row of free balls passes a moving ball's momentum to its far end, whole: the contacts'
    /// normals, which the optimizer's points tilt, don't scatter it
    @Test(arguments: [Float(0.02), 0.004]) func freeRowPassesMomentumThrough(margin: Float) throws {
        let root = Entity(), simulation = SDFSimulation()
        var settings = SDFPhysicsSimulationComponent(simulation: simulation)
        settings.isSleepingEnabled = false
        settings.gravity = .zero
        settings.restitutionThreshold = 0.05
        settings.collisionDetection.contactMargin = margin
        settings.collisionDetection.allowedPenetration = margin
        root.components.set(settings)
        var balls: [Entity] = []
        for index in 0..<5 {
            let ball = Entity()
            ball.position = [Float(index) * 2 * Self.radius + (index == 0 ? -0.05 : 0), 0, 0]
            var body = SDFPhysicsBodyComponent(shape: .sphere(radius: Self.radius))
            body.material = SDFPhysicsMaterial(staticFriction: 0, dynamicFriction: 0, restitution: 1)
            body.linearDamping = 0
            body.angularDamping = 0
            ball.components.set(body)
            ball.components.set(SDFPhysicsMotionComponent(linearVelocity: index == 0 ? [1, 0, 0] : .zero))
            root.addChild(ball)
            balls.append(ball)
        }
        for _ in 0..<20 { simulation.step(root: root) }
        let motion = balls.map { $0.components[SDFPhysicsMotionComponent.self]! }
        #expect(abs(motion[4].linearVelocity.x - 1) < 0.005)
        // Hardly any spin: under 2 mm/s at the surface
        for m in motion {
            #expect(length(m.angularVelocity) * Self.radius < 0.002)
        }
        for m in motion.dropLast() {
            #expect(length(m.linearVelocity) < 0.005)
        }
    }
}

/// The port of Apple's pendulum sample, as the app runs it
@MainActor
@Suite struct PendulumSampleTests {
    /// The farthest each ball swings to +x, in degrees, over `seconds` of the scene
    static func farthest(_ scene: PendulumScene, seconds: Float) -> [Float] {
        let root = scene.content.children.first { $0.components.has(SDFPhysicsSimulationComponent.self) }!
        let pendulums = root.children[0].children
        var farthest = [Float](repeating: -.infinity, count: pendulums.count)
        for _ in 0..<Int(seconds * 60) {
            scene.simulation.step(root: root)
            for (index, pendulum) in pendulums.enumerated() {
                let v = pendulum.findEntity(named: "ball")!.position - pendulum.findEntity(named: "attachment")!.position
                farthest[index] = max(farthest[index], atan2(v.x, -v.y) * 180 / .pi)
            }
        }
        return farthest
    }

    @Test(arguments: 1...3) func liftedBallsComeOutOnTheFarSide(lifted: Int) {
        let scene = PendulumScene()
        scene.lift(lifted)
        // The root's scale of 1/2 slows the swing: the far balls peak about 0.8 s after the release
        let farthest = Self.farthest(scene, seconds: 0.85)
        for (index, angle) in farthest.enumerated() {
            if index >= 5 - lifted {
                #expect(angle > 28, "ball \(index) swung out to \(angle)°")
            } else {
                #expect(angle < 1.5, "ball \(index) swung out to \(angle)°")
            }
        }
    }

    @Test func pushedBallComesOutAsOne() {
        let scene = PendulumScene()
        scene.reset(pendulumCount: 5)
        scene.pushBall()
        let farthest = Self.farthest(scene, seconds: 2)
        print("push: farthest", farthest)
        #expect(farthest[4] > 40)
        #expect(farthest[1...3].allSatisfy { $0 < 2 })
    }
}
