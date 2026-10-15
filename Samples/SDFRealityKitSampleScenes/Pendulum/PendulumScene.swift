// Port of Apple's sample "Simulating physics joints in your RealityKit app":
// https://developer.apple.com/documentation/realitykit/simulating-physics-joints-in-your-realitykit-app
//
// Five chrome balls hang on revolute joints from green blocks, just touching: a Newton's cradle.
// Apple's settings, entities and joints are kept. Differences from Apple's sample:
//
// - SDF bodies in place of PhysicsBodyComponent and CollisionComponent, with the same sizes,
//   masses and materials: frictionless, fully elastic balls without damping.
// - The cradle behaves as one: a ball let go comes out as one ball on the far side, two as two.
//   RealityKit's solver shares the bounce among the touching balls instead, so in Apple's sample
//   one ball in sends four out together. The engine bounces bodies pair by pair before solving
//   their contacts, so the bounce runs down the row.
// - The balls are 8 cm in the simulation's units (Apple's 4 cm, at the root's scale of 1/2), so
//   the contact margin and the restitution threshold are sized for them: the defaults, for bodies
//   of a meter, would hold the balls a centimeter apart, and not bounce at their speeds.
// - "Push ball" gives the first ball the push RealityKit gives it in Apple's sample: measured, the
//   ball leaves at 2 units/s in the simulation's frame and swings to 48 degrees. "Lift" raises one
//   to three balls to 30 degrees and lets them go, as with a real cradle.

import RealityKit
import RealitySDFPhysics
import simd

/// Apple's PendulumSettings
package struct PendulumSettings {
    let ballRadius: Float = 0.04
    let ballMass: Float = 2
    let stringLength: Float = 0.3
    let stringRadius: Float = 0.001
    let stringColor = SimpleMaterial.Color(red: 0.2, green: 0.2, blue: 0.2, alpha: 1)
    let attachmentSize: SIMD3<Float> = [2.0, 0.5, 2.0]
    let attachmentColor = SimpleMaterial.Color(red: 0.75, green: 0.88, blue: 0.75, alpha: 1)
    package var pendulumCount: Int = 5
    let pendulumSpeed: Float = 0.5
    var impulsePower: SIMD3<Float> { [-2.0 / pow(pendulumSpeed, 4), 0, 0] }

    package init(pendulumCount: Int = 5) { self.pendulumCount = pendulumCount }
}

@MainActor
package final class PendulumScene {
    package let content = Entity()
    package let camera = SampleCamera(eye: [0.15, 0.1, 0.75], target: [0, 0, 0])
    package let simulation = SDFSimulation()
    private(set) package var settings = PendulumSettings()
    /// Apple's parentSimulationEntity, the simulation's root
    private var root = Entity()
    private var pendulums: [Entity] = []

    package init() {
        let light = DirectionalLight()
        light.light.intensity = 3000
        light.look(at: .zero, from: [0.5, 1, 1], relativeTo: nil)
        content.addChild(light)
        content.addChild(camera.makeEntity())
    }

    /// Builds Apple's scene anew with `count` pendulums
    package func reset(pendulumCount count: Int) {
        root.removeFromParent()
        settings = PendulumSettings(pendulumCount: count)
        root = buildPendulumScene()
        content.addChild(root)
    }

    // MARK: Apple's MainView+PendulumCreation

    private func buildPendulumScene() -> Entity {
        let parentSimulationEntity = Entity()
        let pendulumParent = Entity()
        parentSimulationEntity.addChild(pendulumParent)

        var simulationComponent = SDFPhysicsSimulationComponent(simulation: simulation)
        simulationComponent.solverIterations.positionIterations = 25
        simulationComponent.solverIterations.velocityIterations = 25
        // Sized for the balls, see above. The simulation's units are twice Apple's lengths.
        simulationComponent.collisionDetection.contactMargin = 0.008
        simulationComponent.collisionDetection.allowedPenetration = 0.008
        simulationComponent.restitutionThreshold = 0.1
        simulationComponent.isSleepingEnabled = false
        parentSimulationEntity.components.set(simulationComponent)

        pendulums = []
        let firstPendulumX = -Float(settings.pendulumCount - 1) / 2
        for pendulum in 0..<settings.pendulumCount {
            let newPendulum = createPendulum(pendulumParent)
            newPendulum.position.x = (firstPendulumX + Float(pendulum)) * settings.attachmentSize.x * settings.ballRadius
            pendulums.append(newPendulum)
        }
        // The pins are set once every pendulum has its place
        for pendulum in pendulums {
            addPinsTo(ballEntity: pendulum.findEntity(named: "ball")!, attachmentEntity: pendulum.findEntity(named: "attachment")!)
        }

        parentSimulationEntity.position.y = settings.stringLength / 2
        // Apple's slow motion: the simulation runs at twice the size it shows
        parentSimulationEntity.scale = SIMD3(repeating: settings.pendulumSpeed)
        pendulumParent.scale = SIMD3(repeating: 1 / settings.pendulumSpeed)
        return parentSimulationEntity
    }

    private func createPendulum(_ simulationParent: Entity) -> Entity {
        let pendulumParent = Entity()
        let ballEntity = makeBallEntity()
        pendulumParent.addChild(ballEntity)
        ballEntity.addChild(makeStringEntity())
        let attachmentEntity = makeAttachmentEntity()
        pendulumParent.addChild(attachmentEntity)
        addBallPhysics(to: ballEntity)
        addAttachmentPhysics(to: attachmentEntity)
        simulationParent.addChild(pendulumParent)
        return pendulumParent
    }

    // MARK: Apple's MainView+PendulumModels

    private func makeBallEntity() -> Entity {
        let ballEntity = Entity()
        ballEntity.name = "ball"
        let ballMesh = MeshResource.generateSphere(radius: settings.ballRadius)
        ballEntity.components.set(ModelComponent(mesh: ballMesh, materials: [SimpleMaterial(color: .lightGray, isMetallic: true)]))
        ballEntity.position.y = -settings.stringLength
        return ballEntity
    }

    private func makeStringEntity() -> Entity {
        let stringEntity = Entity()
        let stringMesh = MeshResource.generateCylinder(height: settings.stringLength, radius: settings.stringRadius)
        var stringMaterial = PhysicallyBasedMaterial()
        stringMaterial.baseColor.tint = settings.stringColor
        stringEntity.components.set(ModelComponent(mesh: stringMesh, materials: [stringMaterial]))
        stringEntity.position.y = settings.stringLength / 2
        return stringEntity
    }

    private func makeAttachmentEntity() -> Entity {
        let attachmentEntity = Entity()
        attachmentEntity.name = "attachment"
        let attachmentMesh = MeshResource.generateBox(size: settings.attachmentSize * settings.ballRadius)
        var attachmentMaterial = PhysicallyBasedMaterial()
        attachmentMaterial.baseColor.tint = settings.attachmentColor
        attachmentEntity.components.set(ModelComponent(mesh: attachmentMesh, materials: [attachmentMaterial]))
        return attachmentEntity
    }

    // MARK: Apple's MainView+PhysicsAndPins

    private func addPinsTo(ballEntity: Entity, attachmentEntity: Entity) {
        let hingeOrientation = simd_quatf(from: [1, 0, 0], to: [0, 0, 1])
        let attachmentPin = attachmentEntity.pins.set(named: "attachment_hinge", position: .zero, orientation: hingeOrientation)
        let relativeJointLocation = attachmentEntity.position(relativeTo: ballEntity)
        let ballPin = ballEntity.pins.set(named: "ball_hinge", position: relativeJointLocation, orientation: hingeOrientation)
        let revoluteJoint = SDFPhysicsRevoluteJoint(pin0: attachmentPin, pin1: ballPin)
        do { try revoluteJoint.addToSimulation() } catch { fatalError(error.localizedDescription) }
    }

    private func addBallPhysics(to ballEntity: Entity) {
        var ballBody = SDFPhysicsBodyComponent(shape: .sphere(radius: settings.ballRadius))
        ballBody.massProperties = .init(mass: settings.ballMass)
        ballBody.material = SDFPhysicsMaterial(staticFriction: 0, dynamicFriction: 0, restitution: 1)
        ballBody.linearDamping = 0
        ballBody.angularDamping = 0
        ballEntity.components.set([ballBody, SDFPhysicsMotionComponent()])
    }

    private func addAttachmentPhysics(to attachmentEntity: Entity) {
        var attachmentBody = SDFPhysicsBodyComponent(shape: .box(halfExtents: settings.attachmentSize * settings.ballRadius / 2), mode: .static)
        attachmentBody.material = SDFPhysicsMaterial(staticFriction: 0, dynamicFriction: 0, restitution: 1)
        attachmentEntity.components.set(attachmentBody)
    }

    /// Apple's push, as RealityKit applies it: the impulse in the simulation's frame is Apple's
    /// times the root's scale cubed
    package func pushBall() {
        guard let ball = pendulums.first?.findEntity(named: "ball") else { return }
        ball.sdf.applyLinearImpulse(settings.impulsePower * pow(settings.pendulumSpeed, 3), relativeTo: root)
    }

    /// Rebuilds the cradle with its first `count` balls raised to `degrees` towards -x, at rest
    package func lift(_ count: Int, degrees: Float = 30) {
        reset(pendulumCount: settings.pendulumCount)
        let turn = simd_quatf(angle: -degrees * .pi / 180, axis: [0, 0, 1])
        for pendulum in pendulums.prefix(count) {
            guard let ball = pendulum.findEntity(named: "ball"), let attachment = pendulum.findEntity(named: "attachment")
            else { continue }
            ball.orientation = turn
            ball.position = attachment.position + turn.act(ball.position - attachment.position)
        }
    }
}
