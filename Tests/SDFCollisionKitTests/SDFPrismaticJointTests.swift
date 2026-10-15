import RealityKit
import SDFCollisionKit
import Testing
import simd

@MainActor
@Suite struct SDFPrismaticJointTests {
    let root = Entity()
    let simulation = SDFSimulation()
    /// The rail's axis, tilted 30° down from +x
    let tilt = simd_quatf(angle: -.pi / 6, axis: [0, 0, 1])
    var axis: SIMD3<Float> { tilt.act([1, 0, 0]) }

    init() {
        var settings = SDFSimulationComponent(simulation: simulation)
        settings.isSleepingEnabled = false
        root.components.set(settings)
    }

    @discardableResult
    func addBody(_ shape: SDFShape, mode: PhysicsBodyMode = .dynamic, at position: SIMD3<Float>) -> Entity {
        let entity = Entity()
        entity.position = position
        entity.components.set(SDFBodyComponent(shape: shape, mode: mode))
        root.addChild(entity)
        return entity
    }

    func run(_ steps: Int) {
        for _ in 0..<steps { simulation.step(root: root) }
    }

    /// A block on a rail down the tilted axis from a static base, the block's pin at its center
    func blockOnRail() -> (block: Entity, joint: SDFPrismaticJoint) {
        let base = addBody(.box(halfExtents: [0.2, 0.2, 0.2]), mode: .static, at: [0, 2, 0])
        let block = addBody(.box(halfExtents: [0.1, 0.1, 0.1]), at: [0.5, 2, 0])
        block.components.set(SDFMotionComponent())
        let rail = base.pins.set(named: "rail", position: [0.5, 0, 0], orientation: tilt)
        let slide = block.pins.set(named: "slide", orientation: tilt)
        return (block, SDFPrismaticJoint(pin0: rail, pin1: slide))
    }

    /// How far the block has turned, in radians
    func turn(_ block: Entity) -> Float { block.orientation.angle }

    /// How far the block's center is off the rail's line
    func offLine(_ block: Entity) -> Float {
        let d = block.position - SIMD3<Float>(0.5, 2, 0)
        return simd_length(d - axis * simd_dot(d, axis))
    }

    @Test func slidesDownToItsLimit() throws {
        var (block, joint) = blockOnRail()
        joint.linearLimit = 0...1
        try joint.addToSimulation()
        var worstTurn: Float = 0, worstLine: Float = 0
        for _ in 0..<180 {
            run(1)
            worstTurn = max(worstTurn, turn(block))
            worstLine = max(worstLine, offLine(block))
        }
        let translation = try #require(simulation.translation(of: joint))
        #expect(abs(translation - 1) < 0.01, "stopped at \(translation)")
        #expect(worstTurn < 0.01, "turned \(worstTurn)")
        #expect(worstLine < 0.01, "off the line by \(worstLine)")
    }

    @Test func motorReachesItsSpeed() throws {
        var (block, joint) = blockOnRail()
        // Up the rail, against gravity
        joint.motor = SDFPrismaticJoint.Motor(targetVelocity: -0.5, maxForce: 1000)
        try joint.addToSimulation()
        run(60)
        let speed = simd_dot(block.components[SDFMotionComponent.self]!.linearVelocity, axis)
        #expect(abs(speed + 0.5) < 0.025, "slides at \(speed)")
        #expect(turn(block) < 0.01, "turned \(turn(block))")
    }

    @Test func springHoldsItsTarget() throws {
        var (block, joint) = blockOnRail()
        joint.spring = SDFPrismaticJoint.Spring(targetPosition: 0.5, hertz: 5, dampingRatio: 1)
        try joint.addToSimulation()
        run(180)
        let translation = try #require(simulation.translation(of: joint))
        #expect(abs(translation - 0.5) < 0.01, "held at \(translation)")
        #expect(turn(block) < 0.01, "turned \(turn(block))")
    }

    @Test func torqueDoesNotTurnTheBlock() throws {
        let (block, joint) = blockOnRail()
        try joint.addToSimulation()
        var worstTurn: Float = 0
        for _ in 0..<120 {
            block.sdfPhysics.addTorque([0.05, 0.05, 0.05], relativeTo: nil)
            run(1)
            worstTurn = max(worstTurn, turn(block))
        }
        #expect(worstTurn < 0.01, "turned \(worstTurn)")
        #expect(offLine(block) < 0.01, "off the line by \(offLine(block))")
    }

    @Test func limitChangesInPlace() throws {
        var (block, joint) = blockOnRail()
        joint.linearLimit = 0...1
        let simulationRoot = try joint.addToSimulation()
        run(60)
        // The same joint with a shorter rail: the block is pushed back to it
        var shorter = joint
        shorter.linearLimit = 0...0.3
        simulationRoot.components.set(SDFJointsComponent(joints: [.prismatic(shorter)]))
        run(120)
        let translation = try #require(simulation.translation(of: shorter))
        #expect(abs(translation - 0.3) < 0.01, "stopped at \(translation)")
        #expect(turn(block) < 0.01)
    }

    @Test func hingesAndSlidersShareTheComponent() throws {
        let (block, slider) = blockOnRail()
        try slider.addToSimulation()
        // A pendulum hanging from the block: the hinge rides the slider
        let bob = addBody(.box(halfExtents: [0.05, 0.3, 0.05]), at: [0.5, 1.6, 0])
        let hinge = SDFRevoluteJoint(pin0: block.pins.set(named: "hook", position: [0, -0.1, 0]),
                                     pin1: bob.pins.set(named: "top", position: [0, 0.3, 0]))
        try hinge.addToSimulation()
        #expect(root.components[SDFJointsComponent.self]?.joints.count == 2)
        run(120)
        let gap = simd_distance(hinge.pin0.position(relativeTo: root)!, hinge.pin1.position(relativeTo: root)!)
        #expect(gap < 0.02, "hinge pins \(gap) apart")
        #expect(offLine(block) < 0.01, "off the line by \(offLine(block))")
        #expect(simulation.angle(of: hinge) != nil && simulation.translation(of: slider) != nil)
    }
}
