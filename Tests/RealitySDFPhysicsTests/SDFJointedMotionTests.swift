import RealityKit
import RealitySDFPhysics
import Testing
import simd

/// Two spiky wheels about y, of 10 spikes each, as gears on hinges to a static base: the spikes
/// reach 0.9 from the axle, the axles are 1.5 apart, and the driven wheel is turned half a spike
/// so that the spikes interleave, their tips 0.2 short of the other hub
@MainActor
func addSpikyWheels(to root: Entity, driverMotor: SDFPhysicsRevoluteJoint.Motor?) throws -> (driver: Entity, driven: Entity) {
    let base = Entity()
    base.name = "base"
    base.position = [0, -5, 0]
    base.components.set(SDFPhysicsBodyComponent(shape: .box(halfExtents: [1, 0.1, 1]), mode: .static))
    root.addChild(base)
    let axle = simd_quatf(angle: .pi / 2, axis: [0, 0, 1]) // The pin's x axis along y
    let shape = SDFShape.spikyWheel(hubRadius: 0.4, hubHalfWidth: 0.2, spikeRadius: 0.12, spikeLength: 0.5, spikes: 10)
    var wheels: [Entity] = []
    for (name, x, turn) in [("driver", Float(0), Float(0)), ("driven", 1.5, Float.pi / 10)] {
        let wheel = Entity()
        wheel.name = name
        wheel.position = [x, 0, 0]
        wheel.orientation = simd_quatf(angle: turn, axis: [0, 1, 0])
        var body = SDFPhysicsBodyComponent(shape: shape)
        body.material = SDFPhysicsMaterial(staticFriction: 0.1, dynamicFriction: 0.1, restitution: 0)
        wheel.components.set(body)
        wheel.components.set(SDFPhysicsMotionComponent())
        root.addChild(wheel)
        let pin0 = base.pins.set(named: name, position: wheel.position, orientation: axle, relativeTo: root)
        let pin1 = wheel.pins.set(named: "hub", orientation: wheel.orientation.inverse * axle)
        var joint = SDFPhysicsRevoluteJoint(pin0: pin0, pin1: pin1)
        if name == "driver" { joint.motor = driverMotor }
        try joint.addToSimulation()
        wheels.append(wheel)
    }
    return (wheels[0], wheels[1])
}

/// Joints over many steps, with the other parts of the engine: plane locks, contact between
/// jointed bodies, motors changed while they work, and determinism
@MainActor
@Suite struct SDFJointedMotionTests {
    let root = Entity()
    let simulation = SDFSimulation()

    init() {
        var settings = SDFPhysicsSimulationComponent(simulation: simulation)
        settings.isSleepingEnabled = false
        root.components.set(settings)
    }

    var joints: [SDFPhysicsJoint] { root.components[SDFPhysicsJointsComponent.self]?.joints ?? [] }

    /// The farthest apart the pins of any joint are, in the root's frame
    var worstGap: Float {
        joints.map { simd_distance($0.pin0.position(relativeTo: root)!, $0.pin1.position(relativeTo: root)!) }.max() ?? 0
    }

    func addBox(halfExtents: SIMD3<Float>, at position: SIMD3<Float>, mode: PhysicsBodyMode = .dynamic) -> Entity {
        let box = Entity()
        box.position = position
        box.components.set(SDFPhysicsBodyComponent(shape: .box(halfExtents: halfExtents), mode: mode))
        root.addChild(box)
        return box
    }

    /// Six capsule links on hinges about z, laid out straight from a static anchor, 45° below the
    /// level, each moving only in the x-y plane and turning only about z, set off out of the plane
    /// all the same. They swing down and through, into a static box on the far side.
    func addPlanarLinkage() throws -> (links: [Entity], box: Entity) {
        let half: Float = 0.2
        let anchor = addBox(halfExtents: [0.05, 0.05, 0.05], at: [0, 4, 0], mode: .static)
        let box = addBox(halfExtents: [0.3, 0.3, 0.6], at: [-1.4, 2.6, 0], mode: .static)
        let axle = simd_quatf(angle: -.pi / 2, axis: [0, 1, 0]) // The pin's x axis along z
        let direction = simd_normalize(SIMD3<Float>(1, -1, 0))
        let slant = simd_quatf(from: [0, 1, 0], to: direction) // The capsule's y axis along the line
        var previous = anchor.pins.set(named: "end", orientation: axle)
        var links: [Entity] = []
        for i in 0..<6 {
            let link = Entity()
            link.position = anchor.position + (half + Float(i) * 2 * half) * direction
            link.orientation = slant
            var body = SDFPhysicsBodyComponent(shape: .capsule(radius: 0.05, halfLength: half))
            body.isTranslationLocked = (false, false, true)
            body.isRotationLocked = (true, true, false)
            link.components.set(body)
            link.components.set(SDFPhysicsMotionComponent(linearVelocity: [0, 0, 1], angularVelocity: [1, 1, 0]))
            root.addChild(link)
            let start = link.pins.set(named: "start", position: [0, -half, 0], orientation: slant.inverse * axle)
            try SDFPhysicsRevoluteJoint(pin0: previous, pin1: start).addToSimulation()
            previous = link.pins.set(named: "end", position: [0, half, 0], orientation: slant.inverse * axle)
            links.append(link)
        }
        return (links, box)
    }

    @Test func planarLinkageStaysInItsPlane() throws {
        let (links, box) = try addPlanarLinkage()
        var worstZ: Float = 0, worstTilt: Float = 0, worstGap: Float = 0, isFinite = true
        var path: Float = 0, previous = links.last!.position, lastSecond: Float = 0, hits = 0
        for step in 0..<240 {
            simulation.step(root: root)
            for link in links {
                let p = link.position
                isFinite = isFinite && p.x.isFinite && p.y.isFinite && p.z.isFinite
                worstZ = max(worstZ, abs(p.z))
                // The link's z axis stays the world's
                worstTilt = max(worstTilt, 1 - link.orientation.act([0, 0, 1]).z)
            }
            worstGap = max(worstGap, self.worstGap)
            hits += simulation.contacts().filter { $0.entityA == box.id || $0.entityB == box.id }.count
            let moved = simd_distance(links.last!.position, previous)
            path += moved
            if step >= 180 { lastSecond += moved }
            previous = links.last!.position
        }
        let report = Comment(rawValue: "z up to \(worstZ), tilt up to \(worstTilt), pins up to \(worstGap) apart, the last link ran \(path), \(lastSecond) in the last second, \(hits) contacts with the box")
        #expect(isFinite, report)
        #expect(worstZ < 1e-3, report)
        #expect(worstTilt < 1e-3, report)
        #expect(worstGap < 0.01, report)
        // It swung down, into the box, and is still moving
        #expect(hits > 0, report)
        #expect(path > 4, report)
        #expect(lastSecond > 0.2, report)
    }

    @Test(arguments: [SDFPhysicsSimulationComponent.Preset.realTime, .paper])
    func motoredWheelTurnsTheOtherByContact(preset: SDFPhysicsSimulationComponent.Preset) throws {
        var settings = SDFPhysicsSimulationComponent(simulation: simulation, preset: preset)
        settings.isSleepingEnabled = false
        root.components.set(settings)
        let (driver, driven) = try addSpikyWheels(to: root, driverMotor: .init(targetVelocity: 1, maxTorque: 500))
        var driverSpeeds: [Float] = [], drivenSpeeds: [Float] = [], worstGap: Float = 0
        // Six seconds, the last two measured
        for step in 0..<360 {
            simulation.step(root: root)
            worstGap = max(worstGap, self.worstGap)
            guard step >= 240 else { continue }
            driverSpeeds.append(driver.components[SDFPhysicsMotionComponent.self]!.angularVelocity.y)
            drivenSpeeds.append(driven.components[SDFPhysicsMotionComponent.self]!.angularVelocity.y)
        }
        let driverMean = driverSpeeds.reduce(0, +) / 120, drivenMean = drivenSpeeds.reduce(0, +) / 120
        let report = Comment(rawValue: "\(preset): driver \(driverMean), driven \(drivenMean) (\(drivenSpeeds.min()!) to \(drivenSpeeds.max()!)), pins up to \(worstGap) apart")
        #expect(abs(driverMean - 1) < 0.1, report)
        // The other way round, about as fast as the spikes carry it, and steadily
        #expect(drivenMean * driverMean < 0 && abs(drivenMean) > 0.5 * abs(driverMean), report)
        #expect(drivenSpeeds.max()! - drivenSpeeds.min()! < abs(driverMean), report)
        #expect(worstGap < 0.01, report)
    }

    @Test func servoTracksATargetChangedEveryStep() throws {
        // A level arm on a hinge about z, carrying a loose box: the motor works against its weight
        // and its contact as it rocks
        let anchor = addBox(halfExtents: [0.05, 0.05, 0.05], at: [0, 1, 0], mode: .static)
        let arm = addBox(halfExtents: [1, 0.05, 0.3], at: [0, 1, 0])
        arm.components.set(SDFPhysicsMotionComponent())
        _ = addBox(halfExtents: [0.15, 0.15, 0.15], at: [0.6, 1.2, 0])
        let axle = simd_quatf(angle: -.pi / 2, axis: [0, 1, 0]) // The pin's x axis along z
        var hinge = SDFPhysicsRevoluteJoint(pin0: anchor.pins.set(named: "axle", orientation: axle),
                                            pin1: arm.pins.set(named: "hub", orientation: axle))
        hinge.motor = .init(targetVelocity: 0, maxTorque: 2000)
        try hinge.addToSimulation()

        var worstMiss: Float = 0, misses: [Float] = [], isFinite = true
        for step in 0..<240 {
            // A servo's speed command, a new one each step: half a radian per second, back and forth
            let target = 0.5 * sin(Float(step) / 60 * .pi)
            hinge.motor!.targetVelocity = target
            root.components[SDFPhysicsJointsComponent.self]!.joints[0] = .revolute(hinge)
            simulation.step(root: root)
            let speed = arm.components[SDFPhysicsMotionComponent.self]!.angularVelocity.z
            isFinite = isFinite && speed.isFinite && arm.position.x.isFinite && arm.position.y.isFinite
            guard step >= 30 else { continue }
            misses.append(abs(speed - target))
            worstMiss = max(worstMiss, abs(speed - target))
        }
        let meanMiss = misses.reduce(0, +) / Float(misses.count)
        let report = Comment(rawValue: "misses the target by \(meanMiss) on average, \(worstMiss) at most, pins \(worstGap) apart")
        #expect(isFinite, report)
        #expect(meanMiss < 0.02, report)
        #expect(worstMiss < 0.1, report)
        #expect(worstGap < 0.01, report)
    }

    @Test(arguments: [false, true])
    func jointedSceneIsDeterministic(isMultithreaded: Bool) throws {
        func run() throws -> [Transform] {
            let root = Entity(), simulation = SDFSimulation()
            var settings = SDFPhysicsSimulationComponent(simulation: simulation)
            settings.isMultithreaded = isMultithreaded
            settings.isSleepingEnabled = false
            root.components.set(settings)
            let (driver, driven) = try addSpikyWheels(to: root, driverMotor: .init(targetVelocity: 1, maxTorque: 500))
            for _ in 0..<180 { simulation.step(root: root) }
            return [driver.transform, driven.transform]
        }
        let first = try run(), second = try run()
        #expect(first.map(\.translation) == second.map(\.translation))
        #expect(first.map(\.rotation.vector) == second.map(\.rotation.vector))
    }
}
