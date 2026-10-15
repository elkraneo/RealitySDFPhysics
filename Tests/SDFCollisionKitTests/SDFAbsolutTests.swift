import Combine
import RealityKit
import SDFCollisionKit
import SDFDemoScenes
import SDFMechanicsScenes
import Testing
import simd

/// The Absolut scene: 48 links around a piñon of 18 teeth on the left and a plato of 29 on the
/// right, the plato motored clockwise, as a bicycle riding to the right pedals. The piñon must
/// turn by the chain alone, the same way, 29/18 as fast, with every body in the x-y plane.
@MainActor
@Suite struct SDFAbsolutTests {
    static let ratio: Float = 29.0 / 18

    /// The small sprocket on the left: the layout was first written for A larger than B
    @Test func layoutClosesWithTheSmallSprocketFirst() {
        let chain = ChainMesh()
        let layout = ChainLayout(chain: chain, links: 48, teethA: 18, teethB: 29)
        let pins = layout.pins
        #expect(pins.count == 48)
        let gaps = pins.indices.map { simd_distance(pins[$0], pins[($0 + 1) % pins.count]) }
        let report = Comment(rawValue: "center distance \(layout.centerDistance), gaps \(gaps.min()!) to \(gaps.max()!), closing \(gaps.last!)")
        #expect(abs(layout.centerDistance - 5.56) < 0.1, report)
        // A pitch around the sprockets, and on the runs closer by the slack, shared out: 4% of
        // the chain over the runs' 22 gaps is about 9% of each
        #expect(gaps.allSatisfy { $0 > 0.9 * chain.pitch && $0 < chain.pitch + 1e-4 }, report)
        #expect(abs(gaps.last! - chain.pitch) < 1e-3, report)
    }

    /// The scene loaded under a preset, with its sprockets and links
    func load(_ preset: SDFSimulationComponent.Preset) async throws
        -> (root: Entity, simulation: SDFSimulation, piñon: Entity, plato: Entity, links: [Entity]) {
        let root = Entity(), simulation = SDFSimulation()
        var settings = SDFSimulationComponent(simulation: simulation, preset: preset)
        settings.isSleepingEnabled = false
        root.components.set(settings)
        try await MechanicsScenes.all.first { $0.name == "Absolut" }!.load(into: root)
        #expect(root.components[SDFSimulationComponent.self]!.gravity == [0, -3, 0])
        let links = root.children.filter { $0.name.hasPrefix("link") }
        #expect(links.count == 48)
        return (root, simulation, root.findEntity(named: "sprocket A")!, root.findEntity(named: "sprocket B")!, links)
    }

    /// The farthest any two pins of a joint are apart
    func worstPin(_ root: Entity) -> Float {
        root.components[SDFJointsComponent.self]!.joints.map {
            simd_distance($0.pin0.position(relativeTo: root)!, $0.pin1.position(relativeTo: root)!)
        }.max()!
    }

    @Test(arguments: [SDFSimulationComponent.Preset.realTime, .paper])
    func platoDrivesThePiñonThroughTheChain(preset: SDFSimulationComponent.Preset) async throws {
        let scene = try await load(preset)
        var speedsA: [Float] = [], speedsB: [Float] = [], pins: [Float] = []
        for _ in 0..<420 {
            scene.simulation.step(root: scene.root)
            speedsA.append(scene.piñon.components[SDFMotionComponent.self]!.angularVelocity.z)
            speedsB.append(scene.plato.components[SDFMotionComponent.self]!.angularVelocity.z)
            pins.append(worstPin(scene.root))
        }
        let lastA = speedsA.suffix(120), lastB = speedsB.suffix(120)
        let meanA = lastA.reduce(0, +) / 120, meanB = lastB.reduce(0, +) / 120
        // A roller landing in a seat stretches its pins for a step or two: the typical step of the
        // last two seconds, and the worst
        let lastPins = pins.suffix(120).sorted(), worst = lastPins[60], spike = lastPins.last!
        let drift = scene.links.map { abs($0.position.z) }.max()!
        let report = Comment(rawValue: "\(preset): piñon \(meanA) (\(lastA.min()!) to \(lastA.max()!)), plato \(meanB) (\(lastB.min()!) to \(lastB.max()!)) over the last two seconds; pins up to \(worst) apart in a typical step, \(spike) at worst, links up to \(drift) off the plane; step \(scene.simulation.statistics.stepMilliseconds) ms")
        #expect(abs(meanB + 1) < 0.15, report)
        #expect(abs(meanA + Self.ratio) < 0.15 * Self.ratio, report)
        #expect(worst < 0.03, report)
        #expect(spike < 0.15, report)
        #expect(drift < 1e-3, report)
        #expect(scene.links.allSatisfy { $0.position.y > 0 && $0.position.y < 8 }, report)
        print(report.rawValue)
    }

    /// A minute of pedaling: the piñon's turn stays 29/18 of the plato's. The chain's slack moves
    /// from the top run to the bottom one as the motor takes it up, which puts the piñon about a
    /// tooth behind in the first seconds and lets it wander a little after; a skipped tooth would
    /// stay.
    @Test func noToothSkippedInAMinute() async throws {
        let scene = try await load(.realTime)
        let timeStep = scene.root.components[SDFSimulationComponent.self]!.timeStep
        let tooth = 2 * Float.pi / 18
        var angleA: Float = 0, angleB: Float = 0, worst: Float = 0
        var slips: [Float] = [] // In teeth of the piñon, every second
        for step in 1...3600 {
            scene.simulation.step(root: scene.root)
            angleA += scene.piñon.components[SDFMotionComponent.self]!.angularVelocity.z * timeStep
            angleB += scene.plato.components[SDFMotionComponent.self]!.angularVelocity.z * timeStep
            // After the first second, which lays the laid out runs straight
            if step > 60 { worst = max(worst, worstPin(scene.root)) }
            if step % 60 == 0 { slips.append((angleA - Self.ratio * angleB) / tooth) }
        }
        let settled = slips[4], last = slips.last!
        let report = Comment(rawValue: "60 s: plato \(angleB) rad, piñon \(angleA) rad (\(angleA / angleB)×); piñon off by \(settled) teeth at 5 s, \(last) at 60 s, \(slips[4...].min()!) to \(slips[4...].max()!) in between; pins up to \(worst) apart, \(worstPin(scene.root)) at the end")
        #expect(abs(angleB + 60) < 6, report)
        #expect(abs(last - settled) < 0.5, report)
        #expect(worst < 0.15, report)
        #expect(worstPin(scene.root) < 0.03, report)
        print(report.rawValue)
    }

    // MARK: Pedals, hung objects and the zone

    /// The world angle of an entity about z
    func roll(_ entity: Entity, in root: Entity) -> Float {
        let q = entity.orientation(relativeTo: root)
        return 2 * atan2(q.imag.z, q.real)
    }

    /// The pedals turn freely on the crank and stay level, as their damping holds them
    @Test func pedalsStayLevel() async throws {
        let scene = try await load(.realTime)
        let pedals = ["pedal front", "pedal back"].map { scene.root.findEntity(named: $0)! }
        let timeStep = scene.root.components[SDFSimulationComponent.self]!.timeStep
        var turned: Float = 0, worst: Float = 0
        // A full turn of the plato at 1 rad/s
        while turned > -2 * .pi {
            scene.simulation.step(root: scene.root)
            turned += scene.plato.components[SDFMotionComponent.self]!.angularVelocity.z * timeStep
            worst = max(worst, pedals.map { abs(PedalDrive.wrap(roll($0, in: scene.root))) }.max()!)
        }
        let report = Comment(rawValue: "a turn of the plato: pedals up to \(worst) rad off level; pins up to \(worstPin(scene.root)) apart")
        #expect(worst < 0.3, report)
        #expect(worstPin(scene.root) < 0.03, report)
        print(report.rawValue)
    }

    @Test func driveMath() {
        #expect(abs(PedalDrive.wrap(3 * .pi / 2) + .pi / 2) < 1e-5)
        #expect(abs(PedalDrive.wrap(-3 * .pi / 2) - .pi / 2) < 1e-5)
        #expect(abs(PedalDrive.wrap(0.5) - 0.5) < 1e-6)
        #expect(abs(PedalDrive.angle(of: [1, 3, 5], about: [1, 1, 0]) - .pi / 2) < 1e-6)
        #expect(PedalDrive.pointOnPlane(origin: [0, 0, 10], direction: [0, 1, -1]) == [0, 10, 0])
        #expect(PedalDrive.pointOnPlane(origin: [0, 0, 10], direction: [0, 1, 1]) == nil)

        var drive = PedalDrive()
        // Released before a grab: the brake
        #expect(drive.motor(crankAngle: 1) == .init(targetVelocity: 0, maxTorque: 300))
        // Grabbed at the pointer's angle 2 with the crank at 1: no jump, the crank holds
        drive.grab(at: 2, crankAngle: 1)
        #expect(drive.motor(crankAngle: 1) == .init(targetVelocity: 0, maxTorque: 15000))
        // A little ahead: proportional; far ahead: clamped
        drive.move(to: 2.1)
        #expect(abs(drive.motor(crankAngle: 1).targetVelocity - 0.8) < 1e-4)
        drive.move(to: 0)
        #expect(drive.motor(crankAngle: 1).targetVelocity == -6)
        // Across the wrap: the pointer just past π, the crank just short of it, the short way round
        drive.grab(at: 0, crankAngle: 0)
        drive.move(to: -3.1)
        #expect(abs(drive.motor(crankAngle: 3.1).targetVelocity - 8 * (2 * .pi - 6.2)) < 1e-3)
        drive.release()
        #expect(drive.target == nil)
        #expect(drive.motor(crankAngle: 3.1) == .init(targetVelocity: 0, maxTorque: 300))
    }

    /// A pointer circling the plato at 1 rad/s clockwise: the crank follows it
    @Test func crankFollowsThePointer() async throws {
        let scene = try await load(.realTime)
        let timeStep = scene.root.components[SDFSimulationComponent.self]!.timeStep
        let (crank, hinge) = try #require(PedalDrive.crank(in: scene.root))
        #expect(crank == scene.plato)
        var drive = PedalDrive()
        var crankHinge: SDFRevoluteJoint {
            get throws { try #require(scene.root.components[SDFJointsComponent.self]!.joints[hinge].revolute) }
        }
        scene.simulation.step(root: scene.root)
        var pointer: Float = 0.3
        drive.grab(at: pointer, crankAngle: scene.simulation.angle(of: try crankHinge)!)
        var lags: [Float] = [], speeds: [Float] = []
        for step in 0..<300 {
            pointer -= timeStep
            drive.move(to: pointer)
            var joint = try crankHinge
            let angle = scene.simulation.angle(of: joint)!
            if step >= 120 { lags.append(PedalDrive.wrap(drive.target! - angle)) }
            joint.motor = drive.motor(crankAngle: angle)
            scene.root.components[SDFJointsComponent.self]!.joints[hinge] = .revolute(joint)
            scene.simulation.step(root: scene.root)
            speeds.append(scene.plato.components[SDFMotionComponent.self]!.angularVelocity.z)
        }
        let speed = speeds.suffix(120).reduce(0, +) / 120
        let report = Comment(rawValue: "following at 1 rad/s: after 2 s the crank lags \(lags.min()!) to \(lags.max()!) rad, turns at \(speed) rad/s")
        #expect(lags.allSatisfy { abs($0) < 0.2 }, report)
        #expect(abs(speed + 1) < 0.1, report)
        print(report.rawValue)
    }

    /// 20 s at 1 rad/s: the chain carries every board through the zone at the top of the plato,
    /// one at a time, each in its link's order; they swing within their limits on their pins,
    /// and the chain still drives at the ratio
    @Test func chainCarriesTheBoardsThroughTheZone() async throws {
        let scene = try await load(.realTime)
        let names = ["bike", "saddle", "handlebar", "Gaudí"]
        let boards = names.map { scene.root.findEntity(named: $0)! }
        let hinges = scene.root.components[SDFJointsComponent.self]!.joints.compactMap(\.revolute).filter { $0.angularLimit != nil }
        #expect(hinges.count == 4)
        let carrier = try #require(CarrierZone(root: scene.root, simulation: scene.simulation))
        var entries: [String] = [], exits: [String] = [], changes = 0, crowded = 0
        var subscriptions: [AnyCancellable] = []
        subscriptions.append(scene.simulation.subscribe(to: SDFCollisionEvents.Began.self, on: carrier.zone) { event in
            entries.append((event.entityA == carrier.zone ? event.entityB : event.entityA).name)
        })
        subscriptions.append(scene.simulation.subscribe(to: SDFCollisionEvents.Ended.self, on: carrier.zone) { event in
            exits.append((event.entityA == carrier.zone ? event.entityB : event.entityA).name)
        })
        carrier.onChange = { _ in changes += 1 }
        let timeStep = scene.root.components[SDFSimulationComponent.self]!.timeStep
        var angleA: Float = 0, angleB: Float = 0, swing: Float = 0, stretch: Float = 0, hanging: Float = 0
        for i in 0..<1200 {
            scene.simulation.step(root: scene.root)
            angleA += scene.piñon.components[SDFMotionComponent.self]!.angularVelocity.z * timeStep
            angleB += scene.plato.components[SDFMotionComponent.self]!.angularVelocity.z * timeStep
            if carrier.inside.count > 1 { crowded += 1 }
            // Under the straight of the bottom run the outward normal is straight down. The run sags
            // over a meter and waves, its links tilting up to 0.3 either way, and the hinges' limit
            // turns the boards with them once they reach it.
            for (board, hinge) in zip(boards, hinges)
            where i >= 120 && abs(PedalDrive.wrap(roll(hinge.pin0.entity!, in: scene.root) - .pi)) < 0.3 {
                hanging = max(hanging, abs(PedalDrive.wrap(roll(board, in: scene.root) - .pi)))
            }
            swing = max(swing, hinges.map { abs(scene.simulation.angle(of: $0)!) }.max()!)
            stretch = max(stretch, hinges.map { simd_distance($0.pin0.position(relativeTo: scene.root)!, $0.pin1.position(relativeTo: scene.root)!) }.max()!)
        }
        let report = Comment(rawValue: "20 s: in \(entries), out \(exits), \(changes) changes, \(crowded) steps with two inside; swing up to \(swing) rad, pins up to \(stretch) apart; boards under the bottom run up to \(hanging) rad off hanging straight down; plato \(angleB) rad, piñon \(angleA / angleB)×")
        #expect(Set(entries) == Set(names), report)
        // In and out in turn: every entry but perhaps the last is followed by its exit
        #expect(exits == Array(entries.prefix(exits.count)) && entries.count - exits.count <= 1, report)
        // The board ahead on the chain, on the higher link, comes first
        let order = ["saddle", "bike", "Gaudí", "handlebar"]
        #expect(entries.enumerated().allSatisfy { $0.element == order[$0.offset % 4] }, report)
        #expect(crowded == 0, report)
        // The active board changes as each one comes in and goes out
        #expect(changes == entries.count + exits.count, report)
        #expect(swing < 0.4, report)
        #expect(stretch < 0.05, report)
        #expect(hanging > 0 && hanging < 0.3 + 0.4, report)
        #expect(abs(angleB + 20) < 2, report)
        #expect(abs(angleA / angleB - Self.ratio) < 0.05 * Self.ratio, report)
        print(report.rawValue)
    }
}

private extension SDFJoint {
    var revolute: SDFRevoluteJoint? {
        if case .revolute(let joint) = self { joint } else { nil }
    }
}
