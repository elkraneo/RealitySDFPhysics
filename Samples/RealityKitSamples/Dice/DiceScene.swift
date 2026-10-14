// Port of Apple's sample "Simulating dice rolls as a component for your game":
// https://developer.apple.com/documentation/tabletopkit/simulating-dice-rolls-as-a-component-for-your-game
//
// Apple's seven dice lie in a row on a round table 1 m across. Pick one up, and throw it, or all of
// them with "Toss all"; the score of the last roll is the sum of the faces on top. Differences from
// Apple's sample, which builds on TabletopKit:
//
// - The dice are SDF bodies baked from Apple's models, on an SDF table, in a simulation whose root
//   is scaled to centimeters. TabletopKit's `TossableRepresentation` is visionOS only.
// - Invisible walls around the table keep the dice on it, as TabletopKit's table does.
// - The dice are damped more than the engine's default, standing in for the rolling resistance
//   that stops real dice within a second or two, which the engine doesn't model yet.
// - The pointer holds a die by the point it grabbed, with a critically damped spring that carries
//   its weight, and "Toss all" pulls the other dice into Apple's hexagon around it the same way.
//   Released dice keep their velocity, plus a random spin.
// - When the dice are released, the simulation forecasts the roll: an exact copy of it steps
//   ahead for a few milliseconds of each step. The engine is deterministic, so the forecast is
//   the roll, and the score shows while the dice still roll.
// - "Max score toss": TabletopKit decides the outcome when a toss starts and makes the dice land
//   on it. Here, once the forecast knows how each die lands, the die's model is turned by one of
//   its symmetries, so that its highest face is the one that lands on top. A die's shape is the
//   same after the turn, so its flight doesn't change, and the turn is invisible while it spins.
//   The real roll is never adjusted.

import Combine
import RealityKit
import SDFCollisionKit
import simd
import SwiftUI

@MainActor
@Observable
final class DiceScene {
    struct Die {
        let kind: DieKind
        /// The body, in the simulation root's centimeters
        let entity: Entity
        /// Apple's model, in meters, turned by "Max score toss"
        let model: Entity
        let shape: SDFShape
    }

    @ObservationIgnored let content = Entity()
    @ObservationIgnored let camera = SampleCamera(eye: [0.09, 0.3, 0.4], target: [0.09, 0, 0], fieldOfView: 40)
    @ObservationIgnored private var root = Entity()
    @ObservationIgnored private var dice: [Die] = []
    @ObservationIgnored private let table = Entity()
    @ObservationIgnored private let rim = Entity()
    /// Invisible walls above the rim
    @ObservationIgnored private var walls: [Entity] = []

    /// A thrown die and the value on top
    struct Roll: Identifiable {
        var id: String { kind.rawValue }
        let kind: DieKind
        let value: Int
    }

    /// The faces the hidden copy of the last toss predicted, as soon as it knows them
    private(set) var predicted: [Roll]?
    /// The faces on top once the last toss came to rest
    private(set) var rolled: [Roll]?
    private(set) var isRolling = false

    // MARK: Setup

    static let tableRadius: Float = 50
    /// The table's collision group, which picking leaves out
    static let tableGroup = CollisionGroup(rawValue: 1 << 2)

    init() {
        let light = DirectionalLight()
        light.light.intensity = 3500
        light.shadow = DirectionalLightComponent.Shadow(maximumDistance: 2, depthBias: 1)
        light.look(at: .zero, from: [0.4, 1, 0.5], relativeTo: nil)
        content.addChild(light)
        content.addChild(camera.makeEntity())
    }

    /// Loads Apple's dice models and bakes their shapes
    func load() async throws {
        // A table 1 m across and its rim, in centimeters
        var tableBody = SDFBodyComponent(shape: .cylinder(radius: Self.tableRadius, halfHeight: 5), mode: .static)
        tableBody.filter = CollisionFilter(group: Self.tableGroup, mask: .all)
        tableBody.material = SDFMaterial(friction: 0.6, restitution: 0.3)
        table.components.set(tableBody)
        table.position = [0, -5, 0]
        let top = ModelEntity(mesh: .generateCylinder(height: 1, radius: Self.tableRadius),
                              materials: [SimpleMaterial(color: .init(red: 0.18, green: 0.32, blue: 0.24, alpha: 1), roughness: 0.9,
                                                         isMetallic: false)])
        top.position = [0, 4.5, 0]
        table.addChild(top)

        // The rim is only seen: the walls hold the dice, and a torus as large as the table would be
        // in collision detection with every die
        let rimShape = SDFShape.torus(majorRadius: Self.tableRadius, minorRadius: 1.2)
        rim.addChild(ModelEntity(mesh: try .generate(from: rimShape),
                                 materials: [SimpleMaterial(color: .init(white: 0.25, alpha: 1), roughness: 0.6, isMetallic: false)]))
        rim.position = [0, 5, 0] // On the table's top face
        table.addChild(rim)

        // 32 panels, 15 cm high, around the rim
        let panels = 32, wallShape = SDFShape.box(halfExtents: [5.2, 7.5, 1])
        for i in 0..<panels {
            let angle = Float(i) / Float(panels) * 2 * .pi
            let wall = Entity()
            wall.position = [cos(angle) * (Self.tableRadius + 1), 7.5, sin(angle) * (Self.tableRadius + 1)]
            wall.orientation = simd_quatf(angle: -angle + .pi / 2, axis: [0, 1, 0])
            var body = SDFBodyComponent(shape: wallShape, mode: .static)
            body.filter = tableBody.filter
            body.material = tableBody.material
            wall.components.set(body)
            walls.append(wall)
        }

        var models: [Entity] = []
        for kind in DieKind.allCases {
            guard let url = Bundle.module.url(forResource: kind.rawValue, withExtension: "usdz", subdirectory: "Models") else {
                throw CocoaError(.fileNoSuchFile)
            }
            models.append(try await Entity(contentsOf: url))
        }
        // Convex, so 48 cells across a die are plenty
        let shapes = try await SDFShape.bake(from: models, resolution: 48)
        for (index, (kind, model)) in zip(DieKind.allCases, models).enumerated() {
            let shape = shapes[index]
            let entity = Entity()
            entity.name = kind.rawValue
            // The model is in meters, the simulation in centimeters
            entity.scale = SIMD3(repeating: 100)
            entity.addChild(model)
            dice.append(Die(kind: kind, entity: entity, model: model, shape: shape))
            // Apple's layout: a row across the center, 6 cm apart
            entity.position = [-15 + 6 * Float(index + 1), 1.5, 0]
        }
        startSimulation(motions: [:])
    }

    /// The bodies of a simulation: the table, the walls and the dice, always in this order, so that
    /// two simulations built from the same poses and velocities run identically
    private func makeRoot(bodies: [(entity: Entity, component: SDFBodyComponent, motion: SDFMotionComponent?)]) -> Entity {
        let root = Entity()
        root.scale = SIMD3(repeating: 0.01)
        var settings = SDFSimulationComponent(metersPerUnit: 0.01)
        // Dice bounce from slower approaches than the default 1 m/s
        settings.restitutionThreshold = 10
        root.components.set(settings)
        for body in bodies {
            body.entity.components.set(body.component)
            if let motion = body.motion { body.entity.components.set(motion) }
            root.addChild(body.entity, preservingWorldTransform: false)
        }
        return root
    }

    private func dieBody(_ die: Die) -> SDFBodyComponent {
        // Plastic: 1.2 g, or 0.0012 kg, per cubic centimeter of the root
        var body = SDFBodyComponent(shape: die.shape, massProperties: .init(density: 0.0012),
                                    material: .init(friction: 0.5, restitution: 0.35))
        body.filter = CollisionFilter(group: .default, mask: .all)
        // Real dice stop within a second or two, from rolling resistance, which the engine doesn't
        // model yet. Without it, a die can creep and spin slowly for half a minute.
        body.linearDamping = 0.5
        body.angularDamping = 3
        return body
    }

    /// Replaces the simulation with a new one, with the bodies at their current poses
    private func startSimulation(motions: [Int: SDFMotionComponent]) {
        let fixed = [table] + walls
        let bodies = fixed.map { ($0, $0.components[SDFBodyComponent.self]!, nil as SDFMotionComponent?) }
            + dice.enumerated().map { index, die in
                (die.entity, dieBody(die), motions[index] ?? SDFMotionComponent())
            }
        root.removeFromParent()
        root = makeRoot(bodies: bodies.map { ($0.0, $0.1, $0.2) })
        content.addChild(root)
    }

    // MARK: Picking up and tossing

    private var simulation: SDFSimulation { root.components[SDFSimulationComponent.self]!.simulation }
    private var gravity: SIMD3<Float> { root.components[SDFSimulationComponent.self]!.gravity }
    /// The held die, and the others pulled around it with "Toss all"
    @ObservationIgnored private var held: [(index: Int, offset: SIMD3<Float>)] = []
    /// The grabbed point, in the held die's frame, and where the pointer wants it, in the root's
    @ObservationIgnored private var grab: (anchor: SIMD3<Float>, target: SIMD3<Float>)?
    @ObservationIgnored private var holding: AnyCancellable?
    /// The height the dice are held at, in centimeters
    private let holdHeight: Float = 6

    /// Grabs the die under a point of the view, and with `tossAll` pulls the others around it
    func pickUp(at point: CGPoint, in size: CGSize, tossAll: Bool) -> Bool {
        let ray = camera.ray(through: point, in: size)
        guard let hit = simulation.raycast(origin: root.convert(position: ray.origin, from: nil),
                                           direction: root.convert(direction: ray.direction, from: nil), length: 1000,
                                           query: .nearest, mask: .all.subtracting(Self.tableGroup)).first,
              let index = dice.firstIndex(where: { $0.entity == hit.entity }) else { return false }
        held = [(index, .zero)]
        if tossAll {
            // Apple's hexagon of the other dice around the held one, 10 cm across
            for (slot, other) in dice.indices.filter({ $0 != index }).enumerated() {
                let angle = Float(slot) * .pi / 3
                held.append((other, [10 * cos(angle), 0, 10 * sin(angle)]))
            }
        }
        let entity = dice[index].entity
        grab = (entity.convert(position: hit.position, from: root), hit.position)
        holding = simulation.subscribe(to: SDFSimulationEvents.WillSimulate.self) { [unowned self] _ in hold() }
        stopWatchingRoll()
        return true
    }

    /// Moves the grabbed point's target under a point of the view, at the hold height
    func move(to point: CGPoint, in size: CGSize) {
        let ray = camera.ray(through: point, in: size)
        let origin = root.convert(position: ray.origin, from: nil), direction = root.convert(direction: ray.direction, from: nil)
        guard grab != nil, abs(direction.y) > 1e-6 else { return }
        grab?.target = origin + direction * ((holdHeight - origin.y) / direction.y)
    }

    /// Before every step: pulls the grabbed point towards its target, and the other dice towards
    /// their places around the held one, with critically damped springs that also carry the dice's
    /// weight and calm their spin
    private func hold() {
        guard let grab, let (index, _) = held.first else { return }
        let entity = dice[index].entity
        guard let mass = simulation.mass(of: entity), let motion = entity.components[SDFMotionComponent.self] else { return }
        let frequency: Float = 15 // Radians per second: the die follows within about a tenth of a second
        let anchor = entity.convert(position: grab.anchor, to: root)
        let anchorVelocity = motion.linearVelocity + cross(motion.angularVelocity, anchor - entity.position)
        let pull = frequency * frequency * (grab.target - anchor) - 2 * frequency * anchorVelocity
        simulation.addForce(mass * pull, at: anchor, to: entity)
        simulation.addForce(-mass * gravity, to: entity)
        simulation.addTorque(-mass * 2 * motion.angularVelocity, to: entity)

        let center = entity.position
        for (other, offset) in held.dropFirst() {
            let entity = dice[other].entity
            guard let mass = simulation.mass(of: entity), let motion = entity.components[SDFMotionComponent.self] else { continue }
            let pull = frequency * frequency * (center + offset - entity.position) - 2 * frequency * motion.linearVelocity
            simulation.addForce(mass * (pull - gravity), to: entity)
        }
    }

    /// Lets the held dice go with their velocities and a random spin, and forecasts the roll
    func toss(maxScore: Bool) {
        guard !held.isEmpty else { return }
        holding = nil
        grab = nil
        for (index, _) in held {
            let entity = dice[index].entity
            var motion = entity.components[SDFMotionComponent.self] ?? SDFMotionComponent()
            // At most 2 m/s, so that 2 cm dice don't pass through each other between substeps
            if length(motion.linearVelocity) > 200 { motion.linearVelocity = normalize(motion.linearVelocity) * 200 }
            motion.angularVelocity += SIMD3<Float>.random(in: -15...15)
            simulation.setMotion(motion, of: entity)
        }
        let tossed = held.map(\.index).sorted()
        held = []
        watchRoll(tossed: tossed, forecast: simulation.forecast(), maxScore: maxScore)
    }

    private static func isAtRest(_ motion: SDFMotionComponent?) -> Bool {
        guard let motion else { return true }
        return length(motion.linearVelocity) < 0.5 && length(motion.angularVelocity) < 0.05
    }

    // MARK: Scoring

    @ObservationIgnored private var rollSubscription: AnyCancellable?

    /// After every step of the roll, advances the forecast for a few milliseconds until its dice
    /// rest, then waits for the real dice to rest and reads them
    private func watchRoll(tossed: [Int], forecast: SDFForecast, maxScore: Bool) {
        predicted = nil
        rolled = nil
        isRolling = true
        var forecastResting = 0, resting = 0
        rollSubscription = simulation.subscribe(to: SDFSimulationEvents.DidSimulate.self) { [unowned self] _ in
            if predicted == nil {
                // A few milliseconds of each step, so that the roll stays smooth
                let budget = Date().addingTimeInterval(0.004)
                while Date() < budget && forecastResting < 30 && forecast.stepCount < 3600 {
                    forecast.step()
                    forecastResting = dice.allSatisfy { Self.isAtRest(forecast.motion(of: $0.entity)) } ? forecastResting + 1 : 0
                }
                if forecastResting >= 30 || forecast.stepCount >= 3600 {
                    let landings = tossed.map { forecast.transform(of: dice[$0].entity)!.rotation }
                    for (index, landing) in zip(tossed, landings) where maxScore {
                        let kind = dice[index].kind
                        if let top = kind.faces.max(by: { $0.value < $1.value }),
                           let symmetry = kind.symmetry(mapping: top, to: kind.face(for: landing)) {
                            dice[index].model.orientation = symmetry
                        }
                    }
                    predicted = zip(tossed, landings).map { read(index: $0, orientation: $1) }
                }
            }
            resting = dice.allSatisfy { Self.isAtRest($0.entity.components[SDFMotionComponent.self]) } ? resting + 1 : 0
            guard predicted != nil, resting >= 30 else { return }
            rolled = tossed.map { read(index: $0, orientation: dice[$0].entity.orientation) }
            isRolling = false
            rollSubscription = nil
        }
    }

    /// The face on top of a die with a body orientation
    private func read(index: Int, orientation: simd_quatf) -> Roll {
        Roll(kind: dice[index].kind, value: dice[index].kind.face(for: orientation * dice[index].model.orientation).value)
    }

    private func stopWatchingRoll() {
        rollSubscription = nil
        isRolling = false
    }

    /// Puts the dice back in their row, with their models unturned
    func reset() {
        held = []
        holding = nil
        grab = nil
        stopWatchingRoll()
        for (index, die) in dice.enumerated() {
            die.entity.transform = Transform(scale: SIMD3(repeating: 100), translation: [-15 + 6 * Float(index + 1), 1.5, 0])
            die.model.orientation = simd_quatf()
        }
        predicted = nil
        rolled = nil
        startSimulation(motions: [:])
    }
}

