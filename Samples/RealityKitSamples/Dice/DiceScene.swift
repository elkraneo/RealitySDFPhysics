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
// - The pointer drags a die physically, with the simulation's drag, and "Toss all" pulls the
//   other dice into Apple's hexagon around it with spring forces. Released dice keep their
//   velocity, plus a random spin.
// - Every toss starts a new simulation, and a hidden copy of it runs ahead for a few milliseconds
//   of each frame. The engine is deterministic, so the copy predicts the roll exactly, and the score is
//   shown while the dice still roll.
// - "Max score toss": TabletopKit decides the outcome when a toss starts and makes the dice land
//   on it. Here, once the copy knows how each die lands, the die's model is turned by one of its
//   symmetries, so that its highest face is the one that lands on top. A die's shape is the same
//   after the turn, so its flight doesn't change, and the turn is invisible while it spins. The
//   real roll is never adjusted.

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

        for (index, kind) in DieKind.allCases.enumerated() {
            guard let url = Bundle.module.url(forResource: kind.rawValue, withExtension: "usdz", subdirectory: "Models") else {
                throw CocoaError(.fileNoSuchFile)
            }
            let model = try await Entity(contentsOf: url)
            let shape = try await SDFShape.bake(from: model, resolution: 96)
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

    /// Replaces the simulation with a new one, with the bodies at their current poses and the given
    /// velocities (in centimeters per second)
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
    /// The dragged die, and the others pulled around it with "Toss all"
    @ObservationIgnored private var held: [(index: Int, offset: SIMD3<Float>)] = []
    @ObservationIgnored private var gather: AnyCancellable?
    /// The height the dice are held at, in centimeters
    private let holdHeight: Float = 6

    /// Grabs the die under a point of the view with the simulation's drag, and with `tossAll` pulls
    /// the others around it
    func pickUp(at point: CGPoint, in size: CGSize, tossAll: Bool) -> Bool {
        let ray = camera.ray(through: point, in: size)
        guard let (entity, _) = simulation.beginDrag(origin: root.convert(position: ray.origin, from: nil),
                                                    direction: root.convert(direction: ray.direction, from: nil)),
              let index = dice.firstIndex(where: { $0.entity == entity }) else { return false }
        held = [(index, .zero)]
        if tossAll {
            // Apple's hexagon of the other dice around the held one, 10 cm across
            for (slot, other) in dice.indices.filter({ $0 != index }).enumerated() {
                let angle = Float(slot) * .pi / 3
                held.append((other, [10 * cos(angle), 0, 10 * sin(angle)]))
            }
            gather = simulation.subscribe(to: SDFSimulationEvents.WillSimulate.self) { [unowned self] _ in pullTogether() }
        }
        stopWatchingRoll()
        return true
    }

    /// Moves the drag target under a point of the view, at the hold height
    func move(to point: CGPoint, in size: CGSize) {
        let ray = camera.ray(through: point, in: size)
        let origin = root.convert(position: ray.origin, from: nil), direction = root.convert(direction: ray.direction, from: nil)
        guard !held.isEmpty, abs(direction.y) > 1e-6 else { return }
        simulation.drag(to: origin + direction * ((holdHeight - origin.y) / direction.y))
    }

    /// Pulls the other dice towards their places around the dragged one with critically damped
    /// springs that also hold up their weight
    private func pullTogether() {
        guard let center = held.first.map({ dice[$0.index].entity.position }) else { return }
        let stiffness: Float = 400, damping = 2 * stiffness.squareRoot()
        for (index, offset) in held.dropFirst() {
            let entity = dice[index].entity
            guard let mass = simulation.mass(of: entity), let motion = entity.components[SDFMotionComponent.self] else { continue }
            let settings = root.components[SDFSimulationComponent.self]!
            let acceleration = stiffness * (center + offset - entity.position) - damping * motion.linearVelocity - settings.gravity
            simulation.addForce(mass * acceleration, to: entity)
        }
    }

    /// Lets the held dice go with their velocities and a random spin, in a new simulation
    func toss(maxScore: Bool) {
        guard !held.isEmpty else { return }
        simulation.endDrag()
        gather = nil
        var motions: [Int: SDFMotionComponent] = [:]
        for index in dice.indices {
            var motion = dice[index].entity.components[SDFMotionComponent.self] ?? SDFMotionComponent()
            if held.contains(where: { $0.index == index }) {
                // At most 2 m/s, so that 2 cm dice don't pass through each other between substeps
                if length(motion.linearVelocity) > 200 { motion.linearVelocity = normalize(motion.linearVelocity) * 200 }
                motion.angularVelocity += SIMD3<Float>.random(in: -15...15)
            }
            motions[index] = motion
        }
        let tossed = held.map(\.index).sorted()
        held = []

        startSimulation(motions: motions)
        watchRoll(tossed: tossed, prediction: makeHiddenCopy(motions: motions), maxScore: maxScore)
    }

    /// A copy of the new simulation, not in the scene, with copies of the dice
    private func makeHiddenCopy(motions: [Int: SDFMotionComponent]) -> (root: Entity, dice: [Entity]) {
        func copy(_ entity: Entity) -> Entity {
            let copy = Entity()
            copy.transform = entity.transform
            return copy
        }
        let copies = dice.map { copy($0.entity) }
        let fixed = [table] + walls
        let bodies = fixed.map { (copy($0), $0.components[SDFBodyComponent.self]!, nil as SDFMotionComponent?) }
            + dice.indices.map { (copies[$0], dieBody(dice[$0]), motions[$0] ?? SDFMotionComponent()) }
        return (makeRoot(bodies: bodies), copies)
    }

    private static func isAtRest(_ entity: Entity) -> Bool {
        guard let motion = entity.components[SDFMotionComponent.self] else { return true }
        return length(motion.linearVelocity) < 0.5 && length(motion.angularVelocity) < 0.05
    }

    // MARK: Scoring

    @ObservationIgnored private var rollSubscription: AnyCancellable?

    /// After every step of the roll, advances the hidden copy until it comes to rest, then waits for
    /// the real dice to rest and reads them
    private func watchRoll(tossed: [Int], prediction: (root: Entity, dice: [Entity]), maxScore: Bool) {
        predicted = nil
        rolled = nil
        isRolling = true
        let hidden = prediction.root.components[SDFSimulationComponent.self]!.simulation
        var predictionSteps = 0, hiddenResting = 0, resting = 0
        rollSubscription = simulation.subscribe(to: SDFSimulationEvents.DidSimulate.self) { [unowned self] _ in
            if predicted == nil {
                // A few milliseconds of each frame, so that the roll stays smooth
                let budget = Date().addingTimeInterval(0.004)
                while Date() < budget && hiddenResting < 30 && predictionSteps < 3600 {
                    hidden.step(root: prediction.root)
                    predictionSteps += 1
                    hiddenResting = prediction.dice.allSatisfy(Self.isAtRest) ? hiddenResting + 1 : 0
                }
                if hiddenResting >= 30 || predictionSteps >= 3600 {
                    for index in tossed where maxScore {
                        let kind = dice[index].kind
                        let landed = kind.face(for: prediction.dice[index].orientation)
                        if let top = kind.faces.max(by: { $0.value < $1.value }),
                           let symmetry = kind.symmetry(mapping: top, to: landed) {
                            dice[index].model.orientation = symmetry
                        }
                    }
                    predicted = tossed.map { read(index: $0, orientation: prediction.dice[$0].orientation) }
                }
            }
            resting = dice.map(\.entity).allSatisfy(Self.isAtRest) ? resting + 1 : 0
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
        gather = nil
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
