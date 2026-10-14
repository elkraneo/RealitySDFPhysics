// Port of Apple's sample "Simulating dice rolls as a component for your game":
// https://developer.apple.com/documentation/tabletopkit/simulating-dice-rolls-as-a-component-for-your-game
//
// Apple's seven dice lie in a row on a round table 1 m across. Pick one up, and throw it, or all of
// them with "Toss all"; the score of the last roll is the sum of the faces on top. Differences from
// Apple's sample, which builds on TabletopKit:
//
// - The dice are SDF bodies baked from Apple's models, on an SDF table, in a simulation whose root
//   is scaled to centimeters. TabletopKit's `TossableRepresentation` is visionOS only.
// - A rim and invisible walls above it keep the dice on the table, as TabletopKit's table does.
// - The dice are picked up with the pointer and thrown with its velocity, plus a random spin.
// - "Max score toss": TabletopKit decides the outcome when a toss starts and makes the dice land
//   on it. Here every toss starts a new simulation, so a hidden copy of the same toss, run first,
//   lands exactly as the real one will: the engine is deterministic. Each die's model is then
//   turned by one of its symmetries, so that its highest face is the one that will land on top.
//   The die's shape, and so its flight, doesn't change. The real roll is never adjusted.

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
    @ObservationIgnored let camera = SampleCamera(eye: [0, 0.38, 0.55], target: [0.06, 0, 0], fieldOfView: 40)
    @ObservationIgnored private var root = Entity()
    @ObservationIgnored private var dice: [Die] = []
    @ObservationIgnored private let table = Entity()
    @ObservationIgnored private let rim = Entity()
    /// Invisible walls above the rim
    @ObservationIgnored private var walls: [Entity] = []

    /// The sum of the faces on top after the last roll
    private(set) var lastRollScore: Int?
    /// The score the hidden copy of the last roll predicted
    private(set) var predictedScore: Int?
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

        let rimShape = SDFShape.torus(majorRadius: Self.tableRadius, minorRadius: 1.2)
        var rimBody = SDFBodyComponent(shape: rimShape, mode: .static)
        rimBody.filter = tableBody.filter
        rimBody.material = tableBody.material
        rim.components.set(rimBody)
        rim.addChild(ModelEntity(mesh: try .generate(from: rimShape),
                                 materials: [SimpleMaterial(color: .init(white: 0.25, alpha: 1), roughness: 0.6, isMetallic: false)]))

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
            let shape = try await SDFShape.bake(from: model, resolution: 48)
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

    /// The bodies of a simulation: the table, its rim, the walls and the dice, always in this order, so that
    /// two simulations built from the same poses and velocities run identically
    private func makeRoot(bodies: [(entity: Entity, component: SDFBodyComponent, motion: SDFMotionComponent?)]) -> Entity {
        let root = Entity()
        root.scale = SIMD3(repeating: 0.01)
        var settings = SDFSimulationComponent()
        settings.gravity = [0, -981, 0] // In centimeters per second squared
        settings.substeps = 8 // Small fast dice
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
        return body
    }

    /// Replaces the simulation with a new one, with the bodies at their current poses and the given
    /// velocities (in centimeters per second)
    private func startSimulation(motions: [Int: SDFMotionComponent]) {
        let fixed = [table, rim] + walls
        let bodies = fixed.map { ($0, $0.components[SDFBodyComponent.self]!, nil as SDFMotionComponent?) }
            + dice.enumerated().map { index, die in
                (die.entity, dieBody(die), motions[index] ?? SDFMotionComponent())
            }
        root.removeFromParent()
        root = makeRoot(bodies: bodies.map { ($0.0, $0.1, $0.2) })
        content.addChild(root)
    }

    // MARK: Picking up and tossing

    private var held: [(index: Int, offset: SIMD3<Float>)] = []
    private var pointerSamples: [(time: Date, position: SIMD3<Float>)] = []
    /// The height the dice are held at, in meters
    private let holdHeight: Float = 0.08

    /// Picks up the die under a point of the view, and all others around it with `tossAll`
    func pickUp(at point: CGPoint, in size: CGSize, tossAll: Bool) -> Bool {
        let ray = camera.ray(through: point, in: size)
        guard let simulation = root.components[SDFSimulationComponent.self]?.simulation,
              let hit = simulation.raycast(origin: root.convert(position: ray.origin, from: nil),
                                           direction: root.convert(direction: ray.direction, from: nil), length: 1000,
                                           query: .nearest, mask: .all.subtracting(Self.tableGroup)).first,
              let index = dice.firstIndex(where: { $0.entity == hit.entity }) else { return false }
        held = [(index, .zero)]
        if tossAll {
            // Apple's hexagon of the other dice around the held one, 10 cm across
            let others = dice.indices.filter { $0 != index }
            for (slot, other) in others.enumerated() {
                let angle = Float(slot) * .pi / 3
                held.append((other, [0.1 * cos(angle), 0, 0.1 * sin(angle)]))
            }
        }
        for (index, _) in held {
            dice[index].entity.components[SDFBodyComponent.self]?.mode = .kinematic
        }
        pointerSamples = []
        isRolling = false
        return true
    }

    /// Moves the held dice under a point of the view, at the hold height
    func move(to point: CGPoint, in size: CGSize, at time: Date) {
        let ray = camera.ray(through: point, in: size)
        guard !held.isEmpty, abs(ray.direction.y) > 1e-4 else { return }
        let position = ray.origin + ray.direction * ((holdHeight - ray.origin.y) / ray.direction.y)
        for (index, offset) in held {
            dice[index].entity.setPosition(position + offset, relativeTo: nil)
        }
        pointerSamples.append((time, position))
        pointerSamples.removeAll { time.timeIntervalSince($0.time) > 0.1 }
    }

    /// Throws the held dice with the pointer's velocity and a random spin
    func toss(maxScore: Bool) {
        guard !held.isEmpty else { return }
        var velocity = SIMD3<Float>.zero
        if let first = pointerSamples.first, let last = pointerSamples.last, last.time > first.time {
            velocity = (last.position - first.position) / Float(last.time.timeIntervalSince(first.time))
        }
        // At most 2 m/s, so that 2 cm dice don't pass through each other between substeps
        if length(velocity) > 2 { velocity = normalize(velocity) * 2 }
        velocity.y += 0.5
        var motions: [Int: SDFMotionComponent] = [:]
        for (index, _) in held {
            motions[index] = SDFMotionComponent(linearVelocity: velocity * 100, angularVelocity: .random(in: -20...20))
        }
        let tossed = held.map(\.index)
        held = []

        // Run the same toss in a hidden simulation first
        let prediction = predict(motions: motions)
        for index in tossed {
            let die = dice[index]
            let landed = die.kind.face(for: prediction[index])
            if maxScore, let top = die.kind.faces.max(by: { $0.value < $1.value }),
               let symmetry = die.kind.symmetry(mapping: top, to: landed) {
                die.model.orientation = symmetry
            }
        }
        predictedScore = tossed.map { dice[$0].kind.face(for: prediction[$0] * dice[$0].model.orientation).value }.reduce(0, +)
        lastRollScore = nil
        rolled = tossed

        startSimulation(motions: motions)
        isRolling = true
        watchRoll()
    }

    /// The resting orientations, in the root's frame, of the dice of a toss run in a hidden copy
    private func predict(motions: [Int: SDFMotionComponent]) -> [simd_quatf] {
        func copy(_ entity: Entity) -> Entity {
            let copy = Entity()
            copy.transform = entity.transform
            return copy
        }
        let copies = dice.map { copy($0.entity) }
        let fixed = [table, rim] + walls
        let bodies = fixed.map { (copy($0), $0.components[SDFBodyComponent.self]!, nil as SDFMotionComponent?) }
            + dice.indices.map { (copies[$0], dieBody(dice[$0]), motions[$0] ?? SDFMotionComponent()) }
        let hidden = makeRoot(bodies: bodies)
        let simulation = hidden.components[SDFSimulationComponent.self]!.simulation
        var resting = 0
        for _ in 0..<1200 where resting < 30 {
            simulation.step(root: hidden)
            resting = copies.allSatisfy(Self.isAtRest) ? resting + 1 : 0
        }
        return copies.map(\.orientation)
    }

    private static func isAtRest(_ entity: Entity) -> Bool {
        guard let motion = entity.components[SDFMotionComponent.self] else { return true }
        return length(motion.linearVelocity) < 0.5 && length(motion.angularVelocity) < 0.05
    }

    // MARK: Scoring

    @ObservationIgnored private var rolled: [Int] = []
    @ObservationIgnored private var rollSubscription: AnyCancellable?

    /// Scores the roll once the dice have been at rest for half a second
    private func watchRoll() {
        guard let simulation = root.components[SDFSimulationComponent.self]?.simulation else { return }
        var resting = 0
        rollSubscription = simulation.subscribe(to: SDFSimulationEvents.DidSimulate.self) { [unowned self] _ in
            resting = dice.map(\.entity).allSatisfy(Self.isAtRest) ? resting + 1 : 0
            guard resting >= 30 else { return }
            lastRollScore = rolled.map { dice[$0].kind.face(for: dice[$0].entity.orientation * dice[$0].model.orientation).value }
                .reduce(0, +)
            isRolling = false
            rollSubscription = nil
        }
    }

    /// Puts the dice back in their row, with their models unturned
    func reset() {
        held = []
        for (index, die) in dice.enumerated() {
            die.entity.transform = Transform(scale: SIMD3(repeating: 100), translation: [-15 + 6 * Float(index + 1), 1.5, 0])
            die.model.orientation = simd_quatf()
        }
        lastRollScore = nil
        predictedScore = nil
        isRolling = false
        rollSubscription = nil
        startSimulation(motions: [:])
    }
}
