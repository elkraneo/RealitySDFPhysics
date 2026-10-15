import Combine
import RealityKit
import RealitySDFPhysics
import SDFDemoScenes
import simd

/// The Absolut scene's crank, pedals and hung objects: bodies on the chain that touch nothing,
/// and a zone at the top of the plato where the chain brings each object in turn.
extension DemoBuilder {
    /// Pedals: free, collide with nothing. Hung objects: limited, touch the zone only.
    static let pedalFilter = CollisionFilter(group: CollisionGroup(rawValue: 1 << 10), mask: [])
    static let hungFilter = CollisionFilter(group: CollisionGroup(rawValue: 1 << 11), mask: CollisionGroup(rawValue: 1 << 12))
    static let zoneFilter = CollisionFilter(group: CollisionGroup(rawValue: 1 << 12), mask: CollisionGroup(rawValue: 1 << 11))

    /// The pin's x axis along z, the chain's axles
    private static let axle = simd_quatf(angle: -.pi / 2, axis: [0, 1, 0])

    /// The chain: tight, a percent of slack, which still drives and leaves the return run a
    /// slight sag
    public static var absolutChain: ChainMesh {
        var chain = ChainMesh()
        chain.slack = 0.01
        return chain
    }

    /// The density of the chain's links, a quarter of water's: some 240 kg of chain
    static let linkDensity: Float = 250
    /// The density of the sprockets, solid discs where a real chainring is thin and spoked: a
    /// tenth of water's makes the plato some 190 kg, a little lighter than the chain, as on a bike.
    /// At water's they weighed ten times the chain, and the crank was heavy to pedal.
    static let sprocketDensity: Float = 100

    /// The mass of an inner link of the chain, the heavier kind, with its rollers
    private var linkMass: Float {
        Self.linkDensity * root.findEntity(named: "link 0")!.components[SDFPhysicsBodyComponent.self]!.shape.volume
    }

    /// Two crank arms drawn on the plato, half a turn apart, the front one at `angle`, and at the
    /// end of each a pedal on a free hinge. The plato's body stays the sprocket; the pedals are
    /// damped so they stay level as the crank turns.
    mutating func addPedals(to plato: Entity, length: Float = 2.7, angle: Float = -.pi / 4) {
        var paint = PhysicallyBasedMaterial()
        paint.baseColor = .init(tint: .init(white: 0.18, alpha: 1))
        paint.roughness = 0.4
        let axis = plato.position(relativeTo: root), mass = 0.5 * linkMass
        for (side, depth, turn) in [("front", Float(0.4), angle), ("back", -0.4, angle + .pi)] {
            let direction = SIMD3(cos(turn), sin(turn), 0), rotation = simd_quatf(angle: turn, axis: [0, 0, 1])
            let arm = ModelEntity(mesh: .generateBox(size: [length + 0.3, 0.22, 0.06]), materials: [paint])
            arm.name = "crank \(side)"
            plato.addChild(arm)
            arm.setPosition(axis + direction * length / 2 + [0, 0, depth], relativeTo: root)
            arm.setOrientation(rotation, relativeTo: root)

            // The pedal in front of its arm, its pin at its center
            let end = axis + direction * length + [0, 0, depth * 1.4]
            let pedal = spawn(.box(halfExtents: [1.05, 0.125, 0.15]), mesh: .generateBox(size: [2.1, 0.25, 0.3]),
                              named: "pedal \(side)", at: end)
            pedal.components[SDFPhysicsBodyComponent.self]!.massProperties = .init(mass: mass)
            pedal.components[SDFPhysicsBodyComponent.self]!.angularDamping = 2
            pedal.components[SDFPhysicsBodyComponent.self]!.filter = Self.pedalFilter
            pedal.components[SDFPhysicsBodyComponent.self]!.lockToPlane()
            let pin0 = plato.pins.set(named: "pedal \(side)", position: end, orientation: Self.axle, relativeTo: root)
            let pin1 = pedal.pins.set(named: "axle", orientation: Self.axle)
            joints.append(SDFPhysicsRevoluteJoint(pin0: pin0, pin1: pin1))
        }
    }

    /// Thin boards clipped to links, each on a hinge in front of the plates at the link's middle,
    /// which lets it swing `swing` either way of the link's outward normal: they stand on the top
    /// run and hang under the bottom one. A quarter of a link's mass, so they can't pull the chain
    /// off a tooth.
    mutating func hang(_ objects: [(name: String, link: Int, size: SIMD2<Float>)], swing: Float = 0.35) {
        let mass = 0.25 * linkMass, front: Float = 0.25
        for object in objects {
            let link = root.findEntity(named: "link \(object.link)")!
            let orientation = link.orientation(relativeTo: root)
            // From the pin, near the end at the link, to the board's center
            let reach = object.size.y / 2 - 0.05
            let center = link.position(relativeTo: root) + [0, 0, front] + orientation.act([0, reach, 0])
            let board = spawn(.box(halfExtents: [object.size.x / 2, object.size.y / 2, 0.05]),
                              mesh: .generateBox(size: [object.size.x, object.size.y, 0.1]),
                              named: object.name, at: center, orientation: orientation)
            board.components[SDFPhysicsBodyComponent.self]!.massProperties = .init(mass: mass)
            board.components[SDFPhysicsBodyComponent.self]!.angularDamping = 1.5
            board.components[SDFPhysicsBodyComponent.self]!.filter = Self.hungFilter
            board.components[SDFPhysicsBodyComponent.self]!.lockToPlane()
            let pin0 = link.pins.set(named: "clip", position: [0, 0, front], orientation: Self.axle)
            let pin1 = board.pins.set(named: "clip", position: [0, -reach, 0], orientation: Self.axle)
            joints.append(SDFPhysicsRevoluteJoint(pin0: pin0, pin1: pin1, angularLimit: -swing...swing))
        }
    }

    /// A static trigger of a board's size above the plato, where the chain carries the hung
    /// objects through one at a time
    mutating func addZone(above plato: Entity, height: Float, size: SIMD2<Float> = [0.8, 0.8]) {
        let shape = SDFShape.box(halfExtents: [size.x / 2, size.y / 2, 0.3])
        let zone = Entity()
        zone.name = CarrierZone.zoneName
        zone.position = plato.position(relativeTo: root) + [0.3, height, 0.25]
        var body = SDFPhysicsBodyComponent(shape: shape, mode: .static)
        body.isTrigger = true
        body.filter = Self.zoneFilter
        zone.components.set(body)
        root.addChild(zone)
    }
}

extension SDFPhysicsBodyComponent {
    /// Moves only in the x-y plane and turns only about z
    mutating func lockToPlane() {
        isTranslationLocked = (false, false, true)
        isRotationLocked = (true, true, false)
    }
}

/// Pedals a crank by hand: a servo on the crank's hinge motor that turns it toward where a
/// pointer holds it, with a capped speed and torque, so a stalled chain stops the hand rather
/// than jumping a tooth. Released, it brakes gently and the chain coasts to a stop.
public struct PedalDrive: Equatable, Sendable {
    /// Speed per radian behind the pointer
    public var gain: Float = 8
    public var maxSpeed: Float = 6
    /// Enough to start the chain briskly; a chain held still takes it without skipping a tooth
    public var maxTorque: Float = 15000
    /// The brake's torque once released, which stops the crank from 1 rad/s in about two seconds
    public var coastTorque: Float = 700
    /// The crank angle the pointer holds, or nil while released
    public private(set) var target: Float?
    /// The pointer's angle less the crank's when grabbed, so the crank doesn't jump to the pointer
    private var grip: Float = 0

    public init() {}

    /// An angle within [-π, π)
    public static func wrap(_ angle: Float) -> Float {
        angle - 2 * .pi * ((angle + .pi) / (2 * .pi)).rounded(.down)
    }

    /// The angle of a point about an axle along z, both projected onto the x-y plane
    public static func angle(of point: SIMD3<Float>, about axle: SIMD3<Float>) -> Float {
        atan2(point.y - axle.y, point.x - axle.x)
    }

    /// Where a ray meets the plane z = 0
    public static func pointOnPlane(origin: SIMD3<Float>, direction: SIMD3<Float>) -> SIMD3<Float>? {
        guard abs(direction.z) > 1e-6 else { return nil }
        let t = -origin.z / direction.z
        return t > 0 ? origin + direction * t : nil
    }

    public mutating func grab(at pointerAngle: Float, crankAngle: Float) {
        grip = Self.wrap(pointerAngle - crankAngle)
        target = crankAngle
    }

    public mutating func move(to pointerAngle: Float) {
        guard target != nil else { return }
        target = pointerAngle - grip
    }

    public mutating func release() {
        target = nil
    }

    /// The hinge's motor for the crank at an angle
    public func motor(crankAngle: Float) -> SDFPhysicsRevoluteJoint.Motor {
        guard let target else { return .init(targetVelocity: 0, maxTorque: coastTorque) }
        let speed = gain * Self.wrap(target - crankAngle)
        return .init(targetVelocity: min(max(speed, -maxSpeed), maxSpeed), maxTorque: maxTorque)
    }

    /// The crank's entity, whose hinges hold the pedals, and the index of its own hinge in the
    /// root's joints
    @MainActor
    public static func crank(in root: Entity) -> (entity: Entity, hinge: Int)? {
        guard let joints = root.components[SDFPhysicsJointsComponent.self]?.joints,
              let crank = joints.first(where: { $0.pin1.entity?.name.hasPrefix("pedal") == true })?.pin0.entity,
              let hinge = joints.firstIndex(where: { $0.pin1.entity == crank })
        else { return nil }
        return (crank, hinge)
    }
}

/// A trigger zone that hung objects the chain carries into become active in: the mechanic of the
/// Absolut scene, where a board clipped to a link comes alive at the top of the plato. Tracks the
/// zone's contact events while it lives, and marks the active object with a halo.
@MainActor
public final class CarrierZone {
    static let zoneName = "zone"
    public let zone: Entity
    /// The objects in the zone, in the order they came in
    public private(set) var inside: [Entity] = []
    /// Called when the active object changes, with the new one or nil
    public var onChange: ((Entity?) -> Void)?
    private var subscriptions: [AnyCancellable] = []

    /// The object the chain last brought in
    public var active: Entity? { inside.last }

    /// Nil when the root has no zone
    public init?(root: Entity, simulation: SDFSimulation) {
        guard let zone = root.findEntity(named: Self.zoneName) else { return nil }
        self.zone = zone
        subscriptions = [
            simulation.subscribe(to: SDFCollisionEvents.Began.self, on: zone) { [weak self] event in
                self?.update(event.entityA == zone ? event.entityB : event.entityA, isInside: true)
            },
            simulation.subscribe(to: SDFCollisionEvents.Ended.self, on: zone) { [weak self] event in
                self?.update(event.entityA == zone ? event.entityB : event.entityA, isInside: false)
            },
        ]
    }

    private func update(_ object: Entity, isInside: Bool) {
        let active = self.active
        inside.removeAll { $0 == object }
        if isInside { inside.append(object) }
        guard self.active != active else { return }
        active?.findEntity(named: "halo")?.removeFromParent()
        if let object = self.active {
            let size = object.visualBounds(relativeTo: object).extents
            let halo = ModelEntity(mesh: .generateBox(size: [size.x + 0.2, size.y + 0.2, 0.02]),
                                   materials: [UnlitMaterial(color: .init(red: 1, green: 0.8, blue: 0.1, alpha: 1))])
            halo.name = "halo"
            halo.position = [0, 0, -size.z / 2 - 0.02]
            object.addChild(halo)
        }
        onChange?(self.active)
    }
}
