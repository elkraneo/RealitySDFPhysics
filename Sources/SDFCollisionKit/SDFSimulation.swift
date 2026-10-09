import RealityKit
import simd

#if canImport(AppKit)
import AppKit
typealias PlatformColor = NSColor
#else
import UIKit
typealias PlatformColor = UIColor
#endif

/// Drives an `SDFWorld` from RealityKit: steps it at a fixed rate from `SDFSimulationSystem` and
/// mirrors its bodies as entities under `root`.
///
/// RealityKit's own physics is not involved. The body entities have no `PhysicsBodyComponent` or
/// `CollisionComponent`; collision detection and response are entirely the SDF engine's.
@MainActor
public final class SDFSimulation {
    public let world: SDFWorld

    /// Parent of the body and debug entities. Add it to a RealityKit scene to run the simulation.
    public let root = Entity()

    public var isPaused = false

    /// Shows the active contact points and their normals
    public var showsContacts = false {
        didSet {
            if showsContacts { updateContacts() }
            contactRoot.isEnabled = showsContacts
        }
    }

    /// Simulation steps taken since the scene was loaded
    public private(set) var stepCount = 0

    /// Upper bound on the steps taken in one frame, so a slow frame does not spiral
    public var maximumStepsPerFrame = 4

    private let bodyRoot = Entity()
    private let contactRoot = Entity()
    private var bodyEntities: [Entity] = []
    private var meshes: [Int: MeshResource] = [:]
    private var contactEntities: [Entity] = []
    private var accumulator: TimeInterval = 0
    private var builtGeneration = -1

    public init(world: SDFWorld = SDFWorld()) {
        Self.registerOnce()
        self.world = world
        root.name = "SDFSimulation"
        root.addChild(bodyRoot)
        root.addChild(contactRoot)
        contactRoot.isEnabled = false
        root.components.set(SDFSimulationComponent(simulation: self))
    }

    /// Replaces the world with a built-in scene and rebuilds the body entities
    public func loadScene(_ index: Int) {
        world.loadScene(index)
        synchronize()
    }

    public func spawn(_ shape: String, at position: SIMD3<Float>) {
        world.spawn(shape, at: position)
        synchronize()
    }

    /// Takes one step regardless of `isPaused`
    public func step() {
        world.step()
        stepCount += 1
        synchronize()
    }

    /// Advances the simulation by real time in fixed steps. Called by `SDFSimulationSystem`.
    public func advance(by deltaTime: TimeInterval) {
        guard !isPaused else { return }
        let timeStep = TimeInterval(world.parameters.timeStep)
        accumulator += min(deltaTime, 0.25)
        var steps = 0
        while accumulator >= timeStep && steps < max(maximumStepsPerFrame, 1) {
            world.step()
            accumulator -= timeStep
            steps += 1
        }
        // Drop the whole steps a slow frame could not catch up on, keep the fraction
        accumulator = accumulator.truncatingRemainder(dividingBy: timeStep)
        stepCount += steps
        if steps > 0 { synchronize() }
    }

    /// Copies the body poses (and contacts, when shown) from the world to the entities. Rebuilds
    /// the entities when the world has loaded another scene.
    public func synchronize() {
        if builtGeneration != world.sceneGeneration {
            for entity in bodyEntities { entity.removeFromParent() }
            bodyEntities.removeAll()
            meshes.removeAll()
            accumulator = 0
            stepCount = 0
            builtGeneration = world.sceneGeneration
        }
        let bodies = world.bodies()
        while bodyEntities.count < bodies.count {
            let entity = makeEntity(for: bodies[bodyEntities.count])
            bodyRoot.addChild(entity)
            bodyEntities.append(entity)
        }
        for (entity, body) in zip(bodyEntities, bodies) {
            entity.transform = Transform(scale: SIMD3(repeating: body.scale), rotation: body.orientation, translation: body.origin)
        }
        if showsContacts { updateContacts() }
    }

    // MARK: Entities

    private func makeEntity(for body: SDFWorld.Body) -> Entity {
        // The ground is a body too, which the demo draws as a separate floor
        guard body.isVisible, let mesh = mesh(forShape: body.shape) else { return Entity() }
        var material = PhysicallyBasedMaterial()
        material.baseColor = .init(tint: PlatformColor(red: CGFloat(body.color.x), green: CGFloat(body.color.y),
                                                       blue: CGFloat(body.color.z), alpha: 1))
        material.roughness = 0.55
        return ModelEntity(mesh: mesh, materials: [material])
    }

    private func mesh(forShape shape: Int) -> MeshResource? {
        if let mesh = meshes[shape] { return mesh }
        let source = world.mesh(forShape: shape)
        guard !source.indices.isEmpty else { return nil }
        var descriptor = MeshDescriptor(name: "SDFShape\(shape)")
        descriptor.positions = MeshBuffers.Positions(source.positions)
        if let normals = source.normals { descriptor.normals = MeshBuffers.Normals(normals) }
        descriptor.primitives = .triangles(source.indices)
        guard let mesh = try? MeshResource.generate(from: [descriptor]) else { return nil }
        meshes[shape] = mesh
        return mesh
    }

    // MARK: Contacts

    private static let contactRadius: Float = 0.035
    private static let normalLength: Float = 0.3
    private lazy var pointMesh = MeshResource.generateSphere(radius: Self.contactRadius)
    private lazy var lineMesh = MeshResource.generateBox(size: [0.012, Self.normalLength, 0.012])
    private lazy var cachedMaterial = UnlitMaterial(color: PlatformColor(red: 0.2, green: 0.8, blue: 0.3, alpha: 1))
    private lazy var newMaterial = UnlitMaterial(color: PlatformColor(red: 0.95, green: 0.25, blue: 0.2, alpha: 1))

    private func updateContacts() {
        let contacts = world.contacts(limit: 2048)
        while contactEntities.count < contacts.count {
            let marker = Entity()
            marker.addChild(ModelEntity(mesh: pointMesh, materials: [cachedMaterial]))
            let line = ModelEntity(mesh: lineMesh, materials: [cachedMaterial])
            line.position = [0, Self.normalLength / 2, 0]
            marker.addChild(line)
            contactRoot.addChild(marker)
            contactEntities.append(marker)
        }
        for (i, marker) in contactEntities.enumerated() {
            guard i < contacts.count else {
                marker.isEnabled = false
                continue
            }
            let contact = contacts[i]
            marker.isEnabled = true
            marker.transform = Transform(rotation: simd_quatf(from: [0, 1, 0], to: normalize(contact.normal)),
                                         translation: contact.position)
            let material: any Material = contact.isCached ? cachedMaterial : newMaterial
            for case let model as ModelEntity in marker.children {
                model.model?.materials = [material]
            }
        }
    }

    // MARK: Registration

    private static var registered = false

    private static func registerOnce() {
        guard !registered else { return }
        SDFSimulationComponent.registerComponent()
        SDFSimulationSystem.registerSystem()
        registered = true
    }
}

/// Marks the root entity of an `SDFSimulation`, which `SDFSimulationSystem` advances every frame
struct SDFSimulationComponent: Component {
    /// Weak, since the simulation owns the entity this component is set on
    weak var simulation: SDFSimulation?
}

/// Steps every `SDFSimulation` in the scene with the frame time
struct SDFSimulationSystem: System {
    private static let query = EntityQuery(where: .has(SDFSimulationComponent.self))

    init(scene: RealityKit.Scene) {}

    mutating func update(context: SceneUpdateContext) {
        for entity in context.entities(matching: Self.query, updatingSystemWhen: .rendering) {
            entity.components[SDFSimulationComponent.self]?.simulation?.advance(by: context.deltaTime)
        }
    }
}
