import RealityKit
import RealitySDFPhysics

/// Shows the contacts of the parent's simulation as points with normals: green when tracked by the
/// temporal cache, red when newly found. Set it on a direct child of the simulation root.
public struct ContactMarkersComponent: Component {
    public init() {
        _ = registration
    }
}

private let registration: Void = {
    ContactMarkersComponent.registerComponent()
}()

@MainActor
public enum ContactMarkers {
    public static func makeEntity() -> Entity {
        _ = systemRegistration
        let entity = Entity()
        entity.name = "Contacts"
        entity.components.set(ContactMarkersComponent())
        return entity
    }

    private static let systemRegistration: Void = ContactMarkersSystem.registerSystem()
}

struct ContactMarkersSystem: System {
    static let dependencies: [SystemDependency] = [.after(SDFSimulationSystem.self)]
    private static let query = EntityQuery(where: .has(ContactMarkersComponent.self))
    private static let normalLength: Float = 0.3

    private let pointMesh: MeshResource
    private let lineMesh: MeshResource
    private let cachedMaterial = UnlitMaterial(color: .init(red: 0.2, green: 0.8, blue: 0.3, alpha: 1))
    private let newMaterial = UnlitMaterial(color: .init(red: 0.95, green: 0.25, blue: 0.2, alpha: 1))

    init(scene: RealityKit.Scene) {
        pointMesh = .generateSphere(radius: 0.035)
        lineMesh = .generateBox(size: [0.012, Self.normalLength, 0.012])
    }

    func update(context: SceneUpdateContext) {
        for markers in context.entities(matching: Self.query, updatingSystemWhen: .rendering) {
            guard let simulation = markers.parent?.components[SDFPhysicsSimulationComponent.self]?.simulation else { continue }
            let contacts = simulation.contacts().prefix(2048)
            while markers.children.count < contacts.count {
                markers.addChild(makeMarker())
            }
            for (i, marker) in markers.children.enumerated() {
                marker.isEnabled = i < contacts.count
                guard i < contacts.count else { continue }
                let contact = contacts[i]
                marker.transform = Transform(rotation: simd_quatf(from: [0, 1, 0], to: normalize(contact.normal)),
                                             translation: contact.position)
                let material = contact.isCached ? cachedMaterial : newMaterial
                for case let model as ModelEntity in marker.children {
                    model.model?.materials = [material]
                }
            }
        }
    }

    @MainActor
    private func makeMarker() -> Entity {
        let marker = Entity()
        marker.addChild(ModelEntity(mesh: pointMesh, materials: [cachedMaterial]))
        let line = ModelEntity(mesh: lineMesh, materials: [cachedMaterial])
        line.position = [0, Self.normalLength / 2, 0]
        marker.addChild(line)
        return marker
    }
}
