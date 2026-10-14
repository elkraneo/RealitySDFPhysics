// Port of the scene of Apple's sample "Simulating physics with collisions in your visionOS app":
// https://developer.apple.com/documentation/realitykit/simulating-physics-with-collisions-in-your-visionos-app
//
// Apple's 16 metallic spheres float without gravity in an invisible containment box, attract each
// other, and follow drags with a spring-like force. Differences from Apple's sample:
//
// - Bodies are `SDFBodyComponent`s in an `SDFSimulationComponent` root, in place of
//   `PhysicsBodyComponent` and `CollisionComponent`.
// - The root is scaled to a tenth, so bodies are modeled in decimeters. SDFCollisionKit's default
//   contact margin and tolerances suit objects about 1 unit across; the spheres are 3 to 15 cm.
//   Masses, forces and the sizes you see are the same as Apple's.
// - The volume is a fixed 1 m cube viewed through a camera, since a macOS window is not a volume.
//   Its faces are 50 cm thick rather than 1 mm, outside the volume, so that fast bodies don't pass
//   through them between two substeps.
// - `Shape` swaps the spheres for concave tori or spiky wheels (the paper's, Fig. 9), which
//   RealityKit's physics can't simulate as moving bodies. They are 8 to 15 cm across rather than 3
//   to 15, so that their thin parts stay well above the collision tolerance.

import RealityKit
import SDFCollisionKit
import simd
import SwiftUI

@MainActor
final class PhysicsBodiesScene {
    enum Shape: String, CaseIterable, Identifiable {
        case spheres = "Spheres"
        case tori = "Tori"
        case spikyWheels = "Spiky wheels"

        var id: Self { self }
    }

    /// The edge of the cubic volume, in meters
    static let volumeSize: Float = 1

    let content = Entity()
    /// The simulation root, scaled so that its units are decimeters
    let root = Entity()
    let camera = PerspectiveCamera()
    let simulation = SDFSimulation()
    private let bodies = Entity()

    init() {
        root.scale = SIMD3(repeating: 0.1)
        root.components.set(SDFSimulationComponent(simulation: simulation))
        root.addChild(makeContainment())
        root.addChild(bodies)
        content.addChild(root)
        content.addChild(makeOutline())

        let light = DirectionalLight()
        light.light.intensity = 3000
        light.look(at: .zero, from: [1, 2, 1.5], relativeTo: nil)
        content.addChild(light)

        camera.camera.fieldOfViewInDegrees = Self.fieldOfView
        camera.camera.fieldOfViewOrientation = .vertical
        camera.look(at: .zero, from: Self.eye, relativeTo: nil)
        content.addChild(camera)
    }

    /// Replaces the bodies with 16 new ones of a shape
    func reset(shape: Shape) {
        bodies.children.removeAll()
        for index in 0..<16 {
            let body = Self.makeBody(shape)
            body.name = "\(shape.rawValue)_\(index)"
            bodies.addChild(body)
            // Place the body in a semi-random place in the scene
            body.setPosition(SIMD3<Float>.random(in: -0.2...0.2), relativeTo: nil)
        }
    }

    // MARK: Bodies

    /// Port of Apple's `Entity.metallicSphere(_:)`, for any of the shapes
    private static func makeBody(_ kind: Shape) -> Entity {
        let radius: Float = kind == .spheres ? 0.15 * .random(in: 0.2...1.0) : 0.15 * .random(in: 0.55...1.0)
        // Sizes in the root's decimeters
        let r = radius * 10
        let shape: SDFShape
        let mesh: MeshResource
        switch kind {
        case .spheres:
            shape = .sphere(radius: r)
            mesh = .generateSphere(radius: r)
        case .tori:
            shape = .torus(majorRadius: 0.7 * r, minorRadius: 0.3 * r)
            mesh = try! .generate(from: shape)
        case .spikyWheels:
            // The paper's wheel (hub 0.35, spikes 1.4 long), scaled to the radius
            let k = r / 1.75
            shape = .spikyWheel(hubRadius: 0.35 * k, hubHalfWidth: 0.48 * k, spikeRadius: 0.08 * k, spikeLength: 1.4 * k,
                                spikes: 16)
            mesh = try! .generate(from: shape)
        }
        let entity = ModelEntity(mesh: mesh, materials: [metallicMaterial()])
        entity.orientation = simd_quatf(angle: .random(in: 0...(2 * .pi)), axis: normalize(.random(in: -1...1)))

        // Apple's density of 10,000 kg/m³ is 10 kg per cubic decimeter
        var body = SDFBodyComponent(shape: shape, massProperties: .init(density: 10))
        // Make each body float in the air by turning off gravity
        body.isAffectedByGravity = false
        entity.components.set(body)
        // Add an attraction force to the body
        entity.components.set(AttractionComponent())
        return entity
    }

    /// Apple's material: a random hue, metallic, with a clear coat
    private static func metallicMaterial() -> PhysicallyBasedMaterial {
        var material = PhysicallyBasedMaterial()
        let color = RealityKit.Material.Color(hue: .random(in: 0...1), saturation: .random(in: 0.5...1), brightness: 0.9,
                                              alpha: 1)
        material.baseColor = .init(tint: color)
        material.metallic = 1.0
        material.roughness = 0.5
        material.clearcoat = 1.0
        material.clearcoatRoughness = 0.1
        return material
    }

    // MARK: Containment

    /// Port of Apple's `ContainmentCollisionBox`: six static faces around the volume
    private func makeContainment() -> Entity {
        let box = Entity()
        let half = 10 * Self.volumeSize / 2, thickness: Float = 5
        for axis in 0..<3 {
            for side: Float in [-1, 1] {
                var extents = SIMD3<Float>(repeating: half + thickness)
                extents[axis] = thickness / 2
                var position = SIMD3<Float>.zero
                position[axis] = side * (half + thickness / 2)
                let face = Entity()
                face.position = position
                face.components.set(SDFBodyComponent(shape: .box(halfExtents: extents), mode: .static))
                box.addChild(face)
            }
        }
        return box
    }

    /// Faint edges of the volume, which Apple's sample shows by its window
    private func makeOutline() -> Entity {
        let outline = Entity()
        let half = Self.volumeSize / 2, width: Float = 0.002
        let material = UnlitMaterial(color: .init(white: 0.6, alpha: 1))
        for axis in 0..<3 {
            for a: Float in [-1, 1] {
                for b: Float in [-1, 1] {
                    var size = SIMD3<Float>(repeating: width)
                    size[axis] = Self.volumeSize
                    var position = SIMD3<Float>(repeating: 0)
                    position[(axis + 1) % 3] = a * half
                    position[(axis + 2) % 3] = b * half
                    let edge = ModelEntity(mesh: .generateBox(size: size), materials: [material])
                    edge.position = position
                    outline.addChild(edge)
                }
            }
        }
        return outline
    }

    // MARK: Dragging

    static let fieldOfView: Float = 50
    static let eye: SIMD3<Float> = [0, 0.45, 1.8]

    /// A world space ray through a point of the view
    func ray(through point: CGPoint, in size: CGSize) -> (origin: SIMD3<Float>, direction: SIMD3<Float>) {
        let forward = normalize(-Self.eye)
        let right = normalize(cross(forward, [0, 1, 0]))
        let up = cross(right, forward)
        let t = tan(Self.fieldOfView * .pi / 360)
        let aspect = Float(size.width / max(size.height, 1))
        let x = Float(2 * point.x / max(size.width, 1) - 1)
        let y = Float(1 - 2 * point.y / max(size.height, 1))
        return (Self.eye, normalize(forward + right * (x * t * aspect) + up * (y * t)))
    }

    /// The body under a point of the view, and its distance from the camera
    func pick(at point: CGPoint, in size: CGSize) -> (entity: Entity, distance: Float)? {
        let ray = ray(through: point, in: size)
        // The simulation's raycast works in the root's frame
        let origin = root.convert(position: ray.origin, from: nil)
        let direction = root.convert(direction: ray.direction, from: nil)
        guard let hit = simulation.raycast(origin: origin, direction: direction, length: 1000, query: .nearest).first,
              hit.entity.components.has(AttractionComponent.self) else { return nil }
        return (hit.entity, simd_distance(root.convert(position: hit.position, to: nil), ray.origin))
    }

    /// Port of Apple's `ForceDragGesture`: a spring-like force towards the drag target
    func pull(_ entity: Entity, towards target: SIMD3<Float>) {
        let direction = target - entity.position(relativeTo: nil)
        var strength = length(direction)
        guard strength > 1e-6 else { return }
        if strength < 1 {
            strength *= strength
        }
        let forceFactor: Float = 3000
        entity.sdfPhysics.addForce(forceFactor * strength * normalize(direction), relativeTo: nil)
    }
}
