import CoreMedia
import RealityKit
import RealitySDFPhysics
import simd

/// The benchmark scene: layers of concave tori dropped into a box 2 m across, with walls 5 m
/// high so that 400 stay in. The same pile is built for RealitySDFPhysics and for RealityKit's
/// physics, where a torus is a compound of capsules.
public enum TorusPile {
    public static let major: Float = 0.15, minor: Float = 0.04, segments = 12
    /// The body counts of the standard table
    public static let counts = [25, 50, 100, 200, 400]

    /// Where the tori start: layers of a 5 × 5 grid above the box, tilted
    public static func placements(_ count: Int) -> [(position: SIMD3<Float>, orientation: simd_quatf)] {
        (0..<count).map { i in
            let layer = i / 25, cell = i % 25
            let x = Float(cell % 5) * 0.36 - 0.72, z = Float(cell / 5) * 0.36 - 0.72
            let angle = Float(i % 7) * 0.4
            return ([x, 0.3 + Float(layer) * 0.25, z], simd_quatf(angle: angle, axis: normalize([1, 0, Float(i % 3) - 1])))
        }
    }

    /// Floor and four walls: centers and half extents
    public static let walls: [(center: SIMD3<Float>, halfExtents: SIMD3<Float>)] = [
        ([0, -0.5, 0], [2, 0.5, 2]),
        ([-1.05, 2.5, 0], [0.05, 2.5, 1.1]), ([1.05, 2.5, 0], [0.05, 2.5, 1.1]),
        ([0, 2.5, -1.05], [1.1, 2.5, 0.05]), ([0, 2.5, 1.05], [1.1, 2.5, 0.05]),
    ]

    /// The pile as SDF bodies under a root with the settings. With `motion`, every torus gets a
    /// motion component, so its velocities can be read.
    @MainActor
    public static func makeSDF(count: Int, settings: SDFPhysicsSimulationComponent, motion: Bool = false) -> (root: Entity, tori: [Entity]) {
        let root = Entity()
        root.components.set(settings)
        for wall in walls {
            let entity = Entity()
            entity.position = wall.center
            entity.components.set(SDFPhysicsBodyComponent(shape: .box(halfExtents: wall.halfExtents), mode: .static))
            root.addChild(entity)
        }
        let torus = SDFShape.torus(majorRadius: major, minorRadius: minor)
        var tori: [Entity] = []
        for placement in placements(count) {
            let entity = Entity()
            entity.transform = Transform(rotation: placement.orientation, translation: placement.position)
            entity.components.set(SDFPhysicsBodyComponent(shape: torus))
            if motion { entity.components.set(SDFPhysicsMotionComponent()) }
            root.addChild(entity)
            tori.append(entity)
        }
        return (root, tori)
    }

    /// The pile as RealityKit physics bodies, each torus a ring of 12 capsules. The simulation
    /// follows the clock when one is given, as a benchmark steps it.
    @MainActor
    public static func makeRealityKit(count: Int, clock: CMTimebase? = nil) -> (root: Entity, tori: [Entity]) {
        let root = Entity()
        var physics = PhysicsSimulationComponent()
        if let clock { physics.clock = clock }
        root.components.set(physics)
        for wall in walls {
            let entity = Entity()
            entity.position = wall.center
            let shape = ShapeResource.generateBox(size: 2 * wall.halfExtents)
            entity.components.set(CollisionComponent(shapes: [shape]))
            entity.components.set(PhysicsBodyComponent(shapes: [shape], mass: 1, mode: .static))
            root.addChild(entity)
        }
        let chord = 2 * major * sin(.pi / Float(segments))
        let compound = (0..<segments).map { k in
            let angle = (Float(k) + 0.5) * 2 * .pi / Float(segments)
            return ShapeResource.generateCapsule(height: chord + 2 * minor, radius: minor)
                .offsetBy(rotation: simd_quatf(from: [0, 1, 0], to: [-sin(angle), 0, cos(angle)]),
                          translation: [major * cos(angle), 0, major * sin(angle)])
        }
        var tori: [Entity] = []
        for placement in placements(count) {
            let entity = Entity()
            entity.transform = Transform(rotation: placement.orientation, translation: placement.position)
            entity.components.set(CollisionComponent(shapes: compound))
            entity.components.set(PhysicsBodyComponent(shapes: compound, density: 1000))
            root.addChild(entity)
            tori.append(entity)
        }
        return (root, tori)
    }

    /// The mean height of the tori and how many left the box, to check that the pile fell and settled
    @MainActor
    public static func check(_ tori: [Entity]) -> String {
        let positions = tori.map { $0.position(relativeTo: nil) }
        let mean = positions.map(\.y).reduce(0, +) / Float(max(positions.count, 1))
        let out = positions.filter { abs($0.x) > 1 || abs($0.z) > 1 || $0.y < -0.1 }.count
        return String(format: "mean y %.2f m, %d out", mean, out)
    }
}
