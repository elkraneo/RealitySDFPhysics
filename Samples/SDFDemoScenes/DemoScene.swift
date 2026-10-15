import CoreGraphics
import RealityKit
import SDFCollisionKit
import simd

/// A scene of the upstream demo, rebuilt on the public API. Positions, sizes and scales are those
/// of upstream scenes.cpp; the random placements differ, since the generators differ.
public struct DemoScene: Identifiable, Sendable {
    public let id: Int
    public let name: String
    public let description: String
    /// Orbit camera target and distance
    public let target: SIMD3<Float>
    public let distance: Float
    /// The pull of gravity, in m/s², which `load(into:)` and `configure(_:)` set on the simulation
    public var gravity: Float = 9.81
    fileprivate let build: @MainActor (inout DemoBuilder) async throws -> Void

    /// Replaces the bodies under `root` with the scene's. Every scene has a static ground whose top
    /// face is at y = 0, which is not drawn; add a floor separately.
    @MainActor
    public func load(into root: Entity) async throws {
        var builder = DemoBuilder(root: Entity())
        try await builder.spawn("Ground", at: [0, -1, 0], mode: .static, isVisible: false)
        try await build(&builder)
        // Swap in the finished scene at once, so a half built scene never simulates
        for child in Array(root.children) where child.components.has(SDFBodyComponent.self) {
            child.removeFromParent()
        }
        for child in Array(builder.root.children) {
            root.addChild(child)
        }
        root.components.set(SDFJointsComponent(joints: builder.joints))
        if var settings = root.components[SDFSimulationComponent.self] {
            configure(&settings)
            root.components.set(settings)
        }
    }

    /// Sets the scene's own simulation settings, for a simulation added after `load(into:)`
    public func configure(_ settings: inout SDFSimulationComponent) {
        settings.gravity = [0, -gravity / settings.metersPerUnit, 0]
    }
}

/// Creates the bodies of a scene
@MainActor
public struct DemoBuilder {
    let root: Entity
    var random: SeededRandom
    /// The joints of the scene, set on the simulation entity once the bodies are in place
    public var joints: [SDFRevoluteJoint] = []

    /// Bodies are added as children of `root`
    public init(root: Entity, seed: UInt64 = 0x853c49e6748fea9b) {
        self.root = root
        random = SeededRandom(seed: seed)
    }

    /// Drops a shape with a random orientation from above a point, as the upstream Space key
    @discardableResult
    public mutating func drop(_ shapeName: String, above target: SIMD3<Float>) async throws -> Entity {
        let position = target + SIMD3(random.uniform(-0.5, 0.5), 4, random.uniform(-0.5, 0.5))
        return try await spawn(shapeName, at: position, orientation: random.orientation())
    }

    /// Adds a body of a shape with a mesh of its own, such as a generated gear
    @discardableResult
    public mutating func spawn(_ shape: SDFShape, mesh: MeshResource, named name: String, at position: SIMD3<Float>,
                               orientation: simd_quatf = simd_quatf(angle: 0, axis: [0, 1, 0]), scale: Float = 1,
                               mode: PhysicsBodyMode = .dynamic) -> Entity {
        let color = mode == .dynamic ? random.pastel() : SIMD3(0.72, 0.72, 0.76)
        var material = PhysicallyBasedMaterial()
        material.baseColor = .init(tint: .init(red: CGFloat(color.x), green: CGFloat(color.y), blue: CGFloat(color.z), alpha: 1))
        material.roughness = 0.55
        let entity = ModelEntity(mesh: mesh, materials: [material])
        entity.name = name
        entity.transform = Transform(scale: SIMD3(repeating: scale), rotation: orientation, translation: position)
        entity.components.set(SDFBodyComponent(shape: shape, mode: mode))
        root.addChild(entity)
        return entity
    }

    /// Adds a body of a named shape with a pastel color, or gray when static
    @discardableResult
    public mutating func spawn(_ shapeName: String, at position: SIMD3<Float>, orientation: simd_quatf = simd_quatf(),
                               scale: Float = 1, mode: PhysicsBodyMode = .dynamic,
                               isVisible: Bool = true) async throws -> Entity {
        let shape = try await DemoShapes.shared.shape(named: shapeName)
        let entity: Entity
        if isVisible {
            let color = mode == .dynamic ? random.pastel() : SIMD3(0.72, 0.72, 0.76)
            var material = PhysicallyBasedMaterial()
            material.baseColor = .init(tint: .init(red: CGFloat(color.x), green: CGFloat(color.y), blue: CGFloat(color.z), alpha: 1))
            material.roughness = 0.55
            entity = ModelEntity(mesh: try await DemoShapes.shared.mesh(named: shapeName), materials: [material])
        } else {
            entity = Entity()
        }
        entity.name = shapeName
        entity.transform = Transform(scale: SIMD3(repeating: scale), rotation: orientation, translation: position)
        entity.components.set(SDFBodyComponent(shape: shape, mode: mode))
        root.addChild(entity)
        return entity
    }
}

extension DemoScene {
    public static let all: [DemoScene] = [
        DemoScene(id: 0, name: "Box Stack", description: "Pyramid of analytical boxes (Fig. 5, left)",
                  target: [0, 2.5, 0], distance: 13) { b in
            // They start a small gap apart, within the contact margin but short of the allowed
            // penetration, so the stack is at rest from the start
            let levels = 7
            let parameters = SDFCollisionDetection()
            let gap = parameters.contactMargin - 0.5 * parameters.allowedPenetration
            for level in 0..<levels {
                for i in 0..<(levels - level) {
                    try await b.spawn("Box", at: [(Float(i) - 0.5 * Float(levels - level - 1)) * 1.05,
                                                  0.5 + gap + Float(level) * (1 + gap), 0])
                }
            }
        },
        DemoScene(id: 1, name: "Gear Stack", description: "Tower of voxel gears (Fig. 5, right)",
                  target: [0, 2, 0], distance: 9) { b in
            for i in 0..<10 {
                let position = SIMD3<Float>(b.random.uniform(-0.05, 0.05), 0.2 + Float(i) * 0.45, b.random.uniform(-0.05, 0.05))
                try await b.spawn("Gear", at: position, orientation: simd_quatf(angle: b.random.uniform(0, 2 * .pi), axis: [0, 1, 0]),
                                  scale: 1.5)
            }
        },
        fallingBoxes(id: 2, name: "Falling Boxes", description: "Undistorted pile of boxes (Fig. 8)", shape: "Plank"),
        fallingBoxes(id: 3, name: "Falling Boxes: Bend", description: "BEND distorted boxes (Sec. 4.5, Fig. 8)", shape: "Bent Plank"),
        fallingBoxes(id: 4, name: "Falling Boxes: Twist", description: "TWIST distorted boxes (Sec. 4.5, Fig. 8)", shape: "Twisted Plank"),
        fallingBoxes(id: 5, name: "Falling Boxes: Displace", description: "DISPLACE distorted boxes (Sec. 4.5, Fig. 8)",
                     shape: "Displaced Plank"),
        DemoScene(id: 6, name: "Pile of Chairs", description: "Voxel chairs with thin, interlocking legs (Fig. 9)",
                  target: [0, 1, 0], distance: 7) { b in
            try await b.grid(layers: 3, columns: 3, rows: 3) { b, layer, x, z in
                try await b.spawn("Chair", at: [(x - 1) * 1.4, 0.8 + layer * 1.6, (z - 1) * 1.3],
                                  orientation: b.random.orientation(), scale: 1.15)
            }
        },
        DemoScene(id: 7, name: "Spiky Wheel", description: "Analytical composite rolling on its spikes (Fig. 9)",
                  target: [1, 1.2, 0], distance: 13) { b in
            let wheel = try await b.spawn("Spiky Wheel", at: [-6, 2, 0], orientation: simd_quatf(angle: .pi / 2, axis: [1, 0, 0]),
                                          scale: 1.2)
            // A heavy wheel, four times the mass of its volume of water
            let volume = try await DemoShapes.shared.shape(named: "Spiky Wheel").volume
            wheel.components[SDFBodyComponent.self]?.massProperties = .init(mass: 4 * 1000 * volume * pow(1.2, 3))
            wheel.components.set(SDFMotionComponent(linearVelocity: [6, 0, 0], angularVelocity: [0, 0, -4]))
            for row in 0..<4 {
                for i in 0...row {
                    try await b.spawn("Capsule", at: [3 + Float(row) * 0.8, 0.7, (Float(i) - 0.5 * Float(row)) * 0.9])
                }
            }
        },
        DemoScene(id: 8, name: "Cows on Spikes", description: "Voxel cows on an analytical bed of spikes (Fig. 1)",
                  target: [0, 1.2, 0], distance: 9) { b in
            try await b.spawn("Spike Plate", at: [0, 0.08 * 4, 0], scale: 4, mode: .static)
            try await b.grid(layers: 2, columns: 3, rows: 3) { b, layer, x, z in
                try await b.spawn("Cow", at: [(x - 1) * 1.3, 2.2 + layer * 1.2, (z - 1) * 1.3],
                                  orientation: b.random.orientation(), scale: 1.3)
            }
        },
        DemoScene(id: 9, name: "Bunny Pile", description: "Pile of voxel bunnies (Fig. 1)",
                  target: [0, 0.8, 0], distance: 7.5) { b in
            try await b.grid(layers: 3, columns: 3, rows: 3) { b, layer, x, z in
                try await b.spawn("Bunny", at: [(x - 1) * 1.1 + 0.2 * layer, 0.8 + layer * 1.1, (z - 1) * 1.1],
                                  orientation: b.random.orientation())
            }
        },
        DemoScene(id: 10, name: "Primitives", description: "All analytical primitives",
                  target: [0, 1, 0], distance: 10) { b in
            let names = ["Box", "Sphere", "Capsule", "Cylinder", "Cone", "Torus"]
            var n = 0
            try await b.grid(layers: 3, columns: 4, rows: 4) { b, layer, x, z in
                try await b.spawn(names[n % names.count], at: [(x - 1.5) * 1.3, 1 + layer * 1.4, (z - 1.5) * 1.3],
                                  orientation: b.random.orientation())
                n += 1
            }
        },
        DemoScene(id: 11, name: "Gears", description: "Involute gears of 20 and 10 teeth on hinges, the big one driven by a motor: the small one turns by contact, twice as fast",
                  target: [0.79, 0.5, 0], distance: 6) { b in
            let module: Float = 0.1, thickness: Float = 0.4
            let axle = simd_quatf(angle: .pi / 2, axis: [0, 0, 1]) // The pin's x axis along y
            let base = try await b.spawn("Box", at: [0.79, -0.5, 0], scale: 0.4, mode: .static, isVisible: false)
            // Pitch radii 1 and 0.5, with 0.08 of backlash: four times the contact skin, which
            // keeps the flanks a skin apart. Oiled steel: the default friction wedges the teeth.
            for (name, teeth, x, turn) in [("driver", 20, Float(0), Float(0)), ("driven", 10, 1.58, Float.pi / 10)] {
                let shape = try GearMesh.shape(teeth: teeth, module: module, thickness: thickness)
                let gear = b.spawn(shape, mesh: try MeshResource.generate(from: shape), named: name, at: [x, 0.5, 0],
                                   orientation: simd_quatf(angle: turn, axis: [0, 1, 0]))
                gear.components[SDFBodyComponent.self]!.material = SDFMaterial(staticFriction: 0.1, dynamicFriction: 0.1, restitution: 0)
                let pin0 = base.pins.set(named: name, position: gear.position, orientation: axle, relativeTo: b.root)
                let pin1 = gear.pins.set(named: "hub", orientation: gear.orientation.inverse * axle)
                var joint = SDFRevoluteJoint(pin0: pin0, pin1: pin1)
                if name == "driver" { joint.motor = SDFRevoluteJoint.Motor(targetVelocity: 1, maxTorque: 500) }
                b.joints.append(joint)
            }
        },
        DemoScene(id: 12, name: "Chain", description: "A roller chain of 48 links around sprockets of 20 and 10 teeth, the big one driven by a motor: the small one turns by the chain alone",
                  target: [4.1, 3, 0], distance: 16) { b in
            try await b.buildChain()
        },
        // The ids are the indices: a scene added on another branch may take this one's place,
        // so this one goes last, whatever its number
        DemoScene(id: 13, name: "Absolut", description: "The Absolut Barcelona chain of 2008: 48 links around a piñon of 18 teeth and a plato of 29, the plato pedaled clockwise by a motor, as a bicycle riding to the right: it pulls the top run and the bottom one sags. Every body is locked in the plane, under a lighter gravity",
                  target: [2.8, 3.5, 0], distance: 16, gravity: 3) { b in
            try await b.buildChain(teethA: 18, teethB: 29, height: 4, motored: .b, motor: .init(targetVelocity: -1, maxTorque: 15000),
                                   linkDamping: (linear: 0.1, angular: 0.5), isPlanar: true)
        },
    ]

    /// Fig. 8: a pile of falling boxes, optionally distorted
    private static func fallingBoxes(id: Int, name: String, description: String, shape: String) -> DemoScene {
        DemoScene(id: id, name: name, description: description, target: [0, 1.2, 0], distance: 11) { b in
            try await b.grid(layers: 2, columns: 3, rows: 2) { b, layer, x, z in
                let position = SIMD3<Float>((x - 1) * 1.85 + b.random.uniform(-0.1, 0.1), 2.2 + layer * 3 + 0.35 * (x + z),
                                            (z - 0.5) * 1.55 + b.random.uniform(-0.1, 0.1))
                try await b.spawn(shape, at: position, orientation: b.random.orientation(), scale: 0.9)
            }
        }
    }
}

extension DemoBuilder {
    /// Calls `body` for every cell of a layers × rows × columns grid, layer by layer
    mutating func grid(layers: Int, columns: Int, rows: Int,
                       _ body: (inout DemoBuilder, Float, Float, Float) async throws -> Void) async throws {
        for layer in 0..<layers {
            for z in 0..<rows {
                for x in 0..<columns {
                    try await body(&self, Float(layer), Float(x), Float(z))
                }
            }
        }
    }
}

/// SplitMix64, so that scenes are the same on every load
struct SeededRandom {
    private var state: UInt64

    init(seed: UInt64) {
        state = seed
    }

    mutating func next() -> UInt64 {
        state &+= 0x9E3779B97F4A7C15
        var z = state
        z = (z ^ (z >> 30)) &* 0xBF58476D1CE4E5B9
        z = (z ^ (z >> 27)) &* 0x94D049BB133111EB
        return z ^ (z >> 31)
    }

    mutating func uniform(_ a: Float = 0, _ b: Float = 1) -> Float {
        a + (b - a) * Float(next() >> 40) / Float(1 << 24)
    }

    /// Uniformly distributed rotation
    mutating func orientation() -> simd_quatf {
        let u = uniform(), v = uniform(0, 2 * .pi), w = uniform(0, 2 * .pi)
        let a = sqrt(1 - u), b = sqrt(u)
        return simd_quatf(ix: a * sin(v), iy: a * cos(v), iz: b * sin(w), r: b * cos(w))
    }

    /// Muted pastel colors similar to the renderings in the paper
    mutating func pastel() -> SIMD3<Float> {
        let h = uniform() * 6, s = uniform(0.25, 0.45), v = uniform(0.75, 0.9)
        let f = h - h.rounded(.down), p = v * (1 - s), q = v * (1 - s * f), t = v * (1 - s * (1 - f))
        switch Int(h) % 6 {
        case 0: return [v, t, p]
        case 1: return [q, v, p]
        case 2: return [p, v, t]
        case 3: return [p, q, v]
        case 4: return [t, p, v]
        default: return [v, p, q]
        }
    }
}

extension DemoBuilder {
    /// A sprocket of the chain scene: A at the origin, B along +x
    public enum ChainSprocket: Sendable {
        case a, b
    }

    /// The chain scene: links pinned to each other around two sprockets on hinges, in the x-y
    /// plane at `height`, with the friction of oiled steel. The `motored` sprocket's hinge drives
    /// it; `linkDamping` replaces the links' default damping; planar bodies move only in the x-y
    /// plane and turn only about z.
    @MainActor
    public mutating func buildChain(chain: ChainMesh = ChainMesh(), links: Int = 48, teethA: Int = 20, teethB: Int = 10,
                                    height: Float = 3, motored: ChainSprocket = .a,
                                    motor: SDFRevoluteJoint.Motor = .init(targetVelocity: 1, maxTorque: 5000),
                                    linkDamping: (linear: Float, angular: Float)? = nil, isPlanar: Bool = false) async throws {
        let layout = ChainLayout(chain: chain, links: links, teethA: teethA, teethB: teethB)
        let origin = SIMD3<Float>(0, height, 0)
        let axle = simd_quatf(angle: -.pi / 2, axis: [0, 1, 0]) // The pin's x axis along z
        let steel = SDFMaterial(staticFriction: 0.1, dynamicFriction: 0.1, restitution: 0)
        let base = try await spawn("Box", at: [layout.centerDistance / 2, -1, 0], scale: 0.5, mode: .static, isVisible: false)

        let phases = layout.phases
        func makePlanar(_ entity: Entity) {
            guard isPlanar else { return }
            entity.components[SDFBodyComponent.self]!.isTranslationLocked = (false, false, true)
            entity.components[SDFBodyComponent.self]!.isRotationLocked = (true, true, false)
        }
        for (name, teeth, x, phase, sprocketID) in [("sprocket A", teethA, Float(0), phases.a, ChainSprocket.a),
                                                     ("sprocket B", teethB, layout.centerDistance, phases.b, .b)] {
            let shape = try chain.sprocketShape(teeth: teeth)
            let sprocket = spawn(shape, mesh: try MeshResource.generate(from: shape), named: name, at: origin + [x, 0, 0],
                                 orientation: simd_quatf(angle: phase, axis: [0, 0, 1]))
            sprocket.components[SDFBodyComponent.self]!.material = steel
            sprocket.components.set(SDFMotionComponent())
            makePlanar(sprocket)
            let pin0 = base.pins.set(named: name, position: sprocket.position, orientation: axle, relativeTo: root)
            let pin1 = sprocket.pins.set(named: "hub", orientation: sprocket.orientation.inverse * axle)
            var joint = SDFRevoluteJoint(pin0: pin0, pin1: pin1)
            if sprocketID == motored { joint.motor = motor }
            joints.append(joint)
        }

        let linkShape = try chain.linkShape()
        let linkMesh = try MeshResource.generate(from: linkShape)
        let pins = layout.pins
        var linkEntities: [Entity] = []
        for k in 0..<links {
            let from = pins[k], to = pins[(k + 1) % links]
            let middle = (from + to) / 2, direction = to - from
            let link = spawn(linkShape, mesh: linkMesh, named: "link \(k)", at: origin + [middle.x, middle.y, 0],
                             orientation: simd_quatf(angle: atan2(direction.y, direction.x), axis: [0, 0, 1]))
            link.components[SDFBodyComponent.self]!.material = steel
            if let linkDamping {
                link.components[SDFBodyComponent.self]!.linearDamping = linkDamping.linear
                link.components[SDFBodyComponent.self]!.angularDamping = linkDamping.angular
            }
            makePlanar(link)
            link.pins.set(named: "left", position: [-chain.pitch / 2, 0, 0], orientation: axle)
            link.pins.set(named: "right", position: [chain.pitch / 2, 0, 0], orientation: axle)
            linkEntities.append(link)
        }
        for k in 0..<links {
            joints.append(SDFRevoluteJoint(pin0: linkEntities[k].pins["right"]!, pin1: linkEntities[(k + 1) % links].pins["left"]!))
        }
    }
}
