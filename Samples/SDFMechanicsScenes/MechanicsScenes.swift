import RealityKit
import SDFCollisionKit
import SDFDemoScenes
import simd

/// Mechanisms built on the joints: gears, chains and a planetary set, beyond the paper's demo
public enum MechanicsScenes {
    /// The ids are the indices
    public static let all: [DemoScene] = [
        DemoScene(id: 0, name: "Gears", description: "Involute gears of 20 and 10 teeth on hinges, the big one driven by a motor: the small one turns by contact, twice as fast",
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
        DemoScene(id: 1, name: "Chain", description: "A roller chain of 48 links around sprockets of 20 and 10 teeth, the big one driven by a motor: the small one turns by the chain alone",
                  target: [4.1, 3, 0], distance: 16) { b in
            try await b.buildChain()
        },
        DemoScene(id: 2, name: "Absolut", description: "The Absolut Barcelona chain of 2008: 48 links around a piñon of 18 teeth and a plato of 29, the plato pedaled clockwise by a motor, as a bicycle riding to the right: it pulls the top run and the bottom one sags. Four boards hang from links, and each comes alive as the chain carries it over the plato; drag a pedal to pedal. Every body is locked in the plane, under a lighter gravity",
                  target: [2.8, 4.5, 0], distance: 16, gravity: 3) { b in
            // A meter higher than the chain needs, for the boards hanging under its sagging run
            try await b.buildChain(chain: DemoBuilder.absolutChain, teethA: 18, teethB: 29, height: 5, motored: .b,
                                   motor: .init(targetVelocity: -1, maxTorque: 15000),
                                   sprocketMass: .init(density: DemoBuilder.sprocketDensity), linkMass: .init(density: DemoBuilder.linkDensity),
                                   linkDamping: (linear: 0.1, angular: 0.5),
                                   isPlanar: true)
            let plato = b.root.findEntity(named: "sprocket B")!
            b.addPedals(to: plato)
            // The links of the 2008 scene, 22, 29, 36 and 45, from an origin that puts the bike on
            // the top run, left of its middle. Sizes guessed.
            b.hang([("bike", 3, [1, 0.7]), ("saddle", 10, [0.6, 0.6]), ("handlebar", 17, [0.9, 0.6]), ("Gaudí", 26, [0.6, 0.8])])
            b.addZone(above: plato, height: ChainMesh().pitchRadius(teeth: 29) + 0.5)
        },
        DemoScene(id: 3, name: "Planetary", description: "A sun of 12 teeth, three planets of 12 on a carrier and a ring of 36 with inward teeth, all on hinges: the sun motored, the ring braked, the carrier turns a quarter as fast",
                  target: [0, 0.6, 0], distance: 7) { b in
            try await b.buildPlanetary()
        },
    ]
}

extension DemoBuilder {
    /// A sprocket of the chain scene: A at the origin, B along +x
    public enum ChainSprocket: Sendable {
        case a, b
    }

    /// The chain scene: inner and outer links in turn, pinned to each other around two sprockets
    /// on hinges, in the x-y plane at `height`, with the friction of oiled steel. The `motored`
    /// sprocket's hinge drives it; `sprocketMass`, `linkMass` and `linkDamping` replace the defaults;
    /// planar bodies move only in the x-y plane and turn only about z.
    @MainActor
    public mutating func buildChain(chain: ChainMesh = ChainMesh(), links: Int = 48, teethA: Int = 20, teethB: Int = 10,
                                    height: Float = 3, motored: ChainSprocket = .a,
                                    motor: SDFRevoluteJoint.Motor = .init(targetVelocity: 1, maxTorque: 5000),
                                    sprocketMass: SDFMassProperties = .default, linkMass: SDFMassProperties = .default,
                                    linkDamping: (linear: Float, angular: Float)? = nil,
                                    isPlanar: Bool = false) async throws {
        let layout = ChainLayout(chain: chain, links: links, teethA: teethA, teethB: teethB)
        let origin = SIMD3<Float>(0, height, 0)
        let axle = simd_quatf(angle: -.pi / 2, axis: [0, 1, 0]) // The pin's x axis along z
        let steel = SDFMaterial(staticFriction: 0.1, dynamicFriction: 0.1, restitution: 0)
        let base = try await spawn("Box", at: [layout.centerDistance / 2, -1, 0], scale: 0.5, mode: .static, isVisible: false)

        let phases = layout.phases
        func makePlanar(_ entity: Entity) {
            if isPlanar { entity.components[SDFBodyComponent.self]!.lockToPlane() }
        }
        for (name, teeth, x, phase, sprocketID) in [("sprocket A", teethA, Float(0), phases.a, ChainSprocket.a),
                                                     ("sprocket B", teethB, layout.centerDistance, phases.b, .b)] {
            let shape = try chain.sprocketShape(teeth: teeth)
            let sprocket = spawn(shape, mesh: try MeshResource.generate(from: shape), named: name, at: origin + [x, 0, 0],
                                 orientation: simd_quatf(angle: phase, axis: [0, 0, 1]))
            sprocket.components[SDFBodyComponent.self]!.material = steel
            sprocket.components[SDFBodyComponent.self]!.massProperties = sprocketMass
            sprocket.components.set(SDFMotionComponent())
            makePlanar(sprocket)
            let pin0 = base.pins.set(named: name, position: sprocket.position, orientation: axle, relativeTo: root)
            let pin1 = sprocket.pins.set(named: "hub", orientation: sprocket.orientation.inverse * axle)
            var joint = SDFRevoluteJoint(pin0: pin0, pin1: pin1)
            if sprocketID == motored { joint.motor = motor }
            joints.append(joint)
        }

        // Inner links, with the rollers, and outer links, with the pins, in turn
        precondition(links.isMultiple(of: 2), "a chain of inner and outer links needs an even number, not \(links)")
        let shapes = [try chain.innerLinkShape(), try chain.outerLinkShape()]
        let meshes = try shapes.map { try MeshResource.generate(from: $0) }
        let pins = layout.pins
        var linkEntities: [Entity] = []
        for k in 0..<links {
            let from = pins[k], to = pins[(k + 1) % links]
            let middle = (from + to) / 2, direction = to - from
            let link = spawn(shapes[k % 2], mesh: meshes[k % 2], named: "link \(k)", at: origin + [middle.x, middle.y, 0],
                             orientation: simd_quatf(angle: atan2(direction.y, direction.x), axis: [0, 0, 1]))
            link.components[SDFBodyComponent.self]!.material = steel
            link.components[SDFBodyComponent.self]!.massProperties = linkMass
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

extension DemoBuilder {
    /// The planetary scene: a sun, three planets on a carrier and a ring, about the y axis at
    /// height 0.8, every gear of module 0.1 on a hinge. The sun is motored at 1 rad/s; with
    /// `brakingRing` the ring is held by a motor at zero speed and the carrier turns at
    /// sun / (1 + ring / sun) = 0.25 rad/s the same way, else the carrier is held and the ring
    /// turns at −sun / 3. The ring is the concave shape: one SDF, where a convex engine needs a
    /// hull per tooth.
    ///
    /// `backlash` is a profile shift: the sun's teeth thinned by it and the ring's spaces widened
    /// by it, each about the backlash of a center distance `backlash` larger, while every gear
    /// stays on its pitch circle. Center distance alone can't give both meshes room: the planets
    /// moved out from the sun by `b` move into the ring by `b`, whose center distance to a planet
    /// is the difference of their pitch radii, and a ring shifted to make room for that and some
    /// backlash besides needs twice the shift, which narrows its spaces to a V at the root.
    @MainActor
    public mutating func buildPlanetary(backlash: Float = 0.08, brakingRing: Bool = true) async throws {
        let module: Float = 0.1, thickness: Float = 0.4
        let sunTeeth = 12, planetTeeth = 12, ringTeeth = sunTeeth + 2 * planetTeeth
        let center = SIMD3<Float>(0, 0.8, 0)
        let axle = simd_quatf(angle: .pi / 2, axis: [0, 0, 1]) // The pin's x axis along y
        let steel = SDFMaterial(staticFriction: 0.1, dynamicFriction: 0.1, restitution: 0)
        // Fine bakes: some 0.009 units a cell for the gears and 0.011 for the ring, against the
        // contact skin of 0.02. At the gear pair's 0.014 and 0.019 the teeth wedge now and then,
        // and one wedged mesh of six stops the set.
        let gearResolution = 160, ringResolution = 400
        let base = try await spawn("Box", at: [0, -0.5, 0], scale: 0.4, mode: .static, isVisible: false)
        let orbit = Float(sunTeeth + planetTeeth) * module / 2

        /// A body on a hinge to `holder`, about the body's y axis through its position
        func hinge(_ body: Entity, to holder: Entity, motor: SDFRevoluteJoint.Motor? = nil) -> SDFRevoluteJoint {
            body.components[SDFBodyComponent.self]!.material = steel
            body.components.set(SDFMotionComponent())
            let pin0 = holder.pins.set(named: body.name, position: body.position, orientation: axle, relativeTo: root)
            let pin1 = body.pins.set(named: "hub", orientation: body.orientation.inverse * axle)
            return SDFRevoluteJoint(pin0: pin0, pin1: pin1, motor: motor)
        }

        // Phases, as angles about y; turning an entity by θ moves its outline by −θ in the x-z
        // plane's angle atan2(z, x). The sun, unturned, has a tooth at 0° and every 30°, so at
        // each planet's direction, 0°, 120° and 240°. A planet must face it with a space, and a
        // planet turned by half a tooth, 15°, has its spaces at every 30°: toward the sun and,
        // 180° on, toward the ring as well. The ring's spaces are at every 10°, its teeth between
        // them, so turned by half a tooth, 5°, it has a tooth at 0°, 120° and 240°: facing each
        // planet's outer space. The three planets can share a phase because (sun + ring) / 3 =
        // 16 is a whole number of teeth: the planets sit 16 sun teeth apart, as seen by both.
        let sunShape = try GearMesh.shape(teeth: sunTeeth, module: module, thickness: thickness, shift: -backlash, resolution: gearResolution)
        let sun = spawn(sunShape, mesh: try MeshResource.generate(from: sunShape), named: "sun", at: center)
        joints.append(hinge(sun, to: base, motor: .init(targetVelocity: 1, maxTorque: 500)))

        let ringShape = try GearMesh.ringShape(teeth: ringTeeth, module: module, thickness: thickness, outerRadius: 2.15,
                                               shift: backlash, resolution: ringResolution)
        let ring = spawn(ringShape, mesh: try MeshResource.generate(from: ringShape), named: "ring", at: center,
                         orientation: simd_quatf(angle: .pi / Float(ringTeeth), axis: [0, 1, 0]))
        joints.append(hinge(ring, to: base, motor: brakingRing ? .init(targetVelocity: 0, maxTorque: 2000) : nil))

        // The carrier, a plate under the gears, clear of the sun and the ring
        let plate = SDFShape.cylinder(radius: orbit + 0.25, halfHeight: 0.05)
        let carrier = spawn(plate, mesh: try MeshResource.generate(from: plate), named: "carrier", at: center - [0, 0.4, 0])
        joints.append(hinge(carrier, to: base, motor: brakingRing ? nil : .init(targetVelocity: 0, maxTorque: 2000)))

        let planetShape = try GearMesh.shape(teeth: planetTeeth, module: module, thickness: thickness, resolution: gearResolution)
        let planetMesh = try MeshResource.generate(from: planetShape)
        for k in 0..<3 {
            let direction = 2 * Float.pi * Float(k) / 3
            let planet = spawn(planetShape, mesh: planetMesh, named: "planet \(k)",
                               at: center + orbit * SIMD3(cos(direction), 0, sin(direction)),
                               orientation: simd_quatf(angle: .pi / Float(planetTeeth), axis: [0, 1, 0]))
            joints.append(hinge(planet, to: carrier))
        }
    }
}
