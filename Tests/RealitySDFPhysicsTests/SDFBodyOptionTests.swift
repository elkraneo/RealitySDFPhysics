import Combine
import RealityKit
import RealitySDFPhysics
import Testing
import simd

/// The options of RealityKit's physics that the SDF bodies mirror
@MainActor
@Suite struct SDFBodyOptionTests {
    let root = Entity()
    let simulation = SDFSimulation()
    let ground: Entity

    init() {
        root.components.set(SDFPhysicsSimulationComponent(simulation: simulation))
        // Ground with its top face at y = 0
        ground = Entity()
        ground.position = [0, -0.5, 0]
        ground.components.set(SDFPhysicsBodyComponent(shape: .box(halfExtents: [5, 0.5, 5]), mode: .static))
        root.addChild(ground)
    }

    @discardableResult
    func addBody(_ shape: SDFShape = .sphere(radius: 0.25), mode: PhysicsBodyMode = .dynamic, at position: SIMD3<Float>,
                 configure: (inout SDFPhysicsBodyComponent) -> Void = { _ in }) -> Entity {
        let entity = Entity()
        entity.position = position
        var body = SDFPhysicsBodyComponent(shape: shape, mode: mode)
        configure(&body)
        entity.components.set(body)
        root.addChild(entity)
        return entity
    }

    func run(_ steps: Int) {
        for _ in 0..<steps { simulation.step(root: root) }
    }

    func highest(of entity: Entity, over steps: Int) -> Float {
        var top = -Float.infinity
        for _ in 0..<steps {
            simulation.step(root: root)
            top = max(top, entity.position.y)
        }
        return top
    }

    @Test func restitutionBounces() {
        let dull = addBody(at: [-1, 1.25, 0])
        let bouncy = addBody(at: [1, 1.25, 0]) { $0.material = SDFPhysicsMaterial(friction: 0.5, restitution: 0.8) }
        run(40) // Both land after about 0.45 s
        #expect(highest(of: bouncy, over: 60) > 0.6)
        #expect(highest(of: dull, over: 60) < 0.3)
    }

    @Test func bodiesWithoutGravityFloat() {
        let ball = addBody(at: [0, 2, 0]) { $0.isAffectedByGravity = false }
        run(60)
        #expect(abs(ball.position.y - 2) < 1e-4)
    }

    @Test func lockedAxesDoNotMove() {
        // A tilted box falling on its edge would turn and slide; locks keep it upright and in place
        let box = addBody(.box(halfExtents: [0.3, 0.3, 0.3]), at: [0, 1, 0]) {
            $0.isTranslationLocked = (true, false, true)
            $0.isRotationLocked = (true, true, true)
        }
        box.orientation = simd_quatf(angle: 0.4, axis: [0, 0, 1])
        box.components.set(SDFPhysicsMotionComponent(linearVelocity: [3, 0, 0], angularVelocity: [0, 0, 5]))
        run(120)
        #expect(abs(box.position.x) < 1e-5 && abs(box.position.z) < 1e-5)
        #expect(abs(box.orientation.angle - 0.4) < 1e-4)
        #expect(box.position.y < 0.6) // It still fell
    }

    @Test func slidingUsesDynamicFriction() {
        // Coulomb friction decelerates a sliding box at about μ_dynamic × g
        func speed(after steps: Int, material: SDFPhysicsMaterial) -> Float {
            let box = addBody(.box(halfExtents: [0.25, 0.25, 0.25]), at: [0, 0.27, 0]) {
                $0.material = material
                $0.linearDamping = 0
            }
            box.components.set(SDFPhysicsMotionComponent(linearVelocity: [3, 0, 0]))
            run(steps)
            box.removeFromParent()
            return box.components[SDFPhysicsMotionComponent.self]!.linearVelocity.x
        }
        let slippery = speed(after: 30, material: SDFPhysicsMaterial(staticFriction: 1, dynamicFriction: 0.1, restitution: 0))
        let rough = speed(after: 30, material: SDFPhysicsMaterial(staticFriction: 1, dynamicFriction: 0.4, restitution: 0))
        // The ground's friction is 0.5, combined by geometric mean
        #expect(abs(slippery - (3 - 0.5 * sqrt(0.1 * 0.5) * 9.81)) < 0.25, "slippery \(slippery)")
        #expect(abs(rough - (3 - 0.5 * sqrt(0.4 * 0.5) * 9.81)) < 0.25, "rough \(rough)")
    }

    @Test func dampingSlowsBodies() {
        let ball = addBody(at: [0, 2, 0]) {
            $0.isAffectedByGravity = false
            $0.linearDamping = 2
        }
        ball.components.set(SDFPhysicsMotionComponent(linearVelocity: [1, 0, 0]))
        run(60)
        let speed = ball.components[SDFPhysicsMotionComponent.self]!.linearVelocity.x
        #expect(abs(speed - exp(-2)) < 0.01)
    }

    @Test func filteredPairsPassThrough() {
        let ghosts = CollisionGroup(rawValue: 1 << 5)
        let ghost = addBody(at: [0, 1, 0]) { $0.filter = CollisionFilter(group: ghosts, mask: .all.subtracting(.default)) }
        run(120)
        #expect(ghost.position.y < -1) // Fell through the ground
    }

    @Test func triggersReportWithoutResponding() {
        // A static trigger volume in the path of a falling ball
        let trigger = addBody(.box(halfExtents: [1, 0.25, 1]), mode: .static, at: [0, 1, 0]) { $0.isTrigger = true }
        let ball = addBody(at: [0, 2, 0])
        var began: [Entity] = [], ended: [Entity] = []
        let subscriptions = [
            simulation.subscribe(to: SDFCollisionEvents.Began.self, on: trigger) { began.append($0.entityA == trigger ? $0.entityB : $0.entityA) },
            simulation.subscribe(to: SDFCollisionEvents.Ended.self, on: trigger) { ended.append($0.entityA == trigger ? $0.entityB : $0.entityA) },
        ]
        withExtendedLifetime(subscriptions) { run(90) }
        #expect(began == [ball] && ended == [ball])
        #expect(ball.position.y < 0.3) // Fell through the trigger onto the ground
    }

    @Test func collisionEventsFollowAContact() {
        let ball = addBody(at: [0, 1, 0])
        var events: [String] = []
        var impulse: Float = 0
        let subscriptions = [
            simulation.subscribe(to: SDFCollisionEvents.Began.self, on: ball) {
                events.append("began")
                impulse = $0.impulse
                #expect($0.contacts.isEmpty == false)
            },
            simulation.subscribe(to: SDFCollisionEvents.Updated.self) { _ in
                if events.last != "updated" { events.append("updated") }
            },
            simulation.subscribe(to: SDFCollisionEvents.Ended.self) { _ in events.append("ended") },
        ]
        withExtendedLifetime(subscriptions) {
            run(60)
            ball.removeFromParent()
            run(1)
        }
        #expect(events == ["began", "updated", "ended"])
        #expect(impulse > 0)
    }

    @Test func cancelledSubscriptionsStop() {
        addBody(at: [0, 1, 0])
        var count = 0
        let subscription = simulation.subscribe(to: SDFPhysicsSimulationEvents.DidSimulate.self) { _ in count += 1 }
        run(3)
        subscription.cancel()
        run(3)
        #expect(count == 3)
    }

    @Test func forcesActForOneUpdate() {
        let ball = addBody(at: [0, 2, 0]) { $0.isAffectedByGravity = false }
        run(1)
        let mass = simulation.mass(of: ball)!
        ball.components.set(SDFPhysicsMotionComponent())
        ball.sdf.addForce([mass * 60, 0, 0], relativeTo: nil) // 60 units/s² for one step of 1/60 s
        run(2)
        let velocity = ball.components[SDFPhysicsMotionComponent.self]!.linearVelocity.x
        #expect(abs(velocity - 1) < 0.01, "velocity \(velocity)")
    }

    @Test func impulsesChangeVelocityAtOnce() {
        let ball = addBody(at: [0, 2, 0]) { $0.isAffectedByGravity = false }
        ball.components.set(SDFPhysicsMotionComponent())
        // Before the body exists: kept for its creation
        ball.sdf.applyLinearImpulse([0, 0, 1], relativeTo: nil)
        run(1)
        let velocity = ball.components[SDFPhysicsMotionComponent.self]!.linearVelocity.z
        #expect(abs(velocity - 1 / simulation.mass(of: ball)!) < 1e-4)
    }

    @Test func raycastFindsTheNearestBody() {
        let near = addBody(mode: .static, at: [0, 1, 0])
        addBody(mode: .static, at: [0, 2, 0])
        run(1)
        let hits = simulation.raycast(origin: [0, 5, 0], direction: [0, -1, 0])
        #expect(hits.count == 3) // Two spheres and the ground
        let nearest = simulation.raycast(origin: [0, 0.5, 0], direction: [0, 1, 0], query: .nearest)
        #expect(nearest.first?.entity == near)
        #expect(abs((nearest.first?.distance ?? 0) - 0.25) < 0.01)
        #expect(simd_distance(nearest.first?.normal ?? .zero, [0, -1, 0]) < 0.05)
    }

    @Test func overlapsAnswerWhetherAShapeFits() {
        // A U shaped holder: a cube fits in the gap, a wider one does not
        for x: Float in [-0.6, 0.6] { addBody(.box(halfExtents: [0.1, 0.5, 0.5]), mode: .static, at: [x, 0.5, 0]) }
        run(1)
        let at = Transform(translation: [0, 0.5, 0])
        let fits = simulation.overlaps(of: .box(halfExtents: [0.4, 0.4, 0.4]), at: at)
        let tooWide = simulation.overlaps(of: .box(halfExtents: [0.6, 0.4, 0.4]), at: at)
        #expect(fits.isEmpty)
        #expect(tooWide.count == 2)
        #expect(tooWide.allSatisfy { $0.separation < 0 })
        // Deterministic, and it leaves the simulation alone
        #expect(simulation.overlaps(of: .box(halfExtents: [0.6, 0.4, 0.4]), at: at).map(\.separation) == tooWide.map(\.separation))
    }

    @Test func kinematicPairsReportWhenAsked() {
        root.components[SDFPhysicsSimulationComponent.self]!.collisionOptions = .reportKinematicVsStatic
        let platform = addBody(.box(halfExtents: [0.5, 0.1, 0.5]), mode: .kinematic, at: [0, 0.05, 0])
        var began = 0
        let subscription = simulation.subscribe(to: SDFCollisionEvents.Began.self, on: platform) { _ in began += 1 }
        withExtendedLifetime(subscription) { run(2) }
        #expect(began == 1)
    }

    @Test func metersPerUnitSetsThePhysicalDefaults() {
        // A 1 m cube modeled in centimeters, in a root scaled to 0.01
        let root = Entity(), simulation = SDFSimulation()
        root.scale = SIMD3(repeating: 0.01)
        let settings = SDFPhysicsSimulationComponent(simulation: simulation, metersPerUnit: 0.01)
        #expect(simd_distance(settings.gravity, [0, -981, 0]) < 1e-3)
        #expect(abs(settings.restitutionThreshold - 100) < 1e-3)
        root.components.set(settings)
        let cube = Entity()
        cube.position = [0, 500, 0]
        cube.components.set(SDFPhysicsBodyComponent(shape: .box(halfExtents: [50, 50, 50])))
        root.addChild(cube)
        simulation.step(root: root)
        #expect(abs(simulation.mass(of: cube)! - 1000) < 30) // Water, integrated over a voxelization
        for _ in 0..<23 { simulation.step(root: root) } // 0.4 s in all
        // Falls 0.5 g t² = 78 cm, plus up to 2 cm for the integration in 48 substeps, less a little
        // for the damping
        let fallen = 500 - cube.position.y
        #expect(fallen > 74 && fallen < 81, "fell \(fallen) cm")
    }
}
