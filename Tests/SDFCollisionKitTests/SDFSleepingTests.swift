import RealityKit
import SDFCollisionKit
import Testing
import simd

@MainActor
@Suite struct SDFSleepingTests {
    let root = Entity()
    let simulation = SDFSimulation()
    let ground: Entity

    init() {
        root.components.set(SDFSimulationComponent(simulation: simulation))
        ground = Entity()
        ground.position = [0, -0.5, 0]
        ground.components.set(SDFBodyComponent(shape: .box(halfExtents: [5, 0.5, 5]), mode: .static))
        root.addChild(ground)
    }

    func addBox(at position: SIMD3<Float>, mode: PhysicsBodyMode = .dynamic) -> Entity {
        let entity = Entity()
        entity.position = position
        entity.components.set(SDFBodyComponent(shape: .box(halfExtents: [0.5, 0.5, 0.5]), mode: mode))
        root.addChild(entity)
        return entity
    }

    func run(_ steps: Int) {
        for _ in 0..<steps { simulation.step(root: root) }
    }

    @Test func aRestingStackFallsAsleepAndKeepsItsContacts() {
        let bottom = addBox(at: [0, 0.6, 0]), top = addBox(at: [0, 1.8, 0])
        run(240)
        #expect(simulation.isSleeping(bottom) && simulation.isSleeping(top))
        #expect(simulation.statistics.sleepingBodies == 2)
        #expect(simulation.statistics.contacts > 0)
        let position = top.position
        run(60)
        #expect(top.position == position)
    }

    @Test func anImpulseWakesTheBodiesThatFellAsleepTogether() {
        let bottom = addBox(at: [0, 0.6, 0]), top = addBox(at: [0, 1.8, 0])
        run(240)
        let height = top.position.y
        top.sdfPhysics.applyLinearImpulse([0, 2000, 0], relativeTo: root) // 2 m/s up
        run(1)
        #expect(!simulation.isSleeping(top) && !simulation.isSleeping(bottom))
        #expect(top.position.y > height + 0.02)
    }

    @Test func aFallingBodyWakesASleeperAndLandsOnIt() {
        let bottom = addBox(at: [0, 0.6, 0])
        run(120)
        #expect(simulation.isSleeping(bottom))
        let top = addBox(at: [0.2, 3, 0])
        var woke = false
        for _ in 0..<240 {
            run(1)
            woke = woke || !simulation.isSleeping(bottom)
        }
        #expect(woke)
        #expect(top.position.y > 1.5 && top.position.y < 1.6, "rests at \(top.position.y)")
    }

    @Test func removingTheGroundWakesWhatRestedOnIt() {
        let box = addBox(at: [0, 0.6, 0])
        run(120)
        #expect(simulation.isSleeping(box))
        ground.removeFromParent()
        run(30)
        #expect(!simulation.isSleeping(box))
        #expect(box.position.y < 0)
    }

    @Test func aMovingKinematicPlatformWakesWhatRestsOnIt() {
        let platform = addBox(at: [0, 0.5, 0], mode: .kinematic)
        let box = addBox(at: [0, 1.6, 0])
        run(120)
        #expect(simulation.isSleeping(box))
        for _ in 0..<60 {
            platform.position.x += 0.01
            run(1)
        }
        #expect(!simulation.isSleeping(box))
        #expect(box.position.x > 0.4, "carried to \(box.position.x)")
    }

    @Test func aSupportThatJumpsAwayWakesWhatRestedOnIt() {
        let platform = addBox(at: [0, 0.5, 0], mode: .kinematic)
        let box = addBox(at: [0, 1.6, 0])
        run(120)
        #expect(simulation.isSleeping(box))
        platform.position.x = 5
        run(30)
        #expect(!simulation.isSleeping(box))
        #expect(box.position.y < 1.4)
    }

    @Test func bodiesFilteredOutDoNotWakeAPile() {
        let box = addBox(at: [0, 0.6, 0])
        let ghost = addBox(at: [0, 0.6, 0], mode: .static)
        ghost.components[SDFBodyComponent.self]!.filter = CollisionFilter(group: .init(rawValue: 2), mask: .init(rawValue: 2))
        run(120)
        #expect(simulation.isSleeping(box))
        for _ in 0..<30 {
            ghost.position.x += 0.001
            run(1)
        }
        #expect(simulation.isSleeping(box))
    }

    @Test func aSleepingContactReportsTheImpulseThatHoldsIt() {
        let box = addBox(at: [0, 0.6, 0])
        run(120)
        #expect(simulation.isSleeping(box))
        var impulse: Float = 0
        // Pairs are tracked from the first subscription on, so the first step reports the
        // contact as begun
        let subscription = simulation.subscribe(to: SDFCollisionEvents.Began.self, on: box) { impulse = $0.impulse }
        run(1)
        _ = subscription
        // About the box's weight over a step: 1000 kg × 9.81 m/s² / 60
        #expect(impulse > 100 && impulse < 250, "impulse \(impulse)")
    }

    @Test func aNewContactMarginWakesSleepers() {
        let box = addBox(at: [0, 0.6, 0])
        run(120)
        #expect(simulation.isSleeping(box))
        root.components[SDFSimulationComponent.self]!.collisionDetection.contactMargin = 0.01
        run(1)
        #expect(!simulation.isSleeping(box))
    }

    @Test func aBodyThatWakesAndSleepsInOneStepIsWrittenBack() {
        var settings = SDFSimulationComponent(simulation: simulation)
        settings.timeStep = 0.5
        root.components.set(settings)
        let box = addBox(at: [0, 0.6, 0])
        run(20)
        #expect(simulation.isSleeping(box))
        let x = box.position.x
        simulation.setMotion(SDFMotionComponent(linearVelocity: [0.04, 0, 0]), of: box)
        run(1)
        #expect(simulation.isSleeping(box))
        #expect(box.position.x > x)
    }

    @Test func thePaperPresetNeverSleeps() {
        root.components.set(SDFSimulationComponent(simulation: simulation, preset: .paper))
        let box = addBox(at: [0, 0.6, 0])
        run(240)
        #expect(!simulation.isSleeping(box))
        #expect(simulation.statistics.sleepingBodies == 0)
    }
}
