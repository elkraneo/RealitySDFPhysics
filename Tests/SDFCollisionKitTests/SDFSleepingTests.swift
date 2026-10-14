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

    @Test func thePaperPresetNeverSleeps() {
        root.components.set(SDFSimulationComponent(simulation: simulation, preset: .paper))
        let box = addBox(at: [0, 0.6, 0])
        run(240)
        #expect(!simulation.isSleeping(box))
        #expect(simulation.statistics.sleepingBodies == 0)
    }
}
