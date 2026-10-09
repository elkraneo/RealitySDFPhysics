// Runs every built-in scene headless and prints timing and sanity statistics.
//
//   swift run -c release sdf-bench [steps] [scene index...]
//
// Steps default to 600 (10 s of simulated time at 60 Hz).

import Foundation
import SDFCollisionKit

@MainActor
func run() {
    func usage() -> Never {
        print("usage: sdf-bench [steps > 0] [scene index...]")
        exit(1)
    }
    let numbers = CommandLine.arguments.dropFirst().compactMap { Int($0) }
    guard numbers.count == CommandLine.arguments.dropFirst().count else { usage() }
    let steps = numbers.first ?? 600
    let selected = numbers.count > 1 ? Array(numbers.dropFirst()) : SDFWorld.scenes.map(\.id)
    guard steps > 0, selected.allSatisfy(SDFWorld.scenes.indices.contains) else { usage() }

    print("scene                     bodies  load ms  step ms (mean / max)  collision ms  pairs  contacts  lowest y  finite")
    for index in selected {
        let scene = SDFWorld.scenes[index]
        let world = SDFWorld()

        let loadStart = Date()
        world.loadScene(index)
        let loadMilliseconds = Date().timeIntervalSince(loadStart) * 1000

        var stepTotal = 0.0, stepMax = 0.0, collisionTotal = 0.0
        for _ in 0..<steps {
            world.step()
            let stats = world.stats
            stepTotal += stats.stepMilliseconds
            stepMax = max(stepMax, stats.stepMilliseconds)
            collisionTotal += stats.collisionMilliseconds
        }

        let stats = world.stats
        let dynamic = world.bodies().filter { !$0.isStatic }
        let lowest = dynamic.map(\.origin.y).min() ?? 0
        let finite = dynamic.allSatisfy { b in [b.origin.x, b.origin.y, b.origin.z, b.orientation.real].allSatisfy(\.isFinite) }
        print(scene.name.padding(toLength: 25, withPad: " ", startingAt: 0) + String(format: " %6d %8.0f %9.2f / %6.2f %13.2f %6d %9d %9.2f  %@",
                     stats.bodies, loadMilliseconds, stepTotal / Double(steps), stepMax,
                     collisionTotal / Double(steps), stats.pairs, stats.contacts, lowest, (finite ? "yes" : "NO") as NSString))
    }
}

MainActor.assumeIsolated { run() }
