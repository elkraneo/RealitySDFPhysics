// swift-tools-version: 6.2

import PackageDescription

let package = Package(
    name: "SDFCollisionKit",
    platforms: [.macOS(.v15), .iOS(.v18), .visionOS(.v2)],
    products: [
        .library(name: "SDFCollisionKit", targets: ["SDFCollisionKit"]),
        .executable(name: "SDFDemos", targets: ["SDFDemos"]),
        .executable(name: "SDFMechanics", targets: ["SDFMechanics"]),
        .executable(name: "RealityKitSamples", targets: ["RealityKitSamples"]),
        .executable(name: "sdf-snapshot", targets: ["sdf-snapshot"]),
        .executable(name: "sdf-video", targets: ["sdf-video"]),
        .executable(name: "Benchmarks", targets: ["Benchmarks"]),
        // The benchmark scene, timings and view, for the device app in Samples/BenchmarksApp
        .library(name: "SDFBenchmarks", targets: ["SDFBenchmarks"]),
    ],
    targets: [
        // Upstream C++ engine (vendored unmodified in upstream/) behind a C interface
        .target(
            name: "SDFCore",
            exclude: ["upstream/LICENSE"],
            cxxSettings: [.headerSearchPath("upstream")]
        ),
        // The library: shapes, components and the RealityKit system
        .target(
            name: "SDFCollisionKit",
            dependencies: ["SDFCore"]
        ),
        // The upstream demo scenes rebuilt on the public API, shared by the app and snapshots
        .target(
            name: "SDFDemoScenes",
            dependencies: ["SDFCollisionKit"],
            path: "Samples/SDFDemoScenes",
            resources: [.copy("Resources/assets")]
        ),
        // The window of the demo apps: a list of scenes, their parameters and an orbit camera
        .target(
            name: "SDFDemoViewer",
            dependencies: ["SDFCollisionKit", "SDFDemoScenes"],
            path: "Samples/SDFDemoViewer"
        ),
        .executableTarget(
            name: "SDFDemos",
            dependencies: ["SDFDemoScenes", "SDFDemoViewer"],
            path: "Samples/SDFDemos"
        ),
        // Mechanisms on the joints, beyond the paper's demo: gears, chains, the Absolut chain and
        // a planetary set, and their app
        .target(
            name: "SDFMechanicsScenes",
            dependencies: ["SDFCollisionKit", "SDFDemoScenes"],
            path: "Samples/SDFMechanicsScenes"
        ),
        .executableTarget(
            name: "SDFMechanics",
            dependencies: ["SDFCollisionKit", "SDFDemoViewer", "SDFMechanicsScenes"],
            path: "Samples/SDFMechanics"
        ),
        // Apple's RealityKit physics samples, ported to SDF bodies: the scenes, shared by the app and
        // the video renderer, and the app
        .target(
            name: "RealityKitSampleScenes",
            dependencies: ["SDFCollisionKit"],
            path: "Samples/RealityKitSampleScenes",
            exclude: ["Dice/Models/LICENSE.txt"],
            resources: [.copy("Dice/Models")]
        ),
        .executableTarget(
            name: "RealityKitSamples",
            dependencies: ["RealityKitSampleScenes"],
            path: "Samples/RealityKitSamples"
        ),
        // Offscreen RealityKit renders of every demo scene
        .executableTarget(
            name: "sdf-snapshot",
            dependencies: ["SDFDemoScenes", "SDFMechanicsScenes"]
        ),
        // The same pile of tori timed with SDFCollisionKit and with RealityKit's physics: the
        // scene, the timings and a view that runs the standard table, shared by the command-line
        // tool, the macOS app and the device app
        .target(
            name: "SDFBenchmarks",
            dependencies: ["SDFCollisionKit", "SDFDemoScenes", "SDFMechanicsScenes"],
            path: "Samples/SDFBenchmarks"
        ),
        .executableTarget(
            name: "Benchmarks",
            dependencies: ["SDFBenchmarks"],
            path: "Samples/Benchmarks"
        ),
        .executableTarget(
            name: "sdf-bench",
            dependencies: ["SDFBenchmarks"]
        ),
        // Showcase video of the demo scenes, rendered offscreen
        .executableTarget(
            name: "sdf-video",
            dependencies: ["SDFDemoScenes", "SDFMechanicsScenes", "RealityKitSampleScenes"]
        ),
        .testTarget(
            name: "SDFCollisionKitTests",
            dependencies: ["SDFCollisionKit", "SDFCore", "SDFDemoScenes", "SDFMechanicsScenes"]
        ),
    ],
    cxxLanguageStandard: .cxx17
)
