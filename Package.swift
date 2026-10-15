// swift-tools-version: 6.2

import PackageDescription

let package = Package(
    name: "RealitySDFPhysics",
    platforms: [.macOS(.v15), .iOS(.v18), .visionOS(.v2)],
    products: [
        .library(name: "RealitySDFPhysics", targets: ["RealitySDFPhysics"]),
        .executable(name: "SDFDemos", targets: ["SDFDemos"]),
        .executable(name: "SDFMechanics", targets: ["SDFMechanics"]),
        .executable(name: "SDFRealityKitSamples", targets: ["SDFRealityKitSamples"]),
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
            name: "RealitySDFPhysics",
            dependencies: ["SDFCore"]
        ),
        // The upstream demo scenes rebuilt on the public API, shared by the app and snapshots
        .target(
            name: "SDFDemoScenes",
            dependencies: ["RealitySDFPhysics"],
            path: "Samples/SDFDemoScenes",
            resources: [.copy("Resources/assets")]
        ),
        // The window of the demo apps: a list of scenes, their parameters and an orbit camera
        .target(
            name: "SDFDemoViewer",
            dependencies: ["RealitySDFPhysics", "SDFDemoScenes"],
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
            dependencies: ["RealitySDFPhysics", "SDFDemoScenes"],
            path: "Samples/SDFMechanicsScenes"
        ),
        .executableTarget(
            name: "SDFMechanics",
            dependencies: ["RealitySDFPhysics", "SDFDemoViewer", "SDFMechanicsScenes"],
            path: "Samples/SDFMechanics"
        ),
        // Apple's RealityKit physics samples, ported to SDF bodies: the scenes, shared by the app and
        // the video renderer, and the app
        .target(
            name: "SDFRealityKitSampleScenes",
            dependencies: ["RealitySDFPhysics"],
            path: "Samples/SDFRealityKitSampleScenes",
            exclude: ["Dice/Models/LICENSE.txt"],
            resources: [.copy("Dice/Models")]
        ),
        .executableTarget(
            name: "SDFRealityKitSamples",
            dependencies: ["SDFRealityKitSampleScenes"],
            path: "Samples/SDFRealityKitSamples"
        ),
        // Offscreen RealityKit renders of every demo scene
        .executableTarget(
            name: "sdf-snapshot",
            dependencies: ["SDFDemoScenes", "SDFMechanicsScenes"]
        ),
        // The same pile of tori timed with RealitySDFPhysics and with RealityKit's physics: the
        // scene, the timings and a view that runs the standard table, shared by the command-line
        // tool, the macOS app and the device app
        .target(
            name: "SDFBenchmarks",
            dependencies: ["RealitySDFPhysics", "SDFDemoScenes", "SDFMechanicsScenes"],
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
            dependencies: ["SDFDemoScenes", "SDFMechanicsScenes", "SDFRealityKitSampleScenes"]
        ),
        .testTarget(
            name: "RealitySDFPhysicsTests",
            dependencies: ["RealitySDFPhysics", "SDFCore", "SDFDemoScenes", "SDFMechanicsScenes"]
        ),
    ],
    cxxLanguageStandard: .cxx17
)
