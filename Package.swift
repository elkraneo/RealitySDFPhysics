// swift-tools-version: 6.2

import PackageDescription

let package = Package(
    name: "SDFCollisionKit",
    platforms: [.macOS(.v15), .iOS(.v18), .visionOS(.v2)],
    products: [
        .library(name: "SDFCollisionKit", targets: ["SDFCollisionKit"]),
        .executable(name: "SDFDemos", targets: ["SDFDemos"]),
        .executable(name: "RealityKitSamples", targets: ["RealityKitSamples"]),
        .executable(name: "sdf-snapshot", targets: ["sdf-snapshot"]),
        .executable(name: "sdf-video", targets: ["sdf-video"]),
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
        .executableTarget(
            name: "SDFDemos",
            dependencies: ["SDFDemoScenes"],
            path: "Samples/SDFDemos"
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
            dependencies: ["SDFDemoScenes"]
        ),
        // The same pile of tori timed with SDFCollisionKit and with RealityKit's physics
        .executableTarget(
            name: "sdf-bench",
            dependencies: ["SDFCollisionKit"]
        ),
        // Showcase video of the demo scenes, rendered offscreen
        .executableTarget(
            name: "sdf-video",
            dependencies: ["SDFDemoScenes", "RealityKitSampleScenes"]
        ),
        .testTarget(
            name: "SDFCollisionKitTests",
            dependencies: ["SDFCollisionKit", "SDFCore", "SDFDemoScenes"]
        ),
    ],
    cxxLanguageStandard: .cxx17
)
