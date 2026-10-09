// swift-tools-version: 6.2

import PackageDescription

let package = Package(
    name: "SDFCollisionKit",
    platforms: [.macOS(.v15), .iOS(.v18), .visionOS(.v2)],
    products: [
        .library(name: "SDFCollisionKit", targets: ["SDFCollisionKit"]),
        .executable(name: "SDFDemos", targets: ["SDFDemos"]),
        .executable(name: "sdf-bench", targets: ["sdf-bench"]),
        .executable(name: "sdf-snapshot", targets: ["sdf-snapshot"]),
    ],
    targets: [
        // Upstream C++ engine (vendored unmodified in upstream/) behind a C interface
        .target(
            name: "SDFCore",
            exclude: ["upstream/LICENSE"],
            cxxSettings: [
                .headerSearchPath("upstream"),
                .headerSearchPath("shim"),
            ]
        ),
        // Swift wrapper and RealityKit integration (components, system, mesh conversion)
        .target(
            name: "SDFCollisionKit",
            dependencies: ["SDFCore"],
            resources: [.copy("Resources/assets")]
        ),
        // macOS RealityKit port of the upstream demo scenes
        .executableTarget(
            name: "SDFDemos",
            dependencies: ["SDFCollisionKit"]
        ),
        // Headless timing of every scene, for comparison with the upstream demo
        .executableTarget(
            name: "sdf-bench",
            dependencies: ["SDFCollisionKit"]
        ),
        // Offscreen RealityKit renders of every scene
        .executableTarget(
            name: "sdf-snapshot",
            dependencies: ["SDFCollisionKit"]
        ),
        .testTarget(
            name: "SDFCollisionKitTests",
            dependencies: ["SDFCollisionKit"]
        ),
    ],
    cxxLanguageStandard: .cxx17
)
