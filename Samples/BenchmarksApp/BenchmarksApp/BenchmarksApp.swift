import SDFBenchmarks
import SwiftUI

/// The benchmark on an iPhone, an iPad or a Vision Pro: the same view as the macOS sample
@main
struct BenchmarksApp: App {
    var body: some Scene {
        WindowGroup {
            NavigationStack {
                BenchmarkView()
            }
        }
    }
}
