import Foundation

/// The machine a benchmark ran on, for its report
public struct DeviceInfo: Sendable {
    public var model: String
    public var system: String
    public var performanceCores: Int
    public var efficiencyCores: Int
    public var thermalState: String

    public static var current: DeviceInfo {
        // hw.machine names the device on iOS and visionOS, and only the architecture on macOS
        let machine = sysctlString("hw.machine")
        return DeviceInfo(model: machine.hasPrefix("arm64") || machine.hasPrefix("x86") ? sysctlString("hw.model") : machine,
                   system: ProcessInfo.processInfo.operatingSystemVersionString,
                   performanceCores: sysctlInt("hw.perflevel0.logicalcpu"),
                   efficiencyCores: sysctlInt("hw.perflevel1.logicalcpu"),
                   thermalState: thermalStateName)
    }

    public var text: String {
        "\(model), \(system), \(performanceCores) performance + \(efficiencyCores) efficiency cores, thermal \(thermalState)"
    }

    /// The engine's solver pool takes the performance cores but one
    public var solverThreads: Int { max(1, performanceCores - 1) }

    private static var thermalStateName: String {
        switch ProcessInfo.processInfo.thermalState {
        case .nominal: "nominal"
        case .fair: "fair"
        case .serious: "serious"
        case .critical: "critical"
        @unknown default: "unknown"
        }
    }

    private static func sysctlString(_ name: String) -> String {
        var size = 0
        guard sysctlbyname(name, nil, &size, nil, 0) == 0, size > 0 else { return "?" }
        var buffer = [CChar](repeating: 0, count: size)
        guard sysctlbyname(name, &buffer, &size, nil, 0) == 0 else { return "?" }
        return String(cString: buffer)
    }

    private static func sysctlInt(_ name: String) -> Int {
        var value: Int32 = 0
        var size = MemoryLayout<Int32>.size
        return sysctlbyname(name, &value, &size, nil, 0) == 0 ? Int(value) : 0
    }
}
