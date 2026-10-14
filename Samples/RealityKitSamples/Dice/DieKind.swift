// The dice of Apple's sample "Simulating dice rolls as a component for your game":
// https://developer.apple.com/documentation/tabletopkit/simulating-dice-rolls-as-a-component-for-your-game
//
// Apple's sample reads faces with TabletopKit's `TossableRepresentation`, which is visionOS only.
// Here every die lists its face normals, in its model's frame, with the value printed on each:
// the normals are the largest planar regions of Apple's meshes, and the values were read from
// renders of each face. Opposite faces add up to the usual totals (7 on the D6, 21 on the D20).

import simd

enum DieKind: String, CaseIterable {
    case d4 = "D4"
    case d6 = "D6"
    case d8 = "D8"
    case customD8 = "D8_customFaces"
    case d10 = "D10"
    case d12 = "D12"
    case d20 = "D20"

    /// The die's name, with the custom D8's values
    var label: String { self == .customD8 ? "D8 (1–35)" : rawValue }

    struct Face {
        var normal: SIMD3<Float>
        var value: Int
    }

    /// A D4 shows its value at the top vertex, the one its bottom face doesn't touch
    var readsBottomFace: Bool { self == .d4 }

    var faces: [Face] {
        let table: [(SIMD3<Float>, Int)] = switch self {
        case .d4: [
            // Values of a D4 are those of the vertex opposite each face
            ([0.8165, 0.3333, -0.4714], 3), ([0, -1, 0], 1), ([-0.8165, 0.3333, -0.4714], 2), ([0, 0.3333, 0.9428], 4),
        ]
        case .d6: [
            ([0, 0, -1], 5), ([-1, 0, 0], 4), ([0, 0, 1], 2), ([1, 0, 0], 3), ([0, 1, 0], 1), ([0, -1, 0], 6),
        ]
        case .d8: [
            ([0.8187, -0.3315, 0.4689], 6), ([0.8154, 0.3369, -0.4708], 7), ([-0.8154, 0.3369, -0.4708], 3),
            ([-0.8187, -0.3315, 0.4689], 2), ([0, 0.3370, 0.9415], 4), ([-0.0033, -1, 0.0020], 8), ([0, 1, 0.0037], 1),
            ([0.0033, -0.3316, -0.9434], 5),
        ]
        case .customD8: [
            ([0.8187, -0.3315, 0.4689], 25), ([0.8154, 0.3369, -0.4708], 30), ([-0.8154, 0.3369, -0.4708], 10),
            ([-0.8187, -0.3315, 0.4689], 5), ([0, 0.3370, 0.9415], 15), ([-0.0033, -1, 0.0020], 35),
            ([0.0033, -0.3316, -0.9434], 20), ([0, 1, 0.0037], 1),
        ]
        case .d10: [
            ([-0.0009, 1, 0.0020], 1), ([0.7082, 0.6179, -0.3415], 7), ([-0.7089, 0.6166, -0.3425], 9),
            ([-0.4374, -0.0029, -0.8993], 5), ([0.4375, -0.0010, -0.8992], 3), ([0.4229, 0.0004, 0.9062], 4),
            ([-0.4460, -0.0118, 0.8949], 6), ([0.7076, -0.6100, 0.3567], 10), ([-0.6990, -0.6300, 0.3384], 2),
            ([0.0008, -1, -0.0064], 8),
        ]
        case .d12: [
            ([-0.8506, -0.4472, -0.2764], 11), ([-0.8506, 0.4473, 0.2764], 6), ([0.8507, 0.4472, 0.2763], 2),
            ([-0.5257, 0.4472, -0.7236], 5), ([0, -0.4473, -0.8944], 9), ([0.8506, -0.4473, -0.2764], 7),
            ([0.5258, 0.4471, -0.7236], 10), ([-0.5257, -0.4472, 0.7236], 3), ([0.5258, -0.4472, 0.7236], 8),
            ([0, 1, 0], 1), ([0, 0.4472, 0.8944], 4), ([0, -1, 0], 12),
        ]
        case .d20: [
            ([-0.9411, 0.3159, 0.1207], 15), ([0.5644, 0.3481, 0.7485], 11), ([-0.9411, -0.3159, -0.1207], 12),
            ([0.9411, -0.3159, -0.1207], 6), ([-0.3382, -0.3358, 0.8791], 18), ([-0.5644, 0.3481, 0.7485], 5),
            ([0.9411, 0.3159, 0.1207], 9), ([0.5644, -0.7584, 0.3259], 14), ([0.0172, 0.7386, 0.6739], 13),
            ([0.5644, -0.3481, -0.7485], 16), ([-0.5644, 0.7584, -0.3259], 7), ([0.3660, -0.3480, 0.8631], 4),
            ([-0.5922, -0.3358, -0.7324], 10), ([-0.0172, 0.9998, -0.0100], 1), ([0.5644, 0.7584, -0.3259], 19),
            ([-0.0172, -0.9998, 0.0100], 20), ([0.0172, -0.7386, -0.6739], 8), ([0.3660, 0.3480, -0.8631], 3),
            ([-0.3382, 0.3358, -0.8791], 17), ([-0.5644, -0.7584, 0.3259], 2),
        ]
        }
        return table.map { Face(normal: normalize($0.0), value: $0.1) }
    }

    /// The face read for a die whose model has an orientation (relative to a frame whose y axis is up)
    func face(for orientation: simd_quatf) -> Face {
        let up = orientation.inverse.act([0, 1, 0]) * (readsBottomFace ? -1 : 1)
        return faces.max { dot($0.normal, up) < dot($1.normal, up) }!
    }

    /// A rotation that maps the die onto itself and one face onto another. Turning the model by it
    /// changes which number shows on a face, while the shape, and so the physics, stays the same.
    func symmetry(mapping from: Face, to target: Face) -> simd_quatf? {
        let normals = faces.map(\.normal)
        // A neighbor of `from`, and the faces at the same angle from `target`
        let neighbor = normals.filter { $0 != from.normal }.max { dot($0, from.normal) < dot($1, from.normal) }!
        let angle = dot(neighbor, from.normal)
        for candidate in normals where abs(dot(candidate, target.normal) - angle) < 0.02 {
            let rotation = simd_quatf(Self.frame(target.normal, candidate).transpose * Self.frame(from.normal, neighbor))
            // A symmetry maps every face onto a face
            let mapsOntoItself = normals.allSatisfy { n in
                let r = rotation.act(n)
                return normals.contains { dot($0, r) > 0.995 }
            }
            if mapsOntoItself { return rotation }
        }
        return nil
    }

    /// The rotation into an orthonormal frame with x along a and y towards b
    private static func frame(_ a: SIMD3<Float>, _ b: SIMD3<Float>) -> simd_float3x3 {
        let y = normalize(b - dot(a, b) * a)
        return simd_float3x3(columns: (a, y, cross(a, y))).transpose
    }
}
