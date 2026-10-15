import SDFCollisionKit
import simd

/// Spur gears with involute teeth, generated as closed meshes to bake: the shape that meshes,
/// unlike the paper's toy gear, whose trapezoid teeth have no clearance and wedge tip on tip.
/// Two gears of the same module mesh when their pitch circles touch, at a center distance of
/// `(teethA + teethB) * module / 2`, and turn at the inverse ratio of their teeth.
public enum GearMesh {
    /// The pitch radius of a gear: half the module times the teeth
    public static func pitchRadius(teeth: Int, module: Float) -> Float { module * Float(teeth) / 2 }

    /// The mesh of a gear about the y axis, its teeth of the standard proportions: addendum one
    /// module, dedendum one and a quarter, pressure angle 20°
    public static func mesh(teeth: Int, module: Float, thickness: Float, pointsPerFlank: Int = 8)
        -> (positions: [SIMD3<Float>], indices: [UInt32]) {
        let pressureAngle: Float = 20 * .pi / 180
        let pitch = pitchRadius(teeth: teeth, module: module)
        let base = pitch * cos(pressureAngle), tip = pitch + module, root = pitch - 1.25 * module
        // The half thickness of a tooth, as an angle, at a radius on the involute
        func involute(_ angle: Float) -> Float { tan(angle) - angle }
        let halfAtPitch = Float.pi / (2 * Float(teeth))
        func halfAngle(at radius: Float) -> Float {
            let angle = acos(min(base / radius, 1))
            return halfAtPitch + involute(pressureAngle) - involute(angle)
        }

        // The profile of one tooth centered on angle zero, counterclockwise: up the left flank,
        // across the tip, down the right flank. Below the base circle the flank is radial.
        var tooth: [(radius: Float, angle: Float)] = []
        let flankStart = max(base, root)
        if root < base { tooth.append((root, -halfAngle(at: base))) }
        for i in 0...pointsPerFlank {
            let r = flankStart + (tip - flankStart) * Float(i) / Float(pointsPerFlank)
            tooth.append((r, -halfAngle(at: r)))
        }
        for i in (0...pointsPerFlank).reversed() {
            let r = flankStart + (tip - flankStart) * Float(i) / Float(pointsPerFlank)
            tooth.append((r, halfAngle(at: r)))
        }
        if root < base { tooth.append((root, halfAngle(at: base))) }

        // The outline, then two caps fanned from their centers and the side
        var outline: [SIMD2<Float>] = []
        for k in 0..<teeth {
            let center = 2 * .pi * Float(k) / Float(teeth)
            for point in tooth {
                let a = center + point.angle
                outline.append([point.radius * cos(a), point.radius * sin(a)])
            }
        }
        let n = outline.count, half = thickness / 2
        var positions: [SIMD3<Float>] = []
        for p in outline { positions.append([p.x, half, p.y]) }    // 0 ..< n, top
        for p in outline { positions.append([p.x, -half, p.y]) }   // n ..< 2n, bottom
        positions.append([0, half, 0])                             // 2n
        positions.append([0, -half, 0])                            // 2n + 1
        let topCenter = UInt32(2 * n), bottomCenter = UInt32(2 * n + 1)
        var indices: [UInt32] = []
        for i in 0..<n {
            let j = (i + 1) % n
            // Outline runs counterclockwise seen from +y, which in RealityKit's frame (x right,
            // z toward the viewer) is clockwise seen from above: wind the top cap the other way
            indices += [topCenter, UInt32(j), UInt32(i)]
            indices += [bottomCenter, UInt32(n + i), UInt32(n + j)]
            indices += [UInt32(i), UInt32(j), UInt32(n + j)]
            indices += [UInt32(i), UInt32(n + j), UInt32(n + i)]
        }
        return (positions, indices)
    }

    /// The gear baked, at `resolution` cells across its diameter
    public static func shape(teeth: Int, module: Float, thickness: Float, resolution: Int = 100) throws -> SDFShape {
        let mesh = mesh(teeth: teeth, module: module, thickness: thickness)
        return try SDFShape(baking: mesh.positions, indices: mesh.indices, resolution: resolution)
    }
}
