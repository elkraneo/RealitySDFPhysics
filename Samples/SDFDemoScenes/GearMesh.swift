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
    /// module, dedendum one and a quarter, pressure angle 20°.
    ///
    /// `shift` moves the teeth outward, as a profile shift of the cutter: tips and roots by
    /// `shift`, and the teeth thicker by `2 * shift * tan(20°)` at the pitch circle. A negative
    /// shift thins the teeth, which gives a pair the backlash of a center distance about
    /// `-shift` larger, with the gears where they are.
    public static func mesh(teeth: Int, module: Float, thickness: Float, shift: Float = 0, pointsPerFlank: Int = 8)
        -> (positions: [SIMD3<Float>], indices: [UInt32]) {
        let pressureAngle: Float = 20 * .pi / 180
        let pitch = pitchRadius(teeth: teeth, module: module)
        let base = pitch * cos(pressureAngle), tip = pitch + module + shift, root = pitch - 1.25 * module + shift
        // The half thickness of a tooth, as an angle, at a radius on the involute
        func involute(_ angle: Float) -> Float { tan(angle) - angle }
        let halfAtPitch = Float.pi / (2 * Float(teeth)) + shift * tan(pressureAngle) / pitch
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
    public static func shape(teeth: Int, module: Float, thickness: Float, shift: Float = 0, resolution: Int = 100) throws -> SDFShape {
        let mesh = mesh(teeth: teeth, module: module, thickness: thickness, shift: shift)
        return try SDFShape(baking: mesh.positions, indices: mesh.indices, resolution: resolution)
    }

    /// The mesh of an internal gear about the y axis: a ring whose teeth point inward, the shape
    /// of a planetary set's outer member. Its tooth spaces are the teeth of an external gear of
    /// as many teeth, so its addendum and dedendum swap: tips one module inside the pitch circle,
    /// roots a module and a quarter outside. A gear of the same module meshes inside it at a
    /// center distance of `(ringTeeth - teeth) * module / 2`.
    ///
    /// `shift` moves the teeth outward, as a profile shift of the cutter: tips and roots by
    /// `shift`, and the spaces wider by `2 * shift * tan(20°)` at the pitch circle. A positive
    /// shift gives the gear inside the backlash of a center distance about `shift` smaller,
    /// with the gears where they are. Shifted far, the widened spaces would close to a point
    /// before the root; the roots then stop just short of it.
    public static func ringMesh(teeth: Int, module: Float, thickness: Float, outerRadius: Float, shift: Float = 0,
                                pointsPerFlank: Int = 8) -> (positions: [SIMD3<Float>], indices: [UInt32]) {
        let pressureAngle: Float = 20 * .pi / 180
        let pitch = pitchRadius(teeth: teeth, module: module)
        let base = pitch * cos(pressureAngle)
        func involute(_ angle: Float) -> Float { tan(angle) - angle }
        // The half width of a space, as an angle, at a radius on the involute: an external
        // gear's tooth, thickened by the shift. Below the base circle the flank is radial.
        let halfAtPitch = Float.pi / (2 * Float(teeth)) + shift * tan(pressureAngle) / pitch
        func halfAngle(at radius: Float) -> Float {
            halfAtPitch + involute(pressureAngle) - involute(acos(min(base / max(radius, base), 1)))
        }
        // The flanks meet where the half width reaches zero, at inv φ = halfAtPitch + inv α
        var pointed = pitch + 2 * module
        if halfAngle(at: pointed) < 0 {
            var low = pitch
            for _ in 0..<30 {
                let middle = (low + pointed) / 2
                if halfAngle(at: middle) > 0 { low = middle } else { pointed = middle }
            }
        }
        let tip = pitch - module + shift, root = min(pitch + 1.25 * module + shift, tip + 0.98 * (pointed - tip))

        // The profile of one space centered on angle zero, counterclockwise: out along the
        // left flank, back in along the right, so that the angles only increase
        var space: [(radius: Float, angle: Float)] = []
        for i in 0...pointsPerFlank {
            let r = tip + (root - tip) * Float(i) / Float(pointsPerFlank)
            space.append((r, -halfAngle(at: r)))
        }
        for i in (0...pointsPerFlank).reversed() {
            let r = tip + (root - tip) * Float(i) / Float(pointsPerFlank)
            space.append((r, halfAngle(at: r)))
        }

        // The inner outline and an outer circle of as many points at the same angles; the ring
        // isn't star-shaped about its center, so its caps are strips between the two
        var inner: [SIMD2<Float>] = [], outer: [SIMD2<Float>] = []
        for k in 0..<teeth {
            let center = 2 * .pi * Float(k) / Float(teeth)
            for point in space {
                let a = center + point.angle
                inner.append([point.radius * cos(a), point.radius * sin(a)])
                outer.append([outerRadius * cos(a), outerRadius * sin(a)])
            }
        }
        let n = inner.count, half = thickness / 2
        var positions: [SIMD3<Float>] = []
        for p in inner { positions.append([p.x, half, p.y]) }    // 0 ..< n, inner top
        for p in inner { positions.append([p.x, -half, p.y]) }   // n ..< 2n, inner bottom
        for p in outer { positions.append([p.x, half, p.y]) }    // 2n ..< 3n, outer top
        for p in outer { positions.append([p.x, -half, p.y]) }   // 3n ..< 4n, outer bottom
        var indices: [UInt32] = []
        for i in 0..<n {
            let j = (i + 1) % n
            let (ti, tj, bi, bj) = (UInt32(i), UInt32(j), UInt32(n + i), UInt32(n + j))
            let (oti, otj, obi, obj) = (UInt32(2 * n + i), UInt32(2 * n + j), UInt32(3 * n + i), UInt32(3 * n + j))
            // Wound as the external gear's: caps the other way round from the outline, the
            // outer wall facing out and the inner wall facing the center
            indices += [ti, tj, otj, ti, otj, oti]
            indices += [bi, obj, bj, bi, obi, obj]
            indices += [oti, otj, obj, oti, obj, obi]
            indices += [ti, bj, tj, ti, bi, bj]
        }
        return (positions, indices)
    }

    /// The internal gear baked, at `resolution` cells across its outer diameter: enough for some
    /// five cells across a tooth's height
    public static func ringShape(teeth: Int, module: Float, thickness: Float, outerRadius: Float, shift: Float = 0,
                                 resolution: Int = 240) throws -> SDFShape {
        let mesh = ringMesh(teeth: teeth, module: module, thickness: thickness, outerRadius: outerRadius, shift: shift)
        return try SDFShape(baking: mesh.positions, indices: mesh.indices, resolution: resolution)
    }
}
