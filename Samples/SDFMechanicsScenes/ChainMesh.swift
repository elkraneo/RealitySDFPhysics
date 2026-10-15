import SDFCollisionKit
import simd

/// A roller chain and its sprockets, generated as closed meshes to bake: links pinned to each
/// other at the rollers, and sprockets whose teeth seat the rollers. As in a bicycle's chain the
/// links alternate: inner links of two plates with a roller at each end, and outer links of two
/// plates outside them with the pins. The chain runs in the x-y plane with its pins along z.
public struct ChainMesh {
    /// Distance between the pins of a link
    public var pitch: Float = 0.5
    public var rollerRadius: Float = 0.12
    /// Half the plate's width, across the link
    public var plateHalfWidth: Float = 0.09
    public var plateThickness: Float = 0.03
    /// The inner links' plates are this far either side of the middle of the chain: the
    /// rollers and the sprocket are between them
    public var innerHalf: Float = 0.1
    /// Between an inner plate and the outer plate over it
    public var plateGap: Float = 0.01
    public var pinRadius: Float = 0.05
    /// How far a pin stands out of its outer plate, riveted
    public var pinHead: Float = 0.01
    public var sprocketThickness: Float = 0.12
    /// How much wider than a roller a seat is
    public var seatClearance: Float = 0.05
    /// How far the teeth rise above the pitch circle, in roller radii
    public var tipHeight: Float = 0.5
    /// The fraction of the chain's length the sprockets are closer than it would reach around:
    /// the return run sags by it. A taut loop is over-constrained, which the solver fights until
    /// the chain climbs off a tooth; up to some 6% drives, and this much hangs.
    public var slack: Float = 0.04

    public init() {}

    /// The outer links' plates are this far either side of the middle, over the inner ones
    public var outerHalf: Float { innerHalf + plateThickness + plateGap }

    /// The pitch radius of a sprocket: its seats are the vertices of a polygon of `teeth` sides
    /// of the chain's pitch
    public func pitchRadius(teeth: Int) -> Float { pitch / (2 * sin(.pi / Float(teeth))) }

    /// An inner link centered between its pins, which are at x = ±pitch / 2: two plates close
    /// together and a roller at each end
    public func innerLinkMesh() -> (positions: [SIMD3<Float>], indices: [UInt32]) {
        var mesh = Mesh()
        mesh.extrude(plateOutline, from: innerHalf, to: innerHalf + plateThickness)
        mesh.extrude(plateOutline, from: -innerHalf - plateThickness, to: -innerHalf)
        for cx in [-pitch / 2, pitch / 2] {
            mesh.extrude(circle(rollerRadius, at: [cx, 0]), from: -innerHalf, to: innerHalf, center: [cx, 0])
        }
        return (mesh.positions, mesh.indices)
    }

    /// An outer link centered between its pins, as an inner link: two plates outside the inner
    /// links' and the pins through them, which run through the inner links' rollers
    public func outerLinkMesh() -> (positions: [SIMD3<Float>], indices: [UInt32]) {
        var mesh = Mesh()
        mesh.extrude(plateOutline, from: outerHalf, to: outerHalf + plateThickness)
        mesh.extrude(plateOutline, from: -outerHalf - plateThickness, to: -outerHalf)
        let end = outerHalf + plateThickness + pinHead
        for cx in [-pitch / 2, pitch / 2] {
            mesh.extrude(circle(pinRadius, at: [cx, 0]), from: -end, to: end, center: [cx, 0])
        }
        return (mesh.positions, mesh.indices)
    }

    /// A plate: a bar between two discs at the pins, outlined by the farthest point along each
    /// ray, which also fills the notch where the bar meets a disc
    var plateOutline: [SIMD2<Float>] {
        let half = pitch / 2, r = rollerRadius, w = plateHalfWidth
        return (0..<180).map { i in
            let a = 2 * .pi * Float(i) / 180
            let d = SIMD2(cos(a), sin(a))
            // The bar, a rectangle of half extents (half, w)
            let tx = d.x != 0 ? half / abs(d.x) : .infinity, ty = d.y != 0 ? w / abs(d.y) : .infinity
            var far = min(tx, ty)
            for cx in [-half, half] {
                let c = SIMD2(cx, 0)
                let b = simd_dot(d, c), disc = b * b - simd_dot(c, c) + r * r
                if disc > 0 { far = max(far, b + sqrt(disc)) }
            }
            return d * far
        }
    }

    func circle(_ radius: Float, at center: SIMD2<Float>) -> [SIMD2<Float>] {
        (0..<48).map { i in
            let a = 2 * .pi * Float(i) / 48
            return center + radius * SIMD2(cos(a), sin(a))
        }
    }

    /// A sprocket about z with a seat for a roller at every tooth, the first seat at angle zero
    public func sprocketMesh(teeth: Int) -> (positions: [SIMD3<Float>], indices: [UInt32]) {
        let radius = pitchRadius(teeth: teeth)
        // The seats clear the rollers by two and a half contact skins; tighter, the solver binds
        // the chain, wider, the teeth thin into needles that wedge between rollers
        let seat = rollerRadius + seatClearance, tip = radius + rollerRadius * tipHeight
        let samples = teeth * 24
        var outline: [SIMD2<Float>] = []
        for i in 0..<samples {
            let a = 2 * .pi * Float(i) / Float(samples)
            // Where the ray enters the nearest seat, if it does, else the tip circle
            let k = (a / (2 * .pi) * Float(teeth)).rounded()
            let delta = a - 2 * .pi * k / Float(teeth)
            var far = tip
            let s = radius * sin(delta)
            if abs(s) < seat {
                far = min(far, radius * cos(delta) - sqrt(seat * seat - s * s))
            }
            outline.append([far * cos(a), far * sin(a)])
        }
        var mesh = Mesh()
        mesh.extrude(outline, from: -sprocketThickness / 2, to: sprocketThickness / 2)
        return (mesh.positions, mesh.indices)
    }

    public func innerLinkShape(resolution: Int = 128) throws -> SDFShape {
        let mesh = innerLinkMesh()
        return try SDFShape.cached(baking: mesh.positions, indices: mesh.indices, resolution: resolution)
    }

    public func outerLinkShape(resolution: Int = 128) throws -> SDFShape {
        let mesh = outerLinkMesh()
        return try SDFShape.cached(baking: mesh.positions, indices: mesh.indices, resolution: resolution)
    }

    public func sprocketShape(teeth: Int, resolution: Int = 160) throws -> SDFShape {
        let mesh = sprocketMesh(teeth: teeth)
        return try SDFShape.cached(baking: mesh.positions, indices: mesh.indices, resolution: resolution)
    }

    /// Triangles of extruded outlines, each fanned from a center it is star-shaped about
    struct Mesh {
        var positions: [SIMD3<Float>] = []
        var indices: [UInt32] = []

        mutating func extrude(_ outline: [SIMD2<Float>], from z0: Float, to z1: Float, center: SIMD2<Float> = .zero) {
            let base = UInt32(positions.count), n = outline.count
            for p in outline { positions.append([p.x, p.y, z1]) }
            for p in outline { positions.append([p.x, p.y, z0]) }
            positions.append([center.x, center.y, z1])
            positions.append([center.x, center.y, z0])
            let top = base + UInt32(2 * n), bottom = top + 1
            for i in 0..<n {
                let j = (i + 1) % n
                let ti = base + UInt32(i), tj = base + UInt32(j), bi = base + UInt32(n + i), bj = base + UInt32(n + j)
                indices += [top, ti, tj]
                indices += [bottom, bj, bi]
                indices += [ti, bi, bj]
                indices += [ti, bj, tj]
            }
        }
    }
}

/// Where the pins of a chain go around two sprockets, A at the origin and B along +x, and how
/// far apart the sprockets must be for a number of links to close the loop
public struct ChainLayout {
    public var chain: ChainMesh
    public var links: Int
    public var teethA: Int, teethB: Int
    public private(set) var centerDistance: Float = 0

    public init(chain: ChainMesh, links: Int, teethA: Int, teethB: Int) {
        self.chain = chain
        self.links = links
        self.teethA = teethA
        self.teethB = teethB
        // The loop's length grows with the distance; bisect for the chain's length less the
        // slack, which the bottom run then hangs out
        let target = Float(links) * chain.pitch * (1 - chain.slack)
        var low = radiusA + radiusB, high = target / 2
        for _ in 0..<60 {
            let mid = (low + high) / 2
            if loopLength(at: mid) < target { low = mid } else { high = mid }
        }
        centerDistance = (low + high) / 2
    }

    public var radiusA: Float { chain.pitchRadius(teeth: teethA) }
    public var radiusB: Float { chain.pitchRadius(teeth: teethB) }

    /// The tangent lines lean by this angle when the sprockets differ
    func lean(at distance: Float) -> Float { asin((radiusA - radiusB) / distance) }

    func loopLength(at distance: Float) -> Float {
        let beta = lean(at: distance)
        let straight = sqrt(distance * distance - (radiusA - radiusB) * (radiusA - radiusB))
        return 2 * straight + radiusB * (.pi - 2 * beta) + radiusA * (.pi + 2 * beta)
    }

    /// The pins, clockwise from where the top run leaves sprocket A. Around each sprocket they
    /// sit a tooth apart from its top tangent point, so that every roller on a sprocket is in a
    /// seat; the two runs share the rest evenly, closer than a pitch by the slack, which the
    /// chain hangs out as sag.
    public var pins: [SIMD2<Float>] {
        let beta = lean(at: centerDistance), c = centerDistance
        let straight = sqrt(c * c - (radiusA - radiusB) * (radiusA - radiusB))
        // The tangent points: the runs' normal leans toward the small sprocket
        let top = Float.pi / 2 - beta, bottom = -Float.pi / 2 + beta
        func onA(_ a: Float) -> SIMD2<Float> { [radiusA * cos(a), radiusA * sin(a)] }
        func onB(_ a: Float) -> SIMD2<Float> { [c + radiusB * cos(a), radiusB * sin(a)] }
        // Around B, clockwise from its top tangent point, within its wrap
        let stepB = 2 * .pi / Float(teethB)
        var wrapB: [SIMD2<Float>] = []
        var a = top
        while a > bottom - 1e-4 { wrapB.append(onB(a)); a -= stepB }
        // Around A, from its top tangent point backwards, then reversed into chain order
        let stepA = 2 * .pi / Float(teethA)
        var wrapA: [SIMD2<Float>] = []
        a = top
        while a < bottom + 2 * .pi + 1e-4 { wrapA.append(onA(a)); a += stepA }
        wrapA.reverse()
        // The runs share what is left, between the last pin of one wrap and the first of the next
        precondition(links >= wrapA.count + wrapB.count + 2, "\(links) links can't reach around \(teethA) and \(teethB) teeth")
        let onRuns = links - wrapA.count - wrapB.count
        let onTop = onRuns / 2, onBottom = onRuns - onTop
        func run(from: SIMD2<Float>, to: SIMD2<Float>, count: Int) -> [SIMD2<Float>] {
            (1...max(count, 1)).prefix(count).map { from + (to - from) * (Float($0) / Float(count + 1)) }
        }
        var pins: [SIMD2<Float>] = []
        pins += wrapA.suffix(1)
        pins += run(from: wrapA.last!, to: wrapB.first!, count: onTop)
        pins += wrapB
        pins += run(from: wrapB.last!, to: wrapA.first!, count: onBottom)
        pins += wrapA.dropLast()
        return Array(pins.prefix(links))
    }

    /// The angle of the first seat of each sprocket: a seat at each top tangent point
    public var phases: (a: Float, b: Float) {
        let top = Float.pi / 2 - lean(at: centerDistance)
        return (top, top)
    }

}
