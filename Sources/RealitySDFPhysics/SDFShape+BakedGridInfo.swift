internal import SDFCore
import simd

extension SDFShape {
    /// Read-only storage diagnostics for a baked voxel shape. These describe live grid samples,
    /// not total shape memory, peak baking memory, process footprint or an accuracy guarantee.
    public struct BakedGridInfo: Equatable, Sendable {
        /// Samples along x, y and z, including the baker's boundary samples and padding.
        /// Short mesh dimensions are cropped; this is not necessarily a cubic grid.
        public let sampleDimensions: SIMD3<Int32>
        /// Cell width in the original shape's local units, with baking normalization undone.
        /// Multiply by a body's uniform scale for simulation-root units, and by the simulation's
        /// meters-per-unit for physical meters. This value does not include an entity transform.
        public let cellSize: Float
        /// Number of stored (distance, gradient) samples.
        public let sampleCount: Int64
        /// Bytes occupied by the live grid samples. Excludes spare vector capacity, meshes,
        /// curvature seeds, solver state and temporary allocations used while baking.
        public let gridPayloadBytes: Int64
        /// Number of samples inside the shape by more than a thousandth of a cell (samples on the
        /// surface store distances of rounding size and either sign, and are not counted). Zero means no query
        /// can find penetration: typically the thinnest part of the mesh is under about one cell
        /// thick (see `minimumResolution(forThickness:longestExtent:)`). A nonzero count does not
        /// show that every thin part kept its interior. Counted on each access of `bakedGridInfo`.
        public let interiorSampleCount: Int64
    }

    /// The smallest bake resolution that puts two cells across a part `thickness` thick, in a mesh
    /// whose longest bounding-box side is `longestExtent` (same units): `ceil(2 × longestExtent /
    /// thickness)`, and at least the baker's minimum of 8. Two cells guarantee a grid node inside a
    /// slab of that thickness at any orientation (an axis-aligned slab needs just over one, an
    /// oblique one up to √3). Without an interior node a part's field stays positive and overlap
    /// queries miss it, with either sign method. Nil for nonpositive or nonfinite inputs, or when
    /// the result does not fit in an `Int32`. Grid memory grows with the cube of the resolution
    /// for a mesh that is not thin in two directions; check `bakedGridInfo` after baking.
    public static func minimumResolution(forThickness thickness: Float, longestExtent: Float) -> Int? {
        guard thickness.isFinite, thickness > 0, longestExtent.isFinite, longestExtent > 0 else { return nil }
        let cells = (2 * Double(longestExtent) / Double(thickness)).rounded(.up)
        guard cells <= Double(Int32.max) else { return nil }
        return max(8, Int(cells))
    }

    /// Actual baked grid dimensions, spacing and live sample payload. Nil for analytical shapes.
    /// Also available after loading `bakedData()`, without serializing or copying the grid.
    public var bakedGridInfo: BakedGridInfo? {
        var info = SDFBakedGridInfo()
        guard sdf_shape_baked_grid_info(handle, &info) else { return nil }
        return BakedGridInfo(
            sampleDimensions: [info.sampleDimensions.0, info.sampleDimensions.1, info.sampleDimensions.2],
            cellSize: info.cellSize, sampleCount: info.sampleCount, gridPayloadBytes: info.gridPayloadBytes,
            interiorSampleCount: info.interiorSampleCount)
    }
}
