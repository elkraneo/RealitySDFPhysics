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
    }

    /// Actual baked grid dimensions, spacing and live sample payload. Nil for analytical shapes.
    /// Also available after loading `bakedData()`, without serializing or copying the grid.
    public var bakedGridInfo: BakedGridInfo? {
        var info = SDFBakedGridInfo()
        guard sdf_shape_baked_grid_info(handle, &info) else { return nil }
        return BakedGridInfo(
            sampleDimensions: [info.sampleDimensions.0, info.sampleDimensions.1, info.sampleDimensions.2],
            cellSize: info.cellSize, sampleCount: info.sampleCount, gridPayloadBytes: info.gridPayloadBytes)
    }
}
