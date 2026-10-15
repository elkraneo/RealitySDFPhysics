import Foundation
import RealityKit
import RealitySDFPhysics

/// A scene's own handling of the pointer and the frame, such as a crank to pedal: the viewer
/// makes one for every load of the scene and offers it each drag before the bodies and the
/// camera get it.
@MainActor
public protocol DemoInteraction: AnyObject {
    /// Called once the scene is loaded under `root`, with the simulation that steps it
    func attach(root: Entity, simulation: SDFSimulation)
    /// True when the interaction takes the drag, along a ray in the root's frame
    func beginDrag(origin: SIMD3<Float>, direction: SIMD3<Float>) -> Bool
    func moveDrag(origin: SIMD3<Float>, direction: SIMD3<Float>)
    func endDrag()
    /// Called every frame
    func update(deltaTime: TimeInterval)
    /// A line for the overlay, or nil
    var status: String? { get }
}
