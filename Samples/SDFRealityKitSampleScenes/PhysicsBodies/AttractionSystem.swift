// Port of SphereAttractionComponent and SphereAttractionSystem from Apple's sample "Simulating
// physics with collisions in your visionOS app":
// https://developer.apple.com/documentation/realitykit/simulating-physics-with-collisions-in-your-visionos-app
//
// The forces are Apple's, in world space, through `entity.sdf.addForce` in place of
// `ModelEntity.addForce`. One difference: the inverse-square law assumes spheres, whose centers
// stay at least 6 cm apart (the two smallest ones touching). Concave bodies can nearly share a
// center, where the force would explode, so distances are clamped to those 6 cm. Apple's spheres
// never come closer, so for them the forces are unchanged.

import RealityKit
import RealitySDFPhysics

/// Marks a body that the other marked bodies and the center attract
package struct AttractionComponent: Component {}

package struct AttractionSystem: System {
    let entityQuery = EntityQuery(where: .has(AttractionComponent.self))
    /// The closest two of Apple's spheres can be, in meters
    let minimumDistance: Float = 0.06

    package init(scene: RealityKit.Scene) {}

    package func update(context: SceneUpdateContext) {
        let bodies = Array(context.entities(matching: entityQuery, updatingSystemWhen: .rendering))
        for body in bodies {
            // Start with a force back to the center, with the inverse-square law
            let centerForceStrength = Float(0.05)
            let position = body.position(relativeTo: nil)
            let distance = length(position)
            guard distance > 1e-6 else { continue }
            var aggregateForce = -centerForceStrength * (position / distance) / pow(max(distance, minimumDistance), 2)

            for neighbor in bodies where neighbor != body {
                let offset = neighbor.position(relativeTo: nil) - position
                let distance = length(offset)
                guard distance > 1e-6 else { continue }
                let forceFactor = Float(0.1)
                aggregateForce += forceFactor * (offset / distance) / pow(max(distance, minimumDistance), 2)
            }

            body.sdf.addForce(aggregateForce, relativeTo: nil)
        }
    }
}
