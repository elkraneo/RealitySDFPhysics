// Port of SphereAttractionComponent and SphereAttractionSystem from Apple's sample "Simulating
// physics with collisions in your visionOS app":
// https://developer.apple.com/documentation/realitykit/simulating-physics-with-collisions-in-your-visionos-app
//
// The forces are Apple's, in world space. Only the call changes: `entity.sdfPhysics.addForce` in
// place of `ModelEntity.addForce`.

import RealityKit
import SDFCollisionKit

/// Marks a body that the other marked bodies and the center attract
struct AttractionComponent: Component {}

struct AttractionSystem: System {
    let entityQuery = EntityQuery(where: .has(AttractionComponent.self))

    init(scene: RealityKit.Scene) {}

    func update(context: SceneUpdateContext) {
        let bodies = Array(context.entities(matching: entityQuery, updatingSystemWhen: .rendering))
        for body in bodies {
            // Start with a force back to the center, with the inverse-square law
            let centerForceStrength = Float(0.05)
            let position = body.position(relativeTo: nil)
            let distanceSquared = length_squared(position)
            guard distanceSquared > 1e-6 else { continue }
            var aggregateForce = -centerForceStrength * normalize(position) / distanceSquared

            for neighbor in bodies where neighbor != body {
                let offset = neighbor.position(relativeTo: nil) - position
                let distance = length(offset)
                // Not in Apple's sample: concave bodies, such as interlocked tori, can share a center
                guard distance > 1e-3 else { continue }
                let forceFactor = Float(0.1)
                aggregateForce += forceFactor * (offset / distance) / pow(distance, 2)
            }

            body.sdfPhysics.addForce(aggregateForce, relativeTo: nil)
        }
    }
}
