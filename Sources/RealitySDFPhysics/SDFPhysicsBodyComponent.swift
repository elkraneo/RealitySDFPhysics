import RealityKit
internal import SDFCore

/// Makes an entity a rigid body of the nearest ancestor with an `SDFPhysicsSimulationComponent`. The
/// counterpart of `PhysicsBodyComponent` and `CollisionComponent` together: the shape is part of
/// the body.
///
/// The body is created on the simulation's next update, and removed with the entity or the
/// component. Changing the shape, mode or the entity's scale recreates it; every other property
/// updates in place. The entity's scale must be uniform.
public struct SDFPhysicsBodyComponent: Component {
    /// The body's geometry, in the entity's frame
    public var shape: SDFShape

    /// `.dynamic` bodies are moved by the simulation, which writes their entity's transform.
    /// `.kinematic` bodies are moved by you through the transform, and push dynamic bodies with the
    /// velocity of that motion. `.static` bodies do not move, or teleport when you set the transform.
    public var mode: PhysicsBodyMode

    /// Mass of a dynamic body. By default that of the shape's volume of water.
    public var massProperties: SDFPhysicsMassProperties

    public var material: SDFPhysicsMaterial

    /// Axes of the simulation root's frame along which a dynamic body does not move
    public var isTranslationLocked: (x: Bool, y: Bool, z: Bool)

    /// Axes of the simulation root's frame around which a dynamic body does not rotate
    public var isRotationLocked: (x: Bool, y: Bool, z: Bool)

    public var isAffectedByGravity: Bool

    /// Exponential damping of the velocities, per second
    public var linearDamping: Float
    public var angularDamping: Float

    /// The groups the body is in and those it collides with. Two bodies collide when each one's
    /// group is in the other's mask.
    public var filter: CollisionFilter

    /// A trigger detects contacts, reported as collision events, without responding to them.
    /// The counterpart of `CollisionComponent.Mode.trigger`.
    public var isTrigger: Bool

    public init(shape: SDFShape, massProperties: SDFPhysicsMassProperties = .default, material: SDFPhysicsMaterial = .default,
                mode: PhysicsBodyMode = .dynamic) {
        _ = registration
        self.shape = shape
        self.mode = mode
        self.massProperties = massProperties
        self.material = material
        let defaults = sdf_default_body_options()
        isTranslationLocked = (false, false, false)
        isRotationLocked = (false, false, false)
        isAffectedByGravity = defaults.affectedByGravity
        linearDamping = defaults.linearDamping
        angularDamping = defaults.angularDamping
        filter = .default
        isTrigger = false
    }

    /// The engine options, which the simulation compares to update bodies in place
    var options: Options {
        Options(material: material, isTranslationLocked: [isTranslationLocked.x, isTranslationLocked.y, isTranslationLocked.z],
                isRotationLocked: [isRotationLocked.x, isRotationLocked.y, isRotationLocked.z],
                isAffectedByGravity: isAffectedByGravity, linearDamping: linearDamping, angularDamping: angularDamping,
                filter: filter, isTrigger: isTrigger)
    }

    struct Options: Equatable {
        var material: SDFPhysicsMaterial
        var isTranslationLocked: [Bool]
        var isRotationLocked: [Bool]
        var isAffectedByGravity: Bool
        var linearDamping: Float
        var angularDamping: Float
        var filter: CollisionFilter
        var isTrigger: Bool

        var engine: SDFBodyOptions {
            var o = sdf_default_body_options()
            o.staticFriction = material.staticFriction
            o.dynamicFriction = material.dynamicFriction
            o.restitution = material.restitution
            o.linearDamping = linearDamping
            o.angularDamping = angularDamping
            o.affectedByGravity = isAffectedByGravity
            o.trigger = isTrigger
            o.group = filter.group.rawValue
            o.mask = filter.mask.rawValue
            o.translationLocked = (isTranslationLocked[0], isTranslationLocked[1], isTranslationLocked[2])
            o.rotationLocked = (isRotationLocked[0], isRotationLocked[1], isRotationLocked[2])
            return o
        }
    }
}

/// Friction and bounce of a body. The counterpart of `PhysicsMaterialResource`.
///
/// The friction of a contact is the geometric mean of the two bodies', and its restitution the
/// larger of the two.
public struct SDFPhysicsMaterial: Equatable, Sendable {
    /// Friction coefficient while the surfaces stick
    public var staticFriction: Float
    /// Friction coefficient while they slide
    public var dynamicFriction: Float
    /// Fraction of the approach speed a contact bounces back with, from 0 to 1
    public var restitution: Float

    public init(staticFriction: Float, dynamicFriction: Float, restitution: Float) {
        self.staticFriction = staticFriction
        self.dynamicFriction = dynamicFriction
        self.restitution = restitution
    }

    public init(friction: Float, restitution: Float = 0) {
        self.init(staticFriction: friction, dynamicFriction: friction, restitution: restitution)
    }

    /// Friction 0.5 and no bounce, as in the paper
    public static let `default` = SDFPhysicsMaterial(friction: 0.5)
}

/// Mass of a dynamic body. Its center of mass and inertia always come from the shape, as for a
/// uniform density. The counterpart of `PhysicsMassProperties`.
public struct SDFPhysicsMassProperties: Equatable, Sendable {
    enum Value: Equatable {
        case water
        case density(Float)
        case mass(Float)
    }

    let value: Value

    private init(_ value: Value) {
        self.value = value
    }

    /// A total mass, in kilograms
    public init(mass: Float) {
        value = .mass(mass)
    }

    /// Mass per unit volume of the root, in kilograms. In a root whose unit is a centimeter, 1 g
    /// per cubic centimeter is 0.001.
    public init(density: Float) {
        value = .density(density)
    }

    /// The density of water, 1000 kg per cubic meter, whatever the root's unit
    public static let `default` = SDFPhysicsMassProperties(.water)

    /// The mass of a shape at a scale, in a root whose unit is `metersPerUnit` long
    func mass(of shape: SDFShape, scale: Float, metersPerUnit: Float) -> Float {
        let volume = shape.volume * scale * scale * scale
        return switch value {
        case .water: 1000 * pow(metersPerUnit, 3) * volume
        case .mass(let mass): mass
        case .density(let density): density * volume
        }
    }
}

/// Velocities of a dynamic body, in the simulation root's frame. The counterpart of
/// `PhysicsMotionComponent`.
///
/// The simulation writes the body's velocities to this component after every step. Changes you make
/// are applied to the body before the next step.
public struct SDFPhysicsMotionComponent: Component, Equatable {
    public var linearVelocity: SIMD3<Float>
    public var angularVelocity: SIMD3<Float>

    public init(linearVelocity: SIMD3<Float> = .zero, angularVelocity: SIMD3<Float> = .zero) {
        _ = registration
        self.linearVelocity = linearVelocity
        self.angularVelocity = angularVelocity
    }
}
