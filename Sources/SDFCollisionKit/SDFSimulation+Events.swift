import Combine
import Foundation
import RealityKit
internal import SDFCore
import Synchronization

/// An event of an `SDFSimulation`, delivered to the handlers of `SDFSimulation.subscribe(to:on:_:)`
public protocol SDFSimulationEvent {
    /// Whether the event concerns an entity, for subscriptions to the events of one entity
    @MainActor func involves(_ entity: Entity) -> Bool
}

/// Contact events between bodies, sent after the step that made them. A pair is in contact for a
/// step when it touched in any of the step's substeps. The counterparts of `CollisionEvents`.
/// Pairs without a dynamic body get them only when the simulation's `collisionOptions` include
/// them.
public enum SDFCollisionEvents {
    /// Two bodies came into contact
    public struct Began: SDFSimulationEvent {
        public let entityA: Entity
        public let entityB: Entity
        /// The deepest contact point, in the root's frame
        public let position: SIMD3<Float>
        /// Total normal impulse of the step, pushing entity B away from entity A
        public let impulse: Float
        public let impulseDirection: SIMD3<Float>
        /// Overlap of the surfaces at the deepest contact, negative while they are apart but within
        /// the contact margin
        public let penetrationDistance: Float
        public let contacts: [SDFSimulation.Contact]

        public func involves(_ entity: Entity) -> Bool { entityA == entity || entityB == entity }
    }

    /// Two bodies stayed in contact for another step
    public struct Updated: SDFSimulationEvent {
        public let entityA: Entity
        public let entityB: Entity
        public let position: SIMD3<Float>
        public let impulse: Float
        public let impulseDirection: SIMD3<Float>
        public let penetrationDistance: Float
        public let contacts: [SDFSimulation.Contact]

        public func involves(_ entity: Entity) -> Bool { entityA == entity || entityB == entity }
    }

    /// Two bodies are no longer in contact, or one of them was removed
    public struct Ended: SDFSimulationEvent {
        public let entityA: Entity
        public let entityB: Entity

        public func involves(_ entity: Entity) -> Bool { entityA == entity || entityB == entity }
    }
}

/// Events around every simulation step. The counterparts of `PhysicsSimulationEvents`.
public enum SDFSimulationEvents {
    /// Sent before a step. Forces added now act during it.
    public struct WillSimulate: SDFSimulationEvent {
        public let deltaTime: TimeInterval
        /// The entity with the `SDFSimulationComponent`
        public let simulationEntity: Entity

        public func involves(_ entity: Entity) -> Bool { simulationEntity == entity }
    }

    /// Sent after a step, once the entities have their new poses
    public struct DidSimulate: SDFSimulationEvent {
        public let deltaTime: TimeInterval
        public let simulationEntity: Entity

        public func involves(_ entity: Entity) -> Bool { simulationEntity == entity }
    }
}

extension SDFSimulation {
    /// Calls a handler with every event of a type, or only those that involve an entity. The
    /// subscription lasts until the returned value is cancelled or released.
    ///
    /// ```swift
    /// subscription = simulation.subscribe(to: SDFCollisionEvents.Began.self, on: gear) { event in
    ///     print("\(event.entityA.name) hit \(event.entityB.name)")
    /// }
    /// ```
    public func subscribe<Event: SDFSimulationEvent>(to event: Event.Type, on entity: Entity? = nil,
                                                    _ handler: @escaping @MainActor (Event) -> Void) -> AnyCancellable {
        let subscription = EventRouter.Subscription()
        events.removeCancelled()
        events.add(Event.self, subscription) { event in
            guard let event = event as? Event, entity.map({ event.involves($0) }) ?? true else { return }
            handler(event)
        }
        return AnyCancellable(subscription)
    }

    /// Sends an event to its handlers, which may subscribe and cancel while it is sent
    func send<Event: SDFSimulationEvent>(_ event: Event) {
        events.removeCancelled(Event.self)
        let router = events
        router.send(event)
    }

    /// Sends the contact events of the last step
    func sendCollisionEvents() {
        let count = Int(sdf_world_pairs(world, nil, 0))
        var pairs = [SDFPair](repeating: SDFPair(), count: count)
        _ = pairs.withUnsafeMutableBufferPointer { sdf_world_pairs(world, $0.baseAddress, Int32(count)) }

        let wantsContacts = events.hasHandlers(for: SDFCollisionEvents.Began.self)
            || events.hasHandlers(for: SDFCollisionEvents.Updated.self)
        let contactsByPair = wantsContacts ? Dictionary(grouping: self.contacts()) { PairKey($0.entityA, $0.entityB) } : [:]

        // The new state is in place before any handler runs
        var previous = touching
        var current: [PairKey: (entityA: Entity, entityB: Entity)] = [:]
        var updates: [any SDFSimulationEvent] = []
        for pair in pairs {
            guard let a = entity(of: pair.bodyA), let b = entity(of: pair.bodyB) else { continue }
            let key = PairKey(a.id, b.id)
            let position = SIMD3(pair.position.0, pair.position.1, pair.position.2)
            let direction = SIMD3(pair.impulseDirection.0, pair.impulseDirection.1, pair.impulseDirection.2)
            let pairContacts = contactsByPair[key] ?? []
            if previous.removeValue(forKey: key) != nil {
                updates.append(SDFCollisionEvents.Updated(entityA: a, entityB: b, position: position, impulse: pair.impulse,
                                                          impulseDirection: direction, penetrationDistance: pair.penetration,
                                                          contacts: pairContacts))
            } else {
                updates.append(SDFCollisionEvents.Began(entityA: a, entityB: b, position: position, impulse: pair.impulse,
                                                        impulseDirection: direction, penetrationDistance: pair.penetration,
                                                        contacts: pairContacts))
            }
            current[key] = (a, b)
        }
        touching = current
        for event in updates {
            send(event)
        }
        for (a, b) in previous.values {
            send(SDFCollisionEvents.Ended(entityA: a, entityB: b))
        }
    }

    /// Ends the contacts of a body that is removed
    func endContacts(of id: SDFBodyID) {
        guard let entity = entity(of: id) else { return }
        for (key, pair) in touching where pair.entityA == entity || pair.entityB == entity {
            touching[key] = nil
            send(SDFCollisionEvents.Ended(entityA: pair.entityA, entityB: pair.entityB))
        }
    }
}

/// An unordered pair of entities
struct PairKey: Hashable {
    let first: Entity.ID, second: Entity.ID

    init(_ a: Entity.ID, _ b: Entity.ID) {
        (first, second) = a < b ? (a, b) : (b, a)
    }
}

/// The handlers of an `SDFSimulation`'s events, by event type
@MainActor
struct EventRouter {
    final class Subscription: Cancellable, Sendable {
        let isCancelled = Mutex(false)

        func cancel() {
            isCancelled.withLock { $0 = true }
        }

        var isActive: Bool { !isCancelled.withLock { $0 } }
    }

    private var handlers: [ObjectIdentifier: [(subscription: Subscription, handler: (any SDFSimulationEvent) -> Void)]] = [:]

    mutating func add<Event: SDFSimulationEvent>(_ type: Event.Type, _ subscription: Subscription,
                                                 _ handler: @escaping (any SDFSimulationEvent) -> Void) {
        handlers[ObjectIdentifier(type), default: []].append((subscription, handler))
    }

    mutating func hasHandlers<Event: SDFSimulationEvent>(for type: Event.Type) -> Bool {
        removeCancelled(type)
        return handlers[ObjectIdentifier(type)]?.isEmpty == false
    }

    /// Releases the handlers of cancelled subscriptions to events of a type, or of all types
    mutating func removeCancelled<Event: SDFSimulationEvent>(_ type: Event.Type) {
        handlers[ObjectIdentifier(type)]?.removeAll { !$0.subscription.isActive }
    }

    mutating func removeCancelled() {
        for key in handlers.keys {
            handlers[key]?.removeAll { !$0.subscription.isActive }
        }
    }

    func send<Event: SDFSimulationEvent>(_ event: Event) {
        // A handler may subscribe or cancel, which takes effect from the next event
        for entry in handlers[ObjectIdentifier(Event.self)] ?? [] where entry.subscription.isActive {
            entry.handler(event)
        }
    }
}
