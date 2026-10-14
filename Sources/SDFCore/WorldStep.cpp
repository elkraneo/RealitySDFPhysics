// Our solver: upstream's Solver::step (upstream/solver.cpp, MIT, Chris Giles) with the per body
// options of BodyRecord, collision filtering and contact totals for events. With every option at
// its default it takes exactly the same steps, which SDFSimulationTests checks against
// sdf_world_step_reference.

#include <algorithm>
#include <chrono>
#include <map>
#include <numeric>

#include "World.h"

namespace
{

// A body with what the solver needs of its record. Locked axes are taken out of the inverse mass
// and inertia, so that no impulse moves the body along them.
struct Mover
{
    Body *body;
    BodyRecord *record;
    bool locked;
    float3x3 invInertia; // World space, with the locked axes removed

    void prepare()
    {
        locked = record->locked() && !body->isStatic;
        invInertia = body->invInertiaWorld;
        if (locked)
        {
            float3x3 mask = diagonal(record->angularMask);
            invInertia = mask * invInertia * mask;
        }
    }

    void applyImpulse(float3 r, float3 impulse)
    {
        if (!locked)
        {
            body->velocity += impulse * body->invMass;
            body->angularVelocity += invInertia * cross(r, impulse);
        }
        else
        {
            body->velocity += impulse * body->invMass * record->linearMask;
            body->angularVelocity += invInertia * cross(r, impulse);
        }
    }

    void applyPseudoImpulse(float3 r, float3 impulse)
    {
        if (!locked)
            body->pseudoVelocity += impulse * body->invMass;
        else
            body->pseudoVelocity += impulse * body->invMass * record->linearMask;
        body->pseudoAngularVelocity += invInertia * cross(r, impulse);
    }

    float3 pseudoPointVelocity(float3 r) const { return body->pseudoVelocity + cross(body->pseudoAngularVelocity, r); }

    // Inverse mass along a direction
    float linearTerm(float3 dir) const { return locked ? body->invMass * dot(dir, dir * record->linearMask) : body->invMass; }
};

// Frictional contact constraint built from an active point of a manifold
struct Contact
{
    Mover *a, *b;
    Body *bodyA, *bodyB; // Of a and b, for the hot loops
    ContactPoint *point;
    float3 rA, rB, normal, tangent[2];
    float normalMass, tangentMass[2];
    float bias, staticFriction, dynamicFriction, restitution;
    float relativeVelocity; // Normal velocity before the solve, for restitution
    float normalImpulse, tangentImpulse[2], pseudoImpulse;
};

float effectiveMass(const Mover &a, const Mover &b, float3 rA, float3 rB, float3 dir)
{
    // At least one of the bodies is dynamic. Locks can still leave it unable to move along the
    // direction, and then the constraint has no effect.
    float3 ra = cross(rA, dir), rb = cross(rB, dir);
    float k = a.linearTerm(dir) + b.linearTerm(dir) + dot(ra, a.invInertia * ra) + dot(rb, b.invInertia * rb);
    return k > 0 ? 1.0f / k : 0.0f;
}

bool isDynamic(const BodyRecord &r) { return r.mode == SDFBodyDynamic; }

// Whether the narrow phase runs for a pair: at least one dynamic body, or a kinematic body when
// the world reports those pairs, and filters that accept each other
bool shouldCollide(const BodyRecord &a, const BodyRecord &b, uint32_t options)
{
    if (!isDynamic(a) && !isDynamic(b))
    {
        bool kinematicA = a.mode == SDFBodyKinematic, kinematicB = b.mode == SDFBodyKinematic;
        bool reported = (kinematicA && kinematicB && (options & SDFReportKinematicVsKinematic)) ||
                        (kinematicA != kinematicB && (options & SDFReportKinematicVsStatic));
        if (!reported)
            return false;
    }
    return (a.group & b.mask) && (b.group & a.mask);
}

} // namespace

void SDFWorld::step()
{
    Solver &s = solver;
    s.stats = CollisionStats();
    float h = s.dt / s.substeps;

    std::vector<Mover> movers(s.bodies.size());
    for (size_t i = 0; i < s.bodies.size(); i++)
        movers[i] = {s.bodies[i].get(), &record(s.bodies[i].get())};

    // Impulse totals of the pairs over the substeps, by body index
    std::map<std::pair<int, int>, std::pair<float, float3>> impulses;

    for (int substep = 0; substep < s.substeps; substep++)
    {
        // Collision detection at the current positions. The work and time add up over the
        // substeps, pairs and contacts are those of the last substep.
        for (auto &body : s.bodies)
            body->updateBounds();
        s.stats.pairs = s.stats.contacts = 0;
        for (Mover &m : movers)
            m.prepare();

        // Broad phase: sort and sweep along x over the bounding boxes grown by half the margin.
        // Pairs that already had a manifold keep it, and with it their cache.
        float r = 0.5f * s.params.margin;
        std::vector<int> order(s.bodies.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](int a, int b) { return s.bodies[a]->bounds.min.x < s.bodies[b]->bounds.min.x; });

        std::map<std::pair<int, int>, Manifold> overlapping;
        for (size_t i = 0; i < order.size(); i++)
        {
            for (size_t j = i + 1; j < order.size(); j++)
            {
                if (s.bodies[order[j]]->bounds.min.x - r > s.bodies[order[i]]->bounds.max.x + r)
                    break;

                std::pair<int, int> key = std::minmax(order[i], order[j]);
                Body *a = s.bodies[key.first].get(), *b = s.bodies[key.second].get();
                if (!shouldCollide(*movers[key.first].record, *movers[key.second].record, pairOptions) ||
                    !overlaps(expand(a->bounds, r), expand(b->bounds, r)))
                    continue;

                Manifold &manifold = overlapping[key];
                auto previous = s.manifolds.find(key);
                if (previous != s.manifolds.end())
                    manifold = std::move(previous->second);
                manifold.bodyA = a, manifold.bodyB = b;
            }
        }
        s.manifolds.swap(overlapping);

        // Narrow phase: Algorithm 1 for every pair
        auto start = std::chrono::steady_clock::now();
        for (auto &entry : s.manifolds)
            entry.second.update(s.params, h, s.rng, s.stats);
        s.stats.milliseconds += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

        // External forces
        for (Mover &m : movers)
        {
            Body *body = m.body;
            if (body->invMass > 0)
            {
                if (m.record->affectedByGravity)
                    body->velocity += s.gravity * h;
                if (lengthSq(m.record->force) > 0)
                    body->velocity += m.record->force * (body->invMass * h);
                if (lengthSq(m.record->torque) > 0)
                    body->angularVelocity += m.invInertia * m.record->torque * h;
                if (m.locked)
                {
                    body->velocity = body->velocity * m.record->linearMask;
                    body->angularVelocity = body->angularVelocity * m.record->angularMask;
                }
            }
        }
        if (s.dragBody)
        {
            // Pull the grabbed point towards the target with a critically damped velocity target
            Body *body = s.dragBody;
            float3 r = rotate(body->orientation, s.dragAnchor);
            float3 anchor = body->position + r;
            float3 targetVelocity = (s.dragTarget - anchor) * (0.2f / h) - body->pointVelocity(anchor) * 0.85f;
            float3x3 K = identity3() * body->invMass - skew(r) * body->invInertiaWorld * skew(r);
            float3 impulse = inverse(K) * targetVelocity;
            float maxImpulse = body->mass * 50.0f * h;
            if (length(impulse) > maxImpulse)
                impulse = impulse * (maxImpulse / length(impulse));
            movers[std::find_if(s.bodies.begin(), s.bodies.end(), [&](auto &b) { return b.get() == body; }) - s.bodies.begin()]
                .applyImpulse(r, impulse);
        }

        // Build the contact constraints and warm start them with the impulses cached in the manifold
        std::vector<Contact> contacts;
        for (auto &entry : s.manifolds)
        {
            Manifold &m = entry.second;
            Mover &a = movers[entry.first.first], &b = movers[entry.first.second];
            // Triggers and pairs without a dynamic body only report their contacts
            bool responds = !a.record->trigger && !b.record->trigger && (a.body->invMass > 0 || b.body->invMass > 0);
            for (ContactPoint &p : m.points)
            {
                if (!p.active || !responds)
                {
                    p.normalImpulse = 0, p.frictionImpulse = {0, 0, 0};
                    continue;
                }

                Contact c;
                c.a = &a, c.b = &b, c.point = &p;
                c.bodyA = a.body, c.bodyB = b.body;
                c.rA = p.x - m.bodyA->position;
                c.rB = p.x - m.bodyB->position;
                c.normal = p.normal;
                tangents(c.normal, c.tangent[0], c.tangent[1]);
                c.normalMass = effectiveMass(a, b, c.rA, c.rB, c.normal);
                for (int i = 0; i < 2; i++)
                    c.tangentMass[i] = effectiveMass(a, b, c.rA, c.rB, c.tangent[i]);
                c.staticFriction = sqrtf(a.record->staticFriction * b.record->staticFriction);
                c.dynamicFriction = sqrtf(a.record->dynamicFriction * b.record->dynamicFriction);
                c.restitution = max(a.record->restitution, b.record->restitution);
                c.relativeVelocity = dot(m.bodyB->pointVelocity(p.x) - m.bodyA->pointVelocity(p.x), c.normal);

                // The contact margin is a skin that keeps the bodies apart. Its penetration is
                // measured with the separation, the full gap (or overlap) of the surfaces. Within
                // the allowed penetration a contact only stops the bodies from approaching, deeper
                // ones are pushed apart by Baumgarte stabilization (below).
                float penetration = s.params.margin - p.separation;
                c.bias = max(-s.baumgarte * max(penetration - s.params.allowedPenetration, 0.0f) / h, -s.maxCorrection);

                c.pseudoImpulse = 0;
                c.normalImpulse = p.normalImpulse;
                c.tangentImpulse[0] = dot(p.frictionImpulse, c.tangent[0]);
                c.tangentImpulse[1] = dot(p.frictionImpulse, c.tangent[1]);
                float3 impulse = c.normal * c.normalImpulse + c.tangent[0] * c.tangentImpulse[0] + c.tangent[1] * c.tangentImpulse[1];
                a.applyImpulse(c.rA, -impulse);
                b.applyImpulse(c.rB, impulse);
                contacts.push_back(c);
            }
        }

        // Sequential impulses (projected Gauss-Seidel)
        for (int it = 0; it < s.iterations; it++)
        {
            for (Contact &c : contacts)
            {
                // Non-penetration: v_n >= 0 with a non-negative accumulated impulse
                float3 dv = c.bodyB->pointVelocity(c.point->x) - c.bodyA->pointVelocity(c.point->x);
                float lambda = -dot(dv, c.normal) * c.normalMass;
                float previous = c.normalImpulse;
                c.normalImpulse = max(previous + lambda, 0.0f);
                float3 impulse = c.normal * (c.normalImpulse - previous);
                c.a->applyImpulse(c.rA, -impulse);
                c.b->applyImpulse(c.rB, impulse);

                // Coulomb friction: sticking within the static cone, sliding at the dynamic one
                dv = c.bodyB->pointVelocity(c.point->x) - c.bodyA->pointVelocity(c.point->x);
                float t0 = c.tangentImpulse[0] - dot(dv, c.tangent[0]) * c.tangentMass[0];
                float t1 = c.tangentImpulse[1] - dot(dv, c.tangent[1]) * c.tangentMass[1];
                float limit = c.staticFriction * c.normalImpulse, len = sqrtf(t0 * t0 + t1 * t1);
                if (len > limit)
                {
                    float sliding = c.dynamicFriction * c.normalImpulse;
                    t0 *= sliding / len, t1 *= sliding / len;
                }
                impulse = c.tangent[0] * (t0 - c.tangentImpulse[0]) + c.tangent[1] * (t1 - c.tangentImpulse[1]);
                c.tangentImpulse[0] = t0, c.tangentImpulse[1] = t1;
                c.a->applyImpulse(c.rA, -impulse);
                c.b->applyImpulse(c.rB, impulse);
            }
        }

        // Restitution (as in Box2D): contacts that approached faster than the threshold leave with
        // the restitution of that speed
        for (Contact &c : contacts)
        {
            if (c.restitution == 0 || c.relativeVelocity > -restitutionThreshold || c.normalImpulse == 0)
                continue;
            float3 dv = c.bodyB->pointVelocity(c.point->x) - c.bodyA->pointVelocity(c.point->x);
            float lambda = -(dot(dv, c.normal) + c.restitution * c.relativeVelocity) * c.normalMass;
            float previous = c.normalImpulse;
            c.normalImpulse = max(previous + lambda, 0.0f);
            float3 impulse = c.normal * (c.normalImpulse - previous);
            c.a->applyImpulse(c.rA, -impulse);
            c.b->applyImpulse(c.rB, impulse);
        }

        // Store the impulses in the manifold for warm starting the next step, and add them up per pair
        for (Contact &c : contacts)
        {
            c.point->normalImpulse = c.normalImpulse;
            c.point->frictionImpulse = c.tangent[0] * c.tangentImpulse[0] + c.tangent[1] * c.tangentImpulse[1];
            auto &total = impulses[{(int)(c.a - movers.data()), (int)(c.b - movers.data())}];
            total.first += c.normalImpulse;
            total.second += c.normal * c.normalImpulse;
        }

        // Baumgarte stabilization with split impulses: v_n + bias >= 0 is solved for pseudo
        // velocities, which move the bodies in this substep and are then discarded
        for (auto &body : s.bodies)
            body->pseudoVelocity = body->pseudoAngularVelocity = {0, 0, 0};
        for (int it = 0; it < positionIterations; it++)
        {
            for (Contact &c : contacts)
            {
                float3 dv = c.b->pseudoPointVelocity(c.rB) - c.a->pseudoPointVelocity(c.rA);
                float lambda = -(dot(dv, c.normal) + c.bias) * c.normalMass;
                float previous = c.pseudoImpulse;
                c.pseudoImpulse = max(previous + lambda, 0.0f);
                float3 impulse = c.normal * (c.pseudoImpulse - previous);
                c.a->applyPseudoImpulse(c.rA, -impulse);
                c.b->applyPseudoImpulse(c.rB, impulse);
            }
        }

        // Integrate positions, with damping
        for (Mover &m : movers)
        {
            Body *body = m.body;
            if (body->isStatic)
                continue;
            body->velocity *= expf(-m.record->linearDamping * h);
            body->angularVelocity *= expf(-m.record->angularDamping * h);
            body->position += (body->velocity + body->pseudoVelocity) * h;
            body->orientation = integrate(body->orientation, body->angularVelocity + body->pseudoAngularVelocity, h);
            body->updateInertia();
        }
    }

    // Pairs touching after the last substep, with their impulses over the step
    pairs.clear();
    for (auto &entry : s.manifolds)
    {
        const Manifold &m = entry.second;
        PairRecord pair = {ids.at(m.bodyA), ids.at(m.bodyB)};
        bool touching = false;
        for (const ContactPoint &p : m.points)
        {
            if (!p.active)
                continue;
            if (!touching || -p.separation > pair.penetration)
                pair.penetration = -p.separation, pair.position = p.x;
            touching = true;
        }
        if (!touching)
            continue;
        auto total = impulses.find(entry.first);
        if (total != impulses.end())
            pair.impulse = total->second.first, pair.impulseVector = total->second.second;
        pairs.push_back(pair);
    }
}
