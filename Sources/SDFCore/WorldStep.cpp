// Derived from upstream/solver.cpp, Copyright (c) 2026 Chris Giles, MIT License (see
// upstream/LICENSE).
//
// Our solver: upstream's Solver::step (upstream/solver.cpp, MIT, Chris Giles) with the per body
// options of BodyRecord, collision filtering and contact totals for events. With every option at
// its default it takes exactly the same steps, which SolverReferenceTests checks against
// sdf_world_step_reference.

#include <algorithm>
#include <chrono>
#include <dispatch/dispatch.h>
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

// Whether a body can move this substep: kinematic, or dynamic and awake
bool canMove(const BodyRecord &r) { return r.mode == SDFBodyKinematic || (isDynamic(r) && !r.asleep); }

bool isMoving(const Body &b) { return lengthSq(b.velocity) > 0 || lengthSq(b.angularVelocity) > 0; }

bool touches(const Manifold &m)
{
    return std::any_of(m.points.begin(), m.points.end(), [](const ContactPoint &p) { return p.active; });
}

// SplitMix64, to derive independent random streams from a seed
uint64_t mix(uint64_t x)
{
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
}

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

    // Pairs in contact in any substep, with their impulses over the step
    std::map<std::pair<SDFBodyID, SDFBodyID>, PairRecord> touched;

    // Sleeping bodies keep zero velocities and no forces; any of them, set since the last step,
    // wakes the body
    if (sleeping)
    {
        for (Mover &m : movers)
        {
            BodyRecord &r = *m.record;
            if (r.asleep && (isMoving(*m.body) || lengthSq(r.force) > 0 || lengthSq(r.torque) > 0 || m.body == s.dragBody))
                wake(r);
        }
    }
    for (Mover &m : movers)
        m.record->restedThroughStep = m.record->asleep;

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
        // Pairs whose bounds parted, such as a sleeper and a support that moved away in one frame,
        // end their contacts here, before the narrow phase. Kept pairs were moved from, and are empty.
        if (sleeping)
        {
            for (auto &entry : overlapping)
            {
                if (!touches(entry.second))
                    continue;
                for (Body *body : {entry.second.bodyA, entry.second.bodyB})
                    if (BodyRecord &r = record(body); r.asleep)
                        wake(r);
            }
        }

        // Narrow phase: Algorithm 1 for every pair with a body that can move. The others keep the
        // contacts they had when they fell asleep.
        auto start = std::chrono::steady_clock::now();
        auto needsUpdate = [&](const std::pair<int, int> &key) {
            return canMove(*movers[key.first].record) || canMove(*movers[key.second].record);
        };
        if (!parallel)
        {
            for (auto &entry : s.manifolds)
                if (needsUpdate(entry.first))
                    entry.second.update(s.params, h, s.rng, s.stats);
        }
        else
        {
            // Pairs are independent but for upstream's single random sequence. Each pair gets its
            // own, seeded from the world's sequence and the pair's bodies, so that the results do
            // not depend on the number of threads or the order the pairs finish in.
            struct Job
            {
                Manifold *manifold;
                Random rng;
                CollisionStats stats;
            };
            uint64_t seed = (uint64_t)s.rng.next() << 32 | s.rng.next();
            std::vector<Job> jobs;
            jobs.reserve(s.manifolds.size());
            for (auto &entry : s.manifolds)
            {
                if (!needsUpdate(entry.first))
                    continue;
                Job job = {&entry.second};
                job.rng.state = mix(seed ^ mix((uint64_t)ids.at(entry.second.bodyA) << 32 | ids.at(entry.second.bodyB)));
                jobs.push_back(job);
            }
            const CollisionParams &params = s.params;
            Job *data = jobs.data();
            dispatch_apply(jobs.size(), DISPATCH_APPLY_AUTO, ^(size_t i) {
                data[i].manifold->update(params, h, data[i].rng, data[i].stats);
            });
            for (const Job &job : jobs)
            {
                s.stats.pairs += job.stats.pairs;
                s.stats.contacts += job.stats.contacts;
                s.stats.iterations += job.stats.iterations;
                s.stats.queries += job.stats.queries;
            }
        }
        s.stats.milliseconds += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        for (const auto &entry : s.manifolds)
        {
            if (needsUpdate(entry.first))
                continue;
            const Manifold &m = entry.second;
            s.stats.pairs++;
            s.stats.contacts += (int)std::count_if(m.points.begin(), m.points.end(), [](const ContactPoint &p) { return p.active; });
            // Its events carry the impulses that held it when it fell asleep, as contacts() does
            for (const ContactPoint &p : m.points)
            {
                if (!p.active)
                    continue;
                PairRecord &pair = touched[{ids.at(m.bodyA), ids.at(m.bodyB)}];
                pair.impulse += p.normalImpulse;
                pair.impulseVector += p.normal * p.normalImpulse;
            }
        }

        // A sleeping body wakes, with its island, when a dynamic body or a moving kinematic one
        // touches it
        if (sleeping)
        {
            for (auto &entry : s.manifolds)
            {
                BodyRecord &a = *movers[entry.first.first].record, &b = *movers[entry.first.second].record;
                if (a.asleep == b.asleep || a.trigger || b.trigger || !touches(entry.second))
                    continue;
                BodyRecord &sleeper = a.asleep ? a : b, &other = a.asleep ? b : a;
                if (isDynamic(other) || (other.mode == SDFBodyKinematic && isMoving(*other.body)))
                    wake(sleeper);
            }
        }

        // Record the pairs in contact, with their deepest point in the latest substep they touched
        for (auto &entry : s.manifolds)
        {
            const Manifold &m = entry.second;
            PairRecord *pair = nullptr;
            for (const ContactPoint &p : m.points)
            {
                if (!p.active)
                    continue;
                if (!pair)
                {
                    pair = &touched[{ids.at(m.bodyA), ids.at(m.bodyB)}];
                    pair->bodyA = ids.at(m.bodyA), pair->bodyB = ids.at(m.bodyB);
                    pair->penetration = -p.separation, pair->position = p.x;
                }
                else if (-p.separation > pair->penetration)
                    pair->penetration = -p.separation, pair->position = p.x;
            }
        }

        // External forces
        for (Mover &m : movers)
        {
            Body *body = m.body;
            if (body->invMass > 0 && !m.record->asleep)
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
            // Upstream limits the pull to 50 m/s², converted to the world's units
            float maxImpulse = body->mass * (50.0f / metersPerUnit) * h;
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
            // A sleeping body's pairs here touch nothing that pushes it, and keep their impulses
            // to warm start the solver when it wakes
            if (a.record->asleep || b.record->asleep)
                continue;
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
                contacts.push_back(c);
            }
        }

        // Approach velocities before any warm start, for restitution
        for (Contact &c : contacts)
            c.relativeVelocity = dot(c.bodyB->pointVelocity(c.point->x) - c.bodyA->pointVelocity(c.point->x), c.normal);

        for (Contact &c : contacts)
        {
            float3 impulse = c.normal * c.normalImpulse + c.tangent[0] * c.tangentImpulse[0] + c.tangent[1] * c.tangentImpulse[1];
            c.a->applyImpulse(c.rA, -impulse);
            c.b->applyImpulse(c.rB, impulse);
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
            PairRecord &pair = touched[{ids.at(c.bodyA), ids.at(c.bodyB)}];
            pair.impulse += c.normalImpulse;
            pair.impulseVector += c.normal * c.normalImpulse;
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
            if (body->isStatic || m.record->asleep)
                continue;
            body->velocity *= expf(-m.record->linearDamping * h);
            body->angularVelocity *= expf(-m.record->angularDamping * h);
            body->position += (body->velocity + body->pseudoVelocity) * h;
            body->orientation = integrate(body->orientation, body->angularVelocity + body->pseudoAngularVelocity, h);
            body->updateInertia();
        }
    }

    pairs.clear();
    for (auto &entry : touched)
        pairs.push_back(entry.second);

    if (sleeping)
        sleep();
}

void SDFWorld::sleep()
{
    // Nearly still: below 0.05 units and 0.05 radians per second, for half a second. Units suit
    // objects about 1 unit across, as the collision tolerances do.
    const float linearTolerance = 0.05f, angularTolerance = 0.05f, timeToSleep = 0.5f;
    Solver &s = solver;
    size_t count = s.bodies.size();
    std::vector<BodyRecord *> bodies(count);
    for (size_t i = 0; i < count; i++)
    {
        BodyRecord &r = *(bodies[i] = &record(s.bodies[i].get()));
        if (!isDynamic(r) || r.asleep)
            continue;
        const Body &b = *r.body;
        bool still = lengthSq(b.velocity) < linearTolerance * linearTolerance &&
                     lengthSq(b.angularVelocity) < angularTolerance * angularTolerance && lengthSq(r.force) == 0 &&
                     lengthSq(r.torque) == 0 && &b != s.dragBody;
        r.stillTime = still ? r.stillTime + s.dt : 0;
    }

    // Islands: awake dynamic bodies joined by their contacts. None touches a sleeping body, which
    // would have woken.
    std::vector<int> parent(count);
    std::iota(parent.begin(), parent.end(), 0);
    auto root = [&](int i) {
        while (parent[i] != i)
            i = parent[i] = parent[parent[i]];
        return i;
    };
    for (auto &entry : s.manifolds)
    {
        auto [a, b] = entry.first;
        if (isDynamic(*bodies[a]) && isDynamic(*bodies[b]) && !bodies[a]->trigger && !bodies[b]->trigger &&
            touches(entry.second))
            parent[root(a)] = root(b);
    }

    // An island sleeps when every body in it has been still long enough
    std::vector<float> stillTime(count, INFINITY);
    for (size_t i = 0; i < count; i++)
        if (isDynamic(*bodies[i]) && !bodies[i]->asleep)
            stillTime[root((int)i)] = std::min(stillTime[root((int)i)], bodies[i]->stillTime);
    std::vector<uint32_t> islands(count, 0);
    for (size_t i = 0; i < count; i++)
    {
        BodyRecord &r = *bodies[i];
        int island = root((int)i);
        if (!isDynamic(r) || r.asleep || stillTime[island] < timeToSleep)
            continue;
        if (!islands[island])
            islands[island] = nextIsland++;
        r.asleep = true, r.island = islands[island];
        r.body->velocity = r.body->angularVelocity = {0, 0, 0};
    }
}

void SDFWorld::wake(BodyRecord &r)
{
    r.stillTime = 0;
    r.restedThroughStep = false;
    if (!r.asleep)
        return;
    for (auto &entry : records)
    {
        BodyRecord &other = entry.second;
        if (other.asleep && other.island == r.island)
            other.asleep = other.restedThroughStep = false, other.stillTime = 0;
    }
}

void SDFWorld::disturb(const Body *body)
{
    wake(record(body));
    for (auto &entry : solver.manifolds)
    {
        if (entry.second.bodyA == body)
            wake(record(entry.second.bodyB));
        else if (entry.second.bodyB == body)
            wake(record(entry.second.bodyA));
    }
    const BodyRecord &r = record(body);
    AABB bounds = expand(body->bounds, solver.params.margin);
    for (auto &other : solver.bodies)
    {
        BodyRecord &o = record(other.get());
        if (other.get() != body && (r.group & o.mask) && (o.group & r.mask) &&
            overlaps(bounds, expand(other->bounds, solver.params.margin)))
            wake(o);
    }
}

void SDFWorld::wakeAll()
{
    for (auto &entry : records)
        entry.second.asleep = entry.second.restedThroughStep = false, entry.second.stillTime = 0;
}
