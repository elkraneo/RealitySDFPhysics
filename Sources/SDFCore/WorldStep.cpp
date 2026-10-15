// Derived from upstream/solver.cpp, Copyright (c) 2026 Chris Giles, MIT License (see
// upstream/LICENSE).
//
// Our solver: upstream's Solver::step (upstream/solver.cpp, MIT, Chris Giles) with the per body
// options of BodyRecord, collision filtering and contact totals for events. With every option at
// its default it takes exactly the same steps, which SolverReferenceTests checks against
// sdf_world_step_reference.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <dispatch/dispatch.h>
#include <map>
#include <numeric>
#include <unordered_map>

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
        if (body->isStatic) // Nothing to move, and threads must not write to a shared body
            return;
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
        if (body->isStatic)
            return;
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
    PairRecord *pair; // Totals of the contact's pair over the step
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

// Finds the pairs whose bounds overlap and merges them into the manifolds
void broadPhase(SDFWorld &world, std::vector<Mover> &movers)
{
    Solver &s = world.solver;
    float r = 0.5f * s.params.margin;
    std::vector<int> order(s.bodies.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) { return s.bodies[a]->bounds.min.x < s.bodies[b]->bounds.min.x; });

    // Bounding spheres, tested after the boxes: a box and a sphere each enclose the shape, and a
    // pair must pass both. The spheres reject pairs whose boxes overlap only at their corners.
    // Only with threads: there every pair draws its own random samples, so a pair fewer changes
    // nothing for the others, whereas the reference path draws them from one sequence.
    std::vector<float3> sphereCenter(world.parallel ? s.bodies.size() : 0);
    for (size_t i = 0; i < sphereCenter.size(); i++)
        sphereCenter[i] = s.bodies[i]->toWorld(center(s.bodies[i]->shape->bounds));
    auto spheresOverlap = [&](int a, int b) {
        if (sphereCenter.empty())
            return true;
        float reach = movers[a].record->boundingRadius + movers[b].record->boundingRadius + 2 * r;
        return lengthSq(sphereCenter[a] - sphereCenter[b]) <= reach * reach;
    };

    std::vector<std::pair<int, int>> overlapping;
    for (size_t i = 0; i < order.size(); i++)
    {
        for (size_t j = i + 1; j < order.size(); j++)
        {
            if (s.bodies[order[j]]->bounds.min.x - r > s.bodies[order[i]]->bounds.max.x + r)
                break;

            std::pair<int, int> key = std::minmax(order[i], order[j]);
            if (shouldCollide(*movers[key.first].record, *movers[key.second].record, world.pairOptions) &&
                overlaps(expand(s.bodies[key.first]->bounds, r), expand(s.bodies[key.second]->bounds, r)) &&
                spheresOverlap(key.first, key.second))
                overlapping.push_back(key);
        }
    }
    std::sort(overlapping.begin(), overlapping.end());

    // Merge them into the manifolds, in place: pairs that still overlap keep their manifold, and
    // with it their cache. Pairs whose bounds parted, such as a sleeper and a support that moved
    // away in one frame, end their contacts here, before the narrow phase.
    auto drop = [&](auto it) {
        if (world.sleeping && touches(it->second))
            for (int i : {it->first.first, it->first.second})
                if (movers[i].record->asleep)
                    world.wake(*movers[i].record);
        return s.manifolds.erase(it);
    };
    auto it = s.manifolds.begin();
    for (const std::pair<int, int> &key : overlapping)
    {
        while (it != s.manifolds.end() && it->first < key)
            it = drop(it);
        if (it == s.manifolds.end() || it->first != key)
            it = s.manifolds.emplace_hint(it, key, Manifold());
        it->second.bodyA = s.bodies[key.first].get(), it->second.bodyB = s.bodies[key.second].get();
        ++it;
    }
    while (it != s.manifolds.end())
        it = drop(it);
    world.broadPhaseCurrent = true;
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
    std::vector<SDFBodyID> bodyIDs(s.bodies.size());
    for (size_t i = 0; i < s.bodies.size(); i++)
        bodyIDs[i] = ids.at(s.bodies[i].get());

    // Pairs in contact in any substep, with their impulses over the step
    std::unordered_map<uint64_t, PairRecord> touched;

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
    solverMilliseconds = 0;

    for (int substep = 0; substep < s.substeps; substep++)
    {
        // Collision detection at the current positions. The work and time add up over the
        // substeps, pairs and contacts are those of the last substep.
        for (auto &body : s.bodies)
            body->updateBounds();
        s.stats.pairs = s.stats.contacts = 0;
        for (Mover &m : movers)
            m.prepare();

        // Broad phase: sort and sweep along x over the bounding boxes grown by half the margin. While
        // nothing can move, the overlapping pairs stay those it found last.
        bool anyCanMove = std::any_of(movers.begin(), movers.end(), [](const Mover &m) { return canMove(*m.record); });
        if (anyCanMove || !broadPhaseCurrent)
            broadPhase(*this, movers);

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
                const CollisionParams *params;
                Random rng;
                CollisionStats stats;
            };
            // A pair of shapes that are high curvature everywhere, such as two tori, skips the
            // scan of their curvature seeds: those are a uniform sampling of the surfaces, and a
            // uniform sample of the overlap, the fallback, finds the contacts as well
            CollisionParams uniform = s.params;
            uniform.curvatureSampling = false;
            uint64_t seed = (uint64_t)s.rng.next() << 32 | s.rng.next();
            std::vector<Job> jobs;
            jobs.reserve(s.manifolds.size());
            for (auto &entry : s.manifolds)
            {
                if (!needsUpdate(entry.first))
                    continue;
                bool skipSeeds = movers[entry.first.first].record->uniformCurvature && movers[entry.first.second].record->uniformCurvature;
                Job job = {&entry.second, skipSeeds ? &uniform : &s.params};
                job.rng.state = mix(seed ^ mix((uint64_t)bodyIDs[entry.first.first] << 32 | bodyIDs[entry.first.second]));
                jobs.push_back(job);
            }
            Job *data = jobs.data();
            dispatch_apply(jobs.size(), DISPATCH_APPLY_AUTO, ^(size_t i) {
                data[i].manifold->update(*data[i].params, h, data[i].rng, data[i].stats);
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
        // Record the pairs in contact, with their deepest point in the latest substep they touched.
        // Pairs that weren't updated count in the stats, and their events carry the impulses that
        // held them when they fell asleep, as contacts() does.
        std::vector<PairRecord *> pairRecords;
        pairRecords.reserve(s.manifolds.size());
        for (auto &entry : s.manifolds)
        {
            const Manifold &m = entry.second;
            bool resting = !needsUpdate(entry.first);
            PairRecord *pair = nullptr;
            for (const ContactPoint &p : m.points)
            {
                if (!p.active)
                    continue;
                if (!pair)
                {
                    pair = &touched[(uint64_t)bodyIDs[entry.first.first] << 32 | bodyIDs[entry.first.second]];
                    pair->bodyA = bodyIDs[entry.first.first], pair->bodyB = bodyIDs[entry.first.second];
                    pair->penetration = -p.separation, pair->position = p.x;
                }
                else if (-p.separation > pair->penetration)
                    pair->penetration = -p.separation, pair->position = p.x;
                if (resting)
                {
                    s.stats.contacts++;
                    pair->impulse += p.normalImpulse;
                    pair->impulseVector += p.normal * p.normalImpulse;
                }
            }
            if (resting)
                s.stats.pairs++;
            pairRecords.push_back(pair);
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
        auto solverStart = std::chrono::steady_clock::now();
        std::vector<Contact> contacts;
        size_t index = 0;
        for (auto &entry : s.manifolds)
        {
            Manifold &m = entry.second;
            PairRecord *pair = pairRecords[index++];
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
                c.a = &a, c.b = &b, c.point = &p, c.pair = pair;
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
        for (auto &body : s.bodies)
            body->pseudoVelocity = body->pseudoAngularVelocity = {0, 0, 0};

        // The passes over the contacts

        auto warmStart = [](Contact &c) {
            float3 impulse = c.normal * c.normalImpulse + c.tangent[0] * c.tangentImpulse[0] + c.tangent[1] * c.tangentImpulse[1];
            c.a->applyImpulse(c.rA, -impulse);
            c.b->applyImpulse(c.rB, impulse);
        };

        // Sequential impulses (projected Gauss-Seidel)
        auto solveVelocity = [](Contact &c) {
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
        };

        // Restitution (as in Box2D): contacts that approached faster than the threshold leave with
        // the restitution of that speed
        auto solveRestitution = [&](Contact &c) {
            if (c.restitution == 0 || c.relativeVelocity > -restitutionThreshold || c.normalImpulse == 0)
                return;
            float3 dv = c.bodyB->pointVelocity(c.point->x) - c.bodyA->pointVelocity(c.point->x);
            float lambda = -(dot(dv, c.normal) + c.restitution * c.relativeVelocity) * c.normalMass;
            float previous = c.normalImpulse;
            c.normalImpulse = max(previous + lambda, 0.0f);
            float3 impulse = c.normal * (c.normalImpulse - previous);
            c.a->applyImpulse(c.rA, -impulse);
            c.b->applyImpulse(c.rB, impulse);
        };

        // Store the impulses in the manifold for warm starting the next step, and add them up per pair
        auto store = [](Contact &c) {
            c.point->normalImpulse = c.normalImpulse;
            c.point->frictionImpulse = c.tangent[0] * c.tangentImpulse[0] + c.tangent[1] * c.tangentImpulse[1];
            c.pair->impulse += c.normalImpulse;
            c.pair->impulseVector += c.normal * c.normalImpulse;
        };

        // Baumgarte stabilization with split impulses: v_n + bias >= 0 is solved for pseudo
        // velocities, which move the bodies in this substep and are then discarded
        auto solvePosition = [](Contact &c) {
            float3 dv = c.b->pseudoPointVelocity(c.rB) - c.a->pseudoPointVelocity(c.rA);
            float lambda = -(dot(dv, c.normal) + c.bias) * c.normalMass;
            float previous = c.pseudoImpulse;
            c.pseudoImpulse = max(previous + lambda, 0.0f);
            float3 impulse = c.normal * (c.pseudoImpulse - previous);
            c.a->applyPseudoImpulse(c.rA, -impulse);
            c.b->applyPseudoImpulse(c.rB, impulse);
        };

        auto solve = [&](auto &&forEach) {
            forEach(warmStart);
            for (int it = 0; it < s.iterations; it++)
                forEach(solveVelocity);
            forEach(solveRestitution);
            forEach(store);
            for (int it = 0; it < positionIterations; it++)
                forEach(solvePosition);
        };

        if (!parallel)
        {
            solve([&](auto &pass) {
                for (Contact &c : contacts)
                    pass(c);
            });
        }
        else
        {
            // Graph coloring, as in Box2D v3: the contacts of a pair are one constraint, and no
            // two constraints of a color share a body that moves, so the constraints of a color
            // can be solved at once, in any order, with the same result. Colors are solved one
            // after the other, and the constraints that fit no color before them, one by one.
            const int maxColors = 32;
            std::vector<std::pair<size_t, size_t>> groups; // Ranges of contacts, one per pair
            for (size_t i = 0; i < contacts.size();)
            {
                size_t j = i + 1;
                while (j < contacts.size() && contacts[j].pair == contacts[i].pair)
                    j++;
                groups.push_back({i, j});
                i = j;
            }
            std::vector<uint8_t> used(maxColors * movers.size(), 0);
            std::vector<int> colorOf(groups.size(), -1);
            std::vector<int> groupsPerColor(maxColors, 0);
            size_t overflowGroups = 0;
            for (size_t g = 0; g < groups.size(); g++)
            {
                const Contact &c = contacts[groups[g].first];
                size_t a = c.a - movers.data(), b = c.b - movers.data();
                bool dynamicA = !c.bodyA->isStatic, dynamicB = !c.bodyB->isStatic;
                for (int k = 0; k < maxColors; k++)
                {
                    uint8_t *bits = used.data() + k * movers.size();
                    if ((dynamicA && bits[a]) || (dynamicB && bits[b]))
                        continue;
                    if (dynamicA)
                        bits[a] = 1;
                    if (dynamicB)
                        bits[b] = 1;
                    colorOf[g] = k, groupsPerColor[k]++;
                    break;
                }
                overflowGroups += colorOf[g] < 0;
            }

            // The contacts in solving order: the overflow, then color by color, with the groups
            // of each color counted so that threads take whole groups
            std::vector<Contact> ordered;
            ordered.reserve(contacts.size());
            std::vector<std::vector<std::pair<size_t, size_t>>> colorGroups(maxColors);
            std::pair<size_t, size_t> overflow = {0, 0};
            auto append = [&](size_t g) {
                size_t begin = ordered.size();
                ordered.insert(ordered.end(), contacts.begin() + groups[g].first, contacts.begin() + groups[g].second);
                return std::make_pair(begin, ordered.size());
            };
            for (size_t g = 0; g < groups.size(); g++)
                if (colorOf[g] < 0)
                    overflow.second = append(g).second;
            for (int k = 0; k < maxColors; k++)
            {
                colorGroups[k].reserve(groupsPerColor[k]);
                for (size_t g = 0; g < groups.size(); g++)
                    if (colorOf[g] == k)
                        colorGroups[k].push_back(append(g));
            }
            contacts.swap(ordered);
            static bool trace = getenv("SDF_SOLVER_TRACE") != nullptr;
            if (trace)
            {
                static std::atomic<int> printed{0};
                if (printed++ % 120 == 0)
                {
                    fprintf(stderr, "solver: %zu contacts in %zu groups, %zu in overflow; groups per color:", contacts.size(), groups.size(), overflowGroups);
                    for (int k = 0; k < maxColors && groupsPerColor[k]; k++)
                        fprintf(stderr, " %d", groupsPerColor[k]);
                    fprintf(stderr, "\n");
                }
            }

            // One thread is enough for a few contacts; the order, and so the result, is the same.
            // SDF_SOLVER_THREADS overrides the pool's size, for measurements.
            if (!solverPool)
            {
                static int forcedThreads = getenv("SDF_SOLVER_THREADS") ? atoi(getenv("SDF_SOLVER_THREADS")) : 0;
                solverPool = std::make_unique<WorkerPool>(forcedThreads > 0 ? forcedThreads : WorkerPool::defaultThreads());
            }
            WorkerPool &pool = *solverPool;
            int threads = contacts.size() < 256 ? 1 : pool.threads();
            auto job = [&](int thread, int threadCount) {
                auto forEach = [&](auto &pass) {
                    if (thread == 0)
                        for (size_t i = overflow.first; i < overflow.second; i++)
                            pass(contacts[i]);
                    if (threadCount > 1)
                        pool.barrier();
                    for (int k = 0; k < maxColors; k++)
                    {
                        const auto &ranges = colorGroups[k];
                        if (ranges.empty())
                            break;
                        size_t from = ranges.size() * thread / threadCount, to = ranges.size() * (thread + 1) / threadCount;
                        for (size_t g = from; g < to; g++)
                            for (size_t i = ranges[g].first; i < ranges[g].second; i++)
                                pass(contacts[i]);
                        if (threadCount > 1)
                            pool.barrier();
                    }
                };
                solve(forEach);
            };
            if (threads == 1)
                job(0, 1);
            else
                pool.run(job);
        }

        solverMilliseconds += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - solverStart).count();

        // Integrate positions, with damping
        if (std::any_of(movers.begin(), movers.end(), [](const Mover &m) { return canMove(*m.record); }))
            broadPhaseCurrent = false;
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
    std::sort(pairs.begin(), pairs.end(), [](const PairRecord &a, const PairRecord &b) {
        return std::pair(a.bodyA, a.bodyB) < std::pair(b.bodyA, b.bodyB);
    });

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
    broadPhaseCurrent = false;
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
    broadPhaseCurrent = false;
    for (auto &entry : records)
        entry.second.asleep = entry.second.restedThroughStep = false, entry.second.stillTime = 0;
}
