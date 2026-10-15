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

    void applyAngularImpulse(float3 impulse)
    {
        if (!body->isStatic)
            body->angularVelocity += invInertia * impulse;
    }

    void applyPseudoAngularImpulse(float3 impulse)
    {
        if (!body->isStatic)
            body->pseudoAngularVelocity += invInertia * impulse;
    }

    // Inverse mass along a direction
    float linearTerm(float3 dir) const { return locked ? body->invMass * dot(dir, dir * record->linearMask) : body->invMass; }

    // Inverse mass as a matrix, with the locked axes removed
    float3x3 linearMass() const { return locked ? diagonal(record->linearMask * body->invMass) : identity3() * body->invMass; }
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

// The least share of a constraint's inverse mass that locks may leave it for it to act. Locks
// make a direction near a locked axis, such as a contact normal or a hinge's row a rounding error
// off the plane of a planar chain, all but immovable: the constraint would see a huge mass, and
// shove the bodies across the plane to resolve an error along the axis they can't move on.
const float lockedShare = 0.01f;

float effectiveMass(const Mover &a, const Mover &b, float3 rA, float3 rB, float3 dir)
{
    // At least one of the bodies is dynamic. Locks can still leave it unable to move along the
    // direction, or nearly, and then the constraint has no effect.
    float3 ra = cross(rA, dir), rb = cross(rB, dir);
    float k = a.linearTerm(dir) + b.linearTerm(dir) + dot(ra, a.invInertia * ra) + dot(rb, b.invInertia * rb);
    float free = a.body->invMass + b.body->invMass + dot(ra, a.body->invInertiaWorld * ra) + dot(rb, b.body->invInertiaWorld * rb);
    return k > lockedShare * free ? 1.0f / k : 0.0f;
}

// A spring and damper on a row, in the soft step's terms (see makeSoft below). The defaults, of
// a spring with no stiffness, apply no impulse.
struct Softness
{
    float biasRate = 0, massScale = 0, impulseScale = 1;
};

// A hinge for one substep: the point constraint, the two rows that keep the axes aligned, and
// the axial rows of the spring, the motor and the limits. The position errors are corrected with pseudo
// velocities, as the contacts' penetration is.
struct Hinge
{
    JointRecord *joint;
    Mover *a, *b;
    Body *bodyA, *bodyB;
    float3 rA, rB;             // Anchors from the centers of mass, world space
    float3 axis;               // Of body A, world space
    float3 row[2];             // Perpendicular to the axis: the directions the axes must not tilt about
    float3x3 pointMass;        // Inverse of the point constraint's K
    float rowMass[2], axialMass;
    float3 pointError;         // Anchor B minus anchor A
    float rowError[2];         // Tilt of B's axis from A's about each row
    float angle;
    // The softness of the rows that hold the hinge (Box2D v3's soft step): a spring and damper
    // of a frequency and damping ratio, as a rate at which errors are corrected, a scale on the
    // effective mass and a scale on the accumulated impulse, with massScale + impulseScale = 1
    float biasRate, massScale, impulseScale;
    Softness spring;           // Of the spring's own frequency and damping ratio
};

// A slider (prismatic joint) for one substep, Box3D's prismatic joint: B's anchor held on the line
// through A's anchor along the axis, by two rows perpendicular to it; the orientations locked, by
// the hinge's two alignment rows and a twist row about the axis; and the axial rows of the spring,
// the motor and the limits. The linear rows act at B's anchor, so A's arm reaches to it.
struct Slider
{
    JointRecord *joint;
    Mover *a, *b;
    Body *bodyA, *bodyB;
    float3 rA, rB;             // From the centers of mass to B's anchor, world space
    float3 axis;               // Of body A, world space
    float3 row[2];             // Perpendicular to the axis: the linear rows, and the tilts of the axes
    float linearMass[2], axialMass;
    float rowMass[2], twistMass;
    float linearError[2];      // Of B's anchor off the line
    float rowError[2];         // Tilt of B's axis from A's about each row
    float translation, angle;
    float biasRate, massScale, impulseScale;
    Softness spring;
};

// Erin Catto's soft constraint coefficients for a frequency, a damping ratio and a substep
void makeSoft(float hertz, float zeta, float h, float &biasRate, float &massScale, float &impulseScale)
{
    float omega = 2 * 3.14159265f * hertz;
    float a1 = 2 * zeta + h * omega, a2 = h * omega * a1, a3 = 1 / (1 + a2);
    biasRate = omega / a1, massScale = a2 * a3, impulseScale = a3;
}

// A spring's softness, where no stiffness means no spring
Softness makeSoft(float hertz, float zeta, float h)
{
    Softness soft;
    if (hertz > 0)
        makeSoft(hertz, zeta, h, soft.biasRate, soft.massScale, soft.impulseScale);
    return soft;
}

// The rotation that takes axis to axisB, as an angle times a direction, measured about each of
// two rows perpendicular to axis; for a small tilt t about a row, axisB is about
// axis + t * cross(row, axis), so the tilt is what cross(axis, axisB) has along the row.
// Antiparallel axes tilt by pi about a row.
void tiltAbout(float3 axis, float3 axisB, const float3 row[2], float error[2])
{
    float3 tilt = cross(axis, axisB);
    float sine = length(tilt), cosine = dot(axis, axisB);
    tilt = sine > 1e-6f ? tilt * (atan2f(sine, cosine) / sine) : (cosine < 0 ? row[0] * 3.14159265f : float3{0, 0, 0});
    for (int i = 0; i < 2; i++)
        error[i] = dot(tilt, row[i]);
}

// An angle brought within (-pi, pi]
float wrapAngle(float angle)
{
    float turn = 2 * 3.14159265f;
    return angle - turn * roundf(angle / turn);
}

// The effective mass of a point constraint, or zero when locks leave nothing to move. Locks
// along a world axis on both bodies, such as a chain held in a plane, leave K without that axis:
// the constraint then holds along the others.
float3x3 pointMass(const Mover &a, const Mover &b, float3 rA, float3 rB)
{
    float3x3 sA = skew(rA), sB = skew(rB);
    float3x3 k = a.linearMass() + b.linearMass() + sA * a.invInertia * transpose(sA) + sB * b.invInertia * transpose(sB);
    float scale = maxComponent(abs(k[0])) + maxComponent(abs(k[1])) + maxComponent(abs(k[2]));
    float3 free = {1, 1, 1};
    for (int i = 0; i < 3; ++i)
    {
        if (!(maxComponent(abs(k[i])) > 1e-6f * scale))
        {
            k[i] = float3{0, 0, 0};
            k[i][i] = 1;
            free[i] = 0;
        }
    }
    float det = dot(k[0], cross(k[1], k[2]));
    if (!(scale > 0) || !(det > 1e-12f * scale * scale * scale))
        return identity3() * 0.0f;
    return diagonal(free) * inverse(k) * diagonal(free);
}

// The effective mass of a rotation about a direction, or zero when locks leave nothing, or
// next to nothing, to turn
float angularMass(const Mover &a, const Mover &b, float3 dir)
{
    float k = dot(dir, a.invInertia * dir) + dot(dir, b.invInertia * dir);
    float free = dot(dir, a.body->invInertiaWorld * dir) + dot(dir, b.body->invInertiaWorld * dir);
    return k > lockedShare * free ? 1.0f / k : 0.0f;
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

    // What the sweep tests, packed in its order so that it reads memory in sequence: the bounds
    // grown by half the margin, and a bounding sphere. A box and a sphere each enclose the shape,
    // and a pair must pass both; the spheres reject pairs whose boxes overlap only at their
    // corners. Only with threads: there every pair draws its own random samples, so a pair fewer
    // changes nothing for the others, whereas the reference path draws them from one sequence.
    struct Proxy
    {
        AABB box;
        float3 center;
        float radius;
        int body;
        const BodyRecord *record;
    };
    std::vector<Proxy> proxies(s.bodies.size());
    for (size_t i = 0; i < proxies.size(); i++)
    {
        const Body &body = *s.bodies[i];
        float3 center = world.parallel ? body.toWorld(::center(body.shape->bounds)) : float3{0, 0, 0};
        proxies[i] = {expand(body.bounds, r), center, movers[i].record->boundingRadius, (int)i, movers[i].record};
    }
    std::sort(proxies.begin(), proxies.end(), [](const Proxy &a, const Proxy &b) { return a.box.min.x < b.box.min.x; });

    // Pairs joined by a joint that keeps them from colliding
    std::vector<std::pair<int, int>> joined;
    if (!world.joints.empty())
    {
        std::unordered_map<const Body *, int> indexOf;
        for (size_t i = 0; i < s.bodies.size(); i++)
            indexOf[s.bodies[i].get()] = (int)i;
        for (const JointRecord &joint : world.joints)
            if (!joint.collideConnected)
                joined.push_back(std::minmax(indexOf.at(joint.a), indexOf.at(joint.b)));
        std::sort(joined.begin(), joined.end());
    }

    std::vector<std::pair<int, int>> overlapping;
    for (size_t i = 0; i < proxies.size(); i++)
    {
        const Proxy &a = proxies[i];
        for (size_t j = i + 1; j < proxies.size() && proxies[j].box.min.x <= a.box.max.x; j++)
        {
            const Proxy &b = proxies[j];
            if (!overlaps(a.box, b.box))
                continue;
            if (world.parallel)
            {
                float reach = a.radius + b.radius + 2 * r;
                if (lengthSq(a.center - b.center) > reach * reach)
                    continue;
            }
            std::pair<int, int> key = std::minmax(a.body, b.body);
            if (shouldCollide(*a.record, *b.record, world.pairOptions) &&
                (joined.empty() || !std::binary_search(joined.begin(), joined.end(), key)))
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

float jointAngle(const JointRecord &joint)
{
    float3 axis = rotate(joint.a->orientation, joint.axisA);
    float3 nA = rotate(joint.a->orientation, joint.normalA), nB = rotate(joint.b->orientation, joint.normalB);
    return atan2f(dot(cross(nA, nB), axis), dot(nA, nB));
}

float jointTranslation(const JointRecord &joint)
{
    float3 d = (joint.b->position + rotate(joint.b->orientation, joint.anchorB)) -
               (joint.a->position + rotate(joint.a->orientation, joint.anchorA));
    return dot(d, rotate(joint.a->orientation, joint.axisA));
}

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

        // A joint wakes a sleeper when the other body moves
        if (sleeping)
        {
            for (JointRecord &joint : joints)
            {
                BodyRecord &a = record(joint.a), &b = record(joint.b);
                const BodyRecord &awake = a.asleep ? b : a;
                if (a.asleep != b.asleep && canMove(awake) && (isDynamic(awake) || isMoving(*awake.body)))
                    wake(a.asleep ? a : b);
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

        // Build the joints. Those of sleeping pairs rest, keeping their impulses.
        std::vector<Hinge> hinges;
        std::vector<Slider> sliders;
        for (JointRecord &joint : joints)
        {
            BodyRecord &ra = record(joint.a), &rb = record(joint.b);
            if (ra.asleep || rb.asleep)
                continue;
            auto indexOf = [&](const Body *body) {
                return (int)(std::find_if(s.bodies.begin(), s.bodies.end(), [&](auto &p) { return p.get() == body; }) - s.bodies.begin());
            };
            Mover *a = &movers[indexOf(joint.a)], *b = &movers[indexOf(joint.b)];
            float3 rA = rotate(joint.a->orientation, joint.anchorA), rB = rotate(joint.b->orientation, joint.anchorB);
            float3 axis = rotate(joint.a->orientation, joint.axisA);
            float3 axisB = rotate(joint.b->orientation, joint.axisB);
            // At most half the substep rate. Box3D caps at a quarter with one solve per substep;
            // swept to convergence the rows take a half, and a hanging chain sags a quarter as much.
            float biasRate, massScale, impulseScale;
            makeSoft(min(jointHertz, 0.5f / h), jointDampingRatio, h, biasRate, massScale, impulseScale);
            Softness spring = joint.hasSpring ? makeSoft(joint.springHertz, joint.springDampingRatio, h) : Softness();

            if (joint.kind == JointRecord::prismatic)
            {
                Slider j;
                j.joint = &joint;
                j.a = a, j.b = b;
                j.bodyA = joint.a, j.bodyB = joint.b;
                float3 d = (joint.b->position + rB) - (joint.a->position + rA);
                j.rA = rA + d, j.rB = rB;
                j.axis = axis;
                tangents(j.axis, j.row[0], j.row[1]);
                for (int i = 0; i < 2; i++)
                {
                    j.linearMass[i] = effectiveMass(*a, *b, j.rA, j.rB, j.row[i]);
                    j.rowMass[i] = angularMass(*a, *b, j.row[i]);
                    j.linearError[i] = dot(d, j.row[i]);
                }
                j.axialMass = effectiveMass(*a, *b, j.rA, j.rB, j.axis);
                j.twistMass = angularMass(*a, *b, j.axis);
                tiltAbout(j.axis, axisB, j.row, j.rowError);
                j.translation = dot(d, j.axis);
                j.angle = jointAngle(joint);
                j.biasRate = biasRate, j.massScale = massScale, j.impulseScale = impulseScale;
                j.spring = spring;
                sliders.push_back(j);
                continue;
            }

            Hinge j;
            j.joint = &joint;
            j.a = a, j.b = b;
            j.bodyA = joint.a, j.bodyB = joint.b;
            j.rA = rA, j.rB = rB;
            j.axis = axis;
            tangents(j.axis, j.row[0], j.row[1]);
            j.pointMass = pointMass(*j.a, *j.b, j.rA, j.rB);
            for (int i = 0; i < 2; i++)
                j.rowMass[i] = angularMass(*j.a, *j.b, j.row[i]);
            j.axialMass = angularMass(*j.a, *j.b, j.axis);
            j.pointError = (joint.b->position + j.rB) - (joint.a->position + j.rA);
            tiltAbout(j.axis, axisB, j.row, j.rowError);
            j.angle = jointAngle(joint);
            j.biasRate = biasRate, j.massScale = massScale, j.impulseScale = impulseScale;
            j.spring = spring;
            hinges.push_back(j);
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

        // The passes over the joints. Each pass runs over the joints after the contacts.

        auto warmStartHinge = [](Hinge &j) {
            JointRecord &joint = *j.joint;
            j.a->applyImpulse(j.rA, -joint.linearImpulse);
            j.b->applyImpulse(j.rB, joint.linearImpulse);
            // The alignment rows' impulse, carried as a vector so that it survives the rows' turn
            joint.angularImpulse = joint.angularImpulse - j.axis * dot(joint.angularImpulse, j.axis);
            if (!joint.hasLimit)
                joint.lowerImpulse = joint.upperImpulse = 0;
            if (!joint.hasMotor)
                joint.motorImpulse = 0;
            if (!joint.hasSpring)
                joint.springImpulse = 0;
            float3 angular = joint.angularImpulse + j.axis * (joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse);
            j.a->applyAngularImpulse(-angular);
            j.b->applyAngularImpulse(angular);
        };

        // The hinge's rows, as Box3D solves a revolute joint (revolute_joint.c): the spring soft by
        // its own frequency and damping, pulling the angle toward its target; the motor rigid; the limits
        // speculative when not reached, soft when reached; the alignment rows and the point
        // constraint soft, pulling toward zero error. Box3D applies the rows once per substep and
        // once more without bias after the positions move; here they are swept to convergence
        // within the iterations, which the relax pass then only disturbed, so there is none.
        auto solveHinge = [&](Hinge &j, bool useBias) {
            JointRecord &joint = *j.joint;
            Body &A = *j.bodyA, &B = *j.bodyB;
            float massScale = useBias ? j.massScale : 1, impulseScale = useBias ? j.impulseScale : 0;

            if (joint.hasSpring)
            {
                // The way to the target the short way round, which the spring takes always soft
                float c = wrapAngle(j.angle - joint.springTarget);
                float cdot = dot(j.axis, B.angularVelocity - A.angularVelocity);
                float lambda = -j.spring.massScale * j.axialMass * (cdot + j.spring.biasRate * c) - j.spring.impulseScale * joint.springImpulse;
                joint.springImpulse += lambda;
                j.a->applyAngularImpulse(-j.axis * lambda);
                j.b->applyAngularImpulse(j.axis * lambda);
            }

            if (joint.hasMotor)
            {
                float cdot = dot(j.axis, B.angularVelocity - A.angularVelocity) - joint.motorSpeed;
                float previous = joint.motorImpulse, limit = joint.maxMotorForce * h;
                joint.motorImpulse = clamp(previous - cdot * j.axialMass, -limit, limit);
                float3 impulse = j.axis * (joint.motorImpulse - previous);
                j.a->applyAngularImpulse(-impulse);
                j.b->applyAngularImpulse(impulse);
            }

            if (joint.hasLimit)
            {
                // The angle measured about the middle of the limits, so that its wrap at a half
                // turn falls as far from both stops as it can
                float middle = 0.5f * (joint.lower + joint.upper), turn = 2 * 3.14159265f;
                float angle = j.angle - middle;
                angle = angle - turn * roundf(angle / turn) + middle;
                // Lower: the angle stays at or above it. Away from it, only an approach that
                // would cross it within the substep is stopped.
                {
                    float c = angle - joint.lower, bias = 0, ms = 1, is = 0;
                    if (c > 0)
                        bias = c / h;
                    else if (useBias)
                        bias = j.biasRate * c, ms = j.massScale, is = j.impulseScale;
                    float cdot = dot(j.axis, B.angularVelocity - A.angularVelocity);
                    float previous = joint.lowerImpulse;
                    joint.lowerImpulse = max(previous - ms * j.axialMass * (cdot + bias) - is * previous, 0.0f);
                    float3 impulse = j.axis * (joint.lowerImpulse - previous);
                    j.a->applyAngularImpulse(-impulse);
                    j.b->applyAngularImpulse(impulse);
                }
                {
                    float c = joint.upper - angle, bias = 0, ms = 1, is = 0;
                    if (c > 0)
                        bias = c / h;
                    else if (useBias)
                        bias = j.biasRate * c, ms = j.massScale, is = j.impulseScale;
                    float cdot = dot(j.axis, A.angularVelocity - B.angularVelocity);
                    float previous = joint.upperImpulse;
                    joint.upperImpulse = max(previous - ms * j.axialMass * (cdot + bias) - is * previous, 0.0f);
                    float3 impulse = j.axis * (joint.upperImpulse - previous);
                    j.a->applyAngularImpulse(impulse);
                    j.b->applyAngularImpulse(-impulse);
                }
            }

            // The axes stay aligned
            for (int i = 0; i < 2; i++)
            {
                float cdot = dot(j.row[i], B.angularVelocity - A.angularVelocity);
                float bias = useBias ? j.biasRate * j.rowError[i] : 0;
                float accumulated = dot(joint.angularImpulse, j.row[i]);
                float lambda = -massScale * j.rowMass[i] * (cdot + bias) - impulseScale * accumulated;
                float3 impulse = j.row[i] * lambda;
                joint.angularImpulse += impulse;
                j.a->applyAngularImpulse(-impulse);
                j.b->applyAngularImpulse(impulse);
            }

            // The anchors move together
            float3 cdot = (B.velocity + cross(B.angularVelocity, j.rB)) - (A.velocity + cross(A.angularVelocity, j.rA));
            float3 bias = useBias ? j.pointError * j.biasRate : float3{0, 0, 0};
            float3 lambda = (j.pointMass * (cdot + bias)) * -massScale - joint.linearImpulse * impulseScale;
            joint.linearImpulse += lambda;
            j.a->applyImpulse(j.rA, -lambda);
            j.b->applyImpulse(j.rB, lambda);
        };

        // A slider's linear rows act along a direction at B's anchor, on A at the point that
        // coincides with it
        auto sliderVelocity = [](const Slider &j, float3 dir) {
            const Body &A = *j.bodyA, &B = *j.bodyB;
            return dot((B.velocity + cross(B.angularVelocity, j.rB)) - (A.velocity + cross(A.angularVelocity, j.rA)), dir);
        };
        auto pushSlider = [](Slider &j, float3 impulse) {
            j.a->applyImpulse(j.rA, -impulse);
            j.b->applyImpulse(j.rB, impulse);
        };

        auto warmStartSlider = [&](Slider &j) {
            JointRecord &joint = *j.joint;
            if (!joint.hasLimit)
                joint.lowerImpulse = joint.upperImpulse = 0;
            if (!joint.hasMotor)
                joint.motorImpulse = 0;
            if (!joint.hasSpring)
                joint.springImpulse = 0;
            // The perpendicular rows' impulse, carried as a vector as the hinge's alignment is
            joint.linearImpulse = joint.linearImpulse - j.axis * dot(joint.linearImpulse, j.axis);
            pushSlider(j, joint.linearImpulse + j.axis * (joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse));
            j.a->applyAngularImpulse(-joint.angularImpulse);
            j.b->applyAngularImpulse(joint.angularImpulse);
        };

        // The slider's rows, as Box3D solves a prismatic joint (prismatic_joint.c), in its order:
        // the spring, the motor and the limits along the axis, as the hinge's about its axis; the
        // orientations, by the hinge's alignment rows and a twist row; and B's anchor on the line.
        // Box3D locks the orientations with one 3 by 3 block; rows, as the hinge's, keep each lock
        // that leaves a direction all but immovable out of the others.
        auto solveSlider = [&](Slider &j) {
            JointRecord &joint = *j.joint;
            Body &A = *j.bodyA, &B = *j.bodyB;

            if (joint.hasSpring)
            {
                float c = j.translation - joint.springTarget;
                float cdot = sliderVelocity(j, j.axis);
                float lambda = -j.spring.massScale * j.axialMass * (cdot + j.spring.biasRate * c) - j.spring.impulseScale * joint.springImpulse;
                joint.springImpulse += lambda;
                pushSlider(j, j.axis * lambda);
            }

            if (joint.hasMotor)
            {
                float cdot = sliderVelocity(j, j.axis) - joint.motorSpeed;
                float previous = joint.motorImpulse, limit = joint.maxMotorForce * h;
                joint.motorImpulse = clamp(previous - cdot * j.axialMass, -limit, limit);
                pushSlider(j, j.axis * (joint.motorImpulse - previous));
            }

            if (joint.hasLimit)
            {
                // Speculative when not reached, soft when reached
                {
                    float c = j.translation - joint.lower, bias = 0, ms = 1, is = 0;
                    if (c > 0)
                        bias = c / h;
                    else
                        bias = j.biasRate * c, ms = j.massScale, is = j.impulseScale;
                    float cdot = sliderVelocity(j, j.axis);
                    float previous = joint.lowerImpulse;
                    joint.lowerImpulse = max(previous - ms * j.axialMass * (cdot + bias) - is * previous, 0.0f);
                    pushSlider(j, j.axis * (joint.lowerImpulse - previous));
                }
                {
                    float c = joint.upper - j.translation, bias = 0, ms = 1, is = 0;
                    if (c > 0)
                        bias = c / h;
                    else
                        bias = j.biasRate * c, ms = j.massScale, is = j.impulseScale;
                    float cdot = -sliderVelocity(j, j.axis);
                    float previous = joint.upperImpulse;
                    joint.upperImpulse = max(previous - ms * j.axialMass * (cdot + bias) - is * previous, 0.0f);
                    pushSlider(j, j.axis * (previous - joint.upperImpulse));
                }
            }

            // The orientations stay locked: the axes aligned, and no twist about them
            for (int i = 0; i < 3; i++)
            {
                float3 dir = i < 2 ? j.row[i] : j.axis;
                float mass = i < 2 ? j.rowMass[i] : j.twistMass, error = i < 2 ? j.rowError[i] : j.angle;
                float cdot = dot(dir, B.angularVelocity - A.angularVelocity);
                float accumulated = dot(joint.angularImpulse, dir);
                float lambda = -j.massScale * mass * (cdot + j.biasRate * error) - j.impulseScale * accumulated;
                float3 impulse = dir * lambda;
                joint.angularImpulse += impulse;
                j.a->applyAngularImpulse(-impulse);
                j.b->applyAngularImpulse(impulse);
            }

            // B's anchor stays on the line
            for (int i = 0; i < 2; i++)
            {
                float cdot = sliderVelocity(j, j.row[i]);
                float accumulated = dot(joint.linearImpulse, j.row[i]);
                float lambda = -j.massScale * j.linearMass[i] * (cdot + j.biasRate * j.linearError[i]) - j.impulseScale * accumulated;
                joint.linearImpulse += j.row[i] * lambda;
                pushSlider(j, j.row[i] * lambda);
            }
        };

        // Every pass takes the contacts, then the joints (hinges and sliders), which so have the last word of each
        // iteration: a jammed chain's contacts must not pull its pins apart. A hinge is cheap
        // next to a contact and a chain of them converges slowly, so each pass sweeps the hinges
        // several times.
        // The hinge passes take the whole set: the warm start once, the solves swept
        const int hingeSweeps = 4;
        auto warmStartHinges = [&] {
            for (Hinge &j : hinges)
                warmStartHinge(j);
            for (Slider &j : sliders)
                warmStartSlider(j);
        };
        auto solveHingeVelocities = [&] {
            for (int i = 0; i < hingeSweeps; i++)
            {
                for (Hinge &j : hinges)
                    solveHinge(j, true);
                for (Slider &j : sliders)
                    solveSlider(j);
            }
        };
        auto noHinges = [] {};
        auto solve = [&](auto &&forEach) {
            forEach(warmStartHinges, warmStart);
            for (int it = 0; it < s.iterations; it++)
                forEach(solveHingeVelocities, solveVelocity);
            forEach(noHinges, solveRestitution);
            forEach(noHinges, store);
            for (int it = 0; it < positionIterations; it++)
                forEach(noHinges, solvePosition);
        };

        if (!parallel)
        {
            solve([&](auto &hingePass, auto &pass) {
                for (Contact &c : contacts)
                    pass(c);
                hingePass();
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
                auto forEach = [&](auto &hingePass, auto &pass) {
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
                    // The joints after every color, on one thread, with the others waiting
                    if (!hinges.empty() || !sliders.empty())
                    {
                        if (thread == 0)
                            hingePass();
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
            // No more than half a turn per substep, so that a runaway stays finite: 377 rad/s at
            // 2 substeps of 60 Hz, well past any wheel (Box3D clamps at an eighth of a turn)
            float spin = length(body->angularVelocity), maxSpin = 3.14159265f / h;
            if (spin > maxSpin)
                body->angularVelocity *= maxSpin / spin;
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
    // A running motor keeps its bodies awake
    for (const JointRecord &joint : joints)
        if (joint.hasMotor && joint.motorSpeed != 0)
            record(joint.a).stillTime = record(joint.b).stillTime = 0;

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
    if (!joints.empty())
    {
        std::unordered_map<const Body *, int> indexOf;
        for (size_t i = 0; i < count; i++)
            indexOf[s.bodies[i].get()] = (int)i;
        for (const JointRecord &joint : joints)
        {
            int a = indexOf.at(joint.a), b = indexOf.at(joint.b);
            if (isDynamic(*bodies[a]) && isDynamic(*bodies[b]))
                parent[root(a)] = root(b);
        }
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
    for (const JointRecord &joint : joints)
    {
        if (joint.a == body)
            wake(record(joint.b));
        else if (joint.b == body)
            wake(record(joint.a));
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
