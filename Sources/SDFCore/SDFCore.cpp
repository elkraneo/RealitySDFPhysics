// Implementation of the C interface in include/SDFCore.h on top of the unmodified upstream engine

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <tuple>
#include <unordered_map>

#include "SDFCore.h"
#include "World.h"

// MARK: Shapes

// An engine shape with the frame it is used in. Baked voxel shapes are normalized to a unit cube
// centered at the origin: shape = (caller - center) * unitScale.
struct SDFShape
{
    ShapePtr shape;
    float3 center = {0, 0, 0};
    float unitScale = 1;
    Mesh display; // In the caller's frame
    bool watertight = true;
};

static float3 toFloat3(const float v[3]) { return {v[0], v[1], v[2]}; }

static void store(float out[3], float3 v)
{
    out[0] = v.x, out[1] = v.y, out[2] = v.z;
}

static SDFShape *makeShape(Shape *shape)
{
    shape->finalize();
    SDFShape *handle = new SDFShape();
    handle->shape = ShapePtr(shape);
    handle->display = shape->mesh;
    return handle;
}

extern "C" {

SDFShape *sdf_shape_box(float halfX, float halfY, float halfZ) { return makeShape(new Box({halfX, halfY, halfZ})); }
SDFShape *sdf_shape_sphere(float radius) { return makeShape(new Sphere(radius)); }
SDFShape *sdf_shape_capsule(float radius, float halfLength) { return makeShape(new Capsule(radius, halfLength)); }
SDFShape *sdf_shape_cylinder(float radius, float halfHeight) { return makeShape(new Cylinder(radius, halfHeight)); }
SDFShape *sdf_shape_cone(float radius, float halfHeight) { return makeShape(new Cone(radius, halfHeight)); }
SDFShape *sdf_shape_torus(float majorRadius, float minorRadius) { return makeShape(new Torus(majorRadius, minorRadius)); }

SDFShape *sdf_shape_spike_plate(float halfX, float halfY, float halfZ, int32_t count, float spikeRadius,
                                float minHeight, float maxHeight)
{
    return makeShape(new SpikePlate({halfX, halfY, halfZ}, std::max(1, count), spikeRadius, minHeight, maxHeight));
}

SDFShape *sdf_shape_spiky_wheel(float hubRadius, float hubHalfWidth, float spikeRadius, float spikeLength, int32_t spikes)
{
    return makeShape(new SpikyWheel(hubRadius, hubHalfWidth, spikeRadius, spikeLength, std::max(1, spikes)));
}

SDFShape *sdf_shape_warped_box(SDFWarp warp, float halfX, float halfY, float halfZ, float k)
{
    return makeShape(new WarpedBox((WarpedBox::Type)std::clamp((int)warp, 0, 2), {halfX, halfY, halfZ}, k));
}

SDFShape *sdf_shape_bake(const float *positions, int32_t vertexCount, const uint32_t *indices, int32_t indexCount,
                         int32_t resolution)
{
    if (!positions || !indices || vertexCount <= 0 || indexCount < 3 || indexCount % 3 != 0)
        return nullptr;
    AABB box = emptyAABB();
    for (int32_t i = 0; i < vertexCount; i++)
        grow(box, {positions[3 * i], positions[3 * i + 1], positions[3 * i + 2]});
    float extent = maxComponent(size(box));
    if (!(extent > 0))
        return nullptr;

    // Weld duplicated positions as upstream's OBJ loader does. Meshes split their vertices at
    // normal and texture seams, which would otherwise break the watertightness check.
    float cell = max(extent * 1e-5f, 1e-9f);
    std::map<std::tuple<int, int, int>, uint32_t> unique;
    std::vector<uint32_t> remap(vertexCount);
    Mesh mesh;
    for (int32_t i = 0; i < vertexCount; i++)
    {
        float3 p = {positions[3 * i], positions[3 * i + 1], positions[3 * i + 2]};
        float3 q = (p - box.min) / cell;
        auto key = std::make_tuple((int)lroundf(q.x), (int)lroundf(q.y), (int)lroundf(q.z));
        auto it = unique.find(key);
        if (it == unique.end())
            it = unique.emplace(key, mesh.addVertex(p, {0, 1, 0})).first;
        remap[i] = it->second;
    }
    for (int32_t i = 0; i < indexCount; i += 3)
    {
        if (indices[i] >= (uint32_t)vertexCount || indices[i + 1] >= (uint32_t)vertexCount || indices[i + 2] >= (uint32_t)vertexCount)
            return nullptr;
        uint32_t a = remap[indices[i]], b = remap[indices[i + 1]], c = remap[indices[i + 2]];
        if (a != b && b != c && c != a)
            mesh.addTriangle(a, b, c);
    }
    if (mesh.indices.empty())
        return nullptr;

    // VoxelSdf normalizes the mesh with the same center and scale
    auto voxel = std::make_shared<VoxelSdf>(mesh, "", std::max(8, resolution));
    voxel->finalize();
    SDFShape *handle = new SDFShape();
    handle->shape = voxel;
    handle->center = center(box);
    handle->unitScale = 1.0f / extent;
    handle->watertight = voxel->watertight;
    handle->display = voxel->mesh;
    for (float3 &p : handle->display.positions)
        p = p / handle->unitScale + handle->center;
    return handle;
}

void sdf_shape_release(SDFShape *shape) { delete shape; }

bool sdf_shape_is_watertight(const SDFShape *shape) { return shape->watertight; }

float sdf_shape_volume(const SDFShape *shape) { return shape->shape->volume / powf(shape->unitScale, 3); }

float sdf_shape_distance(const SDFShape *shape, const float point[3], float gradient[3])
{
    float3 g;
    float d = shape->shape->sdf((toFloat3(point) - shape->center) * shape->unitScale, &g) / shape->unitScale;
    if (gradient)
        store(gradient, g);
    return d;
}

SDFShapeMesh sdf_shape_mesh(const SDFShape *shape)
{
    static_assert(sizeof(float3) == 3 * sizeof(float), "float3 must be tightly packed");
    const Mesh &m = shape->display;
    SDFShapeMesh mesh = {};
    mesh.positions = m.positions.empty() ? nullptr : &m.positions[0].x;
    mesh.normals = m.normals.size() == m.positions.size() && !m.normals.empty() ? &m.normals[0].x : nullptr;
    mesh.indices = m.indices.empty() ? nullptr : m.indices.data();
    mesh.vertexCount = (int32_t)m.positions.size();
    mesh.indexCount = (int32_t)m.indices.size();
    return mesh;
}

} // extern "C"

// MARK: Worlds

// World position of the shape's origin for a pose of the caller's frame
static float3 shapeOrigin(float3 center, SDFPose pose, quat q)
{
    return toFloat3(pose.position) + rotate(q, center * pose.scale);
}

static quat toQuat(const float q[4]) { return normalize(quat{q[0], q[1], q[2], q[3]}); }

static SDFParams paramsOf(const SDFWorld &world)
{
    const Solver &solver = world.solver;
    const CollisionParams &c = solver.params;
    SDFParams p;
    p.optimizer = (int32_t)c.optimizer;
    p.cacheSize = c.cacheSize;
    p.randomSamples = c.randomSamples;
    p.warmStart = c.warmStart;
    p.curvatureSampling = c.curvatureSampling;
    p.skipConvexSamples = c.skipConvexSamples;
    p.epsilon = c.epsilon;
    p.alpha = c.alpha;
    p.replacementThreshold = c.replacementThreshold;
    p.tolerance = c.tolerance;
    p.maxIterations = c.maxIterations;
    p.stepSize = c.stepSize;
    p.margin = c.margin;
    p.allowedPenetration = c.allowedPenetration;
    p.substeps = solver.substeps;
    p.velocityIterations = solver.iterations;
    p.positionIterations = world.positionIterations;
    p.timeStep = solver.dt;
    store(p.gravity, solver.gravity);
    p.restitutionThreshold = world.restitutionThreshold;
    p.pairOptions = world.pairOptions;
    p.parallel = world.parallel;
    p.metersPerUnit = world.metersPerUnit;
    return p;
}

// Non-finite or negative values are ignored, they would poison the solver
static void setNonNegative(float &field, float value)
{
    if (std::isfinite(value) && value >= 0)
        field = value;
}

// Inverse inertia with the locked rotation axes removed
static float3x3 lockedInverseInertia(const BodyRecord &r)
{
    float3x3 mask = diagonal(r.angularMask);
    return mask * r.body->invInertiaWorld * mask;
}

extern "C" {

SDFWorld *sdf_world_create(void) { return new SDFWorld(); }

void sdf_world_destroy(SDFWorld *world) { delete world; }

SDFParams sdf_default_params(void)
{
    static const SDFWorld defaults;
    return paramsOf(defaults);
}

SDFParams sdf_world_params(const SDFWorld *world) { return paramsOf(*world); }

void sdf_world_set_params(SDFWorld *world, SDFParams p)
{
    Solver &solver = world->solver;
    CollisionParams &c = solver.params;
    c.optimizer = (Optimizer)std::clamp(p.optimizer, 0, (int32_t)OPTIMIZER_COUNT - 1);
    c.cacheSize = std::max(0, p.cacheSize);
    c.randomSamples = std::max(0, p.randomSamples);
    c.warmStart = p.warmStart;
    c.curvatureSampling = p.curvatureSampling;
    c.skipConvexSamples = p.skipConvexSamples;
    setNonNegative(c.epsilon, p.epsilon);
    setNonNegative(c.alpha, p.alpha);
    setNonNegative(c.replacementThreshold, p.replacementThreshold);
    setNonNegative(c.tolerance, p.tolerance);
    c.maxIterations = std::max(1, p.maxIterations);
    setNonNegative(c.stepSize, p.stepSize);
    setNonNegative(c.margin, p.margin);
    setNonNegative(c.allowedPenetration, p.allowedPenetration);
    solver.substeps = std::max(1, p.substeps);
    solver.iterations = std::max(1, p.velocityIterations);
    world->positionIterations = std::max(0, p.positionIterations);
    if (p.timeStep > 0)
        setNonNegative(solver.dt, p.timeStep);
    float3 gravity = toFloat3(p.gravity);
    if (std::isfinite(gravity.x) && std::isfinite(gravity.y) && std::isfinite(gravity.z))
        solver.gravity = gravity;
    setNonNegative(world->restitutionThreshold, p.restitutionThreshold);
    world->pairOptions = p.pairOptions;
    world->parallel = p.parallel;
    if (std::isfinite(p.metersPerUnit) && p.metersPerUnit > 0)
        world->metersPerUnit = p.metersPerUnit;
}

void sdf_world_step(SDFWorld *world)
{
    auto start = std::chrono::steady_clock::now();
    world->step();
    world->stepMilliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

void sdf_world_step_reference(SDFWorld *world) { world->solver.step(); }

SDFStats sdf_world_stats(const SDFWorld *world)
{
    const CollisionStats &s = world->solver.stats;
    SDFStats stats;
    stats.bodies = (int32_t)world->solver.bodies.size();
    stats.pairs = s.pairs;
    stats.contacts = s.contacts;
    stats.iterations = s.iterations;
    stats.queries = s.queries;
    stats.collisionMilliseconds = s.milliseconds;
    stats.stepMilliseconds = world->stepMilliseconds;
    return stats;
}

// MARK: Bodies

SDFBodyID sdf_world_add_body(SDFWorld *world, const SDFShape *shape, SDFPose pose, SDFBodyMode mode)
{
    if (!shape || !(pose.scale > 0))
        return 0;
    quat q = toQuat(pose.orientation);
    // shape = (caller - center) * unitScale, so the body scales the shape by scale / unitScale
    Body *body = new Body(shape->shape, shapeOrigin(shape->center, pose, q), q, pose.scale / shape->unitScale,
                          mode != SDFBodyDynamic);
    if (mode == SDFBodyDynamic)
    {
        // Upstream bodies have unit density, ours that of water: 1000 kg per cubic meter
        float density = 1000 * powf(world->metersPerUnit, 3);
        body->mass *= density, body->inertia = body->inertia * density;
        body->updateInertia();
    }
    world->solver.add(body);

    SDFBodyID id = world->nextID++;
    BodyRecord record;
    record.body = body, record.mode = mode, record.center = shape->center, record.unitScale = shape->unitScale;
    world->records[id] = record;
    world->ids[body] = id;
    return id;
}

void sdf_world_remove_body(SDFWorld *world, SDFBodyID id)
{
    BodyRecord *record = world->find(id);
    if (!record)
        return;
    Solver &solver = world->solver;
    Body *body = record->body;
    int index = (int)(std::find_if(solver.bodies.begin(), solver.bodies.end(), [&](auto &b) { return b.get() == body; }) -
                      solver.bodies.begin());

    // Manifolds are keyed by body indices, which shift down past the removed body. Their points and
    // body pointers stay valid, so the other pairs keep their temporal cache.
    std::map<std::pair<int, int>, Manifold> manifolds;
    for (auto &entry : solver.manifolds)
    {
        auto [a, b] = entry.first;
        if (a == index || b == index)
            continue;
        manifolds[{a - (a > index), b - (b > index)}] = std::move(entry.second);
    }
    solver.manifolds.swap(manifolds);

    if (solver.dragBody == body)
        solver.dragBody = nullptr;
    auto &pairs = world->pairs;
    pairs.erase(std::remove_if(pairs.begin(), pairs.end(), [&](const PairRecord &p) { return p.bodyA == id || p.bodyB == id; }),
                pairs.end());
    world->ids.erase(body);
    world->records.erase(id);
    solver.bodies.erase(solver.bodies.begin() + index);
}

SDFPose sdf_world_body_pose(const SDFWorld *world, SDFBodyID id)
{
    SDFPose pose = {};
    const BodyRecord *record = world->find(id);
    if (!record)
        return pose;
    const Body &body = *record->body;
    pose.scale = body.scale * record->unitScale;
    pose.orientation[0] = body.orientation.x;
    pose.orientation[1] = body.orientation.y;
    pose.orientation[2] = body.orientation.z;
    pose.orientation[3] = body.orientation.w;
    store(pose.position, body.toWorld({0, 0, 0}) - rotate(body.orientation, record->center * pose.scale));
    return pose;
}

void sdf_world_set_body_pose(SDFWorld *world, SDFBodyID id, SDFPose pose)
{
    BodyRecord *record = world->find(id);
    if (!record)
        return;
    Body &body = *record->body;
    pose.scale = body.scale * record->unitScale;
    body.orientation = toQuat(pose.orientation);
    body.position = shapeOrigin(record->center, pose, body.orientation) + rotate(body.orientation, body.shape->centerOfMass * body.scale);
    body.updateInertia();
    body.updateBounds();
}

void sdf_world_body_velocity(const SDFWorld *world, SDFBodyID id, float linear[3], float angular[3])
{
    const BodyRecord *record = world->find(id);
    store(linear, record ? record->body->velocity : float3{0, 0, 0});
    store(angular, record ? record->body->angularVelocity : float3{0, 0, 0});
}

void sdf_world_set_body_velocity(SDFWorld *world, SDFBodyID id, const float linear[3], const float angular[3])
{
    if (BodyRecord *record = world->find(id))
    {
        record->body->velocity = toFloat3(linear);
        record->body->angularVelocity = toFloat3(angular);
    }
}

float sdf_world_body_mass(const SDFWorld *world, SDFBodyID id)
{
    const BodyRecord *record = world->find(id);
    return record ? record->body->mass : 0;
}

void sdf_world_set_body_mass(SDFWorld *world, SDFBodyID id, float mass)
{
    BodyRecord *record = world->find(id);
    if (!record || record->mode != SDFBodyDynamic || !(mass > 0) || !std::isfinite(mass))
        return;
    Body &body = *record->body;
    body.inertia = body.inertia * (mass / body.mass);
    body.mass = mass;
    body.updateInertia();
}

SDFBodyOptions sdf_default_body_options(void)
{
    BodyRecord r;
    SDFBodyOptions o = {};
    o.staticFriction = r.staticFriction, o.dynamicFriction = r.dynamicFriction, o.restitution = r.restitution;
    o.linearDamping = r.linearDamping, o.angularDamping = r.angularDamping;
    o.affectedByGravity = r.affectedByGravity, o.trigger = r.trigger;
    o.group = r.group, o.mask = r.mask;
    return o;
}

void sdf_world_set_body_options(SDFWorld *world, SDFBodyID id, SDFBodyOptions o)
{
    BodyRecord *record = world->find(id);
    if (!record)
        return;
    BodyRecord &r = *record;
    setNonNegative(r.staticFriction, o.staticFriction);
    setNonNegative(r.dynamicFriction, o.dynamicFriction);
    setNonNegative(r.restitution, o.restitution);
    setNonNegative(r.linearDamping, o.linearDamping);
    setNonNegative(r.angularDamping, o.angularDamping);
    r.affectedByGravity = o.affectedByGravity;
    r.trigger = o.trigger;
    r.group = o.group, r.mask = o.mask;
    for (int i = 0; i < 3; i++)
    {
        r.linearMask[i] = o.translationLocked[i] ? 0.0f : 1.0f;
        r.angularMask[i] = o.rotationLocked[i] ? 0.0f : 1.0f;
    }
    // Upstream's solver, the reference in tests, has one friction coefficient
    r.body->friction = r.staticFriction;
}

void sdf_world_add_body_force(SDFWorld *world, SDFBodyID id, const float force[3], const float point[3])
{
    BodyRecord *record = world->find(id);
    if (!record || record->mode != SDFBodyDynamic)
        return;
    float3 f = toFloat3(force);
    record->force += f;
    if (point)
        record->torque += cross(toFloat3(point) - record->body->position, f);
}

void sdf_world_add_body_torque(SDFWorld *world, SDFBodyID id, const float torque[3])
{
    BodyRecord *record = world->find(id);
    if (record && record->mode == SDFBodyDynamic)
        record->torque += toFloat3(torque);
}

void sdf_world_clear_body_forces(SDFWorld *world, SDFBodyID id)
{
    if (BodyRecord *record = world->find(id))
        record->force = record->torque = {0, 0, 0};
}

void sdf_world_apply_impulse(SDFWorld *world, SDFBodyID id, const float impulse[3], const float point[3])
{
    BodyRecord *record = world->find(id);
    if (!record || record->mode != SDFBodyDynamic)
        return;
    Body &body = *record->body;
    float3 j = toFloat3(impulse);
    body.velocity += j * body.invMass * record->linearMask;
    if (point)
        body.angularVelocity += lockedInverseInertia(*record) * cross(toFloat3(point) - body.position, j);
}

void sdf_world_apply_angular_impulse(SDFWorld *world, SDFBodyID id, const float impulse[3])
{
    BodyRecord *record = world->find(id);
    if (record && record->mode == SDFBodyDynamic)
        record->body->angularVelocity += lockedInverseInertia(*record) * toFloat3(impulse);
}

// MARK: Contacts

int32_t sdf_world_contacts(const SDFWorld *world, SDFContact *contacts, int32_t capacity)
{
    int32_t count = 0;
    for (const auto &entry : world->solver.manifolds)
    {
        const Manifold &m = entry.second;
        for (const ContactPoint &point : m.points)
        {
            if (!point.active)
                continue;
            if (count < capacity)
            {
                SDFContact &contact = contacts[count];
                contact.bodyA = world->ids.at(m.bodyA);
                contact.bodyB = world->ids.at(m.bodyB);
                store(contact.position, point.x);
                store(contact.normal, point.normal);
                contact.separation = point.separation;
                contact.impulse = point.normalImpulse;
                contact.cached = point.cached;
            }
            count++;
        }
    }
    return count;
}

int32_t sdf_world_pairs(const SDFWorld *world, SDFPair *pairs, int32_t capacity)
{
    int32_t count = 0;
    for (const PairRecord &p : world->pairs)
    {
        if (count < capacity)
        {
            SDFPair &pair = pairs[count];
            pair.bodyA = p.bodyA, pair.bodyB = p.bodyB;
            store(pair.position, p.position);
            pair.penetration = p.penetration;
            pair.impulse = p.impulse;
            store(pair.impulseDirection, lengthSq(p.impulseVector) > 0 ? normalize(p.impulseVector) : float3{0, 0, 0});
        }
        count++;
    }
    return count;
}

// MARK: Queries

int32_t sdf_world_raycast(SDFWorld *world, const float origin[3], const float direction[3], float length, uint32_t mask,
                          SDFRayHit *hits, int32_t capacity)
{
    float3 o = toFloat3(origin), d = normalize(toFloat3(direction));
    int32_t count = 0;
    for (auto &body : world->solver.bodies)
    {
        if (!(world->record(body.get()).group & mask))
            continue;
        // Clip the ray against the bounding box, then sphere trace the SDF, like upstream's picking
        body->updateBounds();
        float t0 = 0, t1 = length;
        for (int i = 0; i < 3; i++)
        {
            if (d[i] == 0)
            {
                // Parallel to the slab: inside it all along, or never
                if (o[i] < body->bounds.min[i] || o[i] > body->bounds.max[i])
                    t1 = -1;
                continue;
            }
            float inv = 1.0f / d[i];
            float a = (body->bounds.min[i] - o[i]) * inv, b = (body->bounds.max[i] - o[i]) * inv;
            t0 = max(t0, min(a, b)), t1 = min(t1, max(a, b));
        }
        float t = t0;
        for (int i = 0; i < 256 && t <= t1; i++)
        {
            float3 gradient;
            float distance = body->sdf(o + d * t, &gradient);
            if (distance < 1e-3f * body->scale)
            {
                if (count < capacity)
                {
                    SDFRayHit &hit = hits[count];
                    hit.body = world->ids.at(body.get());
                    store(hit.position, o + d * t);
                    store(hit.normal, normalize(gradient));
                    hit.distance = t;
                }
                count++;
                break;
            }
            t += max(0.9f * distance, 1e-3f * body->scale); // Under-relaxed, since some fields are only quasi-exact
        }
    }
    return count;
}

int32_t sdf_world_overlaps(SDFWorld *world, const SDFShape *shape, SDFPose pose, float margin, uint32_t mask,
                           SDFOverlap *results, int32_t capacity)
{
    if (!shape || !(pose.scale > 0))
        return 0;
    margin = std::isfinite(margin) ? max(margin, 0.0f) : 0;
    quat q = toQuat(pose.orientation);
    Body query(shape->shape, shapeOrigin(shape->center, pose, q), q, pose.scale / shape->unitScale, true);

    // The narrow phase of a step, repeated so that the cache converges without the step to step
    // coherence it normally has. Its search volume grows by half the margin, so twice ours.
    CollisionParams params = world->solver.params;
    params.margin = 2 * margin;
    params.warmStart = true;
    params.randomSamples = std::max(params.randomSamples, 8);
    const int rounds = 4;

    int32_t count = 0;
    for (auto &body : world->solver.bodies)
    {
        if (!(world->record(body.get()).group & mask))
            continue;
        body->updateBounds();
        if (!overlaps(expand(query.bounds, margin), body->bounds))
            continue;

        Manifold manifold;
        manifold.bodyA = &query, manifold.bodyB = body.get();
        Random rng;
        CollisionStats stats;
        for (int i = 0; i < rounds; i++)
            manifold.update(params, 0, rng, stats);
        auto best = std::min_element(manifold.points.begin(), manifold.points.end(),
                                     [](const ContactPoint &a, const ContactPoint &b) { return a.separation < b.separation; });
        if (best == manifold.points.end() || best->separation > margin)
            continue;
        if (count < capacity)
        {
            SDFOverlap &overlap = results[count];
            overlap.body = world->ids.at(body.get());
            store(overlap.position, best->x);
            store(overlap.normal, best->normal);
            overlap.separation = best->separation;
        }
        count++;
    }
    return count;
}

// MARK: Picking

SDFBodyID sdf_world_begin_drag(SDFWorld *world, const float origin[3], const float direction[3], float hit[3])
{
    Solver &solver = world->solver;
    // Upstream refreshes the bounds before integrating, so they lag the poses after a step
    for (auto &body : solver.bodies)
        body->updateBounds();
    float3 point;
    solver.dragBody = solver.pick(toFloat3(origin), normalize(toFloat3(direction)), point);
    if (!solver.dragBody)
        return 0;
    solver.dragAnchor = rotate(conjugate(solver.dragBody->orientation), point - solver.dragBody->position);
    solver.dragTarget = point;
    if (hit)
        store(hit, point);
    return world->ids.at(solver.dragBody);
}

void sdf_world_drag_to(SDFWorld *world, const float target[3]) { world->solver.dragTarget = toFloat3(target); }

void sdf_world_end_drag(SDFWorld *world) { world->solver.dragBody = nullptr; }

} // extern "C"
