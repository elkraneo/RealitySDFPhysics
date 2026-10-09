// Implementation of the C interface in include/SDFCore.h on top of the unmodified upstream engine

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "SDFCore.h"
#include "scenes.h"

char sdfResourceDirectory[4096] = "";

struct SDFWorld
{
    Solver solver;
    Camera camera;
    double stepMilliseconds = 0;

    // Shapes of the bodies, indexed in order of first use
    std::vector<const Shape *> shapes;
    std::map<const Shape *, int32_t> shapeIndex;

    int32_t indexOf(const Shape *shape)
    {
        auto it = shapeIndex.find(shape);
        if (it != shapeIndex.end())
            return it->second;
        int32_t index = (int32_t)shapes.size();
        shapes.push_back(shape);
        shapeIndex[shape] = index;
        return index;
    }

    void indexShapes()
    {
        for (auto &body : solver.bodies)
            indexOf(body->shape.get());
    }
};

static float3 toFloat3(const float v[3]) { return {v[0], v[1], v[2]}; }

static void store(float out[3], float3 v)
{
    out[0] = v.x, out[1] = v.y, out[2] = v.z;
}

extern "C" {

void sdf_set_resource_directory(const char *path)
{
    std::string directory = path ? path : "";
    if (!directory.empty() && directory.back() != '/')
        directory += '/';
    strncpy(sdfResourceDirectory, directory.c_str(), sizeof(sdfResourceDirectory) - 1);
}

int32_t sdf_scene_count(void) { return sceneCount; }

const char *sdf_scene_name(int32_t index) { return index >= 0 && index < sceneCount ? scenes[index].name : ""; }

const char *sdf_scene_description(int32_t index)
{
    return index >= 0 && index < sceneCount ? scenes[index].description : "";
}

static const std::vector<std::string> &cachedShapeNames()
{
    static std::vector<std::string> names = shapeNames();
    return names;
}

int32_t sdf_shape_name_count(void) { return (int32_t)cachedShapeNames().size(); }

const char *sdf_shape_name(int32_t index)
{
    const auto &names = cachedShapeNames();
    return index >= 0 && index < (int32_t)names.size() ? names[index].c_str() : "";
}

static SDFParams paramsOf(const Solver &solver)
{
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
    p.solverIterations = solver.iterations;
    p.timeStep = solver.dt;
    return p;
}

SDFParams sdf_default_params(void)
{
    static Solver defaults;
    return paramsOf(defaults);
}

SDFWorld *sdf_world_create(void) { return new SDFWorld(); }

void sdf_world_destroy(SDFWorld *world) { delete world; }

void sdf_world_load_scene(SDFWorld *world, int32_t index)
{
    if (index < 0 || index >= sceneCount)
        return;
    world->shapes.clear();
    world->shapeIndex.clear();
    world->stepMilliseconds = 0;
    world->solver.stats = CollisionStats();
    scenes[index].build(world->solver, world->camera);
    world->indexShapes();
}

SDFCamera sdf_world_camera(const SDFWorld *world)
{
    SDFCamera camera;
    store(camera.target, world->camera.target);
    camera.distance = world->camera.distance;
    camera.yaw = world->camera.yaw;
    camera.pitch = world->camera.pitch;
    return camera;
}

void sdf_world_spawn(SDFWorld *world, const char *shape, const float position[3])
{
    Body *body = spawn(world->solver, shape, toFloat3(position), world->solver.rng.orientation());
    world->indexOf(body->shape.get());
}

void sdf_world_step(SDFWorld *world)
{
    auto start = std::chrono::steady_clock::now();
    world->solver.step();
    world->stepMilliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

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

SDFParams sdf_world_params(const SDFWorld *world) { return paramsOf(world->solver); }

// Non-finite or negative values are ignored, they would poison the solver
static void setNonNegative(float &field, float value)
{
    if (std::isfinite(value) && value >= 0)
        field = value;
}

void sdf_world_set_params(SDFWorld *world, SDFParams p)
{
    CollisionParams &c = world->solver.params;
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
    world->solver.substeps = std::max(1, p.substeps);
    world->solver.iterations = std::max(1, p.solverIterations);
    if (p.timeStep > 0)
        setNonNegative(world->solver.dt, p.timeStep);
}

int32_t sdf_world_body_count(const SDFWorld *world) { return (int32_t)world->solver.bodies.size(); }

int32_t sdf_world_body_states(const SDFWorld *world, SDFBodyState *bodies, int32_t capacity)
{
    int32_t count = (int32_t)world->solver.bodies.size();
    for (int32_t i = 0; i < std::min(count, capacity); i++)
    {
        const Body &body = *world->solver.bodies[i];
        SDFBodyState &state = bodies[i];
        store(state.origin, body.toWorld({0, 0, 0}));
        state.orientation[0] = body.orientation.x;
        state.orientation[1] = body.orientation.y;
        state.orientation[2] = body.orientation.z;
        state.orientation[3] = body.orientation.w;
        state.scale = body.scale;
        store(state.color, body.color);
        auto it = world->shapeIndex.find(body.shape.get());
        state.shape = it != world->shapeIndex.end() ? it->second : -1;
        state.visible = body.visible;
        state.isStatic = body.isStatic;
    }
    return count;
}

int32_t sdf_world_shape_count(const SDFWorld *world) { return (int32_t)world->shapes.size(); }

SDFShapeMesh sdf_world_shape_mesh(const SDFWorld *world, int32_t shape)
{
    SDFShapeMesh mesh = {};
    if (shape < 0 || shape >= (int32_t)world->shapes.size())
        return mesh;
    const Mesh &m = world->shapes[shape]->mesh;
    static_assert(sizeof(float3) == 3 * sizeof(float), "float3 must be tightly packed");
    mesh.positions = m.positions.empty() ? nullptr : &m.positions[0].x;
    mesh.normals = m.normals.size() == m.positions.size() && !m.normals.empty() ? &m.normals[0].x : nullptr;
    mesh.indices = m.indices.empty() ? nullptr : m.indices.data();
    mesh.vertexCount = (int32_t)m.positions.size();
    mesh.indexCount = (int32_t)m.indices.size();
    return mesh;
}

const char *sdf_world_shape_name(const SDFWorld *world, int32_t shape)
{
    if (shape < 0 || shape >= (int32_t)world->shapes.size())
        return "";
    if (auto voxel = dynamic_cast<const VoxelSdf *>(world->shapes[shape]))
        return voxel->name.c_str();
    return "";
}

int32_t sdf_world_contacts(const SDFWorld *world, SDFContact *contacts, int32_t capacity)
{
    int32_t count = 0;
    for (const auto &entry : world->solver.manifolds)
    {
        for (const ContactPoint &point : entry.second.points)
        {
            if (!point.active)
                continue;
            if (count < capacity)
            {
                SDFContact &contact = contacts[count];
                store(contact.position, point.x);
                store(contact.normal, point.normal);
                contact.separation = point.separation;
                contact.cached = point.cached;
            }
            count++;
        }
    }
    return count;
}

int32_t sdf_world_begin_drag(SDFWorld *world, const float origin[3], const float direction[3], float hit[3])
{
    Solver &solver = world->solver;
    // Upstream refreshes the bounds before integrating, so they lag the poses after a step
    for (auto &body : solver.bodies)
        body->updateBounds();
    float3 point;
    solver.dragBody = solver.pick(toFloat3(origin), normalize(toFloat3(direction)), point);
    if (!solver.dragBody)
        return -1;
    solver.dragAnchor = rotate(conjugate(solver.dragBody->orientation), point - solver.dragBody->position);
    solver.dragTarget = point;
    if (hit)
        store(hit, point);
    for (size_t i = 0; i < solver.bodies.size(); i++)
        if (solver.bodies[i].get() == solver.dragBody)
            return (int32_t)i;
    return -1;
}

void sdf_world_drag_to(SDFWorld *world, const float target[3]) { world->solver.dragTarget = toFloat3(target); }

void sdf_world_end_drag(SDFWorld *world) { world->solver.dragBody = nullptr; }

} // extern "C"
