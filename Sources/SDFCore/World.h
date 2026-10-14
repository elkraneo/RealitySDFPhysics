// The world behind the C interface: upstream's bodies, manifolds and collision detection, stepped
// by our solver (WorldStep.cpp) so that bodies can have the options of a game physics engine.

#pragma once

#include <unordered_map>
#include <vector>

#include "SDFCore.h"
#include "solver.h"

// Per body state the upstream Body has no fields for. The defaults reproduce upstream's solver.
struct BodyRecord
{
    Body *body;
    SDFBodyMode mode;
    float3 center;      // Of the shape, see SDFShape in SDFCore.cpp
    float unitScale;

    float staticFriction = 0.5f, dynamicFriction = 0.5f, restitution = 0;
    float linearDamping = 0.05f, angularDamping = 0.3f;
    bool affectedByGravity = true;
    bool trigger = false;              // Detects contacts without responding to them
    uint32_t group = 1, mask = UINT32_MAX;
    float3 linearMask = {1, 1, 1};     // 0 locks the axis of the simulation frame
    float3 angularMask = {1, 1, 1};
    float3 force = {0, 0, 0};          // Applied in every step until cleared
    float3 torque = {0, 0, 0};

    bool locked() const { return min(min(linearMask.x, linearMask.y), min(linearMask.z, min(min(angularMask.x, angularMask.y), angularMask.z))) < 1; }
};

// Contact totals of a touching pair over the last step, for collision events
struct PairRecord
{
    SDFBodyID bodyA, bodyB;
    float impulse = 0;
    float3 impulseVector = {0, 0, 0};
    float penetration = 0;
    float3 position = {0, 0, 0};
};

struct SDFWorld
{
    // Upstream's solver holds the bodies, manifolds, parameters and random generator. Its step is
    // only used as the reference in tests (sdf_world_step_reference).
    Solver solver;
    int32_t positionIterations = 20;
    float restitutionThreshold = 1;
    uint32_t pairOptions = 0;          // SDFPairOptions

    double stepMilliseconds = 0;
    SDFBodyID nextID = 1;
    std::unordered_map<SDFBodyID, BodyRecord> records;
    std::unordered_map<const Body *, SDFBodyID> ids;
    std::vector<PairRecord> pairs;

    BodyRecord *find(SDFBodyID id)
    {
        auto it = records.find(id);
        return it != records.end() ? &it->second : nullptr;
    }

    const BodyRecord *find(SDFBodyID id) const
    {
        auto it = records.find(id);
        return it != records.end() ? &it->second : nullptr;
    }

    BodyRecord &record(const Body *body) { return records.at(ids.at(body)); }

    void step();
};
