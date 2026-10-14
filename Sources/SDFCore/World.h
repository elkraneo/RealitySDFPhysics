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

    // Sleeping dynamic bodies keep zero velocities and are neither integrated nor collided with
    // each other or static bodies. Bodies put to sleep together share an island, and wake together.
    bool asleep = false;
    float stillTime = 0;               // Seconds the body has been nearly still
    bool restedThroughStep = false;    // Asleep for the whole of the last step, so it hasn't moved
    uint32_t island = 0;

    bool locked() const { return min(min(linearMask.x, linearMask.y), min(linearMask.z, min(min(angularMask.x, angularMask.y), angularMask.z))) < 1; }
};

// A pair in contact in any substep of the last step, with totals for collision events
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
    bool parallel = false;             // Narrow phase on several threads
    float metersPerUnit = 1;           // For the physical constants that upstream gives in meters
    bool sleeping = false;             // Puts still islands of touching bodies to sleep
    uint32_t nextIsland = 1;
    // The manifolds hold the pairs that overlap at the bodies' poses: nothing has moved, been
    // added, removed or changed since the broad phase found them
    bool broadPhaseCurrent = false;

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

    // Wakes a body and the bodies put to sleep with it
    void wake(BodyRecord &record);
    // Wakes a body, the bodies it had contacts with and those its bounds overlap, after a change
    // that the step can't see: a teleport, new options, a new mass, an addition or a removal
    void disturb(const Body *body);
    void wakeAll();

private:
    void sleep();
};
