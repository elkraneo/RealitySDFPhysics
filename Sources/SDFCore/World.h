// The world behind the C interface: upstream's bodies, manifolds and collision detection, stepped
// by our solver (WorldStep.cpp) so that bodies can have the options of a game physics engine.

#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "SDFCore.h"
#include "WorkerPool.h"
#include "solver.h"

// Per body state the upstream Body has no fields for. The defaults reproduce upstream's solver.
struct BodyRecord
{
    Body *body;
    SDFBodyMode mode;
    float3 center;      // Of the shape, see SDFShape in SDFCore.cpp
    float unitScale;
    float boundingRadius = 0; // World space, about the center of the body's local bounds
    bool uniformCurvature = false; // The shape's curvature seeds point at nothing, see SDFCore.cpp

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

// A joint between two bodies. A hinge (revolute) brings their anchors together and keeps their
// axes aligned, leaving one rotation free; a slider (prismatic) keeps B's anchor on the line along
// A's axis and the bodies' orientations locked, leaving one translation free. A limit can bound
// the free motion, a motor drive it and a spring pull it toward a target. Anchors, axes and the
// reference normals that measure the angle are in each body's frame, from its center of mass.
struct JointRecord
{
    enum Kind
    {
        revolute,
        prismatic
    };
    Kind kind = revolute;
    SDFJointID id;
    SDFBodyID bodyA, bodyB;
    Body *a, *b;
    float3 anchorA, anchorB;
    float3 axisA, axisB;
    float3 normalA, normalB;           // Perpendicular to the axes, aligned when the angle was zero
    // The free motion's limits, motor and spring: an angle in radians for a hinge, a translation
    // for a slider, and the motor's speed and greatest torque or force to match
    bool hasLimit = false;
    float lower = 0, upper = 0;
    bool hasMotor = false;
    float motorSpeed = 0, maxMotorForce = 0;
    bool hasSpring = false;
    float springTarget = 0, springHertz = 0, springDampingRatio = 0;
    bool collideConnected = false;

    // Accumulated impulses of the last substep, to warm start the next: the point constraint (for
    // a slider only its rows perpendicular to the axis), the rows that align the axes (for a
    // slider also the twist about the axis), and the axial rows
    float3 linearImpulse = {0, 0, 0};
    float3 angularImpulse = {0, 0, 0}; // World space, perpendicular to the axis for a hinge
    float motorImpulse = 0, lowerImpulse = 0, upperImpulse = 0, springImpulse = 0;
};

// The angle of body B relative to body A about the joint's axis, in (-pi, pi]
float jointAngle(const JointRecord &joint);

// How far B's anchor is from A's along the joint's axis
float jointTranslation(const JointRecord &joint);

struct SDFWorld
{
    // Upstream's solver holds the bodies, manifolds, parameters and random generator. Its step is
    // only used as the reference in tests (sdf_world_step_reference).
    Solver solver;
    int32_t positionIterations = 20;
    float restitutionThreshold = 1;
    uint32_t pairOptions = 0;          // SDFPairOptions
    bool parallel = false;             // Narrow phase and solver on several threads
    std::unique_ptr<WorkerPool> solverPool; // Created on first use
    float metersPerUnit = 1;           // For the physical constants that upstream gives in meters
    float jointHertz = 60, jointDampingRatio = 2; // Box3D's defaults
    bool sleeping = false;             // Puts still islands of touching bodies to sleep
    // The drag holds upstream's dragBody at dragAnchor, pulled toward dragTarget (solver.h)
    SDFDragParams dragParams = {5, 0.7f, 0, 0, 2}; // The spring of Box2D's mouse joint
    float3 dragImpulse = {0, 0, 0};    // Accumulated in the last substep, to warm start the next
    uint32_t nextIsland = 1;
    // The manifolds hold the pairs that overlap at the bodies' poses: nothing has moved, been
    // added, removed or changed since the broad phase found them
    bool broadPhaseCurrent = false;

    double stepMilliseconds = 0;
    double solverMilliseconds = 0;
    SDFBodyID nextID = 1;
    std::unordered_map<SDFBodyID, BodyRecord> records;
    SDFJointID nextJointID = 1;
    std::vector<JointRecord> joints;
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

    JointRecord *findJoint(SDFJointID id)
    {
        for (JointRecord &joint : joints)
            if (joint.id == id)
                return &joint;
        return nullptr;
    }

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
