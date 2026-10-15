// C interface over the SDF-SDF collision engine of Giles and Andrews, "Real-Time Collision Handling
// for Signed Distance Fields via Caching and Importance Sampling" (Pacific Graphics 2026). The
// engine is vendored unmodified in upstream/. Worlds are stepped by our solver, derived from
// upstream's, which adds the body options of a game physics engine. This header is the only API
// Swift sees, so clients do not need C++ interoperability.
//
// Shapes are immutable once created and may be created and used from any thread. A world must be
// used from one thread at a time.
//
// Poses are given in the frame of the caller's object (an entity): the position of its origin,
// its orientation and its uniform scale. The facade converts to the engine's frames, including
// the unit cube normalization of baked voxel shapes.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// MARK: Shapes

typedef struct SDFShape SDFShape;

// Analytical shapes. Axes of symmetry are y; capsule and cylinder lengths are half lengths.
SDFShape *sdf_shape_box(float halfX, float halfY, float halfZ);
SDFShape *sdf_shape_sphere(float radius);
SDFShape *sdf_shape_capsule(float radius, float halfLength);
SDFShape *sdf_shape_cylinder(float radius, float halfHeight);
SDFShape *sdf_shape_cone(float radius, float halfHeight);
SDFShape *sdf_shape_torus(float majorRadius, float minorRadius);

// Analytical composites and operator distorted boxes of the paper (Sec. 4.5, Figs. 1, 4 and 9)
SDFShape *sdf_shape_spike_plate(float halfX, float halfY, float halfZ, int32_t count, float spikeRadius,
                                float minHeight, float maxHeight);
SDFShape *sdf_shape_spiky_wheel(float hubRadius, float hubHalfWidth, float spikeRadius, float spikeLength,
                                int32_t spikes);
typedef enum
{
    SDFWarpBend = 0,
    SDFWarpTwist = 1,
    SDFWarpDisplace = 2,
} SDFWarp;
SDFShape *sdf_shape_warped_box(SDFWarp warp, float halfX, float halfY, float halfZ, float k);

// Voxel SDF baked from a closed triangle mesh (3 floats per vertex, 3 indices per triangle).
// The grid has `resolution` cells across the longest side of the mesh. Returns NULL for an empty
// mesh. Baking takes from milliseconds to about a second.
SDFShape *sdf_shape_bake(const float *positions, int32_t vertexCount, const uint32_t *indices, int32_t indexCount,
                         int32_t resolution);

void sdf_shape_release(SDFShape *shape);

// False for baked meshes whose edges are not all shared by two triangles; their inside and
// outside may be ambiguous
bool sdf_shape_is_watertight(const SDFShape *shape);

// True for analytical convex shapes, and for baked meshes that are closed and convex. Pairs of
// convex shapes skip random samples while their contact is tracked.
bool sdf_shape_is_convex(const SDFShape *shape);

// Volume enclosed by the shape, in its frame's units
float sdf_shape_volume(const SDFShape *shape);

// Signed distance (negative inside) and its gradient at a point of the shape's frame
float sdf_shape_distance(const SDFShape *shape, const float point[3], float gradient[3]);

// Triangle mesh of the shape's surface in its frame, for display. Valid while the shape lives.
typedef struct
{
    const float *positions;      // 3 floats per vertex
    const float *normals;        // 3 floats per vertex
    const uint32_t *indices;     // 3 per triangle
    int32_t vertexCount;
    int32_t indexCount;
} SDFShapeMesh;
SDFShapeMesh sdf_shape_mesh(const SDFShape *shape);

// MARK: Worlds

typedef struct SDFWorld SDFWorld;

// Pairs without a dynamic body that still get collision detection, to report their contacts
typedef enum
{
    SDFReportKinematicVsStatic = 1,
    SDFReportKinematicVsKinematic = 2,
} SDFPairOptions;

// Collision and solver parameters, defaults are the values of the paper (Sec. 4)
typedef struct
{
    int32_t optimizer;           // 0 Nelder-Mead, 1 ellipsoid method, 2 stochastic gradient descent
    int32_t cacheSize;           // n_cache
    int32_t randomSamples;       // n_rand
    bool warmStart;              // Temporal cache, also warm starts the solver impulses
    bool curvatureSampling;      // Draw stochastic samples from high curvature seeds
    bool skipConvexSamples;      // Skip random samples of convex pairs tracked by the cache
    float epsilon;               // Softmax smoothing
    float alpha;                 // Repulsion weight
    float replacementThreshold;  // Improvement a new sample needs to replace a cached point
    float tolerance;             // Optimizer termination tolerance tau
    int32_t maxIterations;       // Optimizer iteration limit
    float stepSize;              // Initial step size of gradient descent
    float margin;                // Contact skin
    float allowedPenetration;    // Penetration of the skin left uncorrected
    int32_t substeps;            // Substeps per time step, each with its own collision detection
    int32_t velocityIterations;  // Contact solver iterations per substep
    int32_t positionIterations;  // Penetration correction iterations per substep
    float timeStep;              // Seconds per step
    float gravity[3];
    float restitutionThreshold;  // Approach speed below which contacts do not bounce
    uint32_t pairOptions;        // SDFPairOptions
    bool parallel;               // Narrow phase on several threads, with a random sequence per pair
    float metersPerUnit;         // Length of a unit, for the default drag force and density
    bool sleeping;               // Puts islands of touching dynamic bodies to sleep once still
    // How stiffly joints hold: the frequency and damping ratio of the spring and damper their rows
    // amount to, as in Box3D, with the frequency capped at half the substep rate and no lower
    // than 1 Hz, the damping ratio no lower than 0.1
    float jointHertz;
    float jointDampingRatio;
} SDFParams;

// Per step statistics
typedef struct
{
    int32_t bodies;
    int32_t pairs;               // Body pairs with overlapping bounds
    int32_t contacts;            // Active contact points
    int32_t iterations;          // Optimizer iterations
    int32_t queries;             // Objective evaluations
    double collisionMilliseconds; // Narrow phase time, summed over the substeps
    double solverMilliseconds;   // Contact solver time, summed over the substeps
    double stepMilliseconds;     // Whole step, including the solver
    int32_t sleepingBodies;
} SDFStats;

SDFWorld *sdf_world_create(void);
void sdf_world_destroy(SDFWorld *world);
// An independent copy that takes the same steps as the original would: bodies with the same IDs,
// contact caches, forces, dragging and random state
SDFWorld *sdf_world_copy(const SDFWorld *world);

SDFParams sdf_default_params(void);
SDFParams sdf_world_params(const SDFWorld *world);
// Non-finite and negative values are ignored, counts are clamped to at least 1 where needed
void sdf_world_set_params(SDFWorld *world, SDFParams params);

void sdf_world_step(SDFWorld *world);
// Steps with upstream's own solver, which ignores the body options. For tests: with default
// options both steps give the same results.
void sdf_world_step_reference(SDFWorld *world);
SDFStats sdf_world_stats(const SDFWorld *world);

// MARK: Bodies

// Identifies a body in its world, 0 is never a valid body
typedef uint32_t SDFBodyID;

typedef enum
{
    SDFBodyDynamic = 0,
    SDFBodyKinematic = 1,        // Moved by the caller, pushes dynamic bodies with its velocity
    SDFBodyStatic = 2,
} SDFBodyMode;

typedef struct
{
    float position[3];           // Origin of the caller's frame
    float orientation[4];        // Quaternion x, y, z, w
    float scale;                 // Uniform
} SDFPose;

// The shape is retained by the body. Dynamic bodies get the mass of their volume of water. A pose
// that is not finite, or whose orientation has no length, is refused here and ignored below.
SDFBodyID sdf_world_add_body(SDFWorld *world, const SDFShape *shape, SDFPose pose, SDFBodyMode mode);
void sdf_world_remove_body(SDFWorld *world, SDFBodyID body);

SDFPose sdf_world_body_pose(const SDFWorld *world, SDFBodyID body);
// Moves the body to a pose (its scale is fixed at creation and ignored here)
void sdf_world_set_body_pose(SDFWorld *world, SDFBodyID body, SDFPose pose);

void sdf_world_body_velocity(const SDFWorld *world, SDFBodyID body, float linear[3], float angular[3]);
// Whether a dynamic body is asleep, with zero velocities, until something touches or moves it
bool sdf_world_body_is_asleep(const SDFWorld *world, SDFBodyID body);
// Whether a body was asleep for the whole of the last step, so that it hasn't moved
bool sdf_world_body_rested_through_step(const SDFWorld *world, SDFBodyID body);
void sdf_world_set_body_velocity(SDFWorld *world, SDFBodyID body, const float linear[3], const float angular[3]);

float sdf_world_body_mass(const SDFWorld *world, SDFBodyID body);
// Rescales the inertia with the mass. Ignored for kinematic and static bodies.
void sdf_world_set_body_mass(SDFWorld *world, SDFBodyID body, float mass);

// Material and motion options of a body. The defaults reproduce upstream's solver.
typedef struct
{
    float staticFriction;        // Friction coefficients, combined by geometric mean
    float dynamicFriction;
    float restitution;           // Bounce, combined by maximum
    float linearDamping;         // Per second
    float angularDamping;
    bool affectedByGravity;
    bool trigger;                // Detects contacts without responding to them
    uint32_t group, mask;        // Pairs collide when each group is in the other's mask
    bool translationLocked[3];   // Axes of the world frame
    bool rotationLocked[3];
} SDFBodyOptions;
SDFBodyOptions sdf_default_body_options(void);
void sdf_world_set_body_options(SDFWorld *world, SDFBodyID body, SDFBodyOptions options);

// Force (at a world point, or the center of mass when point is NULL) and torque, applied in every
// step until cleared. Ignored for kinematic and static bodies.
void sdf_world_add_body_force(SDFWorld *world, SDFBodyID body, const float force[3], const float point[3]);
void sdf_world_add_body_torque(SDFWorld *world, SDFBodyID body, const float torque[3]);
void sdf_world_clear_body_forces(SDFWorld *world, SDFBodyID body);

// Impulse at a world point (or the center of mass when point is NULL), and angular impulse. They
// change the velocity immediately. Ignored for kinematic and static bodies.
void sdf_world_apply_impulse(SDFWorld *world, SDFBodyID body, const float impulse[3], const float point[3]);
void sdf_world_apply_angular_impulse(SDFWorld *world, SDFBodyID body, const float impulse[3]);

// MARK: Joints

typedef uint32_t SDFJointID;    // 0 is never a valid joint

// A hinge: a frame on each body, given in world space at the bodies' current poses (the scale is
// ignored). The joint brings the frames' origins together and aligns their x axes, which is the
// hinge's axis, leaving the rotation about it free. The angle is that of frame B relative to
// frame A about the axis, zero when the frames coincide; a limit bounds it, in radians within
// (-pi, pi), a motor drives it at a speed with up to a torque, and a spring, a damped one of a
// frequency and damping ratio, pulls it toward a target angle the short way round. Unless asked
// to collide, the two bodies ignore each other.
typedef struct
{
    SDFBodyID bodyA, bodyB;
    SDFPose frameA, frameB;
    bool hasLimit;
    float lowerAngle, upperAngle;
    bool hasMotor;
    float motorSpeed;            // Radians per second
    float maxMotorTorque;
    bool hasSpring;
    float targetAngle;
    float springHertz;
    float springDampingRatio;
    bool collideConnected;
} SDFRevoluteJointDef;
SDFRevoluteJointDef sdf_default_revolute_joint(void);

// A slider: frames as the hinge's. The joint keeps frame B's origin on the line through frame
// A's along its x axis, and the frames' orientations as they are when they coincide, leaving the
// translation along the axis free. The translation is that of frame B's origin from frame A's
// along the axis; a limit bounds it, a motor drives it at a speed with up to a force, and a
// spring pulls it toward a target translation.
typedef struct
{
    SDFBodyID bodyA, bodyB;
    SDFPose frameA, frameB;
    bool hasLimit;
    float lowerTranslation, upperTranslation;
    bool hasMotor;
    float motorSpeed;            // Units per second
    float maxMotorForce;
    bool hasSpring;
    float targetTranslation;
    float springHertz;
    float springDampingRatio;
    bool collideConnected;
} SDFPrismaticJointDef;
SDFPrismaticJointDef sdf_default_prismatic_joint(void);

// Return 0 for bodies that don't exist, are the same, or are both unable to move
SDFJointID sdf_world_add_revolute_joint(SDFWorld *world, SDFRevoluteJointDef def);
SDFJointID sdf_world_add_prismatic_joint(SDFWorld *world, SDFPrismaticJointDef def);
void sdf_world_remove_joint(SDFWorld *world, SDFJointID joint);
// The limit, motor and spring of either kind: angles, angular speeds and torques for a hinge,
// translations, speeds and forces for a slider
void sdf_world_set_joint_limit(SDFWorld *world, SDFJointID joint, bool enabled, float lower, float upper);
void sdf_world_set_joint_motor(SDFWorld *world, SDFJointID joint, bool enabled, float speed, float maxForce);
void sdf_world_set_joint_spring(SDFWorld *world, SDFJointID joint, bool enabled, float target, float hertz, float dampingRatio);
// The angle of body B relative to body A about the axis, in (-pi, pi]
float sdf_world_joint_angle(const SDFWorld *world, SDFJointID joint);
// How far frame B's origin is from frame A's along the axis
float sdf_world_joint_translation(const SDFWorld *world, SDFJointID joint);

// MARK: Contacts and picking

typedef struct
{
    SDFBodyID bodyA, bodyB;
    float position[3];           // x* (world space)
    float normal[3];             // From body A to body B
    float separation;            // Gap (> 0) or overlap (< 0)
    float impulse;               // Normal impulse of the last substep
    bool cached;                 // Warm started from the cache, as opposed to a new sample
} SDFContact;

// Writes min(capacity, count) active contacts of the last substep, returns their count
int32_t sdf_world_contacts(const SDFWorld *world, SDFContact *contacts, int32_t capacity);

// A pair of bodies in contact in any substep of the last step
typedef struct
{
    SDFBodyID bodyA, bodyB;
    float position[3];           // Deepest contact point, in the last substep with contacts
    float penetration;           // Of that contact, negative while within the margin
    float impulse;               // Normal impulse over the step, from A to B
    float impulseDirection[3];
} SDFPair;

// Writes min(capacity, count) pairs of the last step, ordered by body, returns their count
int32_t sdf_world_pairs(const SDFWorld *world, SDFPair *pairs, int32_t capacity);

// MARK: Queries

typedef struct
{
    SDFBodyID body;
    float position[3];
    float normal[3];
    float distance;
} SDFRayHit;

// The first hit of a ray on each body whose group is in the mask, unsorted. Writes
// min(capacity, count) hits and returns their count.
int32_t sdf_world_raycast(SDFWorld *world, const float origin[3], const float direction[3], float length, uint32_t mask,
                          SDFRayHit *hits, int32_t capacity);

typedef struct
{
    SDFBodyID body;
    float position[3];           // Deepest point found
    float normal[3];             // From the query shape towards the body
    float separation;            // Gap (> 0) or overlap (< 0) of the surfaces
} SDFOverlap;

// Bodies within a margin of a shape at a pose, whose group is in the mask. Uses the paper's
// collision detection with a fixed seed, so results are deterministic and the world's random
// sequence is untouched. Writes min(capacity, count) results and returns their count.
int32_t sdf_world_overlaps(SDFWorld *world, const SDFShape *shape, SDFPose pose, float margin, uint32_t mask,
                           SDFOverlap *overlaps, int32_t capacity);

// Dragging, by a mouse, a finger or a hand: the grabbed point of a body is pulled toward a target
// by a damped spring of a frequency and damping ratio, the stiffness of the hand, with up to a
// greatest force and at up to a greatest speed. The pull is solved with the joints, after the
// contacts, so a body that meets resistance stops, pressing with at most that force, rather than
// being pushed through; the speed keeps it from flying into what resists faster than contacts
// catch it. While held, the body's spin is damped as well, per second.
typedef struct
{
    float hertz;
    float dampingRatio;
    float maxForce;              // 0 for 50 times the body's weight (at 9.81 m/s² without gravity)
    float maxSpeed;              // 0 for the contact margin per substep
    float angularDamping;
} SDFDragParams;
SDFDragParams sdf_default_drag_params(void);
SDFDragParams sdf_world_drag_params(const SDFWorld *world);
// Negative and NaN values are ignored; an infinite force or speed is a pull without that limit
void sdf_world_set_drag_params(SDFWorld *world, SDFDragParams params);

// Picks the closest dynamic body on the ray and holds it at the hit point, which then follows
// sdf_world_drag_to. Returns the picked body or 0.
SDFBodyID sdf_world_begin_drag(SDFWorld *world, const float origin[3], const float direction[3], float hit[3]);
// Holds a dynamic body at a world space point, for a hand that targets the body itself. Returns
// false, and drags nothing, for a body that doesn't exist or isn't dynamic.
bool sdf_world_begin_drag_body(SDFWorld *world, SDFBodyID body, const float point[3]);
void sdf_world_drag_to(SDFWorld *world, const float target[3]);
void sdf_world_end_drag(SDFWorld *world);

#ifdef __cplusplus
}
#endif
