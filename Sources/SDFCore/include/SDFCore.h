// C interface over the SDF-SDF collision engine of Giles and Andrews, "Real-Time Collision Handling
// for Signed Distance Fields via Caching and Importance Sampling" (Pacific Graphics 2026). The
// engine itself is vendored unmodified in upstream/, this header is the only API Swift sees, so
// clients do not need C++ interoperability.
//
// All functions on one world must be called from a single thread at a time. Shapes are shared
// between worlds (the upstream shape library is global), so loading scenes is not thread safe.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct SDFWorld SDFWorld;

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
    int32_t solverIterations;    // Solver iterations per substep
    float timeStep;              // Seconds per step
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
    double stepMilliseconds;     // Whole step, including the solver
} SDFStats;

// Render state of a body. The transform maps the shape's local frame (in which its mesh is
// given) to world space: world = origin + rotate(orientation, local * scale).
typedef struct
{
    float origin[3];
    float orientation[4];        // Quaternion x, y, z, w
    float scale;
    float color[3];
    int32_t shape;               // Index for sdf_world_shape_mesh
    bool visible;                // The ground is a body too, drawn as a floor instead
    bool isStatic;
} SDFBodyState;

// Triangle mesh of a shape in its local frame. The pointers stay valid until the next scene load.
typedef struct
{
    const float *positions;      // 3 floats per vertex
    const float *normals;        // 3 floats per vertex
    const uint32_t *indices;     // 3 per triangle
    int32_t vertexCount;
    int32_t indexCount;
} SDFShapeMesh;

typedef struct
{
    float position[3];           // x* (world space)
    float normal[3];             // From body A to body B
    float separation;            // Gap (> 0) or overlap (< 0)
    bool cached;                 // Warm started from the cache, as opposed to a new sample
} SDFContact;

typedef struct
{
    float target[3];
    float distance, yaw, pitch;
} SDFCamera;

// Directory that contains assets/*.obj (the built in voxel shapes). Call before loading a scene
// that uses them.
void sdf_set_resource_directory(const char *path);

// Built in scenes
int32_t sdf_scene_count(void);
const char *sdf_scene_name(int32_t index);
const char *sdf_scene_description(int32_t index);

// Shapes that can be spawned by name
int32_t sdf_shape_name_count(void);
const char *sdf_shape_name(int32_t index);

SDFParams sdf_default_params(void);

SDFWorld *sdf_world_create(void);
void sdf_world_destroy(SDFWorld *world);

void sdf_world_load_scene(SDFWorld *world, int32_t index);
SDFCamera sdf_world_camera(const SDFWorld *world);
void sdf_world_spawn(SDFWorld *world, const char *shape, const float position[3]);

void sdf_world_step(SDFWorld *world);
SDFStats sdf_world_stats(const SDFWorld *world);

SDFParams sdf_world_params(const SDFWorld *world);
void sdf_world_set_params(SDFWorld *world, SDFParams params);

int32_t sdf_world_body_count(const SDFWorld *world);
// Writes min(capacity, count) bodies, returns the body count
int32_t sdf_world_body_states(const SDFWorld *world, SDFBodyState *bodies, int32_t capacity);

int32_t sdf_world_shape_count(const SDFWorld *world);
SDFShapeMesh sdf_world_shape_mesh(const SDFWorld *world, int32_t shape);
const char *sdf_world_shape_name(const SDFWorld *world, int32_t shape);

// Writes min(capacity, count) active contacts of the last substep, returns their count
int32_t sdf_world_contacts(const SDFWorld *world, SDFContact *contacts, int32_t capacity);

// Mouse dragging: picks the closest dynamic body on the ray and holds it at the hit point, which
// then follows sdf_world_drag_to. Returns the picked body or -1.
int32_t sdf_world_begin_drag(SDFWorld *world, const float origin[3], const float direction[3], float hit[3]);
void sdf_world_drag_to(SDFWorld *world, const float target[3]);
void sdf_world_end_drag(SDFWorld *world);

#ifdef __cplusplus
}
#endif
