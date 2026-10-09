// Stand-in for upstream/render.h. The scenes only set up the orbit camera, the OpenGL renderer is
// replaced by RealityKit.
#pragma once

#include "solver.h"

// Orbit camera around a target point
struct Camera
{
    float3 target = {0, 1, 0};
    float distance = 10.0f;
    float yaw = 0.6f, pitch = 0.35f;

    float3 eye() const
    {
        return target + float3{cosf(pitch) * sinf(yaw), sinf(pitch), cosf(pitch) * cosf(yaw)} * distance;
    }
};
