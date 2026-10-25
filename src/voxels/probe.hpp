#pragma once

#include <cstdint>
#include <glm/glm.hpp>

struct Probe
{
    alignas(16) glm::vec3 albedo_sum;
    float depth_moment;
    alignas(16) glm::vec3 last_ray_dir;
    float depth_moment_sq;
};