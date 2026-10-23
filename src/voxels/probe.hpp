#pragma once

#include <cstdint>
#include <glm/glm.hpp>

struct alignas(32) Probe
{
    glm::vec3 albedo_sum;
    glm::vec3 last_ray_dir;
};