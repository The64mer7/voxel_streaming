#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

#include "terrain_generator/terrain_gen.hpp"
#include "voxels/node.hpp"

template <typename NodeT>
class TerrainContree
{
public:
    static glm::vec3 chunk_origin(glm::ivec3 chunk_key, float chunk_size)
    {
        return glm::vec3(chunk_key) * chunk_size;
    }

    static bool create3d(const glm::vec3& origin, float size, typename NodeT::data_t* data)
    {
        glm::vec3 center = origin + size * 0.5f;
        float d = TerrainGen::map(center);
        float h = TerrainGen::hash(center);
        int variation = static_cast<int>(h * 32.0f) % 32;

        uint32_t c = 0;
        c |= (60 + variation) << 0;
        c |= (150) << 4;
        c |= (32) << 8;
        // color ->     0 - 11
        // normal ->    12 - 23

        glm::vec2 minmax = TerrainGen::minmax_map(origin + size * 0.5f, glm::vec3(size * 0.5f));
        *data = minmax.x < 0 ? c : 0;
        if (*data)
        {
            glm::vec3 normal = TerrainGen::map_normal(center);
            uint32_t packed_normal = TerrainGen::pack_normal(normal);
            *data |= packed_normal << 12;
        }

        return true;
    }
    static bool homogeneous3d(const glm::vec3& origin, float size)
    {
        glm::vec2 minmax = TerrainGen::minmax_map(origin + size * 0.5f, glm::vec3(size * 0.5f));
        return TerrainGen::is_uniform(minmax);
    }
};

template <typename NodeT>
class SphereContree
{
public:
    inline static glm::vec4 sphere = glm::vec4{32, 32, 32, 32};
    static bool create3d(const glm::vec3& origin, float size, typename NodeT::data_t* data)
    {
        bool collide = check_sphere_shell_aabb_collision(sphere, origin, glm::vec3(size));
        if (collide)
            *data = 1;
        else
            *data = 0;
        return collide;
    }
    static bool homogeneous3d(const glm::vec3& origin, float size)
    {
        return !check_sphere_shell_aabb_collision(sphere, origin, glm::vec3(size));
    }
    static bool check_sphere_shell_aabb_collision(const glm::vec4& sphere,
                                                  const glm::vec3& aabb_min,
                                                  const glm::vec3& aabb_size)
    {
        glm::vec3 sphere_center = glm::vec3(sphere);
        float radius = sphere.w;
        float r2 = radius * radius;

        glm::vec3 aabb_max = aabb_min + aabb_size;

        glm::vec3 closest_point = glm::clamp(sphere_center, aabb_min, aabb_max);
        float min_dist_sq = glm::distance2(sphere_center, closest_point);

        if (min_dist_sq > r2)
        {
            return false;
        }

        glm::vec3 farthest_point;
        farthest_point.x = (std::abs(sphere_center.x - aabb_min.x) >
                            std::abs(sphere_center.x - aabb_max.x))
                               ? aabb_min.x
                               : aabb_max.x;
        farthest_point.y = (std::abs(sphere_center.y - aabb_min.y) >
                            std::abs(sphere_center.y - aabb_max.y))
                               ? aabb_min.y
                               : aabb_max.y;
        farthest_point.z = (std::abs(sphere_center.z - aabb_min.z) >
                            std::abs(sphere_center.z - aabb_max.z))
                               ? aabb_min.z
                               : aabb_max.z;

        float max_dist_sq = glm::distance2(sphere_center, farthest_point);

        if (max_dist_sq < r2)
        {
            return false;
        }

        return true;
    }
};