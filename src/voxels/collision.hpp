#pragma once
#include <algorithm>
#include <glm/glm.hpp>

#include "graphics/amebvh.hpp"

inline bool triangle_aabb_intersection(const glm::vec3& p0, const glm::vec3& p1,
                                       const glm::vec3& p2, const glm::vec3& box_min,
                                       const glm::vec3& box_max)
{
    glm::vec3 center = (box_min + box_max) * 0.5f;
    glm::vec3 extents = (box_max - box_min) * 0.5f;

    glm::vec3 v0 = p0 - center;
    glm::vec3 v1 = p1 - center;
    glm::vec3 v2 = p2 - center;

    glm::vec3 e0 = v1 - v0;
    glm::vec3 e1 = v2 - v1;
    glm::vec3 e2 = v0 - v2;

    if (std::min({v0.x, v1.x, v2.x}) > extents.x || std::max({v0.x, v1.x, v2.x}) < -extents.x)
        return false;
    if (std::min({v0.y, v1.y, v2.y}) > extents.y || std::max({v0.y, v1.y, v2.y}) < -extents.y)
        return false;
    if (std::min({v0.z, v1.z, v2.z}) > extents.z || std::max({v0.z, v1.z, v2.z}) < -extents.z)
        return false;

    // Category 2: Triangle Face Normal
    glm::vec3 tri_normal = glm::cross(e0, e1);
    float plane_dist = std::abs(glm::dot(tri_normal, v0));
    float box_projected_radius = extents.x * std::abs(tri_normal.x) +
                                 extents.y * std::abs(tri_normal.y) +
                                 extents.z * std::abs(tri_normal.z);
    if (plane_dist > box_projected_radius)
        return false;

    auto separated_on_axis = [&](const glm::vec3& axis) -> bool
    {
        const float p0_projection = glm::dot(v0, axis);
        const float p1_projection = glm::dot(v1, axis);
        const float p2_projection = glm::dot(v2, axis);
        const float radius = glm::dot(extents, glm::abs(axis));
        return std::min({p0_projection, p1_projection, p2_projection}) > radius ||
               std::max({p0_projection, p1_projection, p2_projection}) < -radius;
    };

    const glm::vec3 edges[] = {e0, e1, e2};
    const glm::vec3 box_axes[] = {
        glm::vec3(1.f, 0.f, 0.f),
        glm::vec3(0.f, 1.f, 0.f),
        glm::vec3(0.f, 0.f, 1.f),
    };
    for (const glm::vec3& edge : edges)
        for (const glm::vec3& box_axis : box_axes)
            if (separated_on_axis(glm::cross(edge, box_axis)))
                return false;

    return true;
}

inline bool aabb_aabb_volume_intersection(const glm::vec3& a_min, const glm::vec3& a_max,
                                          const glm::vec3& b_min, const glm::vec3& b_max)
{
    return (a_min.x <= b_max.x && a_max.x >= b_min.x) &&
           (a_min.y <= b_max.y && a_max.y >= b_min.y) && (a_min.z <= b_max.z && a_max.z >= b_min.z);
}

inline bool aabb_aabb_shell_intersection(const glm::vec3& a_min, const glm::vec3& a_max,
                                         const glm::vec3& b_min, const glm::vec3& b_max)
{
    if (!aabb_aabb_volume_intersection(a_min, a_max, b_min, b_max))
    {
        return false;
    }

    bool a_inside_b = (a_min.x > b_min.x && a_max.x < b_max.x) &&
                      (a_min.y > b_min.y && a_max.y < b_max.y) &&
                      (a_min.z > b_min.z && a_max.z < b_max.z);

    bool b_inside_a = (b_min.x > a_min.x && b_max.x < a_max.x) &&
                      (b_min.y > a_min.y && b_max.y < a_max.y) &&
                      (b_min.z > a_min.z && b_max.z < a_max.z);

    return !(a_inside_b || b_inside_a);
}

// TODO:
inline glm::vec3 get_barycentric(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b,
                                 const glm::vec3& c)
{
    glm::vec3 v0 = b - a;
    glm::vec3 v1 = c - a;
    glm::vec3 v2 = p - a;

    float d00 = glm::dot(v0, v0);
    float d01 = glm::dot(v0, v1);
    float d11 = glm::dot(v1, v1);
    float d20 = glm::dot(v2, v0);
    float d21 = glm::dot(v2, v1);

    float denom = d00 * d11 - d01 * d01;

    const float EPSILON = 1e-8f;
    if (std::abs(denom) < EPSILON)
    {
        return glm::vec3(0.0f);
    }

    float inv_denom = 1.0f / denom;
    float wb = (d11 * d20 - d01 * d21) * inv_denom;
    float wc = (d00 * d21 - d01 * d20) * inv_denom;
    float wa = 1.0f - wb - wc;

    return glm::vec3(wa, wb, wc);
}

struct BVHIntersectAABBInfo
{
    glm::vec3 normal;
    uint32_t triangle_idx;
};

inline bool bvh_aabb_intersect(BVHIntersectAABBInfo* info, ame_bvh& bvh, uint32_t idx,
                               const glm::vec3& min, const glm::vec3& max)
{
    auto& aabb = bvh.nodes[idx].aabb;
    if (!aabb_aabb_volume_intersection(glm::make_vec3(aabb.min.raw), glm::make_vec3(aabb.max.raw),
                                       min, max))
        return false;

    auto& node = bvh.nodes[idx];
    if (ame_is_leaf(&node))
    {
        for (uint32_t i = ame_bvh_leaf_begin(&node); i < ame_bvh_leaf_end(&node); i++)
        {
            uint32_t triangle_idx = bvh.sorted_triangles[i];
            uint32_t i0 = bvh.mesh->indices[triangle_idx * 3 + 0];
            uint32_t i1 = bvh.mesh->indices[triangle_idx * 3 + 1];
            uint32_t i2 = bvh.mesh->indices[triangle_idx * 3 + 2];

            glm::vec3 p0 = glm::make_vec3(&bvh.mesh->vertices[i0 * bvh.mesh->vertex_stride]);
            glm::vec3 p1 = glm::make_vec3(&bvh.mesh->vertices[i1 * bvh.mesh->vertex_stride]);
            glm::vec3 p2 = glm::make_vec3(&bvh.mesh->vertices[i2 * bvh.mesh->vertex_stride]);

            if (triangle_aabb_intersection(p0, p1, p2, min, max))
            {
                if (info)
                {
                    info->normal = glm::normalize(glm::cross(p1 - p2, p1 - p0));
                    info->triangle_idx = triangle_idx;
                }
                return true;
            }
        }
    }
    else
    {
        if (bvh_aabb_intersect(info, bvh, bvh.nodes[idx].node.left, min, max))
            return true;

        if (bvh_aabb_intersect(info, bvh, bvh.nodes[idx].node.right, min, max))
            return true;
    }
    return false;
}
