#pragma once
#include <algorithm>
#include <glm/glm.hpp>

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

    auto test_cross_axis = [&](float a, float b, float v0_a, float v0_b, float v1_a, float v1_b,
                               float ext_a, float ext_b) -> bool
    {
        float pa0 = a * v0_a - b * v0_b;
        float pa1 = a * v1_a - b * v1_b;
        float rad = ext_a * std::abs(a) + ext_b * std::abs(b);
        return (std::min(pa0, pa1) > rad || std::max(pa0, pa1) < -rad);
    };

    if (test_cross_axis(e0.z, e0.y, v0.y, v0.z, v2.y, v2.z, extents.y, extents.z))
        return false;
    if (test_cross_axis(-e0.z, e0.x, v0.x, v0.z, v2.x, v2.z, extents.x, extents.z))
        return false;
    if (test_cross_axis(e0.y, -e0.x, v0.x, v0.y, v1.x, v1.y, extents.x, extents.y))
        return false;

    if (test_cross_axis(e1.z, e1.y, v0.y, v0.z, v2.y, v2.z, extents.y, extents.z))
        return false;
    if (test_cross_axis(-e1.z, e1.x, v0.x, v0.z, v2.x, v2.z, extents.x, extents.z))
        return false;
    if (test_cross_axis(e1.y, -e1.x, v0.x, v0.y, v1.x, v1.y, extents.x, extents.y))
        return false;

    if (test_cross_axis(e2.z, e2.y, v0.y, v0.z, v1.y, v1.z, extents.y, extents.z))
        return false;
    if (test_cross_axis(-e2.z, e2.x, v0.x, v0.z, v1.x, v1.z, extents.x, extents.z))
        return false;
    if (test_cross_axis(e2.y, -e2.x, v1.x, v1.y, v2.x, v2.y, extents.x, extents.y))
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