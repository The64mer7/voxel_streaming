#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vec_swizzle.hpp>
#include <tiny_obj_loader.h>
#include <vector>

#include "terrain_generator/terrain_gen.hpp"

#include "graphics/amebvh.hpp"
#include "voxels/builder.hpp"
#include "voxels/collision.hpp"
#include "voxels/node.hpp"

class MeshContree
{
public:
    static void create(const std::string& filepath)
    {
        tinyobj::ObjReader reader;
        tinyobj::ObjReaderConfig config;
        config.triangulate = true;

        if (!reader.ParseFromFile(filepath, config))
        {
            std::fprintf(stderr, "Error loading mesh %s: %s\n", filepath.c_str(),
                         reader.Error().c_str());
            std::exit(1);
        }

        destroy();

        const auto& attrib = reader.GetAttrib();
        vertices = attrib.vertices;

        size_t num_indices = 0;
        for (const auto& shape : reader.GetShapes())
        {
            num_indices += shape.mesh.indices.size();
        }
        indices.reserve(num_indices);

        for (const auto& shape : reader.GetShapes())
        {
            for (const auto& index : shape.mesh.indices)
            {
                indices.push_back(static_cast<uint32_t>(index.vertex_index));
            }
        }
        for (int i = 0; i < vertices.size(); i++)
        {
            vertices[i] = (vertices[i] * 8.f + 1.f) * 4.f;
        }

        mesh.vertices = vertices.data();
        mesh.num_vertices = static_cast<uint32_t>(vertices.size() / 3);
        mesh.vertex_stride = 3;
        mesh.indices = indices.data();
        mesh.num_indices = static_cast<uint32_t>(indices.size());

        if (!ame_create_bvh(&m_bvh, &mesh, 8))
        {
            std::fprintf(stderr, "Error creating BVH for mesh %s\n", filepath.c_str());
            std::exit(1);
        }
    }

    static void destroy()
    {
        ame_destroy_bvh(&m_bvh);
        m_bvh = {};
        mesh = {};
        vertices.clear();
        indices.clear();
    }
    static const ame_bvh& get_bvh() { return m_bvh; }

    static bool create3d(const glm::vec3& origin, float size, uint32_t* data)
    {
        const bool intersects = intersects_mesh_aabb(origin, size);
        *data = intersects ? 10u : 0u;
        return intersects;
    }

    static bool homogeneous3d(const glm::vec3& origin, float size)
    {
        return !intersects_mesh_aabb(origin, size);
    }

private:
    static bool intersects_mesh_aabb(const glm::vec3& origin, float size)
    {
        if (m_bvh.nodes == nullptr || m_bvh.mesh == nullptr || m_bvh.sorted_triangles == nullptr ||
            m_bvh.num_nodes == 0)
            return false;

        const glm::vec3 box_min = origin;
        const glm::vec3 box_max = origin + glm::vec3(size);
        std::vector<uint32_t> stack{0};

        while (!stack.empty())
        {
            const uint32_t node_index = stack.back();
            stack.pop_back();
            const ame_bvh_node& node = m_bvh.nodes[node_index];
            const glm::vec3 node_min(node.aabb.min.x, node.aabb.min.y, node.aabb.min.z);
            const glm::vec3 node_max(node.aabb.max.x, node.aabb.max.y, node.aabb.max.z);
            if (!aabb_aabb_volume_intersection(node_min, node_max, box_min, box_max))
                continue;

            if ((node.node.left & 0x80000000u) != 0)
            {
                const uint32_t begin = node.leaf.begin & 0x7fffffffu;
                for (uint32_t triangle = begin; triangle < node.leaf.end; ++triangle)
                {
                    const uint32_t first_index = m_bvh.sorted_triangles[triangle] * 3;
                    const uint32_t vertex0 = m_bvh.mesh->indices[first_index] *
                                             m_bvh.mesh->vertex_stride;
                    const uint32_t vertex1 = m_bvh.mesh->indices[first_index + 1] *
                                             m_bvh.mesh->vertex_stride;
                    const uint32_t vertex2 = m_bvh.mesh->indices[first_index + 2] *
                                             m_bvh.mesh->vertex_stride;
                    const float* vertices = m_bvh.mesh->vertices;
                    const glm::vec3 v0(vertices[vertex0], vertices[vertex0 + 1],
                                       vertices[vertex0 + 2]);
                    const glm::vec3 v1(vertices[vertex1], vertices[vertex1 + 1],
                                       vertices[vertex1 + 2]);
                    const glm::vec3 v2(vertices[vertex2], vertices[vertex2 + 1],
                                       vertices[vertex2 + 2]);
                    if (triangle_aabb_intersection(v0, v1, v2, box_min, box_max))
                        return true;
                }
            }
            else
            {
                stack.push_back(static_cast<uint32_t>(node.node.left));
                stack.push_back(static_cast<uint32_t>(node.node.right));
            }
        }

        return false;
    }

    inline static ame_bvh m_bvh{};
    inline static ame_mesh mesh{};
    inline static std::vector<float> vertices;
    inline static std::vector<uint32_t> indices;
};

class SphereContree
{
public:
    inline static glm::vec4 sphere = glm::vec4{16, 16, 16, 8};
    static bool create3d(const glm::vec3& origin, float size, NodeT::data_t* data)
    {
        bool collide = check_sphere_shell_aabb_collision(sphere, origin, glm::vec3(size));
        if (collide)
        {
            glm::vec3 normal = glm::normalize((origin + size * 0.5f) - glm::xyz(sphere));
            uint32_t packed_normal = TerrainGen::pack_normal(normal);
            *data = (packed_normal << 12) | 0x00000fff;
        }
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

class BallContree
{
public:
    inline static glm::vec4 ball = glm::vec4{16, 16, 16, 16};

    static bool create3d(const glm::vec3& origin, float size, NodeT::data_t* data)
    {
        if (!intersects_ball_aabb(ball, origin, glm::vec3(size)))
        {
            *data = 0;
            return false;
        }

        glm::vec3 direction = origin + size * 0.5f - glm::vec3(ball);
        glm::vec3 normal = glm::dot(direction, direction) > 0.0f ? glm::normalize(direction)
                                                                 : glm::vec3(0.0f, 1.0f, 0.0f);
        uint32_t packed_normal = TerrainGen::pack_normal(normal);
        *data = (packed_normal << 12) | 0x00000fff;
        return true;
    }

    static bool homogeneous3d(const glm::vec3& origin, float size)
    {
        const glm::vec3 center(ball);
        const glm::vec3 aabb_size(size);
        const glm::vec3 aabb_max = origin + aabb_size;
        const glm::vec3 farthest_point(
            std::abs(center.x - origin.x) > std::abs(center.x - aabb_max.x) ? origin.x : aabb_max.x,
            std::abs(center.y - origin.y) > std::abs(center.y - aabb_max.y) ? origin.y : aabb_max.y,
            std::abs(center.z - origin.z) > std::abs(center.z - aabb_max.z) ? origin.z
                                                                            : aabb_max.z);
        return glm::distance2(center, farthest_point) <= ball.w * ball.w;
    }

private:
    static bool intersects_ball_aabb(const glm::vec4& ball, const glm::vec3& aabb_min,
                                     const glm::vec3& aabb_size)
    {
        const glm::vec3 center(ball);
        const glm::vec3 aabb_max = aabb_min + aabb_size;
        const glm::vec3 closest_point = glm::clamp(center, aabb_min, aabb_max);
        return glm::distance2(center, closest_point) <= ball.w * ball.w;
    }
};

class TerrainContree

{
public:
    static glm::vec3 chunk_origin(glm::ivec3 chunk_key, float chunk_size)
    {
        return glm::vec3(chunk_key) * chunk_size;
    }

    static bool create3d(const glm::vec3& origin, float size, NodeT::data_t* data)
    {
        if (SphereContree::create3d(origin, size, data))
            return true;

        glm::vec3 center = origin + size * 0.5f;
        float d = TerrainGen::map(center);
        float h = TerrainGen::hash(center);
        int variation = static_cast<int>(h * 32.0f) % 32;

        uint32_t r = static_cast<uint32_t>((60.0f + variation) / 17.0f) & 0xf;
        uint32_t g = static_cast<uint32_t>(150.0f / 17.0f) & 0xf;
        uint32_t b = static_cast<uint32_t>(32.0f / 17.0f) & 0xf; 

        uint32_t c = 0;
        c |= (r) << 0;
        c |= (g) << 4;
        c |= (b) << 8;

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
        if (!SphereContree::homogeneous3d(origin, size))
            return false;

        glm::vec2 minmax = TerrainGen::minmax_map(origin + size * 0.5f, glm::vec3(size * 0.5f));
        return TerrainGen::is_uniform(minmax);
    }
};

class WorldContree
{
public:
    static glm::vec3 chunk_origin(glm::ivec3 chunk_key, float chunk_size)
    {
        return glm::vec3(chunk_key) * chunk_size;
    }

    using create_pfn = bool(const glm::vec3& origin, float size, NodeT::data_t* data);
    static bool create3d(const glm::vec3& origin, float size, NodeT::data_t* data)
    {
        glm::vec3 center = origin + size * 0.5f;
        float d = TerrainGen::map(center);
        float h = TerrainGen::hash(center);
        int variation = static_cast<int>(h * 4.0f) % 4;

        uint32_t r = static_cast<uint32_t>((60.0f + variation) / 17.0f) & 0xf;
        uint32_t g = static_cast<uint32_t>(150.0f / 17.0f) & 0xf;
        uint32_t b = static_cast<uint32_t>(32.0f / 17.0f) & 0xf;

        uint32_t c = 0;
        c |= (r) << 0;
        c |= (g) << 4;
        c |= (b) << 8;

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
    using homogeneous_pfn = bool (*)(const glm::vec3& origin, float size);
    static bool homogeneous3d(const glm::vec3& origin, float size)
    {
        glm::vec2 minmax = TerrainGen::minmax_map(origin + size * 0.5f, glm::vec3(size * 0.5f));
        return TerrainGen::is_uniform(minmax);
    }
    static constexpr uint32_t invalid_offset = UINT32_MAX;

    static bool subdivide(const glm::ivec4& coord, NodeT* nodes, uint32_t first_child_idx,
                          uint32_t parent_idx)
    {
        int depth = coord.w;
        glm::ivec3 remaining_coords = glm::ivec3(coord);

        uint32_t node_idx = 0;
        for (int i = 0; i < depth; i++)
        {
            NodeT& node = s_nodes[node_idx];
            glm::ivec3 local = remaining_coords % 4;
            int local_bit = local.x + 4 * local.y + 16 * local.z;
            if (node.childmask.get(local_bit))
            {
                node_idx = svo::get_child_ptr(node, local_bit);
            }
            else
                return false;

            remaining_coords = remaining_coords / 4;
        }

        if (node_idx != invalid_offset)
        {

            NodeT& node = s_nodes[node_idx];
            uint32_t child_count = node.child_count();
            if (child_count > 0)
            {
                for (int i = 0; i < 64; i++)
                {
                    nodes[first_child_idx + i] = s_nodes[node.firstchild + i];
                    nodes[first_child_idx + i].clear_bits();
                    nodes[first_child_idx + i].firstchild = invalid_offset;
                    nodes[first_child_idx + i].parent = parent_idx;
                }
                nodes[parent_idx].firstchild = first_child_idx;
                nodes[parent_idx].childmask = node.childmask;
                return true;
            }
        }
        return false;
    }

    static bool create_3d_dfs(const glm::vec3& chunk_origin, float chunk_size, uint32_t max_depth)
    {
        s_nodes.resize(1024 * 1024 * 20);
        s_allocator.create(s_nodes.size() / 64, 64);
        std::vector<uint32_t> out_pages;
        return create_3d_dfs_fixed_page<create_pfn, homogeneous_pfn>(
            &s_allocator, 0, s_nodes.data(), TerrainContree::create3d,
            TerrainContree::homogeneous3d, chunk_origin, chunk_size, max_depth, out_pages);
    }

    inline static std::vector<NodeT> s_nodes;
    inline static NodeAllocator s_allocator;
};
