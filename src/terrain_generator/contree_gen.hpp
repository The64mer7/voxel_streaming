#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
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

#include <stb_image.h>

struct Texture2D
{
    std::vector<stbi_uc> data;
    int width = 0;
    int height = 0;
    int src_channels = 0;

    bool is_valid() { return width != 0 && height != 0; }

    bool load(const std::string& filepath)
    {
        stbi_uc* pixels = stbi_load(filepath.c_str(), &width, &height, &src_channels,
                                    STBI_rgb_alpha);

        if (!pixels)
        {
            std::fprintf(stderr, "Failed to load image %s: %s\n", filepath.c_str(),
                         stbi_failure_reason());
            return false;
        }
        size_t pixels_end = size_t(width) * height * 4;
        data.assign(pixels, pixels + pixels_end);
        stbi_image_free(pixels);
        return true;
    }

    stbi_uc* sample_texture_nearest2f(const glm::vec2& uv)
    {
        glm::ivec2 coords = glm::ivec2(glm::fract(uv) * glm::vec2(width, height)); // REPEAT
        const size_t pixel_index = size_t(coords.x) + size_t(coords.y) * size_t(width);
        return data.data() + pixel_index * 4;
    }

    glm::ivec2 texture_coords_2f_2i(const glm::vec2& uv)
    {
        return glm::ivec2(glm::fract(uv) * glm::vec2(width, height));
    }

    stbi_uc* sample_texture_nearest2i(const glm::ivec2& coords)
    {
        const size_t pixel_index = size_t(coords.x) + size_t(coords.y) * size_t(width);
        return data.data() + pixel_index * 4;
    }
};

class MeshContree
{
public:
    static void create(const std::string& filepath)
    {
        tinyobj::ObjReader reader;
        tinyobj::ObjReaderConfig config;
        config.triangulate = true;

        printf("importing obj...\n");
        if (!reader.ParseFromFile(filepath, config))
        {
            std::fprintf(stderr, "Error loading mesh %s: %s\n", filepath.c_str(),
                         reader.Error().c_str());
            std::exit(1);
        }
        printf("done\n");

        destroy();

        const auto& attrib = reader.GetAttrib();
        const auto& shapes = reader.GetShapes();
        const auto& materials = reader.GetMaterials();

        std::printf("========================================\n");
        std::printf("Scene Loaded: %s\n", filepath.c_str());
        std::printf("----------------------------------------\n");
        std::printf("  Vertices:   %zu\n", attrib.vertices.size() / 3);
        std::printf("  Normals:    %zu\n", attrib.normals.size() / 3);
        std::printf("  Texcoords:  %zu\n", attrib.texcoords.size() / 2);
        std::printf("  Shapes:     %zu\n", shapes.size());
        std::printf("  Materials:  %zu\n", materials.size());
        std::printf("========================================\n");

        if (!materials.empty())
        {
            std::printf("--- Materials List ---\n");
            for (size_t i = 0; i < materials.size(); ++i)
            {
                const auto& mat = materials[i];
                std::printf("  [%zu] Name: %s\n", i, mat.name.c_str());
                Texture2D texture;
                if (!mat.diffuse_texname.empty())
                {
                    const auto texture_path = std::filesystem::path(filepath).parent_path() /
                                              mat.diffuse_texname;
                    texture.load(texture_path.string());

                    std::printf("       Diffuse Texture: %s\n", mat.diffuse_texname.c_str());
                }
                material_to_texture.push_back(texture);
            }
            std::printf("----------------------\n");
        }
        // ---------------------------------------------------------
        vertices_xyz = attrib.vertices;
        vertices_uv = attrib.texcoords;
        for (int i = 0; i + 1 < vertices_uv.size(); i += 2)
        {
            vertices_uv[i + 1] = 1 - vertices_uv[i + 1];
        }
        vertices_norm = attrib.normals;
        size_t num_indices = 0;
        for (const auto& shape : shapes)
        {
            num_indices += shape.mesh.indices.size();
        }
        indices.reserve(num_indices);
        normal_indices.reserve(num_indices);
        uv_indices.reserve(num_indices);

        for (const auto& shape : shapes)
        {
            for (const auto& index : shape.mesh.indices)
            {
                indices.push_back(index.vertex_index);
                normal_indices.push_back(index.normal_index);
                uv_indices.push_back(index.texcoord_index);
            }
        }

        for (const auto& shape : shapes)
        {
            for (auto& mat_id : shape.mesh.material_ids)
            {
                triangle_material_ids.push_back(mat_id);
            }
        }

        mesh.vertices = vertices_xyz.data();
        mesh.num_vertices = static_cast<uint32_t>(vertices_xyz.size() / 3);
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
        vertices_xyz.clear();
        indices.clear();
    }
    static ame_bvh& get_bvh() { return m_bvh; }

    static bool create3d(const glm::vec3& origin, float size, uint32_t* data)
    {
        BVHIntersectAABBInfo info;
        const bool intersects = bvh_aabb_intersect(&info, m_bvh, 0, origin, origin + size);
        if (intersects)
        {
            uint32_t i0 = info.triangle_idx * 3 + 0;
            uint32_t i1 = info.triangle_idx * 3 + 1;
            uint32_t i2 = info.triangle_idx * 3 + 2;

            uint32_t vi0 = indices[i0];
            uint32_t vi1 = indices[i1];
            uint32_t vi2 = indices[i2];

            int ni0 = normal_indices[i0];
            int ni1 = normal_indices[i1];
            int ni2 = normal_indices[i2];

            int ti0 = uv_indices[i0];
            int ti1 = uv_indices[i1];
            int ti2 = uv_indices[i2];

            glm::vec3 v0 = glm::make_vec3(&mesh.vertices[vi0 * mesh.vertex_stride]);
            glm::vec3 v1 = glm::make_vec3(&mesh.vertices[vi1 * mesh.vertex_stride]);
            glm::vec3 v2 = glm::make_vec3(&mesh.vertices[vi2 * mesh.vertex_stride]);

            glm::vec3 bary = get_barycentric(origin + size * 0.5f, v0, v1, v2);

            glm::vec2 uv0 = glm::make_vec2(&vertices_uv[ti0 * 2]);
            glm::vec2 uv1 = glm::make_vec2(&vertices_uv[ti1 * 2]);
            glm::vec2 uv2 = glm::make_vec2(&vertices_uv[ti2 * 2]);

            int mat = triangle_material_ids[info.triangle_idx];
            uint32_t color_rgba4 = 0xfffu;
            if (mat != -1 && mat < material_to_texture.size())
            {
                Texture2D& texture = material_to_texture[mat];
                if (texture.is_valid())
                {
                    glm::vec2 uv = uv0 * bary.x + uv1 * bary.y + uv2 * bary.z;

                    stbi_uc* color_rgba8 = texture.sample_texture_nearest2f(uv);

                    color_rgba4 = 0;
                    color_rgba4 |= (color_rgba8[0] / 16u) << 0u;
                    color_rgba4 |= (color_rgba8[1] / 16u) << 4u;
                    color_rgba4 |= (color_rgba8[2] / 16u) << 8u;
                    // color_rgba4 = TerrainGen::pack_normal(bary);
                }
            }

            uint32_t material_hash = (mat * 1151 + 165591) & 0xfffu;
            *data = color_rgba4 | (TerrainGen::pack_normal(info.normal) << 12);
        }
        else
            *data = 0u;
        return intersects;
    }

    static bool homogeneous3d(const glm::vec3& origin, float size)
    {
        return !bvh_aabb_intersect(nullptr, m_bvh, 0, origin, origin + size);
    }

private:
    inline static ame_bvh m_bvh{};
    inline static ame_mesh mesh{};

    inline static std::vector<float> vertices_xyz;
    inline static std::vector<float> vertices_norm;
    inline static std::vector<float> vertices_uv;

    inline static std::vector<uint32_t> indices;
    inline static std::vector<int> normal_indices;
    inline static std::vector<int> uv_indices;
    inline static std::vector<int> triangle_material_ids;

    inline static std::vector<Texture2D> material_to_texture;
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
    inline static glm::vec4 ball = glm::vec4{16, 16, 16, 4};
    inline static uint32_t color = 0xfff;

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
        *data = color;
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

    static bool create_3d_dfs_mesh(const glm::vec3& chunk_origin, float chunk_size,
                                   uint32_t max_depth, ame_bvh& bvh)
    {
        s_nodes.resize(1024 * 1024 * 128);
        s_allocator.create(s_nodes.size() / 64, 64);
        std::vector<uint32_t> out_pages;
        create_3d_dfs_mesh_fixed_page(&s_allocator, 0, s_nodes.data(), bvh, chunk_origin,
                                      chunk_size, max_depth, out_pages);
        printf("size%u: \n", out_pages.size());
        return true;
    }

    inline static std::vector<NodeT> s_nodes;
    inline static NodeAllocator s_allocator;
};
