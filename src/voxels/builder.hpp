#pragma once

#include "allocator/node_allocator.hpp"
#include "node.hpp"
#include <glm/glm.hpp>

#include "voxels/collision.hpp"

struct BuildInfo
{
    glm::ivec3 coord;
    glm::vec3 chunk_origin;
    float chunk_size;
    uint32_t depth;
};

template <typename PFNCreate, typename PFNHomogeneous>
struct GenerateInfo
{
    PFNCreate pfn_create;
    PFNHomogeneous pfn_homogeneous;
};

template <typename PFNCreate, typename PFNHomogeneous>
bool subdivide_node(const BuildInfo& info, const GenerateInfo<PFNCreate, PFNHomogeneous>& gen_info,
                    uint32_t node_idx, uint32_t first_child_idx, NodeT* nodes,
                    uint32_t axis_branching_factor)

{
    if (gen_info.pfn_homogeneous(info.chunk_origin, info.chunk_size))
        return false;

    NodeT& node = nodes[node_idx];
    uint32_t write_idx = first_child_idx;
    uint32_t child_bit = 0;

    for (uint32_t z = 0; z < axis_branching_factor; z++)
        for (uint32_t y = 0; y < axis_branching_factor; y++)
            for (uint32_t x = 0; x < axis_branching_factor; x++)
            {
                float child_size = info.chunk_size / axis_branching_factor;
                glm::vec3 child_origin = info.chunk_origin + child_size * glm::vec3(x, y, z);

                NodeT& child_node = nodes[write_idx];

                bool created = gen_info.pfn_create(child_origin, child_size, &child_node.data);
                if (created)
                {
                    child_node.firstchild = NodeT::invalid_offset;
                    child_node.childmask.clear();
                    child_node.parent = node_idx;

                    node.set_bit(child_bit);
                    write_idx++;
                }
                child_bit++;
            }
    if (write_idx == first_child_idx)
    {
        printf("subdivided into 0\n");
        return false;
    }

    node.firstchild = first_child_idx;

    return true;
}

static void subdivide_nodes(NodeT* nodes, uint32_t idx, NodeAllocator* allocator)
{
    if (nodes[idx].firstchild == UINT32_MAX)
    {
        uint32_t c_idx = allocator->allocate();
        nodes[idx].firstchild = c_idx;
        nodes[idx].childmask.fill_mask(0, 63);
        for (int i = 0; i < 64; i++)
        {
            nodes[c_idx + i].parent = idx;
            nodes[c_idx + i].firstchild = UINT32_MAX;
            nodes[c_idx + i].clear_bits();
            nodes[c_idx + i].data = nodes[idx].data;
        }
        return;
    }
    uint32_t first = nodes[idx].firstchild;
    for (int i = 0; i < 64; i++)
    {
        if (nodes[idx].get_bit(i))
        {
        }
        else
        {
            nodes[first + i].firstchild = UINT32_MAX;
            nodes[first + i].data = nodes[idx].data;
            nodes[first + i].clear_bits();
            nodes[first + i].parent = idx;
        }
        subdivide_nodes(nodes, first + i, allocator);
    }
}

static void copy_node_data_to_children(NodeT* nodes, uint32_t idx, uint32_t first_child)
{
    nodes[idx].firstchild = first_child;
    nodes[idx].childmask.fill_mask(0, 63);
    for (int i = 0; i < 64; i++)
    {
        nodes[first_child + i].parent = idx;
        nodes[first_child + i].firstchild = UINT32_MAX;
        nodes[first_child + i].clear_bits();
        nodes[first_child + i].data = nodes[idx].data;
    }
}

static void free_children_3d_fixed_page(NodeAllocator* allocator, uint32_t node_idx, NodeT* nodes)
{
    uint32_t first_child = nodes[node_idx].firstchild;
    if (first_child == UINT32_MAX)
        return;
    nodes[node_idx].firstchild = UINT32_MAX;
    uint32_t children_page_id = first_child / 64;
    for (int i = 0; i < 64; i++)
    {
        if (nodes[first_child + i].firstchild != UINT32_MAX)
        {
            free_children_3d_fixed_page(allocator, first_child + i, nodes);
        }
    }
    allocator->free(children_page_id);
}

template <typename CreateFunc, typename HomogeneousFunc>
struct Contree3DModifier
{
    NodeAllocator* allocator;
    NodeT* nodes;
    CreateFunc modify_fn;
    HomogeneousFunc modify_homogeneous_fn;
    uint32_t max_depth;
    std::vector<glm::vec4>* aabbs;
    glm::uvec2 out_minmax = glm::uvec2(UINT32_MAX, 0);

    bool modify_3d_dfs_rec(const glm::vec3& origin, float size, uint32_t node_idx, uint32_t depth)
    {
        if (node_idx == UINT32_MAX)
            return false;

        uint32_t data = 0;
        bool affects_volume = modify_fn(origin, size, &data);
        if (!affects_volume)
        {
            return false;
        }

        if (depth >= max_depth)
        {
            nodes[node_idx].data = data;

            out_minmax.x = glm::min(node_idx, out_minmax.x);
            out_minmax.y = glm::max(node_idx, out_minmax.y);
            return true;
        }

        bool is_homogeneous = modify_homogeneous_fn(origin, size);
        if (is_homogeneous)
        {
            aabbs->push_back(glm::vec4(origin, size));

            printf("cleared %u %f %u\n", node_idx, size, depth);
            free_children_3d_fixed_page(allocator, node_idx, nodes);
            nodes[node_idx].clear_bits();
            nodes[node_idx].firstchild = UINT32_MAX;
            nodes[node_idx].data = data;
            return true;
        }

        uint32_t first_child = nodes[node_idx].firstchild;
        // if (first_child == UINT32_MAX)
        //     aabbs->push_back(glm::vec4(origin, size));

        if (first_child == UINT32_MAX)
        {
            first_child = allocator->allocate();
            if (first_child == UINT32_MAX)
                exit(1);

            nodes[node_idx].firstchild = first_child;
            nodes[node_idx].childmask.clear();
            for (int i = 0; i < 64; i++)
            {
                nodes[first_child + i].parent = idx;
                nodes[first_child + i].firstchild = UINT32_MAX;
                nodes[first_child + i].clear_bits();
                nodes[first_child + i].data = nodes[idx].data;
            }
            out_minmax.x = glm::min(first_child, out_minmax.x);
            out_minmax.y = glm::max(first_child + 63, out_minmax.y);
        }

        bool modified = false;
        for (int i = 0; i < 64; i++)
        {
            float child_size = size * 0.25f;
            glm::vec3 child_origin = origin +
                                     child_size * glm::vec3(glm::ivec3(i, i / 4, i / 16) % 4);
            uint32_t child_idx = first_child + i;
            // if (!nodes[node_idx].get_bit(i))
            //{
            //     nodes[child_idx].clear_bits();
            //     nodes[child_idx].firstchild = UINT32_MAX;
            //     nodes[child_idx].data = nodes[node_idx].data;
            //     nodes[child_idx].parent = node_idx;
            // }

            modified = modified ||
                       modify_3d_dfs_rec(child_origin, child_size, child_idx, depth + 1);
        }
        return modified;
    }
};

template <typename CreateFunc, typename HomogeneousFunc>
static glm::uvec2
modify_3d_dfs_fixed_page(NodeAllocator* allocator, uint32_t root_idx, NodeT* nodes,
                         CreateFunc modify_fn, HomogeneousFunc modify_homogeneous_fn,
                         const glm::vec3& chunk_origin, float chunk_size, uint32_t max_depth,
                         std::vector<uint32_t>& out_pages, std::vector<glm::vec4>& aabbs)

{
    Contree3DModifier<CreateFunc, HomogeneousFunc> modifier;
    modifier.out_minmax = glm::uvec2(0);
    modifier.allocator = allocator;
    modifier.nodes = nodes;
    modifier.modify_fn = modify_fn;
    modifier.modify_homogeneous_fn = modify_homogeneous_fn;
    modifier.max_depth = max_depth;
    modifier.aabbs = &aabbs;
    modifier.modify_3d_dfs_rec(chunk_origin, chunk_size, root_idx, 0);
    return modifier.out_minmax;
}

template <typename CreateFunc, typename HomogeneousFunc>
static bool create_3d_dfs_fixed_page(NodeAllocator* allocator, uint32_t root_idx, NodeT* nodes,
                                     CreateFunc create_fn, HomogeneousFunc homogeneous_fn,
                                     const glm::vec3& chunk_origin, float chunk_size,
                                     uint32_t max_depth, std::vector<uint32_t>& out_pages)
{
    struct StackFrame
    {
        uint32_t node_idx;
        glm::vec3 origin;
        float size;
        int depth;
    };

    NodeT root_node;
    root_node.clear_bits();
    root_node.parent = UINT32_MAX;
    root_node.firstchild = UINT32_MAX;

    if (!create_fn(chunk_origin, chunk_size, &root_node.data))
    {
        return false;
    }

    auto allocate_nodes = [&]()
    {
        uint32_t absolute_index = allocator->allocate();
        if (absolute_index == UINT32_MAX)
        {
            printf("out of mem");
            exit(1);
        }
        out_pages.push_back(absolute_index / 64);
        return absolute_index;
    };

    nodes[root_idx] = root_node;
    allocate_nodes();

    auto free_nodes = [&](uint32_t page_offset) { allocator->free(page_offset); };

    thread_local std::vector<StackFrame> stack;
    stack.clear();
    stack.push_back({root_idx, chunk_origin, chunk_size, 0});

    while (!stack.empty())
    {
        StackFrame frame = stack.back();
        stack.pop_back();
        NodeT node = nodes[frame.node_idx];

        if (homogeneous_fn(frame.origin, frame.size) || frame.depth >= max_depth)
        {
            continue;
        }
        uint32_t first_child_idx = allocate_nodes();

        uint32_t num_nodes = 0;
        for (int z = 0; z < 4; z++)
            for (int y = 0; y < 4; y++)
                for (int x = 0; x < 4; x++)
                {
                    float child_size = frame.size * 0.25f;
                    glm::vec3 child_origin = frame.origin + child_size * glm::vec3(x, y, z);

                    NodeT child_node;

                    bool created = create_fn(child_origin, child_size, &child_node.data);
                    if (created)
                    {
                        child_node.clear_bits();
                        child_node.firstchild = UINT32_MAX;
                        child_node.parent = frame.node_idx;
                        uint32_t flat_idx = x + 4 * y + 16 * z;
                        nodes[frame.node_idx].set_bit(flat_idx);

                        nodes[first_child_idx + flat_idx] = child_node;
                        stack.emplace_back(first_child_idx + flat_idx, child_origin, child_size,
                                           frame.depth + 1);
                        num_nodes++;
                    }
                }
        if (num_nodes != 0)
        {
            nodes[frame.node_idx].firstchild = first_child_idx;
        }
        else
        {
            free_nodes(first_child_idx);
        }
    }
    printf("pages %u\n", out_pages.size());
    return true;
}

static bool create_3d_dfs_mesh_fixed_page(NodeAllocator* allocator, uint32_t root_idx, NodeT* nodes,
                                          ame_bvh& bvh, const glm::vec3& chunk_origin,
                                          float chunk_size, uint32_t max_depth,
                                          std::vector<uint32_t>& out_pages)
{
    struct StackFrame
    {
        uint32_t node_idx;
        glm::vec3 origin;
        float size;
        int depth;
    };

    NodeT root_node;
    root_node.clear_bits();
    root_node.parent = UINT32_MAX;
    root_node.firstchild = UINT32_MAX;
    BVHIntersectAABBInfo root_info;

    if (!bvh_aabb_intersect(&root_info, bvh, 0, chunk_origin, chunk_origin + chunk_size))
    {
        return false;
    }
    root_node.data = 0xfff;
    root_node.data |= TerrainGen::pack_normal(root_info.normal) << 12;

    auto allocate_nodes = [&]()
    {
        uint32_t absolute_index = allocator->allocate();
        if (absolute_index == UINT32_MAX)
        {
            printf("out of mem");
            exit(1);
        }
        out_pages.push_back(absolute_index / 64);
        return absolute_index;
    };

    nodes[root_idx] = root_node;
    allocate_nodes();

    auto free_nodes = [&](uint32_t page_offset) { allocator->free(page_offset); };

    thread_local std::vector<StackFrame> stack;
    stack.clear();
    stack.push_back({root_idx, chunk_origin, chunk_size, 0});

    while (!stack.empty())
    {
        StackFrame frame = stack.back();
        stack.pop_back();
        NodeT node = nodes[frame.node_idx];

        if (frame.depth >= max_depth)
        {
            continue;
        }
        uint32_t first_child_idx = allocate_nodes();

        uint32_t num_nodes = 0;
        for (int z = 0; z < 4; z++)
            for (int y = 0; y < 4; y++)
                for (int x = 0; x < 4; x++)
                {
                    float child_size = frame.size * 0.25f;
                    glm::vec3 child_origin = frame.origin + child_size * glm::vec3(x, y, z);

                    NodeT child_node;

                    BVHIntersectAABBInfo info;
                    bool created = bvh_aabb_intersect(&info, bvh, 0, child_origin,
                                                      child_origin + child_size);
                    if (created)
                    {
                        child_node.clear_bits();
                        child_node.firstchild = UINT32_MAX;
                        child_node.parent = frame.node_idx;
                        child_node.data = 0xfffu;
                        child_node.data |= TerrainGen::pack_normal(info.normal) << 12;
                        uint32_t flat_idx = x + 4 * y + 16 * z;
                        nodes[frame.node_idx].set_bit(flat_idx);

                        nodes[first_child_idx + flat_idx] = child_node;
                        stack.emplace_back(first_child_idx + flat_idx, child_origin, child_size,
                                           frame.depth + 1);
                        num_nodes++;
                    }
                }
        if (num_nodes != 0)
        {
            nodes[frame.node_idx].firstchild = first_child_idx;
        }
        else
        {
            free_nodes(first_child_idx);
        }
    }
    printf("pages %u\n", out_pages.size());
    return true;
}