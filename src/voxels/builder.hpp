#pragma once

#include "allocator/node_allocator.hpp"
#include "node.hpp"
#include <glm/glm.hpp>

#include <queue>
#include <vector>

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
    for (int i = 0; i < 64; i++)
    {
        if (nodes[node_idx].get_bit(i))
        {
            free_children_3d_fixed_page(allocator, first_child + i, nodes);
        }
    }
    // printf("free %u\n", first_child);
    allocator->free(first_child);
}

using create_pfn = bool (*)(const glm::vec3& origin, float size, NodeT::data_t* data);
using homogeneous_pfn = bool (*)(const glm::vec3& origin, float size);

struct Contree3DModifier
{
    NodeAllocator* allocator;
    NodeT* nodes;
    uint32_t num_nodes;
    create_pfn modify_fn;
    homogeneous_pfn modify_homogeneous_fn;
    uint32_t max_depth;
    std::vector<glm::vec4>* aabbs;
    glm::uvec2 out_minmax = glm::uvec2(UINT32_MAX, 0);

    std::string s = "            ";
    bool erase = false;

    void update_minmax(uint32_t idx)
    {
        out_minmax.x = glm::min(out_minmax.x, idx);
        out_minmax.y = glm::max(out_minmax.y, idx);
    }

    void update_minmax(uint32_t min_idx, uint32_t max_idx)
    {
        out_minmax.x = glm::min(out_minmax.x, min_idx);
        out_minmax.y = glm::max(out_minmax.y, max_idx);
    }

    void modify_3d_dfs_iter(const glm::vec3& origin, float size, uint32_t node_idx, uint32_t depth)
    {
        struct StackFrame
        {
            glm::vec3 origin;
            float size;
            uint32_t node_idx;
            uint32_t depth;
        };
        std::vector<StackFrame> stack;
        stack.push_back({origin, size, node_idx, depth});

        while (!stack.empty())
        {
            StackFrame frame = stack.back();
            stack.pop_back();

            if (frame.node_idx == UINT32_MAX)
                continue;

            NodeT& node = nodes[frame.node_idx];
            bool is_homogeneous = modify_homogeneous_fn(frame.origin, frame.size);
            uint32_t data;
            bool affects_volume = modify_fn(frame.origin, frame.size, &data);

            if (!affects_volume)
                continue;

            update_minmax(frame.node_idx);

            if (frame.depth >= max_depth)
            {
                node.data = data;
                continue;
            }

            if (is_homogeneous)
            {
                free_children_3d_fixed_page(allocator, frame.node_idx, nodes);
                node.firstchild = UINT32_MAX;
                node.clear_bits();

                node.data = data;
                continue;
            }

            if (node.firstchild == UINT32_MAX)
            {
                uint32_t first_child = allocator->allocate();
                update_minmax(first_child, first_child + 64);

                assert(first_child != 0);

                if (first_child == UINT32_MAX)
                    exit(1);
                node.firstchild = first_child;
            }

            for (uint32_t i = 0; i < 64; i++)
            {
                uint32_t child_idx = node.firstchild + i;

                if (!node.get_bit(i))
                {
                    NodeT& child_node = nodes[child_idx];
                    child_node.firstchild = UINT32_MAX;
                    child_node.parent = frame.node_idx;
                    child_node.data = node.data;
                    child_node.clear_bits();
                }
            }
            node.childmask.fill_mask(0, 63);

            for (uint32_t i = 0; i < 64; i++)
            {
                uint32_t child_idx = node.firstchild + i;

                glm::vec3 child_local_origin = glm::vec3(i % 4, (i / 4) % 4, i / 16);
                float child_size = frame.size * 0.25f;

                glm::vec3 child_origin = frame.origin + child_size * child_local_origin;
                stack.push_back({child_origin, child_size, child_idx, frame.depth + 1});
            }
        }
    }

    void modify_3d_dfs_iter2(const glm::vec3& origin, float size, uint32_t node_idx, uint32_t depth)
    {
        struct StackFrame
        {
            glm::vec3 origin;
            float size;
            uint32_t node_idx;
            uint32_t depth;
        };
        std::vector<StackFrame> stack;
        stack.push_back({origin, size, node_idx, depth});

        while (!stack.empty())
        {
            StackFrame frame = stack.back();
            stack.pop_back();

            if (frame.node_idx == UINT32_MAX)
                continue;

            NodeT& node = nodes[frame.node_idx];
            bool is_homogeneous = modify_homogeneous_fn(frame.origin, frame.size);

            update_minmax(frame.node_idx);

            if (is_homogeneous)
            {
                free_children_3d_fixed_page(allocator, frame.node_idx, nodes);
                node.firstchild = UINT32_MAX;
                node.clear_bits();

                uint32_t data;
                modify_fn(frame.origin, frame.size, &node.data);
                continue;
            }

            if (node.firstchild == UINT32_MAX)
            {
                uint32_t first_child = allocator->allocate();
                update_minmax(first_child, first_child + 64);

                if (first_child == UINT32_MAX)
                    exit(1);
                node.firstchild = first_child;
            }

            for (uint32_t i = 0; i < 64; i++)
            {
                uint32_t child_idx = node.firstchild + i;

                if (!node.get_bit(i))
                {
                    NodeT& child_node = nodes[child_idx];
                    child_node.firstchild = UINT32_MAX;
                    child_node.parent = frame.node_idx;
                    child_node.data = node.data;
                    child_node.clear_bits();
                }
            }
            node.childmask.fill_mask(0, 63);

            uint32_t num_nodes = 0;
            for (int z = 0; z < 4; z++)
                for (int y = 0; y < 4; y++)
                    for (int x = 0; x < 4; x++)
                    {
                        uint32_t flat_idx = x + 4 * y + 16 * z;
                        uint32_t child_idx = node.firstchild + flat_idx;

                        float child_size = frame.size * 0.25f;
                        glm::vec3 child_origin = frame.origin + child_size * glm::vec3(x, y, z);

                        NodeT& child_node = nodes[child_idx];

                        uint32_t data;
                        bool modified = modify_fn(child_origin, child_size, &data);
                        if (modified)
                        {
                            num_nodes++;
                            if (frame.depth + 1 < max_depth)
                                stack.push_back(
                                    {child_origin, child_size, child_idx, frame.depth + 1});
                            else
                                child_node.data = data;
                        }
                    }
        }
    }

    void modify_3d_dfs_iter3(const glm::vec3& origin, float size, uint32_t node_idx, uint32_t depth)
    {
        struct StackFrame
        {
            glm::vec3 origin;
            float size;
            uint32_t node_idx;
            uint32_t depth;
        };
        std::vector<StackFrame> stack;
        stack.push_back({origin, size, node_idx, depth});
        std::queue<uint32_t> deepest_non_leaves;
        std::queue<glm::vec4> deepest_non_leaves_aabbs;

        while (!stack.empty())
        {
            StackFrame frame = stack.back();
            stack.pop_back();

            if (frame.node_idx == UINT32_MAX)
                continue;

            NodeT& node = nodes[frame.node_idx];
            bool is_homogeneous = modify_homogeneous_fn(frame.origin, frame.size);

            update_minmax(frame.node_idx);

            if (is_homogeneous)
            {
                free_children_3d_fixed_page(allocator, frame.node_idx, nodes);
                node.firstchild = UINT32_MAX;
                node.clear_bits();

                uint32_t data;
                modify_fn(frame.origin, frame.size, &node.data);
                continue;
            }

            if (node.firstchild == UINT32_MAX)
            {
                uint32_t first_child = allocator->allocate();
                update_minmax(first_child, first_child + 64);

                if (first_child == UINT32_MAX)
                    exit(1);
                node.firstchild = first_child;
            }

            for (uint32_t i = 0; i < 64; i++)
            {
                uint32_t child_idx = node.firstchild + i;

                if (!node.get_bit(i))
                {
                    NodeT& child_node = nodes[child_idx];
                    child_node.firstchild = UINT32_MAX;
                    child_node.parent = frame.node_idx;
                    child_node.data = node.data;
                    child_node.clear_bits();
                }
            }
            node.childmask.fill_mask(0, 63);

            for (int z = 0; z < 4; z++)
                for (int y = 0; y < 4; y++)
                    for (int x = 0; x < 4; x++)
                    {
                        uint32_t flat_idx = x + 4 * y + 16 * z;
                        uint32_t child_idx = node.firstchild + flat_idx;

                        float child_size = frame.size * 0.25f;
                        glm::vec3 child_origin = frame.origin + child_size * glm::vec3(x, y, z);

                        NodeT& child_node = nodes[child_idx];

                        uint32_t data;
                        bool modified = modify_fn(child_origin, child_size, &data);
                        if (modified)
                        {
                            if (frame.depth + 1 < max_depth)
                            {
                                StackFrame child_frame = {child_origin, child_size, child_idx,
                                                          frame.depth + 1};
                                stack.push_back(child_frame);
                            }
                            else
                            {
                                child_node.data = data;
                            }
                        }
                    }
            if (frame.depth + 1 >= max_depth)
            {
                deepest_non_leaves.push(frame.node_idx);
                deepest_non_leaves_aabbs.push(glm::vec4(frame.origin, frame.size));
            }
        }
        printf("TODO");
        return;
        if (erase)
            while (!deepest_non_leaves.empty())
            {
                uint32_t node_idx = deepest_non_leaves.front();
                glm::vec4 aabb = deepest_non_leaves_aabbs.front();
                deepest_non_leaves.pop();
                deepest_non_leaves_aabbs.pop();
                if (node_idx == UINT32_MAX)
                    continue;

                bool homogeneous = true;
                uint32_t data = UINT32_MAX;
                uint32_t full_data = data;
                NodeT& node = nodes[node_idx];
                if (node.firstchild == UINT32_MAX)
                    continue;

                for (int i = 0; i < 64; i++)
                {
                    uint32_t node_data = 0;
                    uint32_t child_idx = node.firstchild + i;
                    if (node.get_bit(i))
                    {
                        node_data = nodes[child_idx].data;
                    }

                    if (data == UINT32_MAX)
                    {
                        data = node_data & ((1u << 12u) - 1u);
                        full_data = node_data;
                    }
                    node_data = node_data & ((1u << 12u) - 1u);

                    if (data != node_data)
                    {
                        homogeneous = false;
                        break;
                    }
                }

                if (homogeneous && data != UINT32_MAX)
                {
                    aabbs->push_back(aabb);
                    free_children_3d_fixed_page(allocator, node_idx, nodes);
                    node.data = full_data;
                    node.firstchild = UINT32_MAX;
                    node.clear_bits();
                    if (node.parent != UINT32_MAX)
                    {
                        deepest_non_leaves.push(node.parent);
                        deepest_non_leaves_aabbs.push(
                            glm::vec4(glm::floor(glm::xyz(aabb) / (aabb.w * 4.f)) * aabb.w * 4.f,
                                      aabb.w * 4.f));
                    }
                }
            }
    }

    void modify_3d_dfs_rec(const glm::vec3& origin, float size, uint32_t node_idx, uint32_t depth)
    {
        if (node_idx == UINT32_MAX)
            return;

        uint32_t data;
        bool affects_volume = modify_fn(origin, size, &data);
        if (!affects_volume)
            return;

        update_minmax(node_idx);

        assert(node_idx < num_nodes);

        NodeT& node = nodes[node_idx];

        if (depth >= max_depth)
        {
            node.data = data;
            return;
        }

        bool is_edit_homogeneous = modify_homogeneous_fn(origin, size);
        if (is_edit_homogeneous)
        {
            free_children_3d_fixed_page(allocator, node_idx, nodes);
            node.firstchild = UINT32_MAX;
            node.clear_bits();
            node.data = data;
            return;
        }

        if (node.firstchild == UINT32_MAX)
        {
            uint32_t first_child = allocator->allocate();
            // printf("alloc %u\n", first_child);
            // assert(!(node_idx >= first_child && node_idx < first_child + 64));
            update_minmax(first_child, first_child + 64);

            assert(first_child != 0);

            if (first_child == UINT32_MAX)
                exit(1);
            node.firstchild = first_child;
        }

        for (uint32_t i = 0; i < 64; i++)
        {
            uint32_t child_idx = node.firstchild + i;

            if (!node.get_bit(i))
            {
                NodeT& child_node = nodes[child_idx];
                child_node.firstchild = UINT32_MAX;
                child_node.parent = node_idx;
                child_node.data = node.data;
                child_node.clear_bits();
            }
        }
        node.childmask.fill_mask(0, 63);

        for (uint32_t i = 0; i < 64; i++)
        {
            uint32_t child_idx = node.firstchild + i;

            glm::vec3 child_local_origin = glm::vec3(i % 4, (i / 4) % 4, i / 16);
            float child_size = size * 0.25f;

            glm::vec3 child_origin = origin + child_size * child_local_origin;
            modify_3d_dfs_rec(child_origin, child_size, child_idx, depth + 1);
        }
    }
};

template <typename CreateFunc, typename HomogeneousFunc>
static bool modify_3d_dfs_fixed_page_iter(NodeAllocator* allocator, uint32_t root_idx, NodeT* nodes,
                                          uint32_t num_nodes, CreateFunc modify_fn,
                                          HomogeneousFunc modify_homogeneous_fn,
                                          const glm::vec3& chunk_origin, float chunk_size,
                                          uint32_t max_depth, std::vector<uint32_t>& out_pages,
                                          std::vector<glm::vec4>& aabbs)

{
    struct StackFrame
    {
        uint32_t node_idx;
        glm::vec3 origin;
        float size;
        int depth;
    };
    thread_local std::vector<StackFrame> stack;
    stack.push_back({root_idx, chunk_origin, chunk_size, 0});

    while (!stack.empty())
    {
        StackFrame frame = stack.back();
        stack.pop_back();

        NodeT& node = nodes[frame.node_idx];

        uint32_t first_child = node.firstchild;
        if (modify_homogeneous_fn(frame.origin, frame.size))
        {
            if (first_child != UINT32_MAX)
            {
                free_children_3d_fixed_page(allocator, frame.node_idx, nodes);
            }
            node.clear_bits();
            node.firstchild = UINT32_MAX;
            continue;
        }
        else
        {
            if (frame.depth >= max_depth)
                continue;
            if (first_child == UINT32_MAX)
            {
                aabbs.push_back(glm::vec4(frame.origin, frame.size));
                first_child = allocator->allocate();
                node.firstchild = first_child;
                node.childmask.fill_mask(0, 63);
                for (int i = 0; i < 64; i++)
                {
                    nodes[first_child + i].data = node.data;
                    nodes[first_child + i].parent = frame.node_idx;
                    nodes[first_child + i].firstchild = UINT32_MAX;
                    nodes[first_child + i].clear_bits();
                }
            }
            for (int i = 0; i < 64; i++)
            {
                StackFrame child;
                child.depth = frame.depth + 1;
                child.size = frame.size / 4;
                child.origin = frame.origin + child.size * glm::vec3(i % 4, (i / 4) % 4, i / 16);
                child.node_idx = first_child + i;

                uint32_t data = 0;
                bool modify_child = modify_fn(child.origin, child.size, &data);
                if (modify_child)
                {
                    if (!node.get_bit(i))
                    {
                        nodes[child.node_idx].parent = frame.node_idx;
                    }
                    nodes[child.node_idx].data = data;
                    stack.push_back(child);

                    node.childmask.set(i);
                }
            }
        }
    }
    return true;
}

template <typename CreateFunc, typename HomogeneousFunc>
static uint32_t create_3d_dfs_fixed_page(NodeAllocator* allocator, NodeT* nodes,
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
        return UINT32_MAX;
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
    auto free_nodes = [&](uint32_t page_offset) { allocator->free(page_offset); };

    uint32_t root_idx = allocate_nodes();
    nodes[root_idx] = root_node;

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
    return root_idx;
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